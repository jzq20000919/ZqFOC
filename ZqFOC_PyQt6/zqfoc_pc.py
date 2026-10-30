"""ZqFOC 串口上位机：PyQt6 负责界面，pyserial 负责串口，pyqtgraph 负责曲线。

阅读顺序与职责：
1. 协议常量：USART2 为 2,000,000 bps、8N1；命令以换行 LF 结束。
2. TraceBox：管理一个图表中的目标 Ref 和反馈 Actual 两条线。
3. MotorMonitor：创建界面，管理连接、命令、接收缓存及历史数据。
4. 三个 Qt 定时器：每 10 ms 收包，每 50 ms 绘图，每 1 s 更新连接统计。
5. main：创建 Qt 应用和主窗口，进入事件循环。

配套固件为 User/foc_comm_usart.c/.h。PC 发送 MS、MP、START、STOP、S速度、A角度；
MCU 回传十个小端 float32（40 字节）及帧尾 00 00 80 7F（4 字节）。通道依次是：
速度目标/反馈（rpm）、Id 目标/反馈（A）、Iq 目标/反馈（A）、Vd/Vq（V）、
单圈机械角目标/反馈（度）。固件内部角度使用 rad，在通信模块里换算成度。

串口收到的字节不一定是一整帧：receive_data 保留半帧并处理连续多帧。
所有操作都在 Qt 主线程；串口读取不等待，收包与绘图分开，避免每帧都重绘界面。
固件没有 UART 命令回执或运行状态字段，界面“已发送”只表示写入串口成功。
"""

from __future__ import annotations

import math
import struct
import sys
import time
from collections import deque

import pyqtgraph as pg
import serial
from serial.tools import list_ports
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

# 一、通信协议和显示容量：与 STM32 帧格式、通道顺序保持一致。
BAUD_RATE = 2_000_000
FRAME_TAIL = b"\x00\x00\x80\x7f"
FRAME_BYTES = 44  # 十个 float32 加四字节 JustFloat 帧尾
VIEW_SECONDS = 8.0
MAX_HISTORY = 1200  # 约 100 Hz 的显示采样，可保存约 12 秒，绘图只取最近 8 秒


# 二、图表组件：统一四张图的外观和曲线更新方法。
class TraceBox(QGroupBox):
    """一个图表容器，橙线显示目标值，蓝线显示实际反馈。"""

    def __init__(self, title: str, unit: str):
        """创建带坐标轴、图例和两条曲线的图表；title 是标题，unit 是纵轴单位。"""
        super().__init__(title)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 16, 10, 8)

        self.plot = pg.PlotWidget(background="#101a2a")
        self.plot.setMenuEnabled(False)
        self.plot.showGrid(x=True, y=True, alpha=0.18)
        self.plot.setLabel("left", unit)
        self.plot.setLabel("bottom", "距当前", units="s")
        self.plot.setXRange(-VIEW_SECONDS, 0, padding=0)
        self.plot.setMouseEnabled(x=False, y=False)
        self.plot.getPlotItem().setClipToView(True)
        self.plot.getPlotItem().addLegend(offset=(8, 6))
        for axis_name in ("left", "bottom"):
            self.plot.getAxis(axis_name).setTextPen("#9caec5")
            self.plot.getAxis(axis_name).setPen("#415169")

        # 四张图统一配色：目标为橙色，实际反馈为蓝色。
        self.ref_line = self.plot.plot(
            pen=pg.mkPen("#ffbf69", width=2), name="Ref"
        )
        self.actual_line = self.plot.plot(
            pen=pg.mkPen("#56c7ea", width=2), name="Actual"
        )
        layout.addWidget(self.plot)

    def update_traces(self, x: list[float], ref: list[float], actual: list[float]):
        """用同一组时间坐标 x（秒）更新目标 ref 和反馈 actual；三个序列等长。"""
        self.ref_line.setData(x, ref)
        self.actual_line.setData(x, actual)

    def clear(self):
        """清除两条曲线的显示数据，保留坐标轴和图例。"""
        self.ref_line.clear()
        self.actual_line.clear()


# 三、主窗口：管理界面、串口和数据，按钮通过 Qt 信号连接到处理函数。
class MotorMonitor(QMainWindow):
    """上位机主窗口；不在启动时自动连接串口或发送电机命令。"""
    def __init__(self):
        """初始化连接状态和缓存，创建界面及三个定时器；连接成功后才启动计时。"""
        super().__init__()
        self.setWindowTitle("ZqFOC · PyQt6 上位机")
        self.resize(1250, 890)
        self.setMinimumSize(960, 720)

        self.port: serial.Serial | None = None
        self.recv_buf = bytearray()
        # history 每项在十个通道前添加 PC 单调时钟（秒），因此绘图索引比帧通道大 1。
        # 每项：(时间, 速度目标, 速度反馈, Id目标, Id反馈, Iq目标, Iq反馈, Vd, Vq, 角度目标, 角度反馈)
        self.history: deque[tuple[float, ...]] = deque(maxlen=MAX_HISTORY)
        self.latest: tuple[float, ...] | None = None
        self.last_rx_time = 0.0
        self.frame_count = 0
        
        self._build_ui()
        self.refresh_ports()

        # 三个任务共用 Qt 事件循环，不创建后台线程，避免跨线程操作界面。
        # 收包定时器：轮询操作系统串口缓存，一次可能拿到半帧或多帧。
        self.rx_timer = QTimer(self)
        self.rx_timer.setInterval(10)
        self.rx_timer.timeout.connect(self.receive_data)

        # 绘图定时器：只画收包任务留下的最新数据，降低重绘开销。
        self.chart_timer = QTimer(self)
        self.chart_timer.setInterval(50)  # 绘图 20 fps；MCU 遥测最多 1 kHz，两者频率独立。
        self.chart_timer.timeout.connect(self.update_charts)

        # 状态定时器：统计每秒有效帧数，检查遥测是否长时间没有更新。
        self.status_timer = QTimer(self)
        self.status_timer.setInterval(1000)
        self.status_timer.timeout.connect(self.update_link_status)

    def _build_ui(self):
        """建立连接区、启停/目标控制区、数值卡片、四张曲线及统一样式。"""
        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(17, 14, 17, 12)
        outer.setSpacing(10)

        title = QLabel("ZqFOC 电机控制与实时监控")
        title.setObjectName("title")
        outer.addWidget(title)
        subtitle = QLabel("STM32G431  ·  USART2  ·  2,000,000 bps  ·  JustFloat")
        subtitle.setObjectName("subtle")
        outer.addWidget(subtitle)

        # 界面 A：串口选择、刷新、连接按钮及连接统计。
        conn_group = QGroupBox("串口连接")
        row = QHBoxLayout(conn_group)
        row.addWidget(QLabel("端口"))
        self.port_list = QComboBox()
        self.port_list.setMinimumWidth(215)
        row.addWidget(self.port_list)
        refresh = QPushButton("刷新端口")
        refresh.clicked.connect(self.refresh_ports)
        row.addWidget(refresh)
        self.connect_button = QPushButton("连接")
        self.connect_button.setObjectName("connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        row.addWidget(self.connect_button)
        self.link_label = QLabel("未连接")
        self.link_label.setObjectName("subtle")
        row.addWidget(self.link_label, stretch=1)
        outer.addWidget(conn_group)

        # 界面 B：启停、速度模式/目标、位置模式/目标；这里只绑定信号，不发送命令。
        controls = QHBoxLayout()
        controls.setSpacing(10)

        motor_group = QGroupBox("电机启停")
        motor_row = QVBoxLayout(motor_group)
        buttons = QHBoxLayout()
        self.start_button = QPushButton("启动 START")
        self.start_button.setObjectName("start")
        self.start_button.clicked.connect(self.start_motor)
        self.stop_button = QPushButton("停止 STOP")
        self.stop_button.setObjectName("stop")
        self.stop_button.clicked.connect(self.stop_motor)
        buttons.addWidget(self.start_button)
        buttons.addWidget(self.stop_button)
        motor_row.addLayout(buttons)
        controls.addWidget(motor_group, 3)

        speed_group = QGroupBox("速度控制")
        speed_row = QVBoxLayout(speed_group)
        self.speed_mode_button = QPushButton("切换速度模式 (MS)")
        self.speed_mode_button.clicked.connect(lambda: self.send_mode("MS"))
        speed_row.addWidget(self.speed_mode_button)
        speed_fields = QHBoxLayout()
        self.speed_input = QSpinBox()
        self.speed_input.setRange(-2600, 2600)
        self.speed_input.setSingleStep(100)
        self.speed_input.setValue(300)
        self.speed_input.setSuffix(" rpm")
        speed_fields.addWidget(self.speed_input, 1)
        self.speed_apply = QPushButton("设置速度")
        self.speed_apply.clicked.connect(self.set_speed)
        speed_fields.addWidget(self.speed_apply)
        speed_row.addLayout(speed_fields)
        controls.addWidget(speed_group, 4)

        position_group = QGroupBox("位置控制（单圈）")
        position_row = QVBoxLayout(position_group)
        self.position_mode_button = QPushButton("切换位置模式 (MP)")
        self.position_mode_button.clicked.connect(lambda: self.send_mode("MP"))
        position_row.addWidget(self.position_mode_button)
        pos_fields = QHBoxLayout()
        self.position_input = QDoubleSpinBox()
        self.position_input.setDecimals(2)
        self.position_input.setRange(0.0, 359.99)
        self.position_input.setSingleStep(10.0)
        self.position_input.setValue(90.0)
        self.position_input.setSuffix(" °")
        pos_fields.addWidget(self.position_input, 1)
        self.position_apply = QPushButton("设置位置")
        self.position_apply.clicked.connect(self.set_position)
        pos_fields.addWidget(self.position_apply)
        position_row.addLayout(pos_fields)
        controls.addWidget(position_group, 4)
        outer.addLayout(controls)

        self.command_info = QLabel("操作：连接串口 → 选模式 → 启动 → 等待约 2 秒对齐完成 → 设置目标")
        self.command_info.setObjectName("hint")
        self.command_info.setWordWrap(True)
        outer.addWidget(self.command_info)

        # 界面 C：最新有效帧的四个数值卡片。
        metrics = QHBoxLayout()
        self.speed_value = QLabel("速度  -- / -- rpm")
        self.iq_value = QLabel("Iq  -- / -- A")
        self.id_value = QLabel("Id  -- / -- A")
        self.pos_value = QLabel("位置  -- / -- °")
        for value in (self.speed_value, self.iq_value, self.id_value, self.pos_value):
            value.setObjectName("metric")
            value.setAlignment(Qt.AlignmentFlag.AlignCenter)
            metrics.addWidget(value)
        outer.addLayout(metrics)

        # 界面 D：速度、机械角、Iq 和 Id 的目标/反馈曲线。
        charts = QGridLayout()
        charts.setHorizontalSpacing(10)
        charts.setVerticalSpacing(10)
        self.speed_graph = TraceBox("速度 · Ref / Actual", "rpm")
        self.position_graph = TraceBox("机械角度 · Ref / Actual", "°")
        self.iq_graph = TraceBox("q 轴电流 · Ref / Actual", "A")
        self.id_graph = TraceBox("d 轴电流 · Ref / Actual", "A")
        charts.addWidget(self.speed_graph, 0, 0)
        charts.addWidget(self.position_graph, 0, 1)
        charts.addWidget(self.iq_graph, 1, 0)
        charts.addWidget(self.id_graph, 1, 1)
        charts.setRowStretch(0, 1)
        charts.setRowStretch(1, 1)
        charts.setColumnStretch(0, 1)
        charts.setColumnStretch(1, 1)
        outer.addLayout(charts, stretch=1)

        foot = QLabel(
            "注意：固件 UART 无 ACK/运行状态字段，‘已发送’不等于已执行。"
            "断线、关窗口不能替代硬件断电/急停。"
        )
        foot.setWordWrap(True)
        foot.setObjectName("subtle")
        outer.addWidget(foot)

        # 界面 E：全局 Qt 样式；对象名用于设置标题、启停按钮等局部颜色。
        self.setStyleSheet("""
            QWidget { background: #0c1422; color: #e0e9f5;
                      font-size: 12px; }
            QLabel#title { font-size: 22px; font-weight: 700; color: #f5f8fc; }
            QLabel#subtle { color: #94a6bd; }
            QLabel#hint { color: #ffcd88; padding: 3px 2px; }
            QLabel#metric { background: #182338; border: 1px solid #2a3850;
                            border-radius: 7px; padding: 9px 3px; font-weight: 600; }
            QGroupBox { border: 1px solid #293950; border-radius: 8px;
                        margin-top: 10px; padding-top: 6px;
                        font-size: 12px; font-weight: 600; }
            QGroupBox::title { subcontrol-origin: margin; left: 11px; padding: 0 4px;
                               color: #afc5db; }
            QPushButton { background: #243650; border: 1px solid #3a5272;
                          border-radius: 6px; padding: 8px 12px; }
            QPushButton:hover { background: #304866; }
            QPushButton:pressed { background: #1c2a3d; }
            QPushButton:disabled { background: #1c293a; color: #68809b; }
            QPushButton#connect { background: #19558c; border-color: #2872a8; }
            QPushButton#start { background: #17634b; border-color: #21845e; }
            QPushButton#stop { background: #953b40; border-color: #bd5159; }
            QSpinBox, QDoubleSpinBox, QComboBox { background: #17243a;
                       border: 1px solid #3c5270; border-radius: 5px;
                       padding: 6px; min-height: 20px; }
            QComboBox QAbstractItemView { background: #17243a; color: #eef4ff;
                                         selection-background-color: #365d83; }
        """)
        self._set_control_enabled(False)

    # 四、串口连接与资源释放。
    def _set_control_enabled(self, enabled: bool):
        """根据 enabled 统一启用或禁用命令按钮，防止未连接时操作。"""
        for widget in (
            self.start_button, self.stop_button,
            self.speed_mode_button, self.position_mode_button,
            self.speed_apply, self.position_apply,
        ):
            widget.setEnabled(enabled)

    def refresh_ports(self):
        """枚举系统串口并尽量保留原选择；已连接时不更改端口列表。"""
        if self.port is not None:
            return
        selected = self.port_list.currentData()
        self.port_list.clear()
        for info in sorted(list_ports.comports(), key=lambda p: p.device):
            self.port_list.addItem(f"{info.device}  ({info.description})", info.device)
        if self.port_list.count() == 0:
            self.port_list.addItem("未发现串口", None)
        else:
            idx = self.port_list.findData(selected)
            if idx >= 0:
                self.port_list.setCurrentIndex(idx)
        self.connect_button.setEnabled(self.port_list.currentData() is not None)

    def toggle_connection(self):
        """连接按钮入口：未连接时打开串口，已连接时尝试 STOP 后断开。"""
        if self.port is None:
            self.connect_port()
        else:
            self.disconnect_port(send_stop=True)

    def connect_port(self):
        """按固定波特率打开所选串口，清空旧数据，启用控制并启动三个定时器。"""
        device = self.port_list.currentData()
        if not device:
            return
        try:
            # timeout=0：读取不等待；写入最长等待 0.3 秒，避免异常时无限阻塞。
            port = serial.Serial(device, BAUD_RATE, timeout=0, write_timeout=0.3)
            port.reset_input_buffer()
            self.port = port
        except (serial.SerialException, OSError, ValueError) as exc:
            QMessageBox.warning(self, "连接失败", f"无法打开 {device}：\n{exc}")
            return

        self.recv_buf.clear()
        self.clear_charts()
        self.last_rx_time = time.monotonic()
        self.frame_count = 0
        self.link_label.setText(f"{device} 已连接 · 等待遥测（停机时无数据）")
        self.command_info.setText("已连接。先选择模式；START 后等待约 2 秒对齐，再设置目标。")
        self.connect_button.setText("断开")
        self.port_list.setEnabled(False)
        self._set_control_enabled(True)
        self.rx_timer.start()
        self.chart_timer.start()
        self.status_timer.start()

    def disconnect_port(self, send_stop: bool = True):
        """停止定时器并关闭串口；send_stop=True 时尽力发送 STOP，但无回执保证。"""
        self.rx_timer.stop()
        self.chart_timer.stop()
        self.status_timer.stop()
        if self.port is not None:
            if send_stop:
                try:
                    # 关闭前尽力请求停机；无回执，成功写出也不保证 MCU 已停机。
                    self.port.write(b"STOP\n")
                    self.port.flush()
                except (serial.SerialException, serial.SerialTimeoutException, OSError):
                    pass
            try:
                self.port.close()
            except (serial.SerialException, OSError):
                pass
            self.port = None
        self._set_control_enabled(False)
        self.port_list.setEnabled(True)
        self.connect_button.setText("连接")
        self.link_label.setText("未连接")
        self.connect_button.setEnabled(self.port_list.currentData() is not None)

    # 五、命令发送：所有按钮都通过 _send 添加统一的换行结束符。
    def _send(self, command: str) -> bool:
        """追加 LF 并按 ASCII 写入命令；返回写入是否成功，异常时断开连接。"""
        if self.port is None:
            QMessageBox.information(self, "串口未连接", "请先连接串口。")
            return False
        try:
            self.port.write((command + "\n").encode("ascii"))
            return True
        except (serial.SerialException, serial.SerialTimeoutException, OSError) as exc:
            self.serial_error(exc)
            return False

    def send_mode(self, command: str):
        """发送 MS（速度）或 MP（位置）；只显示命令已发送，不推断电机状态。"""
        if self._send(command):
            mode_name = "速度" if command == "MS" else "位置"
            self.command_info.setText(f"已发送切换{mode_name}模式命令（固件不回 ACK）。")

    def start_motor(self):
        """发送 START，由固件负责对齐与闭环启动；提示用户等待对齐完成。"""
        if self._send("START"):
            self.command_info.setText(
                "START 已发送。电机需进行约 1.8 秒对齐/释放；请等待约 2 秒后再设置目标。"
            )

    def stop_motor(self):
        """发送 STOP，请求固件关闭电机输出；界面无法确认实际停机结果。"""
        if self._send("STOP"):
            self.command_info.setText("STOP 已发送（无 MCU 回执），请确认电机已实际停机。")

    def set_speed(self):
        """读取输入框的 rpm 数值并发送 S 命令；应先选择速度模式。"""
        speed = self.speed_input.value()
        if self._send(f"S{speed}"):
            self.command_info.setText(
                f"已发送速度目标 {speed} rpm；请确认此前已切换速度模式且对齐完成。"
            )

    def set_position(self):
        """读取单圈绝对机械角并发送 A 命令，单位为度；固件自动选择位置模式。"""
        angle = self.position_input.value()
        # A 的单位为度，固件先切到位置模式，再换算成内部使用的弧度。
        if self._send(f"A{angle:.2f}"):
            self.command_info.setText(
                f"已发送单圈位置目标 {angle:.2f}°（A 命令自动切换位置模式）；"
                "请确认对齐完成。"
            )

    def serial_error(self, exc: Exception):
        """串口异常时关闭连接并报告原因；故障连接不再尝试发送 STOP。"""
        self.disconnect_port(send_stop=False)
        self.command_info.setText(f"串口异常断开：{exc}。请确认电机已停机。")
        QMessageBox.warning(self, "串口通信异常", f"串口已断开：\n{exc}\n\n请确认电机实际状态。")

    # 六、遥测接收：先还原 44 字节帧，再校验并保存最新值。
    @staticmethod
    def valid_frame(v: tuple[float, ...]) -> bool:
        """检查十个通道是否有限且在合理范围；用于减少错位帧，不能替代 CRC。"""
        # 合理性校验用于减少误识别；协议没有 CRC，不能保证识别所有传输错误。
        return (
            all(math.isfinite(x) for x in v)
            and abs(v[0]) <= 2700 and abs(v[1]) < 20000
            and all(abs(x) < 100 for x in v[2:6])
            and all(abs(x) < 500 for x in v[6:8])
            and all(-1.0 <= x <= 361.0 for x in v[8:10])
        )

    def receive_data(self):
        """每 10 ms 非阻塞读串口，按帧尾拆包，验证后保存本次最新有效帧。"""
        if self.port is None:
            return
        try:
            incoming = self.port.read_all()
        except (serial.SerialException, OSError) as exc:
            self.serial_error(exc)
            return
        if not incoming:
            return

        self.recv_buf.extend(incoming)
        # 界面若曾阻塞，只保留最近一段数据，避免处理大量积压遥测拖慢操作。
        if len(self.recv_buf) > 100_000:
            del self.recv_buf[:-5000]

        latest_frame = None
        good_count = 0
        while True:
            pos = self.recv_buf.find(FRAME_TAIL)
            if pos == -1:
                # 没找到完整帧尾，保留最多 43 字节，等待下一次收包拼接。
                if len(self.recv_buf) > FRAME_BYTES - 1:
                    del self.recv_buf[:-(FRAME_BYTES - 1)]
                break
            if pos < FRAME_BYTES - len(FRAME_TAIL):
                # 帧尾前不足 40 字节，不是一整帧：跳过此帧尾继续寻找。
                del self.recv_buf[:pos + 4]
                continue
            # 帧尾前的 40 字节才是载荷；< 表示小端，10f 表示十个 float32。
            payload = bytes(self.recv_buf[pos - 40:pos])
            del self.recv_buf[:pos + 4]
            numbers = struct.unpack("<10f", payload)
            if self.valid_frame(numbers):
                latest_frame = numbers
                good_count += 1

        if latest_frame is not None:
            now = time.monotonic()
            self.latest = latest_frame
            self.last_rx_time = now
            self.frame_count += good_count
            # 每次轮询只存最新帧作为显示点（约 100 Hz），帧率统计仍计入所有有效帧。
            self.history.append((now, *latest_frame))

    # 七、显示刷新：统计、曲线和缓存清理不修改 MCU 控制参数。
    def update_link_status(self):
        """每秒显示收到的有效帧数并清零计数；超过两秒无数据时显示等待状态。"""
        if self.port is None:
            return
        if time.monotonic() - self.last_rx_time > 2.0:
            self.link_label.setText("已连接 · 暂无遥测（停机时正常）")
        else:
            self.link_label.setText(f"已连接 · {self.frame_count} 帧/秒")
        self.frame_count = 0

    def update_charts(self):
        """每 50 ms 取最近八秒历史并更新四张图和卡片；时间采用 PC 收包时刻。"""
        if not self.history or self.latest is None:
            return
        now = time.monotonic()
        visible = [s for s in self.history if s[0] >= now - VIEW_SECONDS]
        if not visible:
            return
        x = [s[0] - now for s in visible]
        for graph, ref_idx, actual_idx in (
            (self.speed_graph, 1, 2),
            (self.position_graph, 9, 10),
            (self.iq_graph, 5, 6),
            (self.id_graph, 3, 4),
        ):
            graph.update_traces(
                x,
                [s[ref_idx] for s in visible],
                [s[actual_idx] for s in visible],
            )

        v = self.latest
        self.speed_value.setText(f"速度  {v[1]:.0f} / {v[0]:.0f} rpm")
        self.iq_value.setText(f"Iq  {v[5]:.3f} / {v[4]:.3f} A")
        self.id_value.setText(f"Id  {v[3]:.3f} / {v[2]:.3f} A")
        self.pos_value.setText(f"位置  {v[9]:.1f} / {v[8]:.1f} °")
        # 卡片统一显示“实际值 / 目标值”；latest 不含时间，所以索引直接对应通道。

    def clear_charts(self):
        """清空历史和最近帧，将卡片及曲线恢复为空，防止混入上次连接的数据。"""
        self.history.clear()
        self.latest = None
        self.speed_value.setText("速度  -- / -- rpm")
        self.iq_value.setText("Iq  -- / -- A")
        self.id_value.setText("Id  -- / -- A")
        self.pos_value.setText("位置  -- / -- °")
        for graph in (self.speed_graph, self.position_graph, self.iq_graph, self.id_graph):
            graph.clear()

    def closeEvent(self, event):
        """处理 Qt 关闭窗口事件：尝试发送 STOP、释放串口，再允许窗口关闭。"""
        self.disconnect_port(send_stop=True)
        event.accept()


# 八、程序入口。
def main():
    """设置绘图选项、Qt 样式和字体，显示主窗口并运行 Qt 事件循环。"""
    pg.setConfigOptions(antialias=False)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setFont(QFont("Microsoft YaHei UI", 10))
    window = MotorMonitor()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
