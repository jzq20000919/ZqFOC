import ctypes
import math
from pathlib import Path


# ============================================================
# 1. 定义与 C 语言对应的结构体
# ============================================================

class FOC_Clarke_t(ctypes.Structure):
    _fields_ = [
        ("I_alpha", ctypes.c_float),
        ("I_beta", ctypes.c_float)
    ]


class FOC_Park_t(ctypes.Structure):
    _fields_ = [
        ("I_d", ctypes.c_float),
        ("I_q", ctypes.c_float)
    ]


class FOC_inv_Park_t(ctypes.Structure):
    _fields_ = [
        ("V_alpha", ctypes.c_float),
        ("V_beta", ctypes.c_float)
    ]


# ============================================================
# 2. 加载 DLL
# ============================================================

dll_path = Path(__file__).with_name("foc_math.dll")
foc_math = ctypes.CDLL(str(dll_path))


# ============================================================
# 3. 告诉 Python 每个 C 函数的输入和输出类型
# ============================================================

foc_math.FOC_Normalize.argtypes = [
    ctypes.c_float
]
foc_math.FOC_Normalize.restype = ctypes.c_float


foc_math.FOC_Clarke.argtypes = [
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_float
]
foc_math.FOC_Clarke.restype = FOC_Clarke_t


foc_math.FOC_Park.argtypes = [
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_float
]
foc_math.FOC_Park.restype = FOC_Park_t


foc_math.FOC_inv_Park.argtypes = [
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_float
]
foc_math.FOC_inv_Park.restype = FOC_inv_Park_t


# ============================================================
# 4. 一个公共的浮点数比较函数
# ============================================================

def assert_close(actual, expected, tolerance=1e-5):
    assert abs(actual - expected) < tolerance, (
        f"actual={actual}, expected={expected}"
    )


# ============================================================
# 5. 各项测试
# ============================================================

def test_normalize():
    result = foc_math.FOC_Normalize(
        2.0 * math.pi + 0.5
    )

    assert_close(result, 0.5)

    print("FOC_Normalize PASS")


def test_clarke():
    result = foc_math.FOC_Clarke(
        1.0,
        -0.5,
        -0.5
    )

    assert_close(result.I_alpha, 1.0)
    assert_close(result.I_beta, 0.0)

    print("FOC_Clarke PASS")


def test_park():
    theta_90 = math.pi / 2.0

    result = foc_math.FOC_Park(
        1.0,
        0.0,
        theta_90
    )

    assert_close(result.I_d, 0.0)
    assert_close(result.I_q, -1.0)

    print("FOC_Park PASS")


def test_inv_park():
    result = foc_math.FOC_inv_Park(
        1.0,
        0.0,
        0.0
    )

    assert_close(result.V_alpha, 1.0)
    assert_close(result.V_beta, 0.0)

    print("FOC_inv_Park PASS")


def test_park_invpark_roundtrip():
    theta = 1.2

    # dq -> alpha beta
    inv_result = foc_math.FOC_inv_Park(
        2.0,
        3.0,
        theta
    )

    # alpha beta -> dq
    park_result = foc_math.FOC_Park(
        inv_result.V_alpha,
        inv_result.V_beta,
        theta
    )

    assert_close(park_result.I_d, 2.0)
    assert_close(park_result.I_q, 3.0)

    print("Park <-> invPark PASS")


# ============================================================
# 6. 依次运行所有测试
# ============================================================

if __name__ == "__main__":

    test_normalize()
    test_clarke()
    test_park()
    test_inv_park()
    test_park_invpark_roundtrip()

    print()
    print("ALL FOC MATH TESTS PASS")