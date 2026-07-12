import time
from pyb import UART, LED

# ============================================================
# OpenMV 模式分发器
# 接收 STM32 (USART1) 发来的 3 字节模式帧:
#   [0]=0x55  [1]=0xBB  [2]=mode
# mode:
#   0 = 空闲 (停止当前脚本, 进入待机)
#   1 = Track_mode         -> 运行 Track_blake.py (黑线巡线)
#   2 = Dynamic_Track_Mode -> 运行 Track_red.py  (红色目标)
#   3 = ThreeD_Track_Mode  -> 同 mode=2 复用 Track_red.py
# ============================================================

HEADER0 = 0x55
HEADER1 = 0xBB

MODE_IDLE        = 0
MODE_TRACK_BLAKE = 1
MODE_TRACK_RED   = 2
MODE_TRACK_3D    = 2

SCRIPT_MAP = {
    MODE_TRACK_BLAKE: "Track_blake.py",
    MODE_TRACK_RED:   "Track_red.py",
    MODE_TRACK_3D:    "Track_red.py",   # 占位, 后续替换
}

# 调试: 打开后每次 _drain_uart 都打印收到的原始字节, 帮助诊断接线/波特率
DEBUG_RX = False

uart = UART(1, 115200)
led = LED(2)   # 绿灯指示分发器在跑

# current_mode 在模块作用域, 子脚本通过 check_mode() 读取最新值并判定是否切换
current_mode = MODE_IDLE
requested_mode = MODE_IDLE


class ModeSwitch(Exception):
    """子脚本在主循环顶部调用 check_mode(), 若 requested_mode 变化则抛出此异常退出"""
    pass


def _drain_uart():
    """读 UART, 解析最近一帧 0x55 0xBB <mode>, 更新 requested_mode."""
    global requested_mode
    n = uart.any()
    if n <= 0:
        return
    data = uart.read(n)
    if not data:
        return
    if DEBUG_RX:
        print("[rx]", " ".join("%02X" % b for b in data))
    # 从后往前找最新一帧, 避免堆积时反复切换.
    # 帧长 3 字节, 起点 i 必须满足 i+2 <= len(data)-1, 即 i <= len(data)-3.
    for i in range(len(data) - 3, -1, -1):
        if data[i] == HEADER0 and data[i + 1] == HEADER1:
            m = data[i + 2]
            if m in (MODE_IDLE, MODE_TRACK_BLAKE, MODE_TRACK_RED, MODE_TRACK_3D):
                requested_mode = m
                return


def check_mode():
    """子脚本每帧顶部调用. 发现要求模式 != 当前模式就抛出 ModeSwitch."""
    _drain_uart()
    if requested_mode != current_mode:
        raise ModeSwitch()


def _run_script(path):
    """以分发器的全局命名空间执行子脚本, 让子脚本能访问 check_mode/ModeSwitch."""
    print("[dispatcher] >>> exec", path)
    led.on()
    try:
        with open(path) as f:
            code = f.read()
        # 注意: 在 globals() 里 exec, 子脚本里 `import` 等都正常工作,
        # 同时 check_mode / ModeSwitch 对子脚本直接可见
        exec(code, globals())
    except ModeSwitch:
        print("[dispatcher] <<< switch out of", path)
    except Exception as e:
        # 任何子脚本异常都打印但不要让分发器死掉, 回到 IDLE 等下一次按键
        print("[dispatcher] !!! script error in", path, ":", e)
    finally:
        led.off()


def _idle_wait():
    """空闲: 关 LED, 阻塞等 UART 给出非 0 mode."""
    print("[dispatcher] IDLE, waiting for STM32 mode key...")
    while requested_mode == MODE_IDLE:
        _drain_uart()
        time.sleep_ms(50)


# ============================================================
# 主循环
# ============================================================
print("[dispatcher] booted, listening on UART1 @115200")
_drain_uart()

while True:
    if requested_mode == MODE_IDLE:
        current_mode = MODE_IDLE
        _idle_wait()
        continue

    current_mode = requested_mode
    script = SCRIPT_MAP.get(current_mode)
    if script is None:
        print("[dispatcher] no script for mode", current_mode, ", back to IDLE")
        requested_mode = MODE_IDLE
        continue

    _run_script(script)
    # 子脚本退出后回到顶端, 由 requested_mode 决定下一步
