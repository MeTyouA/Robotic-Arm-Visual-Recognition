
import sensor, image, time, math
from pyb import UART, LED

# ---------- 图像/检测参数 ----------
img_w = 160
img_h = 80

line_seg_threshold    = 1000
line_merge_dist       = 5
line_max_theta_diff   = 15
line_min_length       = 25
continuity_tol        = 25

track_roi             = (0, 20, img_w, img_h)

# ---------- 拐角检测参数 ----------
corner_confirm_frames  = 3          # 连续 N 帧同时看到横竖线才确认拐角
corner_cooldown_frames = 12         # 拐角后冷却, 防止旧线段残留触发误切换

# ---------- BOOT 投票参数 ----------
boot_v_window_lo      = 30          # theta<30 或 >150 视为垂直
boot_v_window_hi      = 150
boot_h_window_lo      = 60          # 60<theta<120 视为水平
boot_h_window_hi      = 120
boot_min_consensus    = 12
boot_max_frames       = 20

# ---------- 调试 ----------
draw_debug            = True
print_period_ms       = 400

# ============================================================
# 协议常量 — 仅四个直线方向, 无角点指令
# ============================================================
HEADER0 = 0x55
HEADER1 = 0xAB

CMD_IDLE             = 0
CMD_LEFT_TO_RIGHT    = 1   # 向右走
CMD_RIGHT_TO_LEFT    = 2   # 向左走
CMD_TOP_TO_BOTTOM    = 3   # 向下走
CMD_BOTTOM_TO_TOP    = 4   # 向上走

STATE_NAMES = {
    CMD_IDLE:             "空闲",
    CMD_LEFT_TO_RIGHT:    "向右走",
    CMD_RIGHT_TO_LEFT:    "向左走",
    CMD_TOP_TO_BOTTOM:    "向下走",
    CMD_BOTTOM_TO_TOP:    "向上走",
}

# 方向反转禁止表: 向左走时不能直接切向右走, 向下走时不能直接切向上走, 反之亦然
OPPOSITE = {
    CMD_LEFT_TO_RIGHT: CMD_RIGHT_TO_LEFT,
    CMD_RIGHT_TO_LEFT: CMD_LEFT_TO_RIGHT,
    CMD_TOP_TO_BOTTOM: CMD_BOTTOM_TO_TOP,
    CMD_BOTTOM_TO_TOP: CMD_TOP_TO_BOTTOM,
}

def state_name(state):
    if state is None:
        return "无"
    return STATE_NAMES.get(state, "未知(%d)" % state)

# 开局起始方向 (现场根据机械臂初始姿态改这两行)
boot_h_initial = CMD_LEFT_TO_RIGHT      # 看到水平线 → 默认向右走
boot_v_initial = CMD_BOTTOM_TO_TOP      # 看到垂直线 → 默认向上走

def send_cmd(state):
    frame = bytearray(9)
    frame[0] = HEADER0
    frame[1] = HEADER1
    frame[2] = state & 0xFF
    uart.write(frame)

def angle_delta(a, b):
    d = abs(a - b)
    return 180 - d if d > 90 else d

def classify_line(theta):
    # 1 = 水平, 2 = 垂直, 0 = 模糊
    if theta < boot_v_window_lo or theta > boot_v_window_hi:
        return 2
    if boot_h_window_lo < theta < boot_h_window_hi:
        return 1
    return 0

# ============================================================
# 状态
# ============================================================
last_main_theta   = None

boot_locked       = False
boot_h_count      = 0
boot_v_count      = 0
boot_frame_seen   = 0

current_state     = CMD_IDLE

# 拐角状态机
corner_consec     = 0           # 连续同时检出横竖线的帧数
corner_cooldown   = 0           # 拐角后冷却剩余帧数

def boot_vote(theta):
    global boot_h_count, boot_v_count, boot_frame_seen, boot_locked, current_state
    if boot_locked:
        return
    boot_frame_seen += 1
    cls = classify_line(theta)
    if cls == 1:
        boot_h_count += 1
    elif cls == 2:
        boot_v_count += 1

    decision = 0
    if boot_h_count >= boot_min_consensus:
        decision = 1
    elif boot_v_count >= boot_min_consensus:
        decision = 2
    elif boot_frame_seen >= boot_max_frames:
        if boot_h_count > boot_v_count:
            decision = 1
        elif boot_v_count > boot_h_count:
            decision = 2
        else:
            boot_h_count = 0
            boot_v_count = 0
            boot_frame_seen = 0
            return

    if decision == 1:
        current_state = boot_h_initial
        boot_locked = True
    elif decision == 2:
        current_state = boot_v_initial
        boot_locked = True

def find_line(img):
    """返回 (main_seg, corner_dir).
    main_seg:  连续性优选主线段 (用于直线跟踪)
    corner_dir: 拐角时的新方向 (CMD_*), 无拐角时为 None
    """
    global last_main_theta, corner_consec, corner_cooldown

    if corner_cooldown > 0:
        corner_cooldown -= 1

    segs = img.find_line_segments(roi=track_roi,
                                  threshold=line_seg_threshold,
                                  merge_distance=line_merge_dist,
                                  max_theta_difference=line_max_theta_diff)
    valid = [s for s in segs if s.length() >= line_min_length]
    if not valid:
        last_main_theta = None
        corner_consec = 0
        return (None, None)

    # --- 主线段选择: 连续性优先 ---
    valid.sort(key=lambda s: s.length(), reverse=True)
    main = valid[0]
    if last_main_theta is not None:
        for s in valid:
            if angle_delta(s.theta(), last_main_theta) <= continuity_tol \
               and s.length() >= valid[0].length() * 0.6:
                main = s
                break
    last_main_theta = main.theta()

    # --- 拐角检测: 按 theta 属性找横竖线, 不按长度 ---
    line_v = None   # 最佳垂直线
    line_h = None   # 最佳水平线
    for s in valid:
        cls = classify_line(s.theta())
        if cls == 2 and (line_v is None or s.length() > line_v.length()):
            line_v = s
        elif cls == 1 and (line_h is None or s.length() > line_h.length()):
            line_h = s

    if line_v is not None and line_h is not None:
        corner_consec += 1
    else:
        corner_consec = 0

    corner_dir = None
    if corner_consec >= corner_confirm_frames and corner_cooldown == 0:
        corner_dir = decide_corner_direction(current_state, line_v, line_h)
        if corner_dir is not None:
            corner_cooldown = corner_cooldown_frames

    return (main, corner_dir)

def decide_corner_direction(state, line_v, line_h):
    """属性绑定判定: 不依赖谁长谁短, 只用 theta 分类 + 几何中心判定转弯方向.

    纵向行驶 → 岔路是水平线, 比 h_cx vs v_cx 定左右
    横向行驶 → 岔路是垂直线, 比 v_cy vs h_cy 定上下
    """
    v_cx = (line_v.x1() + line_v.x2()) // 2
    v_cy = (line_v.y1() + line_v.y2()) // 2
    h_cx = (line_h.x1() + line_h.x2()) // 2
    h_cy = (line_h.y1() + line_h.y2()) // 2

    if state in (CMD_LEFT_TO_RIGHT, CMD_RIGHT_TO_LEFT):
        # 横向行驶, 岔路垂直线的中心在水平线下方 → 向下走; 上方 → 向上走
        return CMD_TOP_TO_BOTTOM if v_cy > h_cy else CMD_BOTTOM_TO_TOP
    elif state in (CMD_TOP_TO_BOTTOM, CMD_BOTTOM_TO_TOP):
        # 纵向行驶, 岔路水平线的中心在垂直线右边 → 向右走; 左边 → 向左走
        return CMD_LEFT_TO_RIGHT if h_cx > v_cx else CMD_RIGHT_TO_LEFT
    return None

def pick_direction(main_seg):
    """维持当前方向, 绝不跨族切换. 跨族切换由拐角检测独占."""
    cls = classify_line(main_seg.theta())

    if cls == 1:  # 水平线
        if current_state in (CMD_LEFT_TO_RIGHT, CMD_RIGHT_TO_LEFT):
            return current_state   # 已在水平方向 → 维持
        return current_state       # 不在水平方向 → 不跨族, 等拐角
    elif cls == 2:  # 垂直线
        if current_state in (CMD_TOP_TO_BOTTOM, CMD_BOTTOM_TO_TOP):
            return current_state   # 已在垂直方向 → 维持
        return current_state       # 不在垂直方向 → 不跨族, 等拐角
    else:
        return current_state  # 模糊线段, 保持当前

# ============================================================
# Debug
# ============================================================
def draw_overlay(img, main, line_v, line_h, tag):
    if not draw_debug:
        return
    img.draw_rectangle(track_roi, color=160)
    if main is not None:
        img.draw_line(main.x1(), main.y1(), main.x2(), main.y2(), color=255, thickness=2)
    if line_v is not None:
        img.draw_line(line_v.x1(), line_v.y1(), line_v.x2(), line_v.y2(), color=100, thickness=1)
    if line_h is not None:
        img.draw_line(line_h.x1(), line_h.y1(), line_h.x2(), line_h.y2(), color=180, thickness=1)
    img.draw_string(2, 2, tag, color=255, scale=1)

last_print_time = time.ticks_ms()
def print_status(text):
    global last_print_time
    now = time.ticks_ms()
    if time.ticks_diff(now, last_print_time) >= print_period_ms:
        print(text)
        last_print_time = now

# ============================================================
# 初始化
# ============================================================
led = LED(1)
led.on()
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_vflip(True)
sensor.set_hmirror(True)

clock = time.clock()
# 由 main.py 注入 uart 时复用同一对象, 单独跑此脚本时才新建.
# 本脚本对 UART 只调用 write(), RX 完全归 main.py 的 check_mode().
try:
    uart
except NameError:
    uart = UART(1, 115200)

# ============================================================
# 主循环
# ============================================================
while True:
    # 模式切换 hook: main.py 注入 check_mode 时生效, 单独跑此脚本时跳过
    try:
        check_mode()
    except NameError:
        pass
    clock.tick()
    img = sensor.snapshot()
    main_seg, corner_dir = find_line(img)

    # ---------- BOOT: 没识别就给 STM32 发 IDLE, 机械臂不动 ----------
    if not boot_locked:
        if main_seg is not None:
            boot_vote(main_seg.theta())
        send_cmd(CMD_IDLE)
        draw_overlay(img, main_seg, None, None,
                     "BOOT H=%d V=%d" % (boot_h_count, boot_v_count))
        print_status("BOOT 水平=%d 垂直=%d seen=%d locked=%s" %
                     (boot_h_count, boot_v_count, boot_frame_seen, boot_locked))
        continue

    # ---------- 锁定后: 持续发方向指令 ----------
    if corner_dir is not None and corner_dir != OPPOSITE.get(current_state):
        # 拐角确认 → 切换方向, 冷却期内 find_line 不会再返回 corner_dir
        current_state = corner_dir

    elif corner_cooldown > 0:
        # 冷却期中: 锁定当前方向.
        # 拐角后旧线段的长度可能反复超过新线段, 若此时让 pick_direction
        # 按 cls 重选就会回到旧方向, 造成来回振荡.
        pass

    elif main_seg is not None:
        # 无拐角 → 正常按线段选方向
        current_state = pick_direction(main_seg)

    send_cmd(current_state)
    draw_overlay(img, main_seg, None, None, state_name(current_state))
    lock_tag = "LOCK" if corner_cooldown > 0 else "   "
    print_status("%s 状态=%s theta=%.1f cls=%d cd=%d" %
                 (lock_tag,
                  state_name(current_state),
                  main_seg.theta() if main_seg is not None else -1,
                  classify_line(main_seg.theta()) if main_seg is not None else 0,
                  corner_cooldown))
