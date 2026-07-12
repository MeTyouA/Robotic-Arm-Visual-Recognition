import sensor, image, time
from pyb import UART

# 1. 传感器物理锁定 
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 极致帧率，画面中心绝对物理坐标为 (80, 60)
sensor.skip_frames(time = 2000)     # 强行等待两秒，让镜头适应环境光

sensor.set_auto_whitebal(False)     # 锁死白平衡
sensor.set_auto_gain(False)         # 锁死曝光
sensor.set_vflip(True)              # 根据需要翻转图像
sensor.set_hmirror(False)            # 根据需要镜像图像

clock = time.clock()
# 由 main.py 注入 uart 时复用同一对象, 单独跑此脚本时才新建.
# 本脚本对 UART 只调用 write(), RX 完全归 main.py 的 check_mode().
try:
    uart
except NameError:
    uart = UART(1, 115200)

# 2. 颜色阈值
red_ = (15, 44, 25, 90, 10, 127)

# 单目测距光学常数
# 计算公式: K = 5cm(标准悬停距离) * 目标在5cm处的像素宽度
K_ = 470.0

target_dis = 5.0 # 期望悬停深度 (Z轴控制零点),单位cm

# 目标打分权重 (Score = Area - SCORE_ALPHA * Distance^2)
# Distance 为 blob 中心到画面中心 (80, 60) 的像素距离
# 增大 α 让中心目标更受偏好（抗边缘噪点），减小则更偏好大面积
SCORE_ALPHA = 0.1

# ==========================================
# 2.1 轴解耦 / 伺服优先级 (先 XY 后 Z)
# ==========================================
# 【中心放宽滤波死区】不需要太精准的 XY，只要在这个容差圈内，XY 直接报 0 停机
# 0.15 代表左右偏离中心不超过 15%（160*0.15 = 24 像素）
XY_CENTER_DEADZONE = 0.15


# 当 blob 外接矩形距离画面边缘 ≤ EDGE_MARGIN(像素) 时，认为宽度被截断，Z 不可信
EDGE_MARGIN = 2
IMG_W = 160
IMG_H = 120

# 当归一化 XY 误差任一轴超过此阈值，先专心做 XY，把 Z 钳为 0
# 0.30 ≈ 距画面中心 24px(x) / 18px(y)，可按你机臂的整定情况微调
XY_LOCK_THRESHOLD = 0.30

# "太近" 判定阈值: blob 的 w 或 h 超过画面对应维度的此比例时, 认为目标已贴到镜头.
# 即使此时被边缘截断, 仍可断定"太近", 必须发负 Z 让 STM32 后退.
# 调小 → 更早判太近 (更安全, 易误触发); 调大 → 更晚判 (更精确, 但近距死区大)
TOO_CLOSE_RATIO = 0.60

# ==========================================
# 3. 核心封包函数 (与STM32的硬件级合同)
# ==========================================
def send_to_stm32(dx, dy, dz):
    # 将浮点数转换为 2 字节有符号整数，大端模式 (高位在前)
    x_b = int(dx).to_bytes(2, 'big', True)
    y_b = int(dy).to_bytes(2, 'big', True)
    z_b = int(dz).to_bytes(2, 'big', True)
    
    # 累加校验和 (位与 0xFF 确保结果在一个字节以内)
    checksum = (x_b[0] + x_b[1] + y_b[0] + y_b[1] + z_b[0] + z_b[1]) & 0xFF
    
    # 组装 9 字节数据帧：[帧头1, 帧头2, X高, X低, Y高, Y低, Z高, Z低, 校验和]
    frame = bytearray([0x55, 0xAA, x_b[0], x_b[1], y_b[0], y_b[1], z_b[0], z_b[1], checksum])
    uart.write(frame)

# ==========================================
# 4. 非阻塞日志秒表 (解放 USB 算力)
# ==========================================
last_print_time = time.ticks_ms()

# ==========================================
# 5. 视觉主循环 (狂奔模式)
# ==========================================
while(True):
    # 模式切换 hook: main.py 注入 check_mode 时生效, 单独跑此脚本时跳过
    try:
        check_mode()
    except NameError:
        pass
    clock.tick()
    img = sensor.snapshot()
    
    # 形态学滤波：过滤掉面积小于 50 像素的环境噪点
    blobs = img.find_blobs([red_], pixels_threshold=50, area_threshold=50, merge=True)
    
    if blobs:
        # 目标打分：Score = Area - α·Distance²，优先锁定画面中心目标，防止边缘大面积噪点抢占
        target = max(blobs, key=lambda b:
            b.pixels() - SCORE_ALPHA * ((b.cx() - 80) ** 2 + (b.cy() - 60) ** 2))
        
        # --- 数据提取与处理 ---
        # 1. 归一化 XY 偏差 (转化为 -1.0 到 1.0 的比例值，方便 STM32 PID 调参)
        raw_error_x = (target.cx() - 80) / 80.0
        raw_error_y = (target.cy() - 60) / 60.0
        
        # 【核心滤波修改：中心区域截断】
        # 一旦落入 XY_CENTER_DEADZONE 圈内，立刻强制输出 0 坐标给下位机，使其停止微调
        error_x = 0 if abs(raw_error_x) < XY_CENTER_DEADZONE else raw_error_x
        error_y = 0 if abs(raw_error_y) < XY_CENTER_DEADZONE else raw_error_y
        
        # 2. 解算 Z 轴深度偏差 (基于宽度)
        w = target.w()
        if w > 0:
            current_z = K_ / w
            error_z = current_z - target_dis
        else:
            error_z = 0

        # --- 轴解耦保险：先 XY，后 Z; "C" 太近时强制后退 ---
        # 分别记录每条边是否触发, 用于"太近时用未截断的轴反估"
        tx, ty, th = target.x(), target.y(), target.h()
        hit_left   = tx <= EDGE_MARGIN
        hit_right  = (tx + w) >= (IMG_W - EDGE_MARGIN)
        hit_top    = ty <= EDGE_MARGIN
        hit_bottom = (ty + th) >= (IMG_H - EDGE_MARGIN)
        edge_hit = hit_left or hit_right or hit_top or hit_bottom

        # (a) "太近" 判定: blob 占据画面 >= TOO_CLOSE_RATIO. 即使边缘截断, 也敢断定"贴脸",
        #     必须发负 Z 让 STM32 后退, 否则会死锁在 error_z = 0.
        too_close = (w >= IMG_W * TOO_CLOSE_RATIO) or (th >= IMG_H * TOO_CLOSE_RATIO)

        # (b) XY 优先：这里的判断基于过滤前的原始误差 raw_error，确保在大范围未对中时死锁 Z 轴
        xy_locked = (abs(raw_error_x) > XY_LOCK_THRESHOLD) or (abs(raw_error_y) > XY_LOCK_THRESHOLD)

        if too_close:
            # 用未被截断的那个轴重估深度; 若四面都被截断则发固定最大后退量.
            if not (hit_top or hit_bottom) and th > 0:
                error_z = (K_ / th) - target_dis       # 高度未截断, 用 h 反算
            elif not (hit_left or hit_right) and w > 0:
                error_z = (K_ / w) - target_dis        # 宽度未截断, 用 w 反算
            else:
                error_z = -target_dis                  # 四面全截断, 发最大后退
            z_tag = "C"   # Close: 太近, 强制后退
        elif xy_locked:
            error_z = 0
            z_tag = "L"
        elif edge_hit:
            error_z = 0
            z_tag = "E"
        else:
            z_tag = "OK"

        # --- UI 可视化反馈 ---
        img.draw_rectangle(target.rect(), color=(255, 0, 0))
        img.draw_cross(target.cx(), target.cy(), color=(0, 255, 0))
        
        # 额外绘制一个绿色的滤波中心死区边界框，方便你用眼睛观测滤波效果
        deadzone_w = int(80 * XY_CENTER_DEADZONE)
        deadzone_h = int(60 * XY_CENTER_DEADZONE)
        img.draw_rectangle(80 - deadzone_w, 60 - deadzone_h, deadzone_w * 2, deadzone_h * 2, color=(0, 0, 255))
        
        # 状态指示：C=太近后退, E=边缘截断, L=XY未对中, OK=Z正常输出
        img.draw_string(2, 2, "Z:%s" % z_tag, color=(255, 255, 0))

        # --- 数据发送 (保持 50Hz 高频下发) ---
        # 放大 100 倍变整数发送。例如：偏差 -0.15 转换为 -15 发送
        send_to_stm32(error_x * 100, error_y * 100, error_z * 100)

        # --- 截流打印：每 500ms 刷新一次终端，严禁刷屏 ---
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, last_print_time) >= 500:
            print("FPS: %2d | 发送STM32 -> X:%4d, Y:%4d, Z:%4d | W:%d | Z状态:%s" %
                  (clock.fps(), int(error_x * 100), int(error_y * 100), int(error_z * 100),
                   w, z_tag))
            last_print_time = current_time

    else:
        # 目标丢失：立刻发送 0 偏差，指令 STM32 原地停机保命
        send_to_stm32(0, 0, 0)
        
        # 目标丢失的警告也按 500ms 截流打印
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, last_print_time) >= 500:
            print("目标丢失，已发送全零停机指令保护机械臂。")
            last_print_time = current_time