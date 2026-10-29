# steel_ball_full.py —— 钢球检测 + 图传 + 录制 + 按键（仅控录制） + ROI
# 平台：MaixCAM Pro / MaixPy v4
# 模型：model_295152.mud (YOLOv5, 320×320, 单类)
# 图传：使用官方 http.JpegStreamer，默认端口 8000

from maix import camera, display, image, nn, app, uart, pinmap, time, gpio, http, network, err
import os, platform

# ═══════════════════════════════════════════
# WiFi：AP 模式（MaixCAM 开热点）
# ═══════════════════════════════════════════
AP_SSID = platform.node()
AP_PASS = "88888888"

wifi = network.wifi.Wifi()
e = wifi.start_ap(AP_SSID, AP_PASS)
err.check_raise(e, "start wifi ap failed")
ip = wifi.get_ip()
print(f"[WiFi] AP 启动: {AP_SSID}, IP: {ip}")

# ═══════════════════════════════════════════
# 串口（UART1，A19=TX → STM32 PA10）
# ═══════════════════════════════════════════
pinmap.set_pin_function("A18", "UART1_RX")
pinmap.set_pin_function("A19", "UART1_TX")
UART_PORT = "/dev/ttyS0"
BAUDRATE = 115200
SEND_INTERVAL_MS = 50
ser = uart.UART(UART_PORT, BAUDRATE)

# ═══════════════════════════════════════════
# ROI 限定（摆杆凹槽区域，需根据实际画面标定）
# ═══════════════════════════════════════════
ROI_X = 0
ROI_Y = 60
ROI_W = 400
ROI_H = 100

# ═══════════════════════════════════════════
# 追踪器
# ═══════════════════════════════════════════
TRACK_TTL = 3
TRACK_ALPHA = 0.8
LOCK_MAX_DIST = 80
lock = {"cx": 0, "cy": 0, "r": 0, "ttl": 0, "active": False}

STM32_FRAME_W = 320
STM32_FRAME_H = 240

# 管子实际宽度 25cm，占满画面宽度
TUBE_CM = 25.0
# 摄像头水平FOV（度），用于偏离距离计算
CAM_FOV_H = 60.0
# 基于FOV推导：画面宽度对应实际 TUBE_CM，线性映射 dev_cm = (cx-CX_HALF)*TUBE_CM/CAM_W

# ═══════════════════════════════════════════
# 系统状态
# ═══════════════════════════════════════════
g_recording = False
g_last_btn = 0

# ═══════════════════════════════════════════
# 按键（A20，仅控制录制开关）
# ═══════════════════════════════════════════
BTN_PIN = "A20"
BTN_DEBOUNCE_MS = 300

try:
    btn = gpio.GPIO(BTN_PIN, gpio.Mode.IN, gpio.Pull.PULL_UP)
    print(f"[BTN] 按键已配置: {BTN_PIN}")
except Exception as e:
    print(f"[BTN] 按键初始化失败: {e}")
    btn = None

# ═══════════════════════════════════════════
# 辅助函数
# ═══════════════════════════════════════════

def clamp(v, lo, hi):
    return max(lo, min(v, hi))

def send_serial(balls, dev=0.0, spd=0.0, acc=0.0):
    if not balls:
        ser.write_str("N\n")
        return
    cx, cy, _ = balls[0]
    sx = clamp(int(cx * STM32_FRAME_W / CAM_W), 0, STM32_FRAME_W - 1)
    sy = clamp(int(cy * STM32_FRAME_H / CAM_H), 0, STM32_FRAME_H - 1)
    ser.write_str(f"{sx},{sy},{dev:.1f},{spd:.1f},{acc:.1f}\n")

def in_roi(cx, cy):
    return ROI_X <= cx <= ROI_X + ROI_W and ROI_Y <= cy <= ROI_Y + ROI_H

# ═══════════════════════════════════════════
# 模型 & 摄像头
# ═══════════════════════════════════════════

model_path = "model_307023.mud"
if not os.path.exists(model_path):
    model_path = "/root/models/model_307023.mud"
detector = nn.YOLOv5(model=model_path)

CAM_W = detector.input_width()
CAM_H = detector.input_height()
CX_HALF = CAM_W // 2
CY_HALF = CAM_H // 2

cam = camera.Camera(CAM_W, CAM_H, detector.input_format(), buff_num=1)
dis = display.Display()

# ═══════════════════════════════════════════
# 图传（自定义 HTTP 服务器，CSS 居中）
# ═══════════════════════════════════════════

g_jpg = bytearray()

INDEX_HTML = """HTTP/1.1 200 OK\r
Content-Type: text/html; charset=utf-8\r
Connection: close\r
\r
<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<style>*{margin:0;padding:0}body{background:#000;display:flex;justify-content:center;align-items:center;min-height:100vh}img{width:640px;height:640px}</style>
</head><body><img src="/stream"></body></html>"""

STREAM_HEADER = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
STREAM_FOOTER = b"\r\n"

def http_serve():
    import socket as sock
    s = sock.socket(sock.AF_INET, sock.SOCK_STREAM)
    s.setsockopt(sock.SOL_SOCKET, sock.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", 8000))
    s.listen(2)
    s.settimeout(0.1)
    while True:
        try:
            c, addr = s.accept()
        except:
            continue
        try:
            req = c.recv(1024).decode()
            if "/stream" in req:
                c.send(b"HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n")
                c.settimeout(1.0)
                while True:
                    try:
                        jpg = bytes(g_jpg)
                        if jpg:
                            c.send(STREAM_HEADER + jpg + STREAM_FOOTER)
                        time.sleep_ms(33)
                    except:
                        break
            else:
                c.send(INDEX_HTML.encode())
        except:
            pass
        try:
            c.close()
        except:
            pass

try:
    import _thread
    _thread.start_new_thread(http_serve, ())
    print(f"[INFO] 图传地址: http://{ip}:8000")
except:
    stream = http.JpegStreamer()
    stream.start()
    print(f"[INFO] 图传地址: http://{ip}:{stream.port()}")
    print("[INFO] 居中页面不可用，使用原生图传")

# ═══════════════════════════════════════════
# 录制目录
# ═══════════════════════════════════════════
RECORD_DIR = None

# ═══════════════════════════════════════════
# 主循环
# ═══════════════════════════════════════════

last_send = time.ticks_ms()
fps_last = time.ticks_ms()
fps_count = 0
fps_val = 0
frame_id = 0
# 速度测算
prev_cx = None
prev_time = 0
speed_cm = 0.0
prev_speed_cm = 0.0
accel_cm_s2 = 0.0

while not app.need_exit():
    img = cam.read()
    now = time.ticks_ms()
    frame_id += 1

    # FPS
    fps_count += 1
    if now - fps_last >= 1000:
        fps_val = fps_count
        fps_count = 0
        fps_last = now

    # ── 按键（仅控制录制） ──
    if btn is not None:
        btn_val = btn.value()
        if btn_val == 0 and now - g_last_btn > BTN_DEBOUNCE_MS:
            g_last_btn = now
            if not g_recording:
                g_recording = True
                ts_dir = time.strftime("%Y%m%d_%H%M%S")
                RECORD_DIR = f"/sd/record_{ts_dir}"
                try:
                    os.makedirs(RECORD_DIR)
                    print(f"[REC] 开始录制: {RECORD_DIR}")
                except Exception:
                    RECORD_DIR = None
                    print("[REC] 创建录制目录失败（无 SD 卡？）")
            else:
                g_recording = False
                if RECORD_DIR:
                    print(f"[REC] 停止录制，帧保存至: {RECORD_DIR}")
                RECORD_DIR = None

    # ── 检测（始终运行） ──
    objs = detector.detect(img, conf_th=0.35, iou_th=0.45)
    balls = []

    if objs:
        for obj in objs:
            cx = int(obj.x + obj.w / 2)
            cy = int(obj.y + obj.h / 2)
            if not in_roi(cx, cy):
                continue
            r = int(max(obj.w, obj.h) // 2)
            balls.append((cx, cy, r))

    # ── 锁定逻辑 ──
    if balls:
        if lock["active"]:
            best = None
            best_dist = LOCK_MAX_DIST
            for b in balls:
                dist = ((b[0] - lock["cx"])**2 + (b[1] - lock["cy"])**2)**0.5
                if dist < best_dist:
                    best_dist = dist
                    best = b
            if best:
                lock["cx"] = int(TRACK_ALPHA * lock["cx"] + (1 - TRACK_ALPHA) * best[0])
                lock["cy"] = int(TRACK_ALPHA * lock["cy"] + (1 - TRACK_ALPHA) * best[1])
                lock["r"] = best[2]
                lock["ttl"] = TRACK_TTL
                balls = [best]
            else:
                lock["ttl"] -= 1
                if lock["ttl"] <= 0:
                    lock["active"] = False
                    lock["cx"] = balls[0][0]
                    lock["cy"] = balls[0][1]
                    lock["r"] = balls[0][2]
                    lock["ttl"] = TRACK_TTL
                    lock["active"] = True
                    balls = [balls[0]]
                else:
                    balls = [(lock["cx"], lock["cy"], lock["r"])]
        else:
            lock["cx"] = balls[0][0]
            lock["cy"] = balls[0][1]
            lock["r"] = balls[0][2]
            lock["ttl"] = TRACK_TTL
            lock["active"] = True
            balls = [balls[0]]
    else:
        if lock["active"]:
            lock["ttl"] -= 1
            if lock["ttl"] <= 0:
                lock["active"] = False
            else:
                balls = [(lock["cx"], lock["cy"], lock["r"])]

    # ── 图传 & 录制（原图，无绘制叠加）──
    jpg = img.to_jpeg(quality=35)
    if jpg:
        g_jpg[:] = jpg.to_bytes()
        if g_recording and RECORD_DIR:
            try:
                jpg_bytes = jpg.to_bytes()
                fname = f"{RECORD_DIR}/frame_{frame_id:06d}.jpg"
                with open(fname, "wb") as f:
                    f.write(jpg_bytes)
            except Exception:
                pass

    # ── 绘制 ──
    img.draw_rect(ROI_X, ROI_Y, ROI_W, ROI_H, image.COLOR_YELLOW, 1)
    # 中心红色竖线
    img.draw_line(CX_HALF, 0, CX_HALF, CAM_H, image.COLOR_RED, 2)

    dev_cm = 0.0
    if balls:
        cx, cy, r = balls[0]
        img.draw_circle(cx, cy, r, color=image.COLOR_RED, thickness=2)
        img.draw_circle(cx, cy, 3, color=image.COLOR_GREEN, thickness=-1)
        # 偏离中心 cm 数（考虑FOV，画面宽度 CAM_W 对应实际 TUBE_CM）
        dev_cm = (cx - CX_HALF) * TUBE_CM / CAM_W
        # 速度测算（cm/s）—— FOV 映射：像素位移→实际位移→速度
        if prev_cx is not None and now != prev_time:
            dt_s = time.ticks_diff(now, prev_time) / 1000.0
            if dt_s > 0.01:
                px_disp = cx - prev_cx
                cm_disp = px_disp * TUBE_CM / CAM_W  # 像素→cm，比例由FOV和摄像头高度决定
                speed_cm = cm_disp / dt_s
                # 加速度（cm/s²）—— 速度变化÷时间变化
                if prev_speed_cm != 0:
                    accel_cm_s2 = (speed_cm - prev_speed_cm) / dt_s
                prev_speed_cm = speed_cm
        prev_cx = cx
        prev_time = now

    status = "LOCK" if lock["active"] else "DET"
    img.draw_string(2, 2, f'FPS:{fps_val} {status}', color=image.COLOR_GREEN, scale=1.2)
    if balls:
        img.draw_string(2, 20, f'Dev:{dev_cm:+.1f}cm', color=image.COLOR_RED, scale=1.2)
        img.draw_string(2, 38, f'Spd:{speed_cm:+.1f}cm/s', color=image.COLOR_YELLOW, scale=1.2)
        img.draw_string(2, 56, f'Acc:{accel_cm_s2:+.1f}cm/s2', color=image.COLOR_BLUE, scale=1.2)

    if g_recording:
        img.draw_string(CAM_W - 60, 2, 'REC', color=image.COLOR_RED, scale=1.2)

    dis.show(img)

    # ── 串口发送（偏离/速度/加速度）──
    if now - last_send >= SEND_INTERVAL_MS:
        send_serial(balls, dev_cm, speed_cm, accel_cm_s2)
        last_send = now

print("[INFO] 程序退出")