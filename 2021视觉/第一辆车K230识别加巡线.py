
import gc
import os
import time
import math

import aicube
import image
import nncase_runtime as nn
import ujson
import ulab.numpy as np
# from libs.PipeLine import ScopedTiming
from libs.Utils import *
from media.display import *
from media.media import *
from media.sensor import *
from machine import UART
from machine import FPIOA
from machine import Pin

# 配置参数
display_mode = "lcd"
if display_mode == "lcd":
    DISPLAY_WIDTH = ALIGN_UP(800, 16)
    DISPLAY_HEIGHT = 480
else:
    DISPLAY_WIDTH = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080

OUT_RGB888P_WIDTH = ALIGN_UP(640, 16)
OUT_RGB888P_HEIGH = 360

# AI识别配置
root_path = "/sdcard/mp_deployment_source/"
config_path = root_path + "deploy_config.json"
debug_mode = 1

# 巡线阈值（LAB空间，适用于RGB565）
BLACK_THRESHOLD = [(0, 40, -128, 127, -128, 127)]  # 黑线阈值
RED_THRESHOLD = [(20, 60, 20, 127, 20, 127)]      # 红线阈值
# 蓝线
# LINE_THRESHOLD = [(0, 50, -128, 0, -128, 0)]
# LINE_THRESHOLD = [(20, 60, 10, 127, 10, 127)]  # 默认黑线

# ROI区域配置 (x, y, w, h, weight)
ROIS = [
    (0, 400, 800, 80, 0.6),   # 下方
    (0, 240, 800, 80, 0.3),   # 中间
    (0, 80, 800, 80, 0.1)     # 上方
]

weight_sum = sum(r[4] for r in ROIS)

# 功能开关（由状态机自动控制，不手动设置）
# ENABLE_LINE_FOLLOW = False    # 是否启用巡线
# ENABLE_DETECTION = True      # 是否启用AI识别

# 模式状态
MODE_DETECTION = 0
MODE_LINE = 1
mode = MODE_DETECTION
pause_for_m = False

# 识别模式相关变量
first_number = None
number_list = []
detect_count = 0
max_detect_count = 8
match_found = False
match_direction = None

# 返程相关变量
returning = False
return_count = 0
return_direction = None
last_match_direction = None

# 用于延时切换模式
import utime as time_mod
mode_switch_time = 0
mode_pause = False
pause_duration = 0

# 串口通信内容显示缓存
uart_last_send = ""
uart_last_recv = ""
uart_last_send_meaning = ""
uart_last_recv_meaning = ""
# 识别显示缓存
last_detected_number = ""
last_match_result = ""

# 路口转向栈
turn_stack = []

# 按键初始化，GPIO21，如无反应可尝试其他引脚
button = Pin(21, Pin.IN, Pin.PULL_UP)

# 串口接收缓冲区（修复NameError）
uart_rx_buf = bytearray()

def two_side_pad_param(input_size, output_size):
    ratio_w = output_size[0] / input_size[0]
    ratio_h = output_size[1] / input_size[1]
    ratio = min(ratio_w, ratio_h)
    new_w = int(ratio * input_size[0])
    new_h = int(ratio * input_size[1])
    dw = (output_size[0] - new_w) / 2
    dh = (output_size[1] - new_h) / 2
    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw - 0.1))
    return top, bottom, left, right, ratio

def read_deploy_config(config_path):
    with open(config_path, "r") as json_file:
        try:
            config = ujson.load(json_file)
        except ValueError as e:
            print("JSON 解析错误:", e)
    return config
'''
def send_uart_number(uart, number, confidence):
    """串口发送数字和置信度"""
    send_data = bytearray([0xFF, ord(number[0]), int(confidence * 100), 0xFE])
    uart.write(send_data)
    print("send")
'''
def send_uart_cmd(uart, flag, cmd=None):
    global uart_last_send
    if flag == 0x00:
        uart_last_send = cmd
        data = bytearray([0xFF, 0x00, ord(cmd), 0xFE])
        uart.write(data)
    elif flag == 0x01:
        uart_last_send = "angle"
        str_val = "{:.1f}".format(-cmd)  # 保留一位小数
        str_bytes = str_val.encode()    # 转为字节流
        data = bytearray([0xFF, 0x01]) + str_bytes + bytearray([0xFE])
        uart.write(data)
    elif flag == 0x02:
        uart_last_send = f"flag2_{cmd}"
        data = bytearray([0xFF, 0x02, ord(cmd), 0xFE])
        uart.write(data)
    print(f"TX: {uart_last_send}")
#先将浮点数格式化为字符串（保留一位小数），
#再转为字节流发送（encode()），
#最终数据包格式为 [0xFF, 0x01] + str_bytes + [0xFE]。
def line_detection(img, osd_img=None):
    """巡线检测函数，支持在osd_img上画框"""
    centroid_sum = 0
    cross_detected = False
    rects = []
    threshold = RED_THRESHOLD  # 巡线改为红线阈值
    use_color = True
    roi_valid = [True, True, True]
    roi_centers = [None, None, None]
    roi_weights = [r[4] for r in ROIS]
    overwide_count = 0
    OVERWIDE_THRESH = 120  # 红线过宽阈值，可根据实际调整

    for idx, r in enumerate(ROIS):
        blobs = img.find_blobs(threshold, roi=r[0:4], merge=True, color=use_color)
        if blobs:
            largest_blob = max(blobs, key=lambda b: b.pixels())
            rects.append(largest_blob.rect())
            w = largest_blob.w()
            if w > OVERWIDE_THRESH:
                roi_valid[idx] = False
                overwide_count += 1
            else:
                roi_centers[idx] = largest_blob.cx()
            # 路口检测：只用最上面ROI的宽色块
            if idx == 2 and w > 400:
                cross_detected = True
        else:
            rects.append(None)
            roi_valid[idx] = False

    # 只用不过宽的红线块计算偏移角度
    if overwide_count < 3:
        valid_sum = sum(roi_weights[i] for i in range(3) if roi_valid[i])
        centroid_sum = sum((roi_centers[i] or 0) * roi_weights[i] for i in range(3) if roi_valid[i])
        center_pos = (centroid_sum / valid_sum) if valid_sum > 0 else 400
        deflection_angle = -math.atan((center_pos - 400) / 240)
        deflection_angle = math.degrees(deflection_angle)
    else:
        deflection_angle = 0  # 或None，表示不计算

    # 在osd_img上画出每个ROI的检测框
    if osd_img is not None:
        for idx, r in enumerate(ROIS):
            if rects[idx] is not None:
                x, y, w, h = rects[idx]
                osd_img.draw_rectangle(x, y, w, h, color=(255,255,255), thickness=3)
                osd_img.draw_cross(x+w//2, y+h//2, color=(255,255,255), size=32, thickness=3)
    return deflection_angle, cross_detected

# ========== 功能函数封装 ========== #

def reset_all_status():
    """一键复位，重置所有状态变量"""
    global mode, first_number, number_list, detect_count, match_found, match_direction, mode_switch_time, mode_pause, pause_duration, pause_for_m, returning, return_count, return_direction, last_match_direction, last_detected_number, last_match_result, turn_stack, uart_last_send, uart_last_recv
    mode = MODE_DETECTION
    first_number = None
    number_list = []
    detect_count = 0
    match_found = False
    match_direction = None
    mode_switch_time = 0
    mode_pause = False
    pause_duration = 0
    pause_for_m = False
    returning = False
    return_count = 0
    return_direction = None
    last_match_direction = None
    last_detected_number = ""
    last_match_result = ""
    turn_stack.clear()
    uart_last_send = ""
    uart_last_recv = ""

def check_button_reset(osd_img, uart):
    """检测按键复位，执行复位流程"""
    if button.value() == 0:
        send_uart_cmd(uart, 0x00, 's')
        print("TX: s")
        osd_img.clear()
        osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 2, 32, "复位", color=(255,0,0))
        Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
        gc.collect()
        reset_all_status()
        # 等待按键松开，防止连发
        while button.value() == 0:
            pass
        return True
    return False

def receive_uart_cmd(uart, cmd_char):
    """
    通用串口接收函数，接收格式为 [0xFF, 0x00, cmd_char, 0xFE]
    参数：uart 串口对象，cmd_char 字符（如 'm'）
    返回：True（收到该指令），False（未收到）
    """
    global uart_last_recv, uart_rx_buf, mode, returning, return_count, cross_count, black_rect_found
    data = uart.read(128)
    if data:
        uart_rx_buf += data
        while len(uart_rx_buf) >= 4:
            if uart_rx_buf[0] == 0xff and uart_rx_buf[1] == 0x00 and uart_rx_buf[2] == ord(cmd_char) and uart_rx_buf[3] == 0xfe:
                uart_last_recv = cmd_char
                print(f"RX: {cmd_char}")
                uart_rx_buf = uart_rx_buf[4:]
                return True
            else:
                uart_rx_buf = uart_rx_buf[1:]
    return False

def check_uart_m(uart):
    """检测串口m指令，返回True表示收到m"""
    global mode, returning, return_count, cross_count, black_rect_found
    if receive_uart_cmd(uart, 'm'):
        if mode is None:
            if returning:
                mode = MODE_LINE
                print("返程收到m，进入返程巡线")
            else:
                returning = True
                return_count = cross_count
                black_rect_found = False
                mode = MODE_LINE
                print(f"去程终点收到m，进入返程巡线，返程计数return_count={return_count}")
        return True
    return False

def detect_cross(blobs, idx):
    """路口检测：只用最上面ROI的宽色块"""
    if blobs:
        largest_blob = max(blobs, key=lambda b: b.pixels())
        w = largest_blob.w()
        if idx == 2 and w > 400:
            return True
    return False

def detect_end_rects(img, area_min=500, area_max=5000, aspect_min=0.5, aspect_max=2.0, rect_num=6):
    """终点检测：检测多个小黑色矩形块"""
    blobs = img.find_blobs(BLACK_THRESHOLD, merge=True, color=True)
    rect_count = 0
    for b in blobs:
        area = b.w() * b.h()
        aspect = b.w() / b.h() if b.h() > 0 else 0
        if area_min < area < area_max and aspect_min < aspect < aspect_max:
            rect_count += 1
    return rect_count >= rect_num

def detect_red_disappear(img):
    """检测红色色块是否消失"""
    red_blobs = img.find_blobs(RED_THRESHOLD, merge=True, color=True)
    return not red_blobs

def detection_and_line_following():
    global mode, first_number, number_list, detect_count, max_detect_count, match_found, match_direction, mode_switch_time, mode_pause, pause_duration, pause_for_m, returning, return_count, return_direction, last_match_direction, last_detected_number, last_match_result, turn_stack, uart_last_send, uart_last_recv
    print("det_infer start")

    # 初始化AI识别
    deploy_conf = read_deploy_config(config_path)
    kmodel_name = deploy_conf["kmodel_path"]
    labels = deploy_conf["categories"]
    confidence_threshold = deploy_conf["confidence_threshold"]
    nms_threshold = deploy_conf["nms_threshold"]
    img_size = deploy_conf["img_size"]
    num_classes = deploy_conf["num_classes"]
    color_four = get_colors(num_classes)
    nms_option = deploy_conf["nms_option"]
    model_type = deploy_conf["model_type"]
    if model_type == "AnchorBaseDet":
        anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]
    kmodel_frame_size = img_size
    frame_size = [OUT_RGB888P_WIDTH, OUT_RGB888P_HEIGH]
    strides = [8, 16, 32]
    top, bottom, left, right, ratio = two_side_pad_param(frame_size, kmodel_frame_size)
    kpu = nn.kpu()
    kpu.load_kmodel(root_path + kmodel_name)
    ai2d = nn.ai2d()
    ai2d.set_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
    ai2d.set_pad_param(True, [0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
    ai2d_builder = ai2d.build(
        [1, 3, OUT_RGB888P_HEIGH, OUT_RGB888P_WIDTH], [1, 3, kmodel_frame_size[1], kmodel_frame_size[0]]
    )
    sensor = Sensor()
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_565, chn=CAM_CHN_ID_1)
    sensor.set_framesize(width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGH, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)
    sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)
    if display_mode == "lcd":
        Display.init(Display.ST7701, to_ide=True)
    else:
        Display.init(Display.LT9611, to_ide=True)
    osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
    MediaManager.init()
    sensor.run()
    ai2d_input_tensor = None
    data = np.ones((1, 3, kmodel_frame_size[1], kmodel_frame_size[0]), dtype=np.uint8)
    ai2d_output_tensor = nn.from_numpy(data)
    fpioa = FPIOA()
    fpioa.set_function(3, FPIOA.UART1_TXD)
    fpioa.set_function(4, FPIOA.UART1_RXD)
    uart = UART(UART.UART1, 9600)
    cross_flag = 0
    cross_count = 0
    black_rect_found = False
    uart_rx_buf = bytearray()
    # 状态机主循环
    while True:
        # 1. 按键复位优先
        if check_button_reset(osd_img, uart):
            continue
        # 2. LCD状态显示
        osd_img.clear()
        mode_str = ("返程巡线" if returning else ("识别模式" if mode == MODE_DETECTION else "巡线模式" if mode == MODE_LINE else "无模式"))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 2, 32, mode_str, color=(0,255,255))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 40, 24, f"Init: {first_number if first_number else '-'}", color=(255,255,255))
        if mode == MODE_DETECTION:
            osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 70, 24, f"Cur: {last_detected_number if last_detected_number else '-'}", color=(255,255,255))
            osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 100, 24, last_match_result, color=(0,255,255))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 130, 24, f"TX: {uart_last_send}", color=(255,255,0))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 160, 24, f"RX: {uart_last_recv}", color=(0,255,0))
        # 3. 状态机流程
        if mode == MODE_DETECTION:
            # 识别初始数字或路口数字
            rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)
            if rgb888p_img.format() == image.RGBP888:
                ai2d_input = rgb888p_img.to_numpy_ref()
                ai2d_input_tensor = nn.from_numpy(ai2d_input)
                ai2d_builder.run(ai2d_input_tensor, ai2d_output_tensor)
                kpu.set_input_tensor(0, ai2d_output_tensor)
                kpu.run()
                results = []
                for i in range(kpu.outputs_size()):
                    out_data = kpu.get_output_tensor(i)
                    result = out_data.to_numpy()
                    result = result.reshape((result.shape[0] * result.shape[1] * result.shape[2] * result.shape[3]))
                    del out_data
                    results.append(result)
                det_boxes = aicube.anchorbasedet_post_process(
                    results[0], results[1], results[2],
                    kmodel_frame_size, frame_size, strides,
                    num_classes, confidence_threshold, nms_threshold,
                    anchors, nms_option,
                )
                detected_number = None
                detected_direction = None
                match_found_this_round = False
                for det_boxe in det_boxes:
                    x1, y1, x2, y2 = det_boxe[2], det_boxe[3], det_boxe[4], det_boxe[5]
                    x = int(x1 * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                    w = int((x2 - x1) * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                    center_x = x + w // 2
                    position = "左" if center_x < DISPLAY_WIDTH // 2 else "右"
                    number = labels[det_boxe[0]]
                    if det_boxe[1] > 0.7:
                        if first_number is None:
                            first_number = number
                            print(f"保存初始数字: {first_number}")
                            send_uart_cmd(uart, 0x00, 'f')  # 初始识别后自启动，f不入栈
                            mode = MODE_LINE
                            break
                        if number == first_number:
                            turn = 'l' if position == '左' else 'r'
                            turn_stack.append(turn)
                            send_uart_cmd(uart, 0x00, turn)
                            print(f"TX: {turn}")
                            mode = MODE_LINE
                            match_found_this_round = True
                            break
                        if detected_number is None:
                            detected_number = number
                            detected_direction = position
                if not match_found_this_round and detected_number is not None:
                    if first_number is None:
                        first_number = detected_number
                        print(f"保存初始数字: {first_number}")
                        send_uart_cmd(uart, 0x00, 'f')
                        mode = MODE_LINE
                    else:
                        turn_stack.append('f')
                        send_uart_cmd(uart, 0x00, 'f')
                        print("TX: f")
                        mode = MODE_LINE
        elif mode == MODE_LINE:
            # 巡线主流程
            line_img = sensor.snapshot(chn=CAM_CHN_ID_1)
            deflection_angle, cross_detected = line_detection(line_img, osd_img)
            send_uart_cmd(uart, 0x01, deflection_angle)
            # 路口检测
            if cross_detected and cross_flag == 0:
                cross_flag = 1
                cross_count += 1 if not returning else 0
                send_uart_cmd(uart, 0x00, 's')
                if not returning:
                    mode = MODE_DETECTION
                else:
                    return_count -= 1
                    if turn_stack:
                        turn = turn_stack.pop()
                        turn = {'l': 'r', 'r': 'l', 'f': 'f'}[turn]
                        send_uart_cmd(uart, 0x00, turn)
                        print(f"返程TX: {turn}")
                    if return_count <= 0:
                        print("返程路口数为0，等待终点")
            elif not cross_detected and cross_flag == 1:
                cross_flag = 0
            # 终点检测
            blobs = line_img.find_blobs(BLACK_THRESHOLD, merge=True, color=True)
            rect_count = 0
            for b in blobs:
                area = b.w() * b.h()
                aspect = b.w() / b.h() if b.h() > 0 else 0
                if 500 < area < 5000 and 0.5 < aspect < 2.0:
                    rect_count += 1
            if not black_rect_found and rect_count >= 6:
                print(f"检测到{rect_count}个小黑色矩形，等待红色消失")
                black_rect_found = True
            if black_rect_found:
                red_blobs = line_img.find_blobs(RED_THRESHOLD, merge=True, color=True)
                if not red_blobs:
                    if not returning:
                        send_uart_cmd(uart, 0x00, 'b')
                        print("TX: b")
                        # 新增：判断cross_count，发送0xff,0x02,'m',0xfe
                        if cross_count == 2:
                            send_uart_cmd(uart, 0x02, 'm')
                            osd_img.draw_string_advanced(10, 10, 32, "中端", color=(255,0,0))
                        elif cross_count == 4:
                            send_uart_cmd(uart, 0x02, 'f')
                            osd_img.draw_string_advanced(10, 10, 32, "远端", color=(0,0,255))
                        mode = None
                    else:
                        send_uart_cmd(uart, 0x00, 'y')
                        print("TX: y")
                        mode = None
                    black_rect_found = False
                    continue
        elif mode is None:
            # 无模式，等待m指令进入返程
            if receive_uart_cmd(uart, 'm'):
                if not returning:
                    returning = True
                    return_count = cross_count
                    black_rect_found = False
                    print(f"去程终点收到m，进入返程巡线，返程计数return_count={return_count}")
                mode = MODE_LINE
        Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
        gc.collect()

    # 清理资源
    del ai2d_input_tensor
    del ai2d_output_tensor
    sensor.stop()
    Display.deinit()
    MediaManager.deinit()
    gc.collect()
    time.sleep(1)
    nn.shrink_memory_pool()
    print("det_infer end")
    return 0

if __name__ == "__main__":
    detection_and_line_following()
