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
# 黑线
# LINE_THRESHOLD = [(0, 40, -128, 127, -128, 127)]
# 红线
# LINE_THRESHOLD = [(20, 60, 20, 127, 20, 127)]
# 蓝线
# LINE_THRESHOLD = [(0, 50, -128, 0, -128, 0)]
LINE_THRESHOLD = [(20, 60, 20, 127, 20, 127)]  # 默认黑线

# ROI区域配置 (x, y, w, h, weight)
ROIS = [
    (0, 400, 800, 80, 0.7),   # 下方
    (0, 240, 800, 80, 0.2),   # 中间
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
def send_uart_cmd(uart, flag, cmd1=None, cmd2=None):
    global uart_last_send
    if flag == 0x00:
        # 只显示高层内容
        if cmd1 is not None:
            uart_last_send = cmd1
        if cmd2 is not None:
            uart_last_send += f" {cmd2}"
    elif flag == 0x01:
        uart_last_send = "angle"
    # flag: 0x00 指令，0x01 浮点数
    if flag == 0x00:
        data = bytearray([0xFF, 0x00])
        if cmd1 is not None:
            data.append(ord(cmd1))
        if cmd2 is not None:
            data.append(ord(cmd2))
        data.append(0xFE)
        uart.write(data)
    elif flag == 0x01:
        import struct
        float_bytes = struct.pack('f', cmd1)
        data = bytearray([0xFF, 0x01]) + float_bytes + bytearray([0xFE])
        uart.write(data)
    print(f"TX: {uart_last_send}")

def line_detection(img, osd_img=None):
    """巡线检测函数，支持在osd_img上画框"""
    centroid_sum = 0
    cross_detected = False
    rects = []
    threshold = LINE_THRESHOLD
    use_color = True
    for idx, r in enumerate(ROIS):
        blobs = img.find_blobs(threshold, roi=r[0:4], merge=True, color=use_color)
        if blobs:
            largest_blob = max(blobs, key=lambda b: b.pixels())
            # 记录矩形框坐标
            rects.append(largest_blob.rect())
            centroid_sum += largest_blob.cx() * r[4]
            # 路口检测：最上方ROI且宽度大于400
            if idx == 2 and largest_blob.w() > 400:
                cross_detected = True
        else:
            rects.append(None)
    center_pos = (centroid_sum / weight_sum) if weight_sum > 0 else 400
    deflection_angle = -math.atan((center_pos - 400) / 240)
    deflection_angle = math.degrees(deflection_angle)
    # 在osd_img上画出每个ROI的检测框
    if osd_img is not None:
        for idx, r in enumerate(ROIS):
            if rects[idx] is not None:
                x, y, w, h = rects[idx]
                osd_img.draw_rectangle(x, y, w, h, color=(255,255,255), thickness=3)
                osd_img.draw_cross(x+w//2, y+h//2, color=(255,255,255), size=32, thickness=3)
    return deflection_angle, cross_detected

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

    # 计算padding值
    top, bottom, left, right, ratio = two_side_pad_param(frame_size, kmodel_frame_size)

    # 初始化kpu
    kpu = nn.kpu()
    kpu.load_kmodel(root_path + kmodel_name)

    # 初始化ai2d
    ai2d = nn.ai2d()
    ai2d.set_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
    ai2d.set_pad_param(True, [0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
    ai2d_builder = ai2d.build(
        [1, 3, OUT_RGB888P_HEIGH, OUT_RGB888P_WIDTH], [1, 3, kmodel_frame_size[1], kmodel_frame_size[0]]
    )

    # 初始化传感器 - 兼容配置
    sensor = Sensor()
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)

    # 通道0: 显示用 (YUV420)
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)

    # 通道1: 巡线用 (RGB565)
    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_565, chn=CAM_CHN_ID_1)

    # 通道2: AI识别用 (RGB888)
    sensor.set_framesize(width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGH, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)

    # 绑定显示
    sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)

    if display_mode == "lcd":
        Display.init(Display.ST7701, to_ide=True)
    else:
        Display.init(Display.LT9611, to_ide=True)

    # 创建OSD图像
    osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)

    # media初始化
    MediaManager.init()
    sensor.run()

    # 初始化AI处理变量
    rgb888p_img = None
    ai2d_input_tensor = None
    data = np.ones((1, 3, kmodel_frame_size[1], kmodel_frame_size[0]), dtype=np.uint8)
    ai2d_output_tensor = nn.from_numpy(data)

    # 初始化串口
    fpioa = FPIOA()
    fpioa.set_function(3, FPIOA.UART1_TXD)
    fpioa.set_function(4, FPIOA.UART1_RXD)
    uart = UART(UART.UART1, 9600)

    # 巡线状态变量
    cross_flag = 0
    cross_count = 0

    def uart_check_for_m(uart):
        global uart_last_recv
        if uart.any() >= 4:
            data = uart.read(4)
            if data and len(data) == 4 and data[0] == 0xFF and data[1] == 0x00 and data[2] == ord('m') and data[3] == 0xFE:
                uart_last_recv = "m"
                print(f"RX: {uart_last_recv}")
                return True
            else:
                uart_last_recv = "?"
                print(f"RX: {uart_last_recv}")
        return False

    while True:
        if pause_for_m:
            osd_img.clear()
            mode_str = "返程巡线" if returning else ("识别模式" if mode == MODE_DETECTION else "巡线模式" if mode == MODE_LINE else "无模式")
            osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 2, 32, mode_str, color=(0,255,255))
            osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 40, 24, f"Init: {first_number if first_number else '-'}", color=(255,255,255))
            osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 70, 24, uart_last_send, color=(255,255,0))
            osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 100, 24, uart_last_recv, color=(0,255,0))
            osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 130, 24, "等待m指令", color=(255,0,0))
            Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
            gc.collect()
            if uart_check_for_m(uart):
                pause_for_m = False
                # 进入返程模式，只巡线
                returning = True
                mode = MODE_LINE
                return_count = cross_count
                # 取反方向
                if match_direction is not None:
                    last_match_direction = match_direction[0].lower()
                    return_direction = 'l' if last_match_direction == 'r' else 'r'
                else:
                    return_direction = 'f'  # 默认前进
            continue
        osd_img.clear()
        mode_str = "返程巡线" if returning else ("识别模式" if mode == MODE_DETECTION else "巡线模式" if mode == MODE_LINE else "无模式")
        # 右上角显示
        osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 2, 32, mode_str, color=(0,255,255))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 40, 24, f"Init: {first_number if first_number else '-'}", color=(255,255,255))
        if mode == MODE_DETECTION:
            osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 70, 24, f"Cur: {last_detected_number if last_detected_number else '-'}", color=(255,255,255))
            osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 100, 24, last_match_result, color=(0,255,255))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 130, 24, f"TX: {uart_last_send}", color=(255,255,0))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 160, 24, f"RX: {uart_last_recv}", color=(0,255,0))
        if mode_pause:
            if time_mod.ticks_ms() - mode_switch_time > pause_duration:
                mode_pause = False
                mode = MODE_LINE
            else:
                Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
                gc.collect()
                continue
        # 返程逻辑
        if returning:
            line_img = sensor.snapshot(chn=CAM_CHN_ID_1)
            deflection_angle, cross_detected = line_detection(line_img, osd_img)
            print(f"返程偏移角度: {deflection_angle:.2f}")
            send_uart_cmd(uart, 0x01, deflection_angle)
            if cross_detected and cross_flag == 0:
                cross_flag = 1
                return_count -= 1
                print(f"返程路口，剩余: {return_count}")
                send_uart_cmd(uart, 0x00, 's')  # 先停车
                # 出栈并发送反方向
                if turn_stack:
                    turn = turn_stack.pop()
                    if turn == 'l':
                        send_uart_cmd(uart, 0x00, 'r')
                        print("TX: r")
                    elif turn == 'r':
                        send_uart_cmd(uart, 0x00, 'l')
                        print("TX: l")
                    else:
                        send_uart_cmd(uart, 0x00, 'f')
                        print("TX: f")
                    t0 = time_mod.ticks_ms()
                    while time_mod.ticks_ms() - t0 < 2000:
                        pass
                if return_count <= 0:
                    print("返程路口数为0，等待小黑色矩形停止")
            elif not cross_detected and cross_flag == 1:
                cross_flag = 0
            # 显示返程信息
            osd_img.draw_string_advanced(2, 2, 20, f'return_count={return_count}', color=(255,0,0))
            # 返程count为0后，检测小黑色矩形块
            if return_count <= 0:
                img = sensor.snapshot(chn=CAM_CHN_ID_1)
                blobs = img.find_blobs(LINE_THRESHOLD, merge=True, color=True)
                for b in blobs:
                    area = b.w() * b.h()
                    aspect = b.w() / b.h() if b.h() > 0 else 0
                    # 设定面积和宽高比阈值，排除大长条线
                    if 1000 < area < 10000 and 0.5 < aspect < 2.0:
                        print("返程检测到小黑色矩形，发送s，停止")
                        send_uart_cmd(uart, 0x00, 's')
                        returning = False
                        mode = None
                        break
            Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
            gc.collect()
            continue
        # 原有识别/巡线逻辑
        if mode == MODE_DETECTION:
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
                display_texts = []
                detected_number = None
                detected_direction = None
                match_found_this_round = False
                for det_boxe in det_boxes:
                    x1, y1, x2, y2 = det_boxe[2], det_boxe[3], det_boxe[4], det_boxe[5]
                    x = int(x1 * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                    y = int(y1 * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
                    w = int((x2 - x1) * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                    h = int((y2 - y1) * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
                    osd_img.draw_rectangle(x, y, w, h, color=color_four[det_boxe[0]][1:])
                    text = labels[det_boxe[0]] + " " + str(round(det_boxe[1], 2))
                    osd_img.draw_string_advanced(x, y - 40, 32, text, color=color_four[det_boxe[0]][1:])
                    if det_boxe[1] > 0.7:
                        number = labels[det_boxe[0]]
                        confidence = det_boxe[1]
                        center_x = x + w // 2
                        position = "左" if center_x < DISPLAY_WIDTH // 2 else "右"
                        osd_img.draw_string_advanced(x, y - 80, 32, position, color=(0, 255, 255))
                        print(f"识别数字: {number}, 置信度: {confidence:.2f}, 位置: {position}")
                        # 遍历每个数字都和init比较
                        if number == first_number:
                            turn = 'l' if position == '左' else 'r'
                            turn_stack.append(turn)
                            send_uart_cmd(uart, 0x00, turn)
                            print(f"TX: {turn}")
                            t0 = time_mod.ticks_ms()
                            while time_mod.ticks_ms() - t0 < 2000:
                                pass
                            last_match_result = f"yes {turn}"
                            uart_last_send = f"{turn}"
                            mode_pause = True
                            pause_duration = 2000
                            # 清空采集，切回巡线
                            number_list = []
                            detect_count = 0
                            mode_switch_time = time_mod.ticks_ms()
                            mode = MODE_LINE
                            match_found_this_round = True
                            break
                        # 只采集第一个数字和方向
                        if detected_number is None:
                            detected_number = number
                            detected_direction = position
                if not match_found_this_round and detected_number is not None:
                    last_detected_number = detected_number
                    if first_number is None:
                        first_number = detected_number
                        print(f"保存初始数字: {first_number}")
                        mode = MODE_LINE
                    else:
                        if detect_count < max_detect_count:
                            number_list.append((detected_number, detected_direction))
                            detect_count += 1
                            print(f"采集第{detect_count}次: {detected_number}, {detected_direction}")
                        if detect_count >= max_detect_count:
                            turn_stack.append('f')
                            send_uart_cmd(uart, 0x00, 'f')
                            print("TX: f")
                            last_match_result = "none f"
                            uart_last_send = "f"
                            mode_pause = True
                            pause_duration = 2000
                            number_list = []
                            detect_count = 0
                            mode_switch_time = time_mod.ticks_ms()
                            mode = MODE_LINE
        elif mode == MODE_LINE:
            line_img = sensor.snapshot(chn=CAM_CHN_ID_1)
            deflection_angle, cross_detected = line_detection(line_img, osd_img)
            print(f"偏移角度: {deflection_angle:.2f}")
            send_uart_cmd(uart, 0x01, deflection_angle)
            # 路口状态机
            if cross_detected and cross_flag == 0:
                cross_flag = 1
                cross_count += 1
                print("cross=1, count=%d" % cross_count)
                send_uart_cmd(uart, 0x00, 's')  # 先停车
                print("TX: s")
                uart_last_send = "s"
                # 路口检测到，切换到识别模式
                mode = MODE_DETECTION
            elif not cross_detected and cross_flag == 1:
                cross_flag = 0
                print("cross=0")
            # 巡线模式下检测多个小黑色矩形块作为终点
            blobs = line_img.find_blobs([(0, 40, -128, 127, -128, 127)], merge=True, color=True)
            rect_count = 0
            for b in blobs:
                area = b.w() * b.h()
                aspect = b.w() / b.h() if b.h() > 0 else 0
                if 500 < area < 5000 and 0.5 < aspect < 2.0:
                    rect_count += 1
            if rect_count >= 6:
                print(f"巡线检测到{rect_count}个小黑色矩形，发送s，停止")
                send_uart_cmd(uart, 0x00, 's')
                mode = None
            # 显示巡线信息
            osd_img.draw_string_advanced(2, 2, 20, f'cross={cross_flag}', color=(255,0,0))
            osd_img.draw_string_advanced(2, 24, 20, f'angle={deflection_angle:.1f}', color=(0,255,255))
            osd_img.draw_string_advanced(2, 46, 20, f'count={cross_count}', color=(0,255,255))

        # 最后统一显示
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
