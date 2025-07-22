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

# 适配2.4寸mipi屏参数
display_mode = "lcd"
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
OUT_RGB888P_WIDTH = 640
OUT_RGB888P_HEIGH = 480

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
LINE_THRESHOLD = [(20, 60, 10, 127, 10, 127)]  # 默认黑线
BLACK_THRESHOLD = [(0, 40, -128, 127, -128, 127)]
RED_THRESHOLD = [(20, 60, 20, 127, 20, 127)]

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
MODE_EXPAND = 2  # 拓展模式
MODE_NONE = 3    # 无模式
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

def check_button_reset(osd_img, uart):
    # 检查按键是否按下，按下则复位
    if button.value() == 0:  # 按下为低电平
        osd_img.draw_string_advanced(10, 10, 32, "复位中...", color=(255,0,0))
        Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
        time.sleep(1)  # 延时防抖
        return True
    return False

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
    print(f"TX: {uart_last_send}")
#先将浮点数格式化为字符串（保留一位小数），
#再转为字节流发送（encode()），
#最终数据包格式为 [0xFF, 0x01] + str_bytes + [0xFE]。
def line_detection(img, osd_img=None):
    """巡线检测函数，支持在osd_img上画框"""
    centroid_sum = 0
    cross_detected = False
    rects = []
    threshold = LINE_THRESHOLD
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
            if idx == 2 and w > 170:
                cross_detected = True
        else:
            rects.append(None)
            roi_valid[idx] = False

    # 只用不过宽的红线块计算偏移角度
    if overwide_count < 3:
        valid_sum = sum(roi_weights[i] for i in range(3) if roi_valid[i])
        centroid_sum = sum((roi_centers[i] or 0) * roi_weights[i] for i in range(3) if roi_valid[i])
        center_pos = (centroid_sum / valid_sum) if valid_sum > 0 else 400
        deflection_angle = -math.atan((center_pos - 320) / 240)
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

def detection_and_line_following():
    global mode, first_number, number_list, detect_count, max_detect_count, match_found, match_direction, mode_switch_time, mode_pause, pause_duration, pause_for_m, returning, return_count, return_direction, last_match_direction, last_detected_number, last_match_result, turn_stack, uart_last_send, uart_last_recv, normal_cross_count
    print("det_infer start")

    # 初始化AI识别、传感器、显示、串口等（略，保持原有初始化代码）
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

    # 初始化传感器 - 兼容配置
    sensor = Sensor()
    sensor.reset()
    sensor.set_hmirror(False)
    # 通道0直接给到显示VO，格式为YUV420
    sensor.set_framesize(width=640, height=480)  # 2.4寸屏分辨率
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
    # 通道1: 巡线用 (RGB565)
    sensor.set_framesize(width=640, height=480, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_565, chn=CAM_CHN_ID_1)
    # 通道2给到AI做算法处理，格式为RGB888
    sensor.set_framesize(width=640, height=480, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)
    # 绑定通道0的输出到vo
    sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)
    # 适配2.4寸mipi屏
    Display.init(Display.ST7701, width=640, height=480, to_ide=True)
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
    ai2d_input_tensor = None
    data = np.ones((1, 3, kmodel_frame_size[1], kmodel_frame_size[0]), dtype=np.uint8)
    ai2d_output_tensor = nn.from_numpy(data)
    fpioa = FPIOA()
    fpioa.set_function(3, FPIOA.UART1_TXD)
    fpioa.set_function(4, FPIOA.UART1_RXD)
    uart = UART(UART.UART1, 9600)
    # 状态变量
    cross_flag = 0
    cross_count = 0
    black_rect_found = False
    uart_rx_buf = bytearray()
    expand_first_left = False  # 拓展模式下第一个路口左转标志
    expand_after_m = False     # m后拓展模式返程标志
    expand_m_count = 0         # 记录收到m的次数
    normal_cross_count = 0     # 记录普通去程阶段的路口数
    # 主循环
    while True:
        # G. 按键复位
        if check_button_reset(osd_img, uart):
            cross_flag = 0
            cross_count = 0
            black_rect_found = False
            expand_first_left = False
            expand_after_m = False
            expand_m_count = 0
            returning = False
            return_count = 0
            mode = MODE_DETECTION
            first_number = None
            turn_stack.clear()
            continue
        # LCD状态显示
        osd_img.clear()
        mode_str = ("拓展返程" if mode==MODE_EXPAND and expand_after_m else "拓展模式" if mode==MODE_EXPAND else "返程巡线" if returning else ("识别模式" if mode==MODE_DETECTION else "巡线模式" if mode==MODE_LINE else "无模式"))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 2, 32, mode_str, color=(0,255,255))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 40, 24, f"Init: {first_number if first_number else '-'}", color=(255,255,255))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-320, DISPLAY_HEIGHT-80, 20, f"去程cross:{cross_count}", color=(0,255,0))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-320, DISPLAY_HEIGHT-60, 20, f"返程ret:{return_count}", color=(0,255,255))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-320, DISPLAY_HEIGHT-40, 20, f"栈长:{len(turn_stack)}", color=(255,255,0))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-320, DISPLAY_HEIGHT-20, 20, f"栈:{turn_stack}", color=(255,255,255))
        if returning and (return_count != len(turn_stack)):
            osd_img.draw_string_advanced(DISPLAY_WIDTH-320, DISPLAY_HEIGHT-100, 24, "栈与返程数不一致!", color=(255,0,0))
        if mode == MODE_DETECTION:
            osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 70, 24, f"Cur: {last_detected_number if last_detected_number else '-'}", color=(255,255,255))
            osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 100, 24, last_match_result, color=(0,255,255))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 130, 24, f"TX: {uart_last_send}", color=(255,255,0))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 160, 24, f"RX: {uart_last_recv}", color=(0,255,0))
        # F. 无模式，等待m指令
        if mode == MODE_NONE:
            data = uart.read(128)
            if data:
                uart_rx_buf += data
                while len(uart_rx_buf) >= 4:
                    if uart_rx_buf[0] == 0xff and uart_rx_buf[1] == 0x00 and uart_rx_buf[2] == ord('m') and uart_rx_buf[3] == 0xfe:
                        uart_last_recv = 'm'
                        expand_m_count += 1
                        send_uart_cmd(uart, 0x00, 'f')
                        print(f"收到第{expand_m_count}次m，发f")
                        uart_rx_buf = uart_rx_buf[4:]
                        if expand_m_count == 1:
                            # 第一次m，进入拓展返程
                            mode = MODE_EXPAND
                            expand_after_m = True
                            cross_count = 0
                            expand_first_left = False
                            black_rect_found = False
                        elif expand_m_count == 2:
                            # 第二次m，进入普通返程
                            mode = MODE_LINE
                            returning = True
                            return_count = normal_cross_count  # 用普通去程阶段的路口数
                            expand_after_m = False
                            expand_first_left = False
                            black_rect_found = False
                        break
                    else:
                        uart_rx_buf = uart_rx_buf[1:]
            Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
            gc.collect()
            continue
        # A. 上电/复位后，MODE_DETECTION
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
                            # 初始识别后自动进入拓展模式
                            mode = MODE_EXPAND
                            cross_count = 0
                            expand_first_left = False
                            expand_after_m = False
                            break
                        if number == first_number:
                            turn = 'l' if position == '左' else 'r'
                            turn_stack.append(turn)
                            send_uart_cmd(uart, 0x00, turn)
                            print(f"入栈: {turn}, 当前栈: {turn_stack}, cross_count: {cross_count}")
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
                        mode = MODE_EXPAND
                        cross_count = 0
                        expand_first_left = False
                        expand_after_m = False
                    else:
                        turn_stack.append('f')
                        send_uart_cmd(uart, 0x00, 'f')
                        print("入栈: f, 当前栈: {turn_stack}, cross_count: {cross_count}")
                        mode = MODE_LINE
        # B/C. 拓展模式/拓展返程模式
        elif mode == MODE_EXPAND:
            line_img = sensor.snapshot(chn=CAM_CHN_ID_1)
            deflection_angle, cross_detected = line_detection(line_img, osd_img)
            send_uart_cmd(uart, 0x01, deflection_angle)
            if cross_detected and cross_flag == 0:
                cross_flag = 1
                cross_count += 1
                if (not expand_after_m and cross_count == 1):
                    # 去程拓展，第一次路口强制左转，不入栈
                    send_uart_cmd(uart, 0x00, 'l')
                    print("拓展模式：第一个路口强制左转")
                    expand_first_left = True
                elif (expand_after_m and cross_count == 1):
                    # 拓展返程，第一次路口强制左转，入栈f，切回普通巡线
                    send_uart_cmd(uart, 0x00, 'l')
                    print("拓展返程：第一个路口强制左转并入栈f")
                    turn_stack.append('f')
                    expand_first_left = True
                    expand_after_m = False
                    mode = MODE_LINE
                    returning = False
                    print(f"切回普通巡线，栈:{turn_stack}")
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
                print(f"拓展模式检测到{rect_count}个小黑色矩形，等待红色消失")
                black_rect_found = True
            if black_rect_found:
                red_blobs = line_img.find_blobs(RED_THRESHOLD, merge=True, color=True)
                if not red_blobs:
                    send_uart_cmd(uart, 0x00, 'b')
                    print("拓展模式终点，发b进入无模式")
                    mode = MODE_NONE
                    cross_count = 0
                    expand_first_left = False
                    expand_after_m = False
                    black_rect_found = False
                    continue
        # D/E. 普通返程/普通去程巡线
        elif mode == MODE_LINE:
            line_img = sensor.snapshot(chn=CAM_CHN_ID_1)
            deflection_angle, cross_detected = line_detection(line_img, osd_img)
            send_uart_cmd(uart, 0x01, deflection_angle)
            if cross_detected and cross_flag == 0:
                cross_flag = 1
                if not returning:
                    cross_count += 1
                    send_uart_cmd(uart, 0x00, 's')
                    print(f"去程路口，count={cross_count}")
                    mode = MODE_DETECTION
                else:
                    return_count -= 1
                    send_uart_cmd(uart, 0x00, 's')
                    print(f"返程路口，剩余{return_count}")
                    if turn_stack:
                        turn = turn_stack.pop()
                        turn = {'l': 'r', 'r': 'l', 'f': 'f'}[turn]
                        send_uart_cmd(uart, 0x00, turn)
                        print(f"返程出栈: {turn}, 当前栈: {turn_stack}, return_count: {return_count}")
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
                        # 普通去程终点，记录normal_cross_count
                        normal_cross_count = cross_count
                        send_uart_cmd(uart, 0x00, 'b')
                        print("TX: b")
                        mode = MODE_NONE
                    else:
                        send_uart_cmd(uart, 0x00, 'y')
                        print("TX: y")
                        mode = MODE_NONE
                    black_rect_found = False
                    continue
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




'''
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

# 适配2.4寸mipi屏参数
display_mode = "lcd"
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
OUT_RGB888P_WIDTH = 640
OUT_RGB888P_HEIGH = 480

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
LINE_THRESHOLD = [(20, 60, 10, 127, 10, 127)]  # 默认黑线
BLACK_THRESHOLD = [(0, 40, -128, 127, -128, 127)]
RED_THRESHOLD = [(20, 60, 20, 127, 20, 127)]

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
MODE_EXPAND = 2  # 拓展模式
MODE_NONE = 3    # 无模式
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

def check_button_reset(osd_img, uart):
    # 检查按键是否按下，按下则复位
    if button.value() == 0:  # 按下为低电平
        osd_img.draw_string_advanced(10, 10, 32, "复位中...", color=(255,0,0))
        Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
        time.sleep(1)  # 延时防抖
        return True
    return False

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
    print(f"TX: {uart_last_send}")
#先将浮点数格式化为字符串（保留一位小数），
#再转为字节流发送（encode()），
#最终数据包格式为 [0xFF, 0x01] + str_bytes + [0xFE]。
def line_detection(img, osd_img=None):
    """巡线检测函数，支持在osd_img上画框"""
    centroid_sum = 0
    cross_detected = False
    rects = []
    threshold = LINE_THRESHOLD
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

def detection_and_line_following():
    global mode, first_number, number_list, detect_count, max_detect_count, match_found, match_direction, mode_switch_time, mode_pause, pause_duration, pause_for_m, returning, return_count, return_direction, last_match_direction, last_detected_number, last_match_result, turn_stack, uart_last_send, uart_last_recv, normal_cross_count
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
    # 通道0直接给到显示VO，格式为YUV420
    sensor.set_framesize(width=640, height=480)  # 2.4寸屏分辨率
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
    # 通道1: 巡线用 (RGB565)
    sensor.set_framesize(width=640, height=480, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_565, chn=CAM_CHN_ID_1)
    # 通道2给到AI做算法处理，格式为RGB888
    sensor.set_framesize(width=640, height=480, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)
    # 绑定通道0的输出到vo
    sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)
    # 适配2.4寸mipi屏
    Display.init(Display.ST7701, width=640, height=480, to_ide=True)
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
    ai2d_input_tensor = None
    data = np.ones((1, 3, kmodel_frame_size[1], kmodel_frame_size[0]), dtype=np.uint8)
    ai2d_output_tensor = nn.from_numpy(data)
    fpioa = FPIOA()
    fpioa.set_function(3, FPIOA.UART1_TXD)
    fpioa.set_function(4, FPIOA.UART1_RXD)
    uart = UART(UART.UART1, 9600)
    # 状态变量
    cross_flag = 0
    cross_count = 0
    black_rect_found = False
    uart_rx_buf = bytearray()
    expand_first_left = False  # 拓展模式下第一个路口左转标志
    expand_after_m = False     # m后拓展模式返程标志
    expand_m_count = 0         # 记录收到m的次数
    normal_cross_count = 0     # 记录普通去程阶段的路口数
    # 主循环
    while True:
        # G. 按键复位
        if check_button_reset(osd_img, uart):
            cross_flag = 0
            cross_count = 0
            black_rect_found = False
            expand_first_left = False
            expand_after_m = False
            expand_m_count = 0
            returning = False
            return_count = 0
            mode = MODE_DETECTION
            first_number = None
            turn_stack.clear()
            continue
        # LCD状态显示
        osd_img.clear()
        mode_str = ("拓展返程" if mode==MODE_EXPAND and expand_after_m else "拓展模式" if mode==MODE_EXPAND else "返程巡线" if returning else ("识别模式" if mode==MODE_DETECTION else "巡线模式" if mode==MODE_LINE else "无模式"))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 2, 32, mode_str, color=(0,255,255))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 40, 24, f"Init: {first_number if first_number else '-'}", color=(255,255,255))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-320, DISPLAY_HEIGHT-80, 20, f"去程cross:{cross_count}", color=(0,255,0))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-320, DISPLAY_HEIGHT-60, 20, f"返程ret:{return_count}", color=(0,255,255))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-320, DISPLAY_HEIGHT-40, 20, f"栈长:{len(turn_stack)}", color=(255,255,0))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-320, DISPLAY_HEIGHT-20, 20, f"栈:{turn_stack}", color=(255,255,255))
        if returning and (return_count != len(turn_stack)):
            osd_img.draw_string_advanced(DISPLAY_WIDTH-320, DISPLAY_HEIGHT-100, 24, "栈与返程数不一致!", color=(255,0,0))
        if mode == MODE_DETECTION:
            osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 70, 24, f"Cur: {last_detected_number if last_detected_number else '-'}", color=(255,255,255))
            osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 100, 24, last_match_result, color=(0,255,255))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 130, 24, f"TX: {uart_last_send}", color=(255,255,0))
        osd_img.draw_string_advanced(DISPLAY_WIDTH-160, 160, 24, f"RX: {uart_last_recv}", color=(0,255,0))
        # F. 无模式，等待m指令
        if mode == MODE_NONE:
            data = uart.read(128)
            if data:
                uart_rx_buf += data
                while len(uart_rx_buf) >= 4:
                    if uart_rx_buf[0] == 0xff and uart_rx_buf[1] == 0x00 and uart_rx_buf[2] == ord('m') and uart_rx_buf[3] == 0xfe:
                        uart_last_recv = 'm'
                        expand_m_count += 1
                        send_uart_cmd(uart, 0x00, 'f')
                        print(f"收到第{expand_m_count}次m，发f")
                        uart_rx_buf = uart_rx_buf[4:]
                        if expand_m_count == 1:
                            # 第一次m，进入拓展返程
                            mode = MODE_EXPAND
                            expand_after_m = True
                            cross_count = 0
                            expand_first_left = False
                            black_rect_found = False
                        elif expand_m_count == 2:
                            # 第二次m，进入普通返程
                            mode = MODE_LINE
                            returning = True
                            return_count = normal_cross_count  # 用普通去程阶段的路口数
                            expand_after_m = False
                            expand_first_left = False
                            black_rect_found = False
                        break
                    else:
                        uart_rx_buf = uart_rx_buf[1:]
            Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
            gc.collect()
            continue
        # A. 上电/复位后，MODE_DETECTION
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
                            # 初始识别后自动进入拓展模式
                            mode = MODE_EXPAND
                            cross_count = 0
                            expand_first_left = False
                            expand_after_m = False
                            break
                        if number == first_number:
                            turn = 'l' if position == '左' else 'r'
                            turn_stack.append(turn)
                            send_uart_cmd(uart, 0x00, turn)
                            print(f"入栈: {turn}, 当前栈: {turn_stack}, cross_count: {cross_count}")
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
                        mode = MODE_EXPAND
                        cross_count = 0
                        expand_first_left = False
                        expand_after_m = False
                    else:
                        turn_stack.append('f')
                        send_uart_cmd(uart, 0x00, 'f')
                        print("入栈: f, 当前栈: {turn_stack}, cross_count: {cross_count}")
                        mode = MODE_LINE
        # B/C. 拓展模式/拓展返程模式
        elif mode == MODE_EXPAND:
            line_img = sensor.snapshot(chn=CAM_CHN_ID_1)
            deflection_angle, cross_detected = line_detection(line_img, osd_img)
            send_uart_cmd(uart, 0x01, deflection_angle)
            if cross_detected and cross_flag == 0:
                cross_flag = 1
                cross_count += 1
                if (not expand_after_m and cross_count == 1):
                    # 去程拓展，第一次路口强制左转，不入栈
                    send_uart_cmd(uart, 0x00, 'l')
                    print("拓展模式：第一个路口强制左转")
                    expand_first_left = True
                elif (expand_after_m and cross_count == 1):
                    # 拓展返程，第一次路口强制左转，入栈f，切回普通巡线
                    send_uart_cmd(uart, 0x00, 'l')
                    print("拓展返程：第一个路口强制左转并入栈f")
                    turn_stack.append('f')
                    expand_first_left = True
                    expand_after_m = False
                    mode = MODE_LINE
                    returning = False
                    print(f"切回普通巡线，栈:{turn_stack}")
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
                print(f"拓展模式检测到{rect_count}个小黑色矩形，等待红色消失")
                black_rect_found = True
            if black_rect_found:
                red_blobs = line_img.find_blobs(RED_THRESHOLD, merge=True, color=True)
                if not red_blobs:
                    send_uart_cmd(uart, 0x00, 'b')
                    print("拓展模式终点，发b进入无模式")
                    mode = MODE_NONE
                    cross_count = 0
                    expand_first_left = False
                    expand_after_m = False
                    black_rect_found = False
                    continue
        # D/E. 普通返程/普通去程巡线
        elif mode == MODE_LINE:
            line_img = sensor.snapshot(chn=CAM_CHN_ID_1)
            deflection_angle, cross_detected = line_detection(line_img, osd_img)
            send_uart_cmd(uart, 0x01, deflection_angle)
            if cross_detected and cross_flag == 0:
                cross_flag = 1
                if not returning:
                    cross_count += 1
                    send_uart_cmd(uart, 0x00, 's')
                    print(f"去程路口，count={cross_count}")
                    mode = MODE_DETECTION
                else:
                    return_count -= 1
                    send_uart_cmd(uart, 0x00, 's')
                    print(f"返程路口，剩余{return_count}")
                    if turn_stack:
                        turn = turn_stack.pop()
                        turn = {'l': 'r', 'r': 'l', 'f': 'f'}[turn]
                        send_uart_cmd(uart, 0x00, turn)
                        print(f"返程出栈: {turn}, 当前栈: {turn_stack}, return_count: {return_count}")
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
                        # 普通去程终点，记录normal_cross_count
                        normal_cross_count = cross_count
                        send_uart_cmd(uart, 0x00, 'b')
                        print("TX: b")
                        mode = MODE_NONE
                    else:
                        send_uart_cmd(uart, 0x00, 'y')
                        print("TX: y")
                        mode = MODE_NONE
                    black_rect_found = False
                    continue
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
'''
