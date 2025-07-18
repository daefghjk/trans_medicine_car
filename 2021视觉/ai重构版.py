# -*- coding: utf-8 -*-
import gc
import math
import time
import aicube
import image
import nncase_runtime as nn
import ujson
import ulab.numpy as np
from machine import FPIOA, Pin, UART
from media.display import *
from media.media import *
from media.sensor import *
# 假设 get_colors 和 ALIGN_UP 函数来自该库
from libs.Utils import ALIGN_UP, get_colors

# ==============================================================================
# 1. 配置参数 (Constants)
# ==============================================================================

# 显示配置
DISPLAY_MODE = "lcd"
DISPLAY_WIDTH = ALIGN_UP(800, 16) if DISPLAY_MODE == "lcd" else ALIGN_UP(1920, 16)
DISPLAY_HEIGHT = 480 if DISPLAY_MODE == "lcd" else 1080

# AI输入图像尺寸
AI_INPUT_WIDTH = ALIGN_UP(640, 16)
AI_INPUT_HEIGHT = 360

# AI模型配置路径
ROOT_PATH = "/sdcard/mp_deployment_source/"
CONFIG_PATH = ROOT_PATH + "deploy_config.json"

# 巡线阈值 (LAB色彩空间) - 适用于红色巡线
LINE_THRESHOLD = [(20, 60, 10, 127, 10, 127)]
# 终点红色检测阈值
RED_THRESHOLD_FOR_END = [(20, 60, 20, 127, 20, 127)]
# 终点标记黑色检测阈值
BLACK_THRESHOLD_FOR_END = [(0, 40, -128, 127, -128, 127)]


# 巡线ROI区域配置 (x, y, w, h, weight)
ROIS = [
    (0, 400, 800, 80, 0.6),  # 下方
    (0, 240, 800, 80, 0.3),  # 中间
    (0, 80, 800, 80, 0.1)    # 上方
]

# 状态机模式定义
MODE_DETECTION = 0  # AI识别模式
MODE_LINE = 1       # 巡线模式
MODE_PAUSED = 2     # 暂停模式（用于转向后短暂延时）
MODE_WAIT_M = 3     # 等待'm'指令模式（去程结束后）
MODE_FINISHED = 4   # 完成模式（返程结束后）

class LineFollowingCar:
    """
    一个集成了AI数字识别和巡线功能的状态机类。
    通过将状态和逻辑封装在此类中，避免了全局变量的滥用，并使代码结构更清晰。
    """
    def __init__(self):
        # 初始化硬件和AI所需的对象
        self.sensor = None
        self.kpu = None
        self.ai2d_builder = None
        self.ai2d_input_tensor = None
        self.ai2d_output_tensor = None
        self.uart = None
        self.button = None
        self.osd_img = None

        # AI模型参数
        self.ai_config = {}
        self.ai_labels = []
        self.ai_padding_params = {}

        # 状态变量
        self._reset_state()

    def _reset_state(self):
        """重置所有状态变量到初始状态"""
        print("System state has been reset.")
        self.mode = MODE_DETECTION
        self.is_returning = False
        self.black_marker_found = False

        self.initial_number = None
        self.turn_stack = []
        self.cross_count = 0
        self.return_trip_crosses = 0

        self.pause_until_time = 0

        self.uart_rx_buf = bytearray()
        
        # OSD显示缓存
        self.osd_info = {
            "mode": "识别模式",
            "init_num": "-",
            "cur_num": "-",
            "match_res": "",
            "tx": "",
            "rx": "",
            "angle": 0.0,
            "cross_count": 0,
            "info": ""
        }

    def _setup_hardware(self):
        """初始化摄像头、显示屏、UART和按键"""
        print("Initializing hardware...")
        # 1. 显示和OSD
        if DISPLAY_MODE == "lcd":
            Display.init(Display.ST7701, to_ide=True)
        else:
            Display.init(Display.LT9611, to_ide=True)
        self.osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)

        # 2. 摄像头传感器
        self.sensor = Sensor()
        self.sensor.reset()
        # Chn0: For Display (YUV420)
        self.sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
        self.sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
        # Chn1: For Line Following (RGB565)
        self.sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, chn=CAM_CHN_ID_1)
        self.sensor.set_pixformat(PIXEL_FORMAT_RGB_565, chn=CAM_CHN_ID_1)
        # Chn2: For AI Detection (RGB888P)
        self.sensor.set_framesize(width=AI_INPUT_WIDTH, height=AI_INPUT_HEIGHT, chn=CAM_CHN_ID_2)
        self.sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)
        
        sensor_bind_info = self.sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
        Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)
        
        # 3. UART
        fpioa = FPIOA()
        fpioa.set_function(3, FPIOA.UART1_TXD)
        fpioa.set_function(4, FPIOA.UART1_RXD)
        self.uart = UART(UART.UART1, 9600)

        # 4. 按键
        self.button = Pin(21, Pin.IN, Pin.PULL_UP)

        # 5. 启动 Media
        MediaManager.init()
        self.sensor.run()

    def _setup_ai(self):
        """加载AI模型并初始化AI处理器"""
        print("Initializing AI model...")
        with open(CONFIG_PATH, "r") as f:
            self.ai_config = ujson.load(f)

        self.ai_labels = self.ai_config["categories"]
        kmodel_frame_size = self.ai_config["img_size"]
        
        # 1. KPU
        self.kpu = nn.kpu()
        self.kpu.load_kmodel(ROOT_PATH + self.ai_config["kmodel_path"])

        # 2. AI2D
        frame_size = [AI_INPUT_WIDTH, AI_INPUT_HEIGHT]
        top, bottom, left, right, ratio = self._get_padding_param(frame_size, kmodel_frame_size)
        self.ai_padding_params = {'top': top, 'bottom': bottom, 'left': left, 'right': right, 'ratio': ratio}

        ai2d = nn.ai2d()
        ai2d.set_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
        ai2d.set_pad_param(True, [0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
        ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
        
        self.ai2d_builder = ai2d.build(
            [1, 3, AI_INPUT_HEIGHT, AI_INPUT_WIDTH], 
            [1, 3, kmodel_frame_size[1], kmodel_frame_size[0]]
        )

        # 3. AI Tensors
        data = np.ones((1, 3, kmodel_frame_size[1], kmodel_frame_size[0]), dtype=np.uint8)
        self.ai2d_output_tensor = nn.from_numpy(data)

    @staticmethod
    def _get_padding_param(input_size, output_size):
        """计算图像填充参数以保持宽高比"""
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
        right = int(round(dw + 0.1))
        return top, bottom, left, right, ratio

    def _send_uart_cmd(self, flag, cmd=None):
        """统一的UART发送接口"""
        if flag == 0x00:  # 发送字符指令
            self.osd_info["tx"] = f"CMD: {cmd}"
            data = bytearray([0xFF, 0x00, ord(cmd), 0xFE])
            self.uart.write(data)
        elif flag == 0x01:  # 发送角度值
            angle_str = "{:.1f}".format(-cmd)
            self.osd_info["tx"] = f"ANGLE: {angle_str}"
            data = bytearray([0xFF, 0x01]) + angle_str.encode() + bytearray([0xFE])
            self.uart.write(data)
        print(f"TX > {self.osd_info['tx']}")

    def _handle_uart_rx(self):
        """处理UART接收到的'm'指令"""
        data = self.uart.read(128)
        if not data:
            return

        self.uart_rx_buf += data
        while len(self.uart_rx_buf) >= 4:
            # 滑动窗口查找指令包 [0xFF, 0x00, 'm', 0xFE]
            if (self.uart_rx_buf[0] == 0xff and self.uart_rx_buf[1] == 0x00 and \
                self.uart_rx_buf[2] == ord('m') and self.uart_rx_buf[3] == 0xfe):
                
                self.osd_info["rx"] = "CMD: m"
                print("RX < CMD: m")
                
                if self.mode == MODE_WAIT_M:
                    print("Received 'm', starting return trip.")
                    self.is_returning = True
                    self.return_trip_crosses = self.cross_count
                    self.black_marker_found = False
                    self.mode = MODE_LINE
                
                self.uart_rx_buf = self.uart_rx_buf[4:]
            else:
                self.uart_rx_buf = self.uart_rx_buf[1:]

    def _handle_button_reset(self):
        """检查并处理按键复位"""
        if self.button.value() == 0:
            self._send_uart_cmd(0x00, 's')
            self._reset_state()
            self.osd_info["info"] = "复位!"
            # 等待按键释放
            while self.button.value() == 0:
                time.sleep_ms(50)
            return True
        return False

    def _is_end_marker_present(self, img):
        """检测赛道终点标记（多个小黑块且无红线）"""
        # 1. 检查是否存在足够多的小黑块
        blobs = img.find_blobs([BLACK_THRESHOLD_FOR_END], merge=True, color=True)
        rect_count = 0
        for b in blobs:
            # 根据面积和长宽比筛选
            if 500 < (b.w() * b.h()) < 5000 and 0.5 < (b.w() / b.h() if b.h() > 0 else 0) < 2.0:
                rect_count += 1
        
        if rect_count < 6:
            return False

        # 2. 如果黑块足够，再检查是否已没有红线
        if not self.black_marker_found:
            self.black_marker_found = True
            print(f"Found {rect_count} black markers, now waiting for red line to disappear.")

        red_blobs = img.find_blobs([RED_THRESHOLD_FOR_END], merge=True, color=True)
        if not red_blobs:
            print("Red line disappeared. End of track confirmed.")
            return True
            
        return False

    def _line_detection(self, img, osd_img):
        """
        巡线检测，返回偏移角度和是否检测到路口。
        """
        centroid_sum = 0
        cross_detected = False
        
        # ROI权重和中心位置
        roi_weights = [r[4] for r in ROIS]
        roi_centers = [None] * len(ROIS)
        is_roi_valid = [True] * len(ROIS)
        overwide_count = 0
        OVERWIDE_THRESH = 120 # 用于过滤路口横线的宽度阈值

        for i, r in enumerate(ROIS):
            blobs = img.find_blobs([LINE_THRESHOLD], roi=r[0:4], merge=True, color=True)
            if blobs:
                largest_blob = max(blobs, key=lambda b: b.pixels())
                # 在OSD上绘制检测框
                osd_img.draw_rectangle(largest_blob.rect(), color=(255, 255, 255), thickness=2)
                
                # 检查色块是否过宽（可能是路口）
                if largest_blob.w() > OVERWIDE_THRESH:
                    is_roi_valid[i] = False
                    overwide_count += 1
                else:
                    roi_centers[i] = largest_blob.cx()
                
                # 仅使用最上方的ROI检测路口（宽度超过400像素）
                if i == 2 and largest_blob.w() > 400:
                    cross_detected = True
            else:
                is_roi_valid[i] = False

        # 计算加权平均偏移角
        deflection_angle = 0
        if overwide_count < len(ROIS):
            valid_weight_sum = sum(roi_weights[i] for i in range(len(ROIS)) if is_roi_valid[i])
            if valid_weight_sum > 0:
                weighted_center_sum = sum((roi_centers[i] or 0) * roi_weights[i] for i in range(len(ROIS)) if is_roi_valid[i])
                center_pos = weighted_center_sum / valid_weight_sum
                # 转换成角度
                deflection_angle = -math.degrees(math.atan((center_pos - DISPLAY_WIDTH / 2) / (DISPLAY_HEIGHT / 2)))

        return deflection_angle, cross_detected

    def _run_ai_inference(self, img):
        """执行一次AI推理"""
        ai2d_input = img.to_numpy_ref()
        self.ai2d_input_tensor = nn.from_numpy(ai2d_input)
        self.ai2d_builder.run(self.ai2d_input_tensor, self.ai2d_output_tensor)
        self.kpu.set_input_tensor(0, self.ai2d_output_tensor)
        self.kpu.run()
        
        results = []
        for i in range(self.kpu.outputs_size()):
            data = self.kpu.get_output_tensor(i).to_numpy()
            results.append(data.flatten())

        cfg = self.ai_config
        anchors = cfg["anchors"][0] + cfg["anchors"][1] + cfg["anchors"][2]
        
        det_boxes = aicube.anchorbasedet_post_process(
            results[0], results[1], results[2],
            cfg["img_size"], [AI_INPUT_WIDTH, AI_INPUT_HEIGHT],
            [8, 16, 32], cfg["num_classes"], cfg["confidence_threshold"],
            cfg["nms_threshold"], anchors, cfg["nms_option"]
        )
        return det_boxes

    def _update_osd(self):
        """统一更新OSD上的所有信息"""
        self.osd_img.clear()
        
        # 模式
        self.osd_img.draw_string_advanced(DISPLAY_WIDTH - 200, 10, 32, self.osd_info["mode"], color=(0, 255, 255))
        # 初始数字
        self.osd_img.draw_string_advanced(DISPLAY_WIDTH - 200, 50, 24, f"Init: {self.osd_info['init_num']}", color=(255, 255, 255))
        # 当前识别数字
        self.osd_img.draw_string_advanced(DISPLAY_WIDTH - 200, 80, 24, f"Cur: {self.osd_info['cur_num']}", color=(255, 255, 255))
        # 匹配结果
        self.osd_img.draw_string_advanced(DISPLAY_WIDTH - 200, 110, 24, self.osd_info["match_res"], color=(0, 255, 255))
        # 串口通信
        self.osd_img.draw_string_advanced(DISPLAY_WIDTH - 200, 140, 24, f"TX: {self.osd_info['tx']}", color=(255, 255, 0))
        self.osd_img.draw_string_advanced(DISPLAY_WIDTH - 200, 170, 24, f"RX: {self.osd_info['rx']}", color=(0, 255, 0))

        # 巡线信息
        if self.mode == MODE_LINE:
            prefix = "Ret" if self.is_returning else "Fwd"
            count = self.return_trip_crosses if self.is_returning else self.cross_count
            self.osd_img.draw_string_advanced(10, 10, 24, f"{prefix} Angle: {self.osd_info['angle']:.1f}", color=(0, 255, 255))
            self.osd_img.draw_string_advanced(10, 40, 24, f"{prefix} Cross: {count}", color=(0, 255, 255))
        
        # 全局提示信息
        if self.osd_info["info"]:
            self.osd_img.draw_string_advanced(DISPLAY_WIDTH//2 - 100, DISPLAY_HEIGHT//2 - 20, 40, self.osd_info["info"], color=(255, 0, 0))

    def run(self):
        """主循环，运行状态机"""
        self._setup_hardware()
        self._setup_ai()
        
        cross_flag = 0 # 用于防止在同一路口重复触发

        while True:
            # 1. 处理高优先级任务：复位和UART指令
            if self._handle_button_reset():
                continue
            self._handle_uart_rx()

            # 2. 如果处于暂停状态，检查是否可以恢复
            if self.mode == MODE_PAUSED:
                if time.ticks_ms() > self.pause_until_time:
                    self.mode = MODE_LINE # 暂停结束，切回巡线
                    self.osd_info["info"] = ""
                else: # 仍在暂停中，刷新屏幕并跳过本轮循环
                    self._update_osd()
                    Display.show_image(self.osd_img, 0, 0, Display.LAYER_OSD3)
                    gc.collect()
                    continue

            # 3. 主状态机逻辑
            # ----------------- AI识别模式 (在路口) -----------------
            if self.mode == MODE_DETECTION:
                self.osd_info["mode"] = "识别模式"
                ai_img = self.sensor.snapshot(chn=CAM_CHN_ID_2)
                if ai_img and ai_img.format() == image.RGBP888:
                    det_boxes = self._run_ai_inference(ai_img)
                    
                    # 分析识别结果
                    match_found = False
                    for box in det_boxes:
                        if box[1] < 0.7: continue # 置信度过滤

                        number = self.ai_labels[box[0]]
                        self.osd_info["cur_num"] = number
                        center_x = (box[2] + box[4]) / 2
                        position = "左" if center_x < AI_INPUT_WIDTH / 2 else "右"
                        turn_cmd = 'l' if position == "左" else 'r'

                        # 如果是第一个路口，记录初始数字并前进
                        if self.initial_number is None:
                            self.initial_number = number
                            self.osd_info["init_num"] = number
                            print(f"Initial number set to: {self.initial_number}")
                            self._send_uart_cmd(0x00, 'f')
                            self.mode = MODE_LINE
                            match_found = True
                            break
                        
                        # 如果识别到的数字与初始数字匹配
                        if number == self.initial_number:
                            print(f"Match found! Turning '{turn_cmd}'")
                            self.turn_stack.append(turn_cmd)
                            self._send_uart_cmd(0x00, turn_cmd)
                            
                            self.osd_info["match_res"] = f"MATCH! Turn {turn_cmd.upper()}"
                            self.osd_info["info"] = f"转向: {turn_cmd.upper()}"
                            self.mode = MODE_PAUSED
                            self.pause_until_time = time.ticks_ms() + 2000 # 暂停2秒
                            match_found = True
                            break
                    
                    # 如果遍历完所有目标都没有匹配上，则直行
                    if not match_found and self.initial_number is not None:
                        print("No match found, going forward.")
                        self.turn_stack.append('f')
                        self._send_uart_cmd(0x00, 'f')
                        self.osd_info["match_res"] = "No Match"
                        self.osd_info["info"] = "直行"
                        self.mode = MODE_PAUSED
                        self.pause_until_time = time.ticks_ms() + 2000 # 暂停2秒

            # ----------------- 巡线模式 -----------------
            elif self.mode == MODE_LINE:
                line_img = self.sensor.snapshot(chn=CAM_CHN_ID_1)
                if not line_img: continue
                
                # A. 检查是否到达终点
                if (self.is_returning and self.return_trip_crosses <= 0) or \
                   (not self.is_returning and self.initial_number is not None):
                    if self._is_end_marker_present(line_img):
                        if self.is_returning: # 返程到达终点
                            print("Return trip finished.")
                            self._send_uart_cmd(0x00, 'y')
                            self.mode = MODE_FINISHED
                            self.osd_info["mode"] = "完成"
                            self.osd_info["info"] = "已到达起点!"
                        else: # 去程到达终点
                            print("Forward trip finished, waiting for 'm'.")
                            self._send_uart_cmd(0x00, 'b')
                            self.mode = MODE_WAIT_M
                            self.osd_info["mode"] = "等待返程"
                            self.osd_info["info"] = "等待m指令..."
                        continue

                # B. 进行巡线和路口检测
                angle, cross_detected = self._line_detection(line_img, self.osd_img)
                self.osd_info["angle"] = angle
                self._send_uart_cmd(0x01, angle)

                if self.is_returning:
                    self.osd_info["mode"] = "返程巡线"
                else:
                    self.osd_info["mode"] = "去程巡线"

                # C. 处理路口
                if cross_detected and cross_flag == 0:
                    cross_flag = 1
                    self._send_uart_cmd(0x00, 's') # 路口停车
                    
                    if self.is_returning: # 返程路口处理
                        self.return_trip_crosses -= 1
                        print(f"Return cross detected. Remaining: {self.return_trip_crosses}")
                        if self.turn_stack:
                            last_turn = self.turn_stack.pop()
                            # 执行相反的转向
                            turn_cmd = {'l': 'r', 'r': 'l', 'f': 'f'}[last_turn]
                            self._send_uart_cmd(0x00, turn_cmd)
                            self.osd_info["info"] = f"返程转向: {turn_cmd.upper()}"
                            self.mode = MODE_PAUSED # 转向后暂停
                            self.pause_until_time = time.ticks_ms() + 2000
                        else: # 如果转向栈为空，说明已回到起点附近
                             print("Turn stack empty on return, continuing line follow.")

                    else: # 去程路口处理
                        self.cross_count += 1
                        self.osd_info["cross_count"] = self.cross_count
                        print(f"Forward cross detected, switching to AI. Count: {self.cross_count}")
                        self.mode = MODE_DETECTION
                
                elif not cross_detected and cross_flag == 1:
                    cross_flag = 0 # 离开路口，重置标志

            # ----------------- 等待'm'指令模式 -----------------
            elif self.mode == MODE_WAIT_M:
                self.osd_info["mode"] = "等待返程"
                self.osd_info["info"] = "等待m指令..."
            
            # ----------------- 完成模式 -----------------
            elif self.mode == MODE_FINISHED:
                self.osd_info["mode"] = "完成"
                self.osd_info["info"] = "任务结束!"

            # 4. 统一刷新屏幕显示并进行垃圾回收
            self._update_osd()
            Display.show_image(self.osd_img, 0, 0, Display.LAYER_OSD3)
            gc.collect()

if __name__ == "__main__":
    car = LineFollowingCar()
    try:
        car.run()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # 清理资源
        if car.sensor:
            car.sensor.stop()
        Display.deinit()
        MediaManager.deinit()
        nn.shrink_memory_pool()
        print("Application terminated and resources released.")