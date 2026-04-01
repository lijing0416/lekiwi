import cv2
import numpy as np
import time

# --- 参数配置 ---
FX, FY = 420, 420
CX, CY = 320.0, 200.0
INSTALL_H = 25.0  # 安装高度(mm)
INSTALL_L = 100.0  # 安装水平偏移(mm)
VIDEO_INDEX = 4
EXPECTED_SIZE = 512000 
DEPTH_SCALE = 0.0627 # 深度缩放因子
LIMIT_MM = 400.0     # 限制距离：40cm

class NuwaFinal:
    def __init__(self):
        self.mx, self.my = -1, -1
        self.depth_data = None

    def click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.mx, self.my = x, y

def run():
    ui = NuwaFinal()
    # 开启 V4L2
    cap = cv2.VideoCapture(VIDEO_INDEX, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print(f"无法打开设备 /dev/video{VIDEO_INDEX}")
        return

    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    win_name = 'Nuwa_30cm_Monitor'
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL) 
    cv2.resizeWindow(win_name, 640, 400)
    cv2.setMouseCallback(win_name, ui.click_event)

    print(f"正在读取数据... 限制范围: {LIMIT_MM}mm 以内")

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            if frame.size != EXPECTED_SIZE:
                continue

            # 1. 深度图解析
            depth = frame.view(np.uint16).reshape((400, 640))
            ui.depth_data = depth

            # 2. 核心过滤：只保留 400mm 以内的数据用于可视化
            # 将原始数据转为浮点计算实际毫米，超出限制的设为 0
            depth_mm = depth.astype(np.float32) * DEPTH_SCALE
            display_mask = (depth_mm > 0) & (depth_mm <= LIMIT_MM)
            # 过滤后的深度图：40cm外全部黑掉
            filtered_depth = np.where(display_mask, depth, 0).astype(np.uint16)

            # 3. 生成伪彩色图
            vis = cv2.normalize(filtered_depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            color_map = cv2.applyColorMap(vis, cv2.COLORMAP_JET)

            # 4. 坐标计算与显示
            if ui.mx != -1:
                z_raw = float(depth[ui.my, ui.mx])
                z_c = z_raw * DEPTH_SCALE
                
                # 判断点击点是否在 40cm 内
                if 0 < z_c <= LIMIT_MM:
                    # 相机坐标系转换
                    x_c = (ui.mx - CX) * z_c / FX
                    y_c = (ui.my - CY) * z_c / FY
                    
                    # 机器人/物理坐标系转换
                    rx = z_c + INSTALL_L
                    ry = -x_c
                    rz = INSTALL_H - y_c
                    
                    # 打印结果并画绿圈
                    print(f"\r[有效] X: {rx:4.0f}mm | Y: {ry:4.0f}mm | Z: {rz:4.0f}mm | 深度: {z_c:4.1f}mm", end="")
                    cv2.circle(color_map, (ui.mx, ui.my), 8, (0, 255, 0), 2)
                else:
                    # 超出范围画红圈
                    print(f"\r[无效] 当前点深度 {z_c:4.1f}mm 已超过 300mm 限制        ", end="")
                    cv2.circle(color_map, (ui.mx, ui.my), 8, (0, 0, 255), 2)

            cv2.imshow(win_name, color_map)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("\n程序已退出")

if __name__ == "__main__":
    run()