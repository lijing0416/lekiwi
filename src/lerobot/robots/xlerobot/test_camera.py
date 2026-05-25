import cv2


def list_cameras(max_index=10):
    available = []
    for idx in range(max_index):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if cap.isOpened():
            available.append(idx)
        cap.release()
    return available


camera = list_cameras()

if not camera:
    print("没有找到相机.")
    exit(1)

print(f"可用相机: {camera}")

selected = int(input(f"从列表里选择相机 {camera}: "))

cap = cv2.VideoCapture(selected, cv2.CAP_V4L2)

# 双目拼接模式：左目 640x480 + 右目 640x480 = 1280x480
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("相机打开失败.")
    exit(1)

success, frame = cap.read()

if not success:
    print("相机读取失败.")
    cap.release()
    exit(1)

height, width = frame.shape[:2]
print(f"实际分辨率：{width}×{height}，像素：{width * height / 10000:.2f}万")

if width < 1000:
    print("警告：当前不是双目拼接分辨率，可能只读到了单目画面。")
else:
    print("双目读取成功，按 q 退出。")

while cap.isOpened():
    success, frame = cap.read()

    if not success:
        print("读取失败.")
        break

    height, width = frame.shape[:2]

    # 左右切分
    left_frame = frame[:, :width // 2]
    right_frame = frame[:, width // 2:]

    cv2.imshow("left camera", left_frame)
    cv2.imshow("right camera", right_frame)
    cv2.imshow("stereo raw", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        print("退出相机.")
        break

cap.release()
cv2.destroyAllWindows()