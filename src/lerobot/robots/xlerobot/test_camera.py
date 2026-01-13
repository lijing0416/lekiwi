import cv2
def list_cameras(max_index=5):
    available = []
    for idx in range(max_index):
        cap = cv2.VideoCapture(idx)
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
cap = cv2.VideoCapture(selected)
success, frame = cap.read()
if success:
    print("相机读取成功,按'q'键退出.")
else:
    print("相机读取失败.")
while cap.isOpened():
    success, frame = cap.read()
    # Read a frame from the camera
    if success:
        # Display the annotated frame in a window
        cv2.imshow("camera", frame)
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("退出相机.")
            break
    else:
        # Break the loop if the camera is not providing frames
        break