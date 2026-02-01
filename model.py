from ultralytics import YOLO
import cv2
import numpy as np

# Load a Model
model = YOLO('/Users/xiaoyangliu/Desktop/codes/cv/runs/detect/train7/weights/best.pt')

# Video Path
video_path = "20260201_024853.mp4"

# Video Capture
cap = cv2.VideoCapture(video_path)
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    results = model.predict(
        source=video_path,
        conf=0.1,
        save=True,
        save_txt=True,
        save_conf=True,
        show=False,     
        stream=False    
    )
    annotated = results[0].plot()
    cv2.imshow("Robot Detection", annotated)

    if cv2.waitKey(33) & 0xFF == ord('q'):
        break
    cap.release()
    cv2.destroyAllWindows()
