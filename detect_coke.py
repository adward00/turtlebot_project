import cv2
from ultralytics import YOLO
import math

model = YOLO(r'C:\Users\202-06\Downloads\cocacola.yolov8\runs\detect\coke_yolo\weights\last.pt')
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    results = model(frame, stream=True, conf=0.7)
    coke_detected = False

    for result in results:
        boxes = result.boxes
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = box.conf[0]
            cls = int(box.cls[0])
            name = model.names[cls]

            if name == "cocacola":
                coke_detected = True

            color = (127, 0, 127) # 원하는 색상 아무거나 하세요
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{name} {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    height, width, channels = frame.shape
    if coke_detected:
        cv2.putText(frame, "WARNING", (50, height//2), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 255), 3)

    cv2.imshow('frame', frame)

    if cv2.waitKey(100) == 27:
        break

cap.release()
cv2.destroyAllWindows()
