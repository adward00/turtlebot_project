import cv2
from ultralytics import YOLO

# =========================
# 모델 경로
# =========================
COKE_MODEL_PATH = r"C:\Users\202-06\Downloads\cocacola.yolov8\runs\detect\coke_yolo\weights\last.pt"
CHILSUNG_MODEL_PATH = r"C:\Users\202-06\Downloads\cocacola.yolov8\chilsung_yolov8\runs\detect\chilsung_yolo\weights\last.pt"

CONF = 0.6
WARNING_FRAMES = 5

# =========================
# 모델 로드
# =========================
coke_model = YOLO(COKE_MODEL_PATH)
chilsung_model = YOLO(CHILSUNG_MODEL_PATH)

cap = cv2.VideoCapture(0)

chilsung_count = 0

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)

    # =========================
    # 코카콜라 탐지
    # =========================
    coke_results = coke_model(frame, conf=CONF)
    coke_detected = False

    for r in coke_results:
        if r.boxes is None:
            continue
        for box in r.boxes:
            coke_detected = True
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"cocacola {conf:.2f}",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 2)

    # =========================
    # 칠성사이다 탐지
    # =========================
    chilsung_results = chilsung_model(frame, conf=CONF)
    chilsung_detected = False

    for r in chilsung_results:
        if r.boxes is None:
            continue
        for box in r.boxes:
            chilsung_detected = True
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(frame, f"chilsung {conf:.2f}",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), 2)

    # =========================
    # 경고 로직
    # =========================
    if chilsung_detected:
        chilsung_count += 1
    else:
        chilsung_count = 0

    if chilsung_count >= WARNING_FRAMES:
        h, w, _ = frame.shape
        cv2.putText(frame, "WARNING: WRONG PRODUCT",
                    (50, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 2.2,
                    (0, 0, 255), 5)

    cv2.imshow("Shelf Monitor", frame)

    if cv2.waitKey(100) == 27:
        break

cap.release()
cv2.destroyAllWindows()
