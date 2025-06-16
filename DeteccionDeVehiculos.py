import torch
import cv2
import serial
import time

# Carga el modelo
model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)

input_video = r"C:\Users\Liamsito\Downloads\WhatsApp Video 2025-06-15 at 18.19.29.mp4"
cap = cv2.VideoCapture(input_video)

conf_threshold = 0.5
area_threshold = 0.20

arduino = serial.Serial('COM8', 9600)
print("Conectado a Arduino")

delay_after_lost = 3  # segundos
last_close_time = None
servo_state = 0  # 0 = abajo, 1 = arriba

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)
    detections = results.xyxy[0]

    car_detections = detections[(detections[:, 5] == 2) & (detections[:, 4] >= conf_threshold)]

    frame_area = frame.shape[0] * frame.shape[1]
    close = False

    for *xyxy, conf, cls in car_detections:
        x1, y1, x2, y2 = map(int, xyxy)
        bbox_area = (x2 - x1) * (y2 - y1)
        if bbox_area / frame_area > area_threshold:
            close = True
            break

    current_time = time.time()

    if close:
        # Si el servo está abajo y carro se acerca, activar servo y marcar tiempo
        if servo_state == 0:
            print("Carro muy cerca - LEVANTANDO la pluma")
            arduino.write(b'1')
            servo_state = 1
        last_close_time = current_time

    else:
        # Carro no está cerca
        if servo_state == 1:
            # Si pasó el delay desde la última vez que estuvo cerca, bajar la pluma
            if last_close_time is not None and (current_time - last_close_time >= delay_after_lost):
                print("Carro se fue - BAJANDO la pluma después de espera")
                arduino.write(b'0')
                servo_state = 0
                last_close_time = None

    # Dibuja las cajas
    for *xyxy, conf, cls in car_detections:
        label = f'Carro {conf:.2f}'
        x1, y1, x2, y2 = map(int, xyxy)
        color = (0, 0, 255)
        thickness = 2
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        cv2.putText(frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

    frame_resized = cv2.resize(frame, (0, 0), fx=0.6, fy=0.6)
    cv2.imshow("Detección de Carros en Video", frame_resized)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
arduino.close()
print("Conexión Serial cerrada")
