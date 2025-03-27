import cv2
import torch
import numpy as np
from ultralytics import YOLO

model = YOLO('vest_det/new_weights.pt')

TARGET_CLASS_NAME = "designated-target"

target_class_id = None
for class_id, name in model.model.names.items():
    if name == TARGET_CLASS_NAME:
        target_class_id = class_id
        break

if target_class_id is None:
    raise ValueError(f"Target class '{TARGET_CLASS_NAME}' not found in model.")

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
center_x = frame_width // 2 

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    results = model(frame, conf=0.4, verbose = False)

    largest_box = None
    max_area = 0

    for result in results:
        for box in result.boxes:
            cls = int(box.cls[0].item())  
            if cls == target_class_id:  
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())  
                area = (x2 - x1) * (y2 - y1)

                if area > max_area:
                    max_area = area
                    largest_box = (x1, y1, x2, y2)

    if largest_box:
        x1, y1, x2, y2 = largest_box
        centroid_x = (x1 + x2) // 2
        centroid_y = (y1 + y2) // 2

        fov = 60  
        angle_per_pixel = fov / frame_width
        relative_angle = (centroid_x - center_x) * angle_per_pixel
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2)  

        label = "designated-target" 
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


        cv2.circle(frame, (centroid_x, centroid_y), 5, (255, 255, 255), -1)

        cv2.putText(frame, f"Angle: {relative_angle:.2f} degrees", (centroid_x - 50, centroid_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        print(f"Target Angle: {relative_angle:.2f} degrees")

    cv2.imshow("Target Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
