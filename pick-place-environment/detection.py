import cv2
import numpy as np
import time
from ultralytics import YOLO
import tkinter as tk
from tkinter import filedialog

# Step 1: Open a file picker dialog
root = tk.Tk()
root.withdraw()  # Hide the main tkinter window

model = YOLO("runs/detect/train29/weights/best.pt")
# model = YOLO("../models/colorcubes6.pt")

px_to_mm = 2.618

keep_running = True
while keep_running:
    image_path = filedialog.askopenfilename(
        title="Select an Image",
        filetypes=[("Image Files", "*.jpg *.jpeg *.png")]
    )

    if image_path:
        start_time = time.perf_counter()

        # Load an image
        image = cv2.imread(image_path)

        # Perform inference
        results = model.predict(image, conf=0.1)

        for result in results:
            for idx, box in enumerate(result.boxes):
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Convert to integers
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                cls_id = int(box.cls[0].item())
                label = model.names[cls_id]
                if label == 'redcube':
                    center_x = cx
                    center_y = cy

        # Get bounding boxes and coordinates
        for result in results:
            for idx, box in enumerate(result.boxes):
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Convert to integers
                conf = box.conf[0].item()  # Confidence score
                cls = int(box.cls[0].item())  # Class ID

                # Draw bounding box
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2

                # cv2.circle(image, (int(cx), int(cy)), radius=3, color=(0, 0, 255), thickness=-1)  # Red dot
                cv2.putText(image, str(idx), (int(cx), int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                cls_id = int(box.cls[0].item())
                label = model.names[cls_id]
                cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                distance_y = (cy - center_y) / px_to_mm
                distance_x = (cx - center_x) / px_to_mm

                print(f"Idx: {idx}, Object: {label} X_mid: {cx} Y_mid: {cy}, X1:{x1},X2:{x2},Y1:{y1},Y2:{y2} Confidence score {conf}. \
                      Distance from center in mm (x, y): {distance_x, distance_y}")

        end_time = time.perf_counter()

        print(f"Time taken for inference: {end_time - start_time:.2f}")

        # Save and display the image with boxes
        cv2.imwrite("cube_detection.jpg", image)

        # Show the image in a window
        # TODO: Fix a crash when closing the image window
        cv2.imshow('Detection Results', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        keep_running = False
        print("No image selected. Will exit.")