import cv2
import numpy as np
import time
from ultralytics import YOLO
import tkinter as tk
from tkinter import filedialog

def x_correction(x_value):
    return 1.021 * x_value - 1.0

def y_correction(y_value):
    return 0.96 * y_value + 5.28

if __name__ == '__main__':

    # Step 1: Open a file picker dialog
    root = tk.Tk()
    root.withdraw()  # Hide the main tkinter window

    model = YOLO("runs/detect/train29/weights/best.pt")
    # model = YOLO("../models/colorcubes6.pt")

    px_to_mm = 2.618

    # Yellow cube (or any origin cube) location in Roboarm coordinates
    yellow_origin_x_mm = 8
    yellow_origin_y_mm = -85
    
    # Roboarm end effector has some offset error
    pointer_offset_x_mm = 18
    pointer_offset_y_mm = 0

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
            results = model.predict(image, conf=0.5)

            center_x = 1920 / 2
            center_y = 1080 / 2

            for result in results:
                for idx, box in enumerate(result.boxes):
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # Convert to integers
                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    cls_id = int(box.cls[0].item())
                    label = model.names[cls_id]
                    distance_y = (cy - center_y) / px_to_mm
                    distance_x = (cx - center_x) / px_to_mm

                    # Apply correction
                    distance_y = y_correction(distance_y)
                    distance_x = x_correction(distance_x)

                    if label == 'yellowcube':
                        anchor_x = distance_x
                        anchor_y = distance_y
                    print(f"Anchor XY: {anchor_x}, {anchor_y}")

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

                    cv2.circle(image, (int(cx), int(cy)), radius=3, color=(0, 0, 255), thickness=-1)  # Red dot
                    cv2.putText(image, str(idx), (int(cx), int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    cls_id = int(box.cls[0].item())
                    label = model.names[cls_id]
                    cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    distance_y = (cy - center_y) / px_to_mm
                    distance_x = (cx - center_x) / px_to_mm

                    # Apply correction
                    distance_y = y_correction(distance_y)
                    distance_x = x_correction(distance_x)

                    print(f"Idx: {idx}, Object: {label} X_mid: {cx} Y_mid: {cy}, X1:{x1},X2:{x2},Y1:{y1},Y2:{y2} Confidence score {conf}.\n")
                    print(f"    Distance from center in mm (x, y): {distance_x, distance_y}.\n")
                    print(f"    Roboarm (X,Y): {(-(distance_y - anchor_y) + yellow_origin_x_mm + pointer_offset_x_mm), (-(distance_x - anchor_x) + yellow_origin_y_mm + pointer_offset_y_mm) }\n")

            end_time = time.perf_counter()

            print(f"Time taken for inference: {end_time - start_time:.2f}")

            # Save and display the image with boxes
            cv2.imwrite("cube_detection.jpg", image)

            # Show the image in a window
            # TODO: Fix a crash when closing the image window
            cv2.resize(image, (1000, 500))
            cv2.namedWindow('Detection Results', cv2.WINDOW_GUI_EXPANDED)
            cv2.imshow('Detection Results', image)
            cv2.resizeWindow('Detection Results', 1000, 500)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            keep_running = False
            print("No image selected. Will exit.")