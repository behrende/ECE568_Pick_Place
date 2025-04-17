import cv2
import time
from ultralytics import YOLO
import tkinter as tk
from tkinter import filedialog

# Step 1: Open a file picker dialog
root = tk.Tk()
root.withdraw()  # Hide the main tkinter window

model = YOLO("runs/detect/train29/weights/best.pt")

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

        # Get bounding boxes and coordinates
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Convert to integers
                conf = box.conf[0].item()  # Confidence score
                cls = int(box.cls[0].item())  # Class ID

                # Draw bounding box
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                cls_id = int(box.cls[0].item())
                label = model.names[cls_id]
                cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                print(f"Object: {label} X_mid: {cx} Y_mid: {cy}, X1:{x1},X2:{x2},Y1:{y1},Y2:{y2} Confidence score {conf}")

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