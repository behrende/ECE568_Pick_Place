import cv2
from ultralytics import YOLO

image_path = "img4.jpg"
model = YOLO("./models/yolo_cubes.pt")
# results = model.predict(image_path)
# print(results)

# Load an image
image = cv2.imread(image_path)

# Perform inference
results = model(image)

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
        print("Object: " + str(label) + " X: " + str(cx) + " Y: " + str(cy))        

# Save and display the image with boxes
cv2.imwrite("cube_detection.jpg", image)
# cv2.imshow("Detection", image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# model.train(data="coco8.yaml", epochs=3)
# metrics = model.val()
# model.export(format="onnx")
# results.pandas().xyxy
