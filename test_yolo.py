import argparse
import cv2
from ultralytics import YOLO

parser = argparse.ArgumentParser(description="Process an image file path.")
parser.add_argument("--image_path", default="cube1.png", type=str, help="Path to the image file")

args = parser.parse_args()

model = YOLO("./models/colorcubes2.pt")
# results = model.predict(image_path)
# print(results)

while True:
    image_path = input("Enter the image path: ")
    # Load an image
    image = cv2.imread("./test_images/" + image_path)

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
            print(f"Object: {label} X: {cx} Y: {cy} Confidence score {conf}")        

    # Save and display the image with boxes
    cv2.imwrite("cube_detection.jpg", image)
    # cv2.imshow("Detection", image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # model.train(data="coco8.yaml", epochs=3)
    # metrics = model.val()
    # model.export(format="onnx")
    # results.pandas().xyxy
