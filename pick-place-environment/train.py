from ultralytics import YOLO

USE_GPU = False
# Load a pre-trained model yolov8l.pt
model = YOLO("yolov8n.pt")

# Train the pre-trained model on dataset
if USE_GPU:
    model.train(data="data.yaml", epochs=50, imgsz=640, device=0)
else:
    model.train(data="data.yaml", epochs=50, imgsz=640)