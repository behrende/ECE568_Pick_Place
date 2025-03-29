import os
from ultralytics import YOLO

# os.environ['CUDA_LAUNCH_BLOCKING'] = '1'
# os.environ['TORCH_USE_CUDA_DSA'] = '1'

if __name__=='__main__':
    # Load a model
    model = YOLO("../yolo11n.pt")  # load a pretrained model (recommended for training)
    model.to('cpu')

    # Train the model
    results = model.train(data="./data.yaml", epochs=100, imgsz=640, batch=0.9)

    # Save the trained model
    model.save("../models/non_cuda_yolo_cubes.pt")  # Save the model to a file