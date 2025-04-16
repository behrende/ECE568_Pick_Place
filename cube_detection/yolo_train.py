import os
from ultralytics import YOLO

os.environ['CUDA_LAUNCH_BLOCKING'] = '1'
os.environ['TORCH_USE_CUDA_DSA'] = '1'

if __name__=='__main__':
    # Load a model
    model = YOLO("yolo11s.pt")  # load a pretrained model (recommended for training)
    model.to('cuda')

    # Train the model
    results = model.train(data="./colorcubes_v6i/data.yaml", 
                          lr0=0.01,
                          lrf=0.01,
                          epochs=300, 
                          imgsz=640, 
                          batch=0.9,
                          plots=True,
                          optimizer='SGD')

    # Save the trained model
    model.save("../models/colorcubes6.pt")  # Save the model to a file