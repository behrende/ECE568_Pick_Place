from ultralytics import YOLO
import cv2


def init_cam():
    #Init models and camera
    model = YOLO("yolov8n.pt")
    #Init Open CV Cam
    cam = cv2.VideoCapture(-1)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
    return model,cam


def run_model(model,frame): 
    results = model(frame)
    return results

#Display Coordinates
def grab_block_info(model,out,image):
    box_info = [] 
    for box in out[0].boxes:
        x1,y1,x2,y2 = box.xyxy[0].tolist()
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        cls_id = int(box.cls[0].item())
        label = model.names[cls_id]
        r,g,b = image[int(cy),int(cx)]
        print("Object: " + str(label) + " X: " + str(cx) + " Y: " + str(cy))
        data =  {"R" : r, "G": g, "B": b,"X": int(cx),"Y": int(cy) } 
        print(data)
        box_info.append(data)

    return box_info



