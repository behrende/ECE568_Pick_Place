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
def grab_xy(model,out):
    box_info = [] 
    for box in out[0].boxes:
        x1,y1,x2,y2 = box.xyxy[0].tolist()
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        cls_id = int(box.cls[0].item())
        label = model.names[cls_id]
        print("Object: " + str(label) + " X: " + str(cx) + " Y: " + str(cy))
        data_xy =  {"X": cx,"Y": cy } 
        box_info.append(data_xy)

    return box_info


#Grab Color from Bounded Block and Return RGB
def grab_color(out, image):
    color_list = []  
    for box in out[0].boxes:
        # Assuming box.xyxy gives [x1, y1, x2, y2] as floats or tensors
        x1, y1, x2, y2 = map(int, box.xyxy[0])  # Convert to integers
        # Crop region from the image
        region = image[y1:y2, x1:x2]
        # Compute average color (BGR by default if using OpenCV)
        if region.size > 0:
            mean_color = region.mean(axis=(0, 1))  # [B, G, R] or [R, G, B]
            color_list.append(tuple(map(int, mean_color)))
    
    return color_list
