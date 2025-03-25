from ultralytics import YOLO
import cv2

#Init models and camera
model = YOLO("yolov8n.pt")
#fourcc = cv2.VideoWriter_fourcc(*'mp4v') #USE IF YOU WANT TO SAVE VIDEO#

cam = cv2.VideoCapture(-1)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)


def run_model(frame): 
    results = model(frame)
    return results

#Display Coordinates
def show_xy(out):
    for box in out[0].boxes:
        x1,y1,x2,y2 = box.xyxy[0].tolist()
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        cls_id = int(box.cls[0].item())
        label = model.names[cls_id]
        print("Object: " + str(label) + " X: " + str(cx) + " Y: " + str(cy))
#Calculate delta for classification algorithm
#def calculate_xy_delta(out):


def main():

    #OpenCV Logic 
    while True: 
        ret,frame = cam.read()
        
        #Show model Output - IE Understand what you just wrote here lol 
        out = run_model(frame)
        annotate = out[0].plot()
        cv2.imshow('yolov8n cam',annotate)
        show_xy(out)

        # Check for incoming frames, if none then kill screen 
        if not ret:
            print("Can't receive frame. Exiting ...")
            break
        # Quit at input 'q'
        if cv2.waitKey(1) == ord('q'):
            print("Exiting...")
            break
        #Save Output to help capture xy - Not needed for our current purposes. 
        #video_writer = cv2.VideoWriter('output.mp4', fourcc, 20.0, (640, 480))
       
    cam.release() 
    cv.destroyAllWindows()

main()