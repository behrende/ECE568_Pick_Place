import numpy as np
import cv2
import multifruit
import comms

#Single Image or Video Mode
mode = False 
#Socket Reciever Info 
IP = '127.0.0.1'
PORT = 65432

def main():
    model,cam = multifruit.init_cam()
    #client = comms.server_TCP()
    #Video
    if mode == True:  
        while True: 
            ret, frame = cam.read()
            
            #Show model Output - IE Understand what you just wrote here lol 
            out = multifruit.run_model(model,frame)
            annotate = out[0].plot()
            cv2.imshow('yolov8n cam', annotate)
            coordinates = multifruit.grab_xy(model,out)
            color = multifruit.grab_color(out, frame)

            # Check for incoming frames, if none then kill screen 
            if not ret:
                print("Can't receive frame. Exiting ...")
                break
            # Quit at input 'q'
            if cv2.waitKey(1) == ord('q'):
                print("Exiting...")
                break
    #Single Image
    else: 
        image_path = "/home/epbehren/yolo/ECE568_Pick_Place/src/test3.png"
        image  = cv2.imread(image_path)
        out = multifruit.run_model(model,image)
        annotate = out[0].plot()
        #cv2.imshow('yolov8n cam', annotate)
        cv2.waitKey(0)
        coordinates = multifruit.grab_xy(model,out)
        color = multifruit.grab_color(out, image)
        
        #Handle Packet
        assert len(coordinates) == len(color), "Functions must be the same size"
        packets = []
        for i, (x,y) in enumerate(coordinates):
            c = color[i]
            X = coordinates[0]["X"]
            Y = coordinates[0]["Y"]

            packets.append(comms.create_packet(c, X, Y))
        print(packets)

        #Send packet data
        #for packet in packets:
            #comms.send_tcp_data(client, packet)

    cam.release() 
    cv2.destroyAllWindows()
if __name__ == "__main__":
    main()
