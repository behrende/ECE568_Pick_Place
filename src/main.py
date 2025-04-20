import numpy as np
import cv2
import multifruit
import sortlogic
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
            data = multifruit.grab_block_info(model,out)
            #color = multifruit.grab_color(out, frame)
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
        box_info = multifruit.grab_block_info(model,out,image)
        #color = multifruit.grab_color(out, image)
        
        #Handle Packet
        #No longer need to assert lengths because it's in the same structure 
       # assert len(coordinates) == len(color), "Functions must be the same size"
        packets = []
        for i in range(len(box_info)):
            R = box_info[i]["R"]
            G = box_info[i]["G"]
            B = box_info[i]["B"]
            X = box_info[i]["X"]
            Y = box_info[i]["Y"]
            
            #packets.append(comms.create_packet(R,G,B,X,Y))
            packets.append({"R": R, "G": G, "B": B, "X": X, "Y": Y})
        #print(packets)
        #Sort_packet
        #Send packet data

        #color Sorted Packets
        hashed_packet = sortlogic.packet_hash(packets)
        #Color sorted packets with new coordinates 
        new_coords = sortlogic.color_sort(hashed_packet)
        
        print(f"Result: {new_coords}")

    cv2.destroyAllWindows()
if __name__ == "__main__":
    main()








#Sand box 


