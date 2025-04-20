#Python Sort Logic for the MicroProcessor
import cv2
import numpy as np
import json

#Sandbox for Incoming Sort Logic 

color_ranges = {
    "Red":    [(0, 100, 100), (10, 255, 255)],      # Lower red
    "Red2":   [(160, 100, 100), (179, 255, 255)],   # Upper red (wraps around)
    "Green":  [(40, 50, 50), (80, 255, 255)],
    "Blue":   [(100, 100, 100), (130, 255, 255)],
    "Yellow": [(20, 100, 100), (30, 255, 255)],
    "Orange": [(10, 100, 100), (20, 255, 255)]
}


#Hash Function 
def packet_hash(packets):
    global color_ranges
    color_hash = {color: [] for color in color_ranges}

    for packet in packets:
        print(packet)
        current = (packet["R"], packet["G"], packet["B"])
        hsv = cv2.cvtColor(np.uint8([[current]]), cv2.COLOR_RGB2HSV)[0][0]
        # Check if the color is within the defined ranges
        matched = False
        for color, (low,high) in color_ranges.items(): 
            if all(high[i] >= hsv[i] >= low[i] for i in range(3)):
                color_hash[color].append(current) 
                matched = True 
                break 
        if not matched: 
            print("Unrecognized color: {current}")
    return color_hash



#Create new XY Coords based on coordinate defintions. 
def color_sort(hashed_packet): 
        #XY Length in mm hardwaire later 
    new_hash = hashed_packet
    # Define the coordinates for the Blocks
    xpos,xneg = 200,-200 
    ypos,yneg = 600, 0
    # Define the spacing between Blocks
    xspace = 40 
    yspace = 50
        #Create board coordinates
    secleny = ypos / len(hashed_packet)
    ystart = ypos
        #Create coordinates 
    color_order = ["Red", "Orange", "Yellow", "Green", "Blue"]
    for color in color_order:
        if color in hashed_packet:
            for i, packet in enumerate(hashed_packet[color]):
                # Assign new coordinates
                if i % 2 == 0:  # Even index
                    packet["X"] = xpos
                    packet["Y"] = ystart - (i // 2) * yspace
                else:  # Odd index
                    packet["X"] = xneg
                    packet["Y"] = ystart - ((i - 1) // 2) * yspace

                # Update the new_hash with the modified packet
                new_hash[color][i] = packet
    return new_hash