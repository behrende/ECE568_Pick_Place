#Python Sort Logic for the MicroProcessor
import cv2
import numpy as np
import json

#Sandbox for Incoming Sort Logic 

color_ranges = {
    "Red":    [(0, 100, 100), (10, 255, 255)],      # Lower red
    "Red2":   [(160, 100, 100), (179, 255, 255)],   # Upper red (wraps around)
    "Green":  [(36, 50, 50), (90, 255, 255)],       # Expanded green range
    "Blue":   [(91, 100, 100), (130, 255, 255)],    # Blue
    "Yellow": [(20, 50, 50), (35, 255, 255)],       # Expanded yellow range
    "Orange": [(10, 100, 100), (20, 255, 255)]      # Orange
}


#Hash Function 
def packet_hash(packets):
    global color_ranges
    color_hash = {color: [] for color in color_ranges}

    for packet in packets:
        #print(packet)
        current = (packet["R"], packet["G"], packet["B"])
        # Convert RGB to HSV
        hsv = cv2.cvtColor(np.uint8([[current]]), cv2.COLOR_RGB2HSV)[0][0]
        # Check if the color is within the defined ranges
        matched = False
        for color, (low,high) in color_ranges.items(): 
            if all(high[i] >= hsv[i] >= low[i] for i in range(3)):
                color_hash[color].append({
                    "R" : packet["R"],
                    "G": packet["G"],
                    "B": packet["B"], 
                    "X": packet["X"], 
                    "Y": packet["Y"]}) 
                matched = True 
                break 
        if not matched: 
            print(f"Unrecognized color: {current}")
    return color_hash



#Create new XY Coords based on coordinate defintions. 
def color_sort(hashed_packet): 
        #XY Length in mm hardwaire later 
    end_hash = []
    # Define the coordinates for the Blocks
    xpos,xneg = 200,-200 
    ypos,yneg = 600, 0
    # Define the spacing between Blocks
    xspace = 40 
    yspace = 50
        #Create board coordinates
    #secleny = ypos / len(hashed_packet)
    ystart = ypos
        #Create coordinates 
    color_order = ["Red","Red2", "Green", "Blue", "Yellow", "Orange"]
    new_hash = {color: [] for color in color_order}
    coordinates = {}
    for color in color_order:
        if color in hashed_packet:
            for i, value in enumerate(hashed_packet[color]): 
                new_hash[color].append(
                    {"R ": value["R"], 
                     "G" : value["G"], 
                     "B ": value["B"],
                     "X": xstart, 
                     "Y": ystart})
                
                xstart += xspace
        xstart = xneg
        ystart -= yspace

    return new_hash