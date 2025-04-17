#Python Sort Logic for the MicroProcessor



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
        current = (packet["R"],packet["G"],packet["B"])
        matched = False
        for color, (low,high) in color_ranges.items(): 
            if all(high[i] >= current[i] >= low[i] for i in range(3)):
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
    xpos,xneg = 200,-200 
    ypos,yneg = 600, 0
    xspace = 40 
    yspace = 50
        #Create board coordinates
    secleny = ypos / size(hashed_packet)
    ystart = ypos
        #Create coordinates 
    color_order = ["Red", "Orange", "Yellow", "Green", "Blue"]

    for color_index, color in enumerate(color_order): 
        packets = new_hash[color]
        xstart = xneg
        for i, val in enumerate(packet): 

             # Assign new XY coordinates
            packet["X"] = x_start + i * x_spacing
            packet["Y"] = y_start - color_index * y_spacing
    return new_hash