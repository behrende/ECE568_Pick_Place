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

color_hash = {
    "Green": None,
    "Blue": None,
    "Red": None,
    "Yellow": None,
    "Orange": None
}





#Hash Function 
def packet_hash(packets, color_hash):
    for packet in packets:
        current = (packet["R"],packet["G"],packet["B"])
        for color in color_ranges: 
            if current > color[0] && current < color[1]
                color_hash[color].append(current) 