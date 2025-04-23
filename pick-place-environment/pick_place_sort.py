import cv2
import time
from ultralytics import YOLO
import tkinter as tk
from tkinter import filedialog
import requests
import json
import time
from datetime import datetime
import numpy as np
import subprocess
import math

# Step 1: Open a file picker dialog
root = tk.Tk()
root.withdraw()  # Hide the main tkinter window

model = YOLO("runs/detect/train29/weights/best.pt")

CAM_PORT = 1
PIXEL_TO_MM = 2.618
GRIPPER_OPEN_ANGLE = 1.62
GRIPPER_CLOSED_ANGLE = 3.24


CAMERA_TO_ARM_BASE = 45.7 # mm

cubes_labels = ['bluecube', 'greencube', 'orangecube', 'purplecube', 'redcube', 'yellowcube']

class PickPlaceRobot:
    def __init__(self, ip="192.168.4.1"):
        self.robot_ip = ip
        self.arm_hotspot_name = "RosArm-M2"
        self.arm_hotspot_pass = "12345678"

        # Initialze Arm with the following intial pose.
        self.arm_x_home_cord = 20
        self.arm_y_home_cord = 0
        self.arm_z_home_cord = 234
        self.arm_gripper_home_angle = GRIPPER_CLOSED_ANGLE

        self.arm_x_pt = self.arm_x_home_cord
        self.arm_y_pt = self.arm_y_home_cord
        self.arm_z_pt = self.arm_z_home_cord
        self.arm_gripper_angle = self.arm_gripper_home_angle

    def img_x_correction(self, x_value):
        return 1.021 * x_value - 1.0

    def img_y_correction(self, y_value):
        return 0.96 * y_value + 5.28
    
    def reset_arm_postion(self):
        self.send_command(self.arm_x_home_cord, self.arm_y_home_cord, self.arm_z_home_cord, self.arm_gripper_home_angle)

    def open_gripper(self):
        self.arm_gripper_angle = GRIPPER_OPEN_ANGLE
        self.send_command(self.arm_x_pt, self.arm_y_pt, self.arm_z_pt, self.arm_gripper_angle)

    def close_gripper(self):
        self.arm_gripper_angle = GRIPPER_CLOSED_ANGLE
        self.send_command(self.arm_x_pt, self.arm_y_pt, self.arm_z_pt, self.arm_gripper_angle)

    def move_arm(self, x, y, z):
        self.arm_x_pt = x
        self.arm_y_pt = y
        self.arm_z_pt = z

        self.send_command(self.arm_x_pt, self.arm_y_pt, self.arm_z_pt, self.arm_gripper_angle)

    def send_command(self, x, y, z, t, T=1041):
        """
        Send a JSON command to the ESP32 via HTTP to move the robotic arm.
        """
        command_dict = {"T": T, "x": x, "y": y, "z": z, "t": t}
        url = f"http://{self.robot_ip}/js"

        try:
            response = requests.post(url, json=command_dict, timeout=5)
            if response.status_code == 200:
                print(f"Command sent: {json.dumps(command_dict)}")
                print(f"Response: {response.text}")
            else:
                print(f"Failed to send command. HTTP {response.status_code}: {response.text}")
        except requests.exceptions.Timeout:
            print("Request timed out. Check your ESP32 connection.")
        except requests.exceptions.RequestException as e:
            print(f"Network error: {e}")

    def capture_image(self):
        # initialize the camera 
        cam = cv2.VideoCapture(CAM_PORT) 

        # reading the input using the camera 
        result, image = cam.read() 

        # If image will detected without any error, show result 
        if result:
            # Show captured image.
            cv2.imshow("Captured Image", image) 

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            # Save Image
            cv2.imwrite(f"CapturedImage_{timestamp}.png", image) 

            # If keyboard interrupt occurs, destroy image window 
            cv2.waitKey(0) 
            cv2.destroyWindow("Captured Image")

            return image
        # If captured image is corrupted, moving to else part 
        else: 
            raise Exception("No image detected. Please! try again")

    def detect_cubes_w_anchor(self, image):
        objs_dist = []
        image = cv2.imread('./test1.jpg')

        # cube height
        cube_pickup_height = -120

        # Yellow cube (or any origin cube) location in Roboarm coordinates
        yellow_origin_x_mm = 8
        yellow_origin_y_mm = -85
        
        # Roboarm end effector has some offset error
        pointer_offset_x_mm = 18
        pointer_offset_y_mm = 0

        # Perform inference
        results = model.predict(image, conf=0.2)

        center_x = 1920 / 2
        center_y = 1080 / 2

        for result in results:
            for idx, box in enumerate(result.boxes):
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Convert to integers
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                conf = box.conf[0].item()  # Confidence score
                cls_id = int(box.cls[0].item())
                label = model.names[cls_id]
                distance_y = (cy - center_y) / PIXEL_TO_MM
                distance_x = (cx - center_x) / PIXEL_TO_MM

                # Apply correction
                distance_y = self.img_y_correction(distance_y)
                distance_x = self.img_x_correction(distance_x)

                print(f"Idx: {idx}, Object: {label} X_mid: {cx} Y_mid: {cy}, X1:{x1},X2:{x2},Y1:{y1},Y2:{y2} Confidence score {conf}.\n")

                if label == 'yellowcube':
                    anchor_x = distance_x
                    anchor_y = distance_y
                    print(f"Anchor XY: {anchor_x}, {anchor_y}")

        # Get bounding boxes and coordinates
        for result in results:
            for idx, box in enumerate(result.boxes):
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Convert to integers
                conf = box.conf[0].item()  # Confidence score
                cls = int(box.cls[0].item())  # Class ID

                # Draw bounding box
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2

                cv2.circle(image, (int(cx), int(cy)), radius=3, color=(0, 0, 255), thickness=-1)  # Red dot
                cv2.putText(image, str(idx), (int(cx), int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                cls_id = int(box.cls[0].item())
                label = model.names[cls_id]
                cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                distance_y = (cy - center_y) / PIXEL_TO_MM
                distance_x = (cx - center_x) / PIXEL_TO_MM

                # Apply correction
                distance_y = self.img_y_correction(distance_y)
                distance_x = self.img_x_correction(distance_x)

                robo_x = (-(distance_y - anchor_y) + yellow_origin_x_mm + pointer_offset_x_mm)
                robo_y = (-(distance_x - anchor_x) + yellow_origin_y_mm + pointer_offset_y_mm)

                print(f"Idx: {idx}, Object: {label} X_mid: {cx} Y_mid: {cy}, X1:{x1},X2:{x2},Y1:{y1},Y2:{y2} Confidence score {conf}.\n")
                print(f"    Distance from center in mm (x, y): {distance_x, distance_y}.\n")
                print(f"    Roboarm (X,Y): {robo_x, robo_y}\n")

                if label == 'redcube':
                    objs_dist.append((robo_x, robo_y, cube_pickup_height,label))
        
        # Save and display the image with boxes
        cv2.imwrite("cube_detection.jpg", image)

        # Show the image in a window
        # TODO: Fix a crash when closing the image window
        # cv2.resize(image, (1000, 500))
        cv2.namedWindow('Detection Results', cv2.WINDOW_GUI_EXPANDED)
        cv2.imshow('Detection Results', image)
        # cv2.resizeWindow('Detection Results', 1000, 500)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return objs_dist

    def detect_cubes(self, image):
        # Perform inference
        results = model.predict(image, conf=0.1)

        objs_dist = []

        # Get bounding boxes and coordinates
        for result in results:
            for idx, box in enumerate(result.boxes):
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Convert to integers
                conf = box.conf[0].item()  # Confidence score
                cls = int(box.cls[0].item())  # Class ID

                # Draw bounding box
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2

                cv2.putText(image, str(idx), (int(cx), int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                cls_id = int(box.cls[0].item())
                label = model.names[cls_id]
                cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                distance_y = cy / PIXEL_TO_MM
                distance_x = cx / PIXEL_TO_MM

                print(f"Idx: {idx}, Object: {label} X_mid: {cx} Y_mid: {cy}, X1:{x1},X2:{x2},Y1:{y1},Y2:{y2} Confidence score {conf}. \
                      Distance from center in mm (x, y): {distance_x, distance_y}")
                
                objs_dist.append((distance_x, distance_y))
        
        return objs_dist
                
    def transform_img_to_real_world(self):
        pass

    def image_to_arm_base(x_img, y_img, height_mm=47.5):
        """
        Transforms image coordinates (camera frame) to robot base frame.
        
        Parameters:
        - x_img: float - X in image (pixels or mm, depending on calibration)
        - y_img: float - Y in image
        - height_mm: float - height of the camera above the base in mm (default: 47.5 mm)
        
        Returns:
        - np.array([x_base, y_base, z_base]) - Coordinates in robot base frame
        """
        # Rotation matrix from camera frame to base frame
        R = np.array([
            [ 0,  1,  0],
            [-1,  0,  0],
            [ 0,  0, -1]
        ])

        # Point in camera frame (Z is 0 since it's a 2D image coordinate)
        cam_point = np.array([x_img, y_img, 0])

        # Apply rotation
        base_point = R @ cam_point

        # Add translation (camera is 47.5 mm above base, so Z in base = -47.5 mm)
        base_point[2] = -height_mm

        return base_point
    
    def image_to_base(self, x_img, y_img):
        pass

    def pick_object(self,x,y,z):
        # x_img, y_img = objs[0]
        # x, y, z = self.image_to_arm_base(x_img, y_img)

        self.open_gripper()
        time.sleep(5)
        self.move_arm(x, y, z)
        time.sleep(5)
        self.close_gripper()
        time.sleep(5)
        self.reset_arm_postion()



    #Simple function to place object in new position. 
    def place_object(self,x,y,z):


        self.move_arm(x, y, z)
        time.sleep(1)
        self.open_gripper()
        time.sleep(1)
        self.reset_arm_postion()
        self.close_gripper()
        time.sleep(1)



    #Sort multiple objects by distance from anchor for pick and place  
    def sort_cubes(self): 

        # cube height
        cube_pickup_height = -120

        # Yellow cube (or any origin cube) location in Roboarm coordinates
        anchor_origin_x_mm = 8
        anchor_origin_y_mm = -85
        

        #Sets initial Arm Position 
        self.reset_arm_postion()
        image = self.capture_image()

        # objs = self.detect_cubes(image)
        objs = self.detect_cubes_w_anchor(image)


        #Determine sort starting positions by color from anchor. 
        # color_start = []
        # for i,color in enumerate(cubes_labels):
        #     x = anchor_origin_x_mm 
        #     y = anchor_origin_y_mm + (i * 10)
        #     z = cube_pickup_height
        #     color_start.append((x, y, z, color))
        
        #color Map
        color_positions = {
            color: [anchor_origin_x_mm, anchor_origin_y_mm + (i * 10), cube_pickup_height]
            for i, color in enumerate(cubes_labels)
        }

        #Determine Sort Positions based on distance from Anchor 
        obj_dist = []
    
        for current in objs:
            x, y, z, label = current
            # Calculate distance from anchor
            dist = math.sqrt((x - anchor_origin_x_mm)**2 + (y - anchor_origin_y_mm)**2)
            obj_dist.append() (x, y, z, label,dist)

        # Sort objects by distance from anchor
        sorted_obj = sorted(obj_dist, key=lambda dist: dist[4])


        #Automation for sorted objects. 
        #obj_new_xy = [[] for x,y,z,label,dist in objs]
        
        # for i, current in enumerate(sorted_obj):
        #     x, y, z, label, dist = current
        #     # Find the corresponding color start position
        #     for j, (color_x, color_y, color_z, color) in enumerate(color_start):
        #         if label == color:
        #             #Move Object
        #             self.pick_object(x, y, z)
        #             self.place_object(color_x, color_y, color_z)
        #             time.sleep(1)
        #             #update color start position for next object
        #             color_start[j] = (color_x, color_y + 10, color_z, color)
        #             break

        #Color Map Update
        for x, y, z, label in sorted_obj:
            # Find the corresponding color start position
            color_x, color_y, color_z = color_positions[label]
            # Move Object
            self.pick_object(x, y, z)
            self.place_object(color_x, color_y, color_z)
            time.sleep(1)
            # Update color position for next object
            color_positions[label][1] += 10





        
    def connect_arm(self):
        """Connects to the Robot Arm Wi-Fi hotspot."""
        try:
            command = f'netsh wlan connect name="{self.arm_hotspot_name}" ssid="{self.arm_hotspot_name}"'
            if self.arm_hotspot_pass:
                command += f' keyMaterial="{self.arm_hotspot_pass}"'
            
            process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = process.communicate()

            if process.returncode == 0:
                print(f"Successfully connected to {self.arm_hotspot_name}")
                return True
            else:
                print(f"Failed to connect to {self.arm_hotspot_name}: {stderr.decode()}")
                return False
        except Exception as e:
            print(f"An error occurred: {e}")
            return False

if __name__ == "__main__":
    arm = PickPlaceRobot("192.168.4.1")

    # arm.capture_image()
    arm.sort_cubes