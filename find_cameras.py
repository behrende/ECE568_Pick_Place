import cv2

for i in range(5):  # Check up to 5 camera indices
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"Camera found at index {i}")
        cap.release()