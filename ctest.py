import cv2

# Initialize the camera
camera = cv2.VideoCapture(0)  # Assuming the first camera

try:
    while True:
        ret, frame = camera.read()
        if ret:
            cv2.imshow("Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit the loop
                break
        else:
            print("Failed to capture frame")
finally:
    camera.release()
    cv2.destroyAllWindows()
