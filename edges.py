import cv2  # OpenCV for image processing
import asyncio  # For asynchronous operations
import numpy as np  # Array processing
import random  # For random decisions

# Initialize the webcam
camera = cv2.VideoCapture(0)  # Use the default webcam (index 0)

# Function to capture a single frame from the webcam
def capture_frame():
    ret, frame = camera.read()  # Capture a frame
    if ret:
        return frame  # Return the captured frame
    else:
        raise RuntimeError("Failed to capture frame")

# Function to process the frame and extract visual information
def process_frame(frame):
    # Convert to grayscale for simple processing
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Example: Detect edges with Canny edge detection
    edges = cv2.Canny(gray_frame, 100, 200)
    
    # Example: Detect contours in the image
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    return contours  # Return the detected contours

# Asynchronous function for webcam-based operations
async def webcam_operations():
    print("Starting webcam-based operations")

    # Capture a frame from the webcam
    frame = capture_frame()

    # Process the frame to get visual information
    contours = process_frame(frame)

    # Decide robot action based on visual information
    if contours:
        # If contours are detected, decide a course of action
        largest_contour = max(contours, key=cv2.contourArea)  # Find the largest contour
        
        # Example logic: Follow the largest contour
        if cv2.contourArea(largest_contour) > 1000:  # If it's large enough
            print("Large object detected, following")
            await control_robot("forward")  # Move forward
        else:
            print("Small object detected, cautious approach")
            await control_robot("explore")  # Explore

    # Clean up and ensure the robot stops at the end
    await control_robot("stop")
