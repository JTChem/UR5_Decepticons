import cv2
import numpy as np
import pandas as pd
import warnings
import time

warnings.filterwarnings("ignore")

# Colour Ranges (You can change this to match your solution colour)
lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([30, 255, 255])
lower_blue = np.array([100, 100, 50])
upper_blue = np.array([130, 255, 255])

"""Counts Pixels"""
def colour_pixels(mask):
    return np.sum(mask > 0)

"""Crop Only the Vial Area"""
def crop_vial(image):
    height, width, _ = image.shape
    return image[int(height * 0.3):int(height * 0.8), int(width * 0.4):int(width * 0.6)]

"""Colour count Function"""
def colour_change_detector():
    camera = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FPS, 30)

    print("[INFO] Taking Before Image ...")
    ret, before = camera.read()
    if not ret:
        print("Camera Error: Before Image Not Captured")
        return
    before = crop_vial(before)
    cv2.imwrite("before.jpg", before)

    print("[INFO] Waiting for Hotplate to Stir the Solution ...")
    time.sleep(30)  # <-- This is where you're giving the hotplate 30 seconds to stir

    print("[INFO] Taking After Image ...")
    ret, after = camera.read()
    if not ret:
        print("Camera Error: After Image Not Captured")
        return
    after = crop_vial(after)
    cv2.imwrite("after.jpg", after)

    # Convert to HSV for Colour Detection
    before_hsv = cv2.cvtColor(before, cv2.COLOR_BGR2HSV)
    after_hsv = cv2.cvtColor(after, cv2.COLOR_BGR2HSV)

    # Blue Colour Detection
    before_blue_mask = cv2.inRange(before_hsv, lower_blue, upper_blue)
    after_blue_mask = cv2.inRange(after_hsv, lower_blue, upper_blue)

    # Count Blue Pixels
    before_pixels = colour_pixels(before_blue_mask)
    after_pixels = colour_pixels(after_blue_mask)

    # Colour Change Percentage
    if before_pixels == 0:
        change_percentage = 100
    else:
        change_percentage = ((after_pixels - before_pixels) / before_pixels) * 100

    print(f"Colour Change Percentage: {round(change_percentage, 2)}%")

    if after_pixels > before_pixels:
        print("Colour Change Detected")
        result = "YES"
    else:
        print("No Colour Change")
        result = "NO"

    # Save Results to CSV
    data = {
        "Before Pixels": [before_pixels],
        "After Pixels": [after_pixels],
        "Colour Change %": [round(change_percentage, 2)],
        "Result": [result],
    }
    df = pd.DataFrame(data)
    df.to_csv("results.csv", index=False, mode="a", header=not pd.io.common.file_exists("results.csv"))

    print("[INFO] Data Saved to results.csv ")

    camera.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    colour_change_detector()
