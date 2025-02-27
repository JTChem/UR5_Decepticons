import sys
import os
import json
import math
import asyncio
import threading
import cv2
import numpy as np

# Add the directory containing robotiq_preamble.py to the Python search path
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, 'robotiq'))

from utils.UR_Functions import URfunctions as URControl
from robotiq.robotiq_gripper import RobotiqGripper
from camera import open, box, detect

# Constants
HOST = "192.168.0.2"
PORT = 30003
UPPER_BLUE = np.array([140, 255, 255])
LOWER_BLUE = np.array([100, 50, 50])
ITERATIONS = 4
MAX_RETRIES = 2

# Load positions from JSON
with open("positions.json", "r") as json_file:
    POSITIONS = json.load(json_file)

class RobotController:
    def __init__(self, host, port):
        self.robot = URControl(ip=host, port=port)
        self.gripper = RobotiqGripper()
        self.gripper.connect(host, 63352)

    async def move_to(self, pos):
        """Moves the robot to the specified position."""
        self.robot.move_joint_list(pos, 0.7, 0.5, 0.05)

    async def grab(self, close: bool):
        """Controls the gripper (True = close, False = open)."""
        self.gripper.move(255 if close else 0, 125, 125)

class CameraController:
    @staticmethod
    async def open_camera():
        """Opens the camera and processes frames."""
        await open()

    @staticmethod
    def detect_color(frame):
        """Detects if the specified color is present in the frame."""
        return detect(frame, LOWER_BLUE, UPPER_BLUE)


async def camera_ai() -> bool:
    ret, frame = cv2.VideoCapture(0).read()
    if not ret:
        print("Failed to grab frame from camera.")
        return False  # Return False if frame capture fails

    if detect(frame, LOWER_BLUE, UPPER_BLUE):
        print("Color change detected.")
        return True  # Color change detected

    print(f"No color change detected. ")
    return False  # 

async def process_workflow(rex, step):
    """Processes the workflow for a single vial."""
    position = step["coords"]
    await rex.move_to(position)
    if step["grab"] is not None:
        await rex.grab(step["grab"])
        

async def process_vial(rex, iteration):
    """Processes a single vial through the entire workflow."""
    retries = 0
    color_change = False

    while retries < MAX_RETRIES:
        for key, workflow in POSITIONS.items():
            if key == "loop":
                while retries < MAX_RETRIES and not color_change:
                    color_change = await camera_ai(retries)
                    if color_change:
                        await process_workflow(rex, workflow)
                    retries += 1
            else:
                await process_workflow(rex, workflow)

            # Retry if no color change detected
            if not color_change:
                print(f"No color change detected, retrying workflow for vial {iteration}")
                await rex.move_to(POSITIONS["stirer_intermediate"]["coords"])  # Go back to stirrer position
                await asyncio.sleep(5)  # Wait before retrying
                continue  # Retry the workflow

        retries += 1

    print("Workflow complete, moving to next vial.")

async def main():
    """Main execution function."""
    rex = RobotController(HOST, PORT)
    cam1 = threading.Thread(target=asyncio.run, args=(CameraController.open_camera(),))
    cam1.start()

    for i in range(ITERATIONS):
        await process_vial(rex, i)

    print("All vials processed. Exiting program.")

if __name__ == '__main__':
    asyncio.run(main())