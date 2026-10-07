import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.robot import RobotController

def main():
    robot = RobotController(config_dir="config")
    robot.go_home(speed=0.5, acc=0.5)

if __name__ == "__main__":
    main()
