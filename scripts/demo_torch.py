import os
import sys
import math

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.robot import RobotController
from motion.pose_generators import torch_angle_from_line

OUTPUT_DIR = "data_gocator"
FOV_WIDTH = 0.12
#WELDING_LINE = ([0.41808, 0.11279, 0.047219], [0.11817, 0.15639, 0.04653])
WELDING_LINE = ([0.11817, 0.15639, 0.04653], [0.41808, 0.11279, 0.047219])
#WELDING_LINE = ([0.0, 0.0, 0.0], [0.51, 0.31, 0.0])
#WELDING_LINE = ([0.51, 0.31, 0.0], [0.0, 0.0, 0.0])

def main():
    
    print("TORCH SEQUENCE")

    # Init robot and camera
    robot = RobotController(config_dir="config")

    rz_angle = torch_angle_from_line(WELDING_LINE[0], WELDING_LINE[1])
    
    # Move to the first pose (approach position) to prevent collision
    robot.set_active_tcp("torch")
    robot.go_to_table_center(z=0.6)
    robot.go_to_table_center(z=0.6, rz=rz_angle)

    robot.move_to_pose(
                x=WELDING_LINE[0][0] + 0.1, 
                y=WELDING_LINE[0][1] + 0.1, 
                z=WELDING_LINE[0][2] + 0.1,
                rx=0.0,
                ry=0.0, 
                rz=rz_angle,
                frame="table"
            )

    robot.move_to_pose(
                x=WELDING_LINE[0][0], 
                y=WELDING_LINE[0][1], 
                z=WELDING_LINE[0][2],
                rx=-math.pi/4,
                ry=0.0, 
                rz=rz_angle,
                frame="table"
            )
    
    robot.move_to_pose(
                x=WELDING_LINE[1][0], 
                y=WELDING_LINE[1][1], 
                z=WELDING_LINE[1][2],
                rx=-math.pi/4,
                ry=0.0, 
                rz=rz_angle,
                frame="table",
                speed=0.02,
                acc=0.02
            )
    
    print("Sequence completed.")


if __name__ == "__main__":
    main()