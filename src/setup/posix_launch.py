import subprocess
import rospy
import sys
import os
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
import roslaunch

launchfile = "posix_sitl.launch"
fullpath = os.path.join("/root/src/Firmware/launch/", launchfile)
subprocess.Popen(["roslaunch",fullpath])
