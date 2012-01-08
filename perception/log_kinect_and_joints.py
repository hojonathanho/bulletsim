import subprocess
import signal
from time import sleep

DATA_ROOT = "/home/joschu/Data/comm_pr2_knot"
CLOUD_TOPIC = "kinect"
JOINT_TOPIC = "joints"

p1 = subprocess.Popen(["/home/joschu/python/comm/publish_ros_topic.py","/joint_states","--out",JOINT_TOPIC])
p2 = subprocess.Popen(["/home/joschu/usr/bin/publish_pcd_files","-t","kinect"])


try:
    sleep(10**10)
except KeyboardInterrupt:
    p1.send_signal(signal.SIGINT)
    p2.send_signal(signal.SIGINT)

