from jds_utils.shell import call_and_print
from glob import glob
from os.path import join
import subprocess


#dirs = glob("/home/joschu/Data/tracking_results/robotjs_tie*")
#bagfiles = []
#for dir in dirs:
    #bagfiles.extend(glob(join(dir,"tracking*/*.bag")))


##call_and_print("test_eval_bag_error.py /home/joschu/Data/tracking_results/robotjs_tie_overhand_2012-09-16-00-12-52/tracking_nocolor/2012-09-17-16-15-49.bag --robot rope")

#dir1 = "/home/joschu/Data/tracking_results/tracker"
#bagfiles.extend(glob(join(dir1,"*.bag")))


dir = "/home/joschu/Data/Experiments"

def bag_cmp(bag1, bag2):
    task1,meth1 = bag1.split("2012")[0:2]
    task2,meth2 = bag2.split("2012")[0:2]
    if task1 == task2:
        if "color_1camera" in task1: return -1
        if "color_1camera" in task2: return 1
        if "nocolor_2camera" in task1: return -1
        if "nocolor_2camera" in task2: return 1
        return -1
    else:
        return cmp(task1, task2)

bags = sorted(glob(join(dir,"*.bag")),cmp=bag_cmp)
for bag in bags:
    if "tie" in bag or "knot" in bag: 
        if "robot" in bag: 
            call_and_print("python test_eval_bag_error2.py --redo_bag --reset_cache --robot --item=rope %s"%bag)
        else:
            continue
            call_and_print("python test_eval_bag_error2.py --item=rope %s"%bag)
        
    else:
        continue
        call_and_print("python test_eval_bag_error2.py --item=cloth %s"%bag)
    