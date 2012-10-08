import subprocess, os
import os.path as osp
from glob import glob
bs_path = osp.abspath(__file__).split("bulletsim_python")[0]
gph_path = osp.join(osp.dirname(osp.abspath(__file__)), "generate_ptr_header.py")
os.chdir(osp.join(bs_path,"src"))
for dir in ["simulation","robots","clouds","sqp","tracking"]:
    ptr_header = osp.join(dir, "%s_fwd.h"%dir)
    headers = glob(osp.join(dir, "*.h"))
    if ptr_header in headers: headers.remove(ptr_header)
    print headers
    cmd = " ".join(["python", gph_path] + headers + [">", osp.join(dir, "%s_fwd.h"%dir)])
    subprocess.check_call(cmd,shell=True)
    #print cmd