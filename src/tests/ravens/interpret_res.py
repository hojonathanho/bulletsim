import cPickle
import numpy as np

perturb_scale = 1
runrange = (1200, 1263)


dir  = "/media/0846DE7846DE65CA/suturing_runs/runs_%d_2" % perturb_scale
resf = dir  + "/results_%d_2.txt" % perturb_scale
fformat = dir + "/run%d-costs.txt"
interp_fname = dir + "/interp_%d_2.txt" % perturb_scale

res_dict = {}

resf = open(resf, 'r')
for line in resf:
    ss = line.split()
    assert len(ss) >= 2
    runnum = int(ss[0])
    res    = bool(int(ss[1]))

    res_dict[runnum]  = {'res':res}
    if res==False:
        res_dict[runnum]['comm'] = " ".join(ss[2:])
    else:
        res_dict[runnum]['comm'] = ""

for i in xrange(runrange[0], runrange[1]):
    ff = open(fformat%i, 'rb')
    irun_dict = cPickle.load(ff)
    

    max_warp_cost= max([seg['warp_costs'][0] + seg['warp_costs'][1] for seg in irun_dict['segments_info']])  
    res_dict[i]['hamm'] = np.count_nonzero(irun_dict['perturbations'])
    res_dict[i]['pert'] = irun_dict['perturbations']
    res_dict[i]['max_warp'] = max_warp_cost
    
   
#import pprint
#pp = pprint.PrettyPrinter(depth=6)
#pp.pprint(res_dict)

import yaml
interpf = open(interp_fname, 'w')
interpf.write(yaml.dump(res_dict, default_flow_style=False))
interpf.close()
