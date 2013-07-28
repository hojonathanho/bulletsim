import cPickle
import numpy as np

perturb_scale = 1
runrange = (12000, 12063)

np.set_printoptions(precision=3)

dir  = "/media/0846DE7846DE65CA/suturing_runs/runs_%d_2_fixed_peg" % perturb_scale
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

for i in xrange(runrange[0], runrange[1]+1):
    ff = open(fformat%i, 'rb')
    irun_dict = cPickle.load(ff)

    try:
        max_warp_cost= max([seg['warp_costs'][0] + seg['warp_costs'][1] for seg in irun_dict['segments_info']])  
        res_dict[i]['hamm'] = np.count_nonzero(irun_dict['perturbations'])
        res_dict[i]['pert'] = irun_dict['perturbations']
        res_dict[i]['max_warp'] = max_warp_cost
    except KeyError:
        pass
    

fail_dict = {}
for k in res_dict.keys():
    if not res_dict[k]['res']:
        fail_dict[k] = res_dict[k]

import pprint
pp = pprint.PrettyPrinter(depth=6)
pprint.pprint(fail_dict, open(interp_fname, 'w'))
