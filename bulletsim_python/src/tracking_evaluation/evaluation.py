from __future__ import division
import numpy as np
norm = np.linalg.norm
import scipy.spatial.distance as ssd

def normalize(x):
    return x / np.sqrt((x**2).sum())

def interp2d(x,xp,yp):
    yp = np.asarray(yp)
    assert yp.ndim == 2
    return np.array([np.interp(x,xp,col) for col in yp.T]).T

def interpnd(newx, oldx, oldy):
    oldy2d = oldy.reshape(oldy.shape[0], -1)
    newy2d = interp2d(newx, oldx, oldy2d)
    return newy2d.reshape((len(newx),) + oldy[0].shape)
    
def norms(x,ax):
    return np.sqrt((x**2).sum(axis=ax))    
    

def apply_trans(hmat, vec):
    return hmat[:3, :3].dot(vec) + hmat[:3,3]
def apply_inv_trans(hmat, vec):
    return np.linalg.solve(hmat[:3,:3], vec - hmat[:3,3])


def get_tri_transform(pt0, pt1, pt2):
    """
    origin is at pt0
    x axis is parallel to pt1-pt0
    y axis is in plane of (pt0, pt1, pt2) orthogonal to x
    z = x cross y
    ideally we'd use barycentric coordinates to be more tolerant to large deformations
    """
    
    x = normalize(pt1 - pt0)
    z = normalize(np.cross(x, pt2-pt0))
    y = np.cross(z,x)
    
    out = np.eye(4)
    out[:3,0] = x
    out[:3,1] = y
    out[:3,2] = z
    out[:3,3] = pt0
    
    return out
    
def calculate_cloth_tracking_score1(tracker_xyz_sn3, tracker_stamp_s, tracker_triangles_m3, 
                                   phasespace_led_tk3, phasespace_stamp_t, ind_pairs):
    
    phasespace_led_tk3 = phasespace_led_tk3[::4]
    phasespace_stamp_t = phasespace_stamp_t[::4]
    
    S,N,_ = tracker_xyz_sn3.shape
    T,K,_ = phasespace_led_tk3.shape
    M = len(tracker_triangles_m3)
    
    led2tri = {}
    
    led2led = {}
    for (i0, i1) in ind_pairs:
        led2led[i0] = i1
        led2led[i1] = i0
        

    marker2info = [{} for _ in xrange(K)]
    
    for k in xrange(K):
        valid_inds = np.flatnonzero(np.isfinite(phasespace_led_tk3[:,k,0]))
        
        if len(valid_inds) > 0:
        
            V = len(valid_inds)
            valid_times = phasespace_stamp_t[valid_inds]
            ifirst = valid_inds[0]
            phasespace_led_v3 = phasespace_led_tk3[valid_inds,k]
            tracker_xyz_v13 = interpnd(valid_times[0:1], tracker_stamp_s, tracker_xyz_sn3)
            
            dist_to_triangles = np.empty(M)
            led = phasespace_led_v3[0]

            for (m,(i0, i1, i2)) in enumerate(tracker_triangles_m3):
                pt0, pt1, pt2 = tracker_xyz_v13[0, [i0, i1, i2],:]
                sum_of_dists = norm(pt0 - led) + norm(pt1 - led) + norm(pt2 - led)
                dist_to_triangles[m] = sum_of_dists
                                                
            if k not in led2tri: 
                led2tri[led2led[k]] = led2tri[k] = dist_to_triangles.argmin(axis=0)
                print "match leds to triangle at time %.2f: %i,%i->%i"%(valid_times[0],k,led2led[k], led2tri[k])
    
    
            
            i0, i1, i2 = tracker_triangles_m3[led2tri[k]]
            tracker_xyz_v33 = interpnd(valid_times, tracker_stamp_s, tracker_xyz_sn3[:,[i0, i1, i2], :])            
            
            led_in_tri_frame_v3 = np.empty((V,3))
            tri_trans_v44 = np.empty((V,4,4))
            
            for v in xrange(V):
                led_pt = phasespace_led_v3[v]
                pt0, pt1, pt2 = tracker_xyz_v33[v]
                tri_trans_v44[v] = get_tri_transform(pt0, pt1, pt2)
                led_in_tri_frame_v3[v] = apply_inv_trans(tri_trans_v44[v], led_pt)
                
            led_position = led_in_tri_frame_v3.mean(axis=0)
            
            marker2info[k]["times"] = valid_times
            marker2info[k]["position_relative"] = led_in_tri_frame_v3 - led_position[None,:]
            marker2info[k]["position_prediction"] = np.array([apply_trans(tri_trans_v44[v], led_position) for v in xrange(V)])
            marker2info[k]["position_actual"] = phasespace_led_v3

        else:
            marker2info[k]["times"] = np.array([])
            marker2info[k]["position_relative"] = np.zeros((0,3))
            marker2info[k]["position_prediction"] = np.zeros((0,3))
            marker2info[k]["position_actual"] = np.zeros((0,3))
            
            

    return marker2info

    

#def calculate_cloth_tracking_score(tracker_xyz_sn3, tracker_stamp_s, tracker_triangles_m3, 
                                   #phasespace_led_tk3, phasespace_stamp_t,
                                   #ind_pairs, demean_at_beginning=True):
    #S,N,_ = tracker_xyz_sn3.shape
    #T,K,_ = phasespace_led_tk3.shape
    #M = len(tracker_triangles_m3)
    
    #led2tri = {}
    
    #led2led = {}
    #for (i0, i1) in ind_pairs:
        #led2led[i0] = i1
        #led2led[i1] = i0
        
        
    
    ## resample everything at 100hz
    #new_times = np.arange(phasespace_stamp_t[0], phasespace_stamp_t[-1], .01)
    #Q = new_times.size
    
    #tr_qn3 = interpnd(new_times, tracker_stamp_s, tracker_xyz_sn3) # tracker
    #ps_qk3 = interpnd(new_times, phasespace_stamp_t, phasespace_led_tk3) # phasespace
            
        
    #for (q, ps_k3) in enumerate(ps_qk3):
        #good_k = ~np.isnan(ps_k3[:,0])
        #for k in np.flatnonzero(good_k):
            #if k not in led2tri:
                #led = ps_k3[k]
                #dist_to_triangles = np.empty(M)
                #for (m,(i0, i1, i2)) in enumerate(tracker_triangles_m3):
                    #pt0, pt1, pt2 = tr_qn3[q, [i0, i1, i2],:]
                    #sum_of_dists = norm(pt0 - led) + norm(pt1 - led) + norm(pt2 - led)
                    #dist_to_triangles[m] = sum_of_dists
                #led2tri[led2led[k]] = led2tri[k] = dist_to_triangles.argmin(axis=0)
                #print "match leds to triangle at time %.2f: %i,%i->%i"%(new_times[q],k,led2led[k], led2tri[k])
            
        
        
    #led_in_tri_frame_qk3 = np.empty((Q,K,3))
    #led_in_tri_frame_qk3.fill(np.nan)    
    
    #good_qk = ~np.isnan(ps_qk3).any(axis=2)
    
    
    #tri_transform_qk44 = np.empty((Q,K,4,4))
    
    
    #for q in xrange(Q):
        #for k in xrange(K):
            #if good_qk[q,k]:
                #i0, i1, i2 = tracker_triangles_m3[led2tri[k]]
                #pt0, pt1, pt2 = tr_qn3[q,i0], tr_qn3[q,i1], tr_qn3[q,i2]
                #tri_transform_qk44[q,k] = get_tri_transform(pt0, pt1, pt2)
                #led_in_tri_frame_qk3[q,k] = np.linalg.solve(tri_transform_qk44[q,k], np.r_[ps_qk3[q,k],1])[:3]
            
    #led_tri_frame_k3 = np.empty((K,3))
    #litf_demeaned_qk3 = np.empty((Q,K,3))
    
    
    #marker2info = [{} for _ in xrange(K)]
    
    #for k in xrange(K):        
        #good_times = np.flatnonzero(good_qk[:,k])
        #if demean_at_beginning:
            #led_tri_frame_k3[k] = led_in_tri_frame_qk3[good_times[:10],k,:].mean(axis=0)
            #litf_demeaned_qk3[:,k,:] = led_in_tri_frame_qk3[:,k,:] - led_tri_frame_k3[k][None,:]
        #else:
            #led_tri_frame_k3[k] = led_in_tri_frame_qk3[good_times,k,:].mean(axis=0)
            #litf_demeaned_qk3[:,k,:] = led_in_tri_frame_qk3[:,k,:] - led_tri_frame_k3[k][None,:]
        
           
        #marker2info[k]["times"] = new_times[good_times]
        #marker2info[k]["position_relative"] = litf_demeaned_qk3[good_times,k,:]
        #marker2info[k]["position_prediction"] = np.array([apply_trans(tri_transform_qk44[q,k],led_tri_frame_k3[k]) for q in good_times])
        #marker2info[k]["position_actual"] = ps_qk3[good_times, k]
    #print "offsets of pairs:"
    #for (i0, i1) in ind_pairs:
        #print i0,led_tri_frame_k3[i0], i1, led_tri_frame_k3[i1]

    #return marker2info
    ##litf_inpainted_qk3 = interpnd(np.arange(q), good_times, litf_demeaned_qk3[good_times])
    ##error_qk = norms(litf_inpainted_qk3,2)
    
    ##return error_qk, new_times

def calculate_rope_tracking_score(tracker_xyz_sn3, tracker_stamp_s,
                                   phasespace_led_tk3, phasespace_stamp_t,
                                   inds, demean_at_beginning=True):
    phasespace_led_tk3 = phasespace_led_tk3[::4]
    phasespace_stamp_t = phasespace_stamp_t[::4]
    
    S,N,_ = tracker_xyz_sn3.shape
    T,K,_ = phasespace_led_tk3.shape
    
    led2node = {}

    marker2info = [{} for _ in xrange(K)]
    
    for k in xrange(K):
        valid_inds = np.flatnonzero(np.isfinite(phasespace_led_tk3[:,k,0]))
        valid_times = phasespace_stamp_t[valid_inds]
        if len(valid_times) > 0:
        
            phasespace_led_v3 = phasespace_led_tk3[valid_inds,k]
            tracker_xyz_1n3 = interpnd(valid_times[0:1], tracker_stamp_s, tracker_xyz_sn3)
            led2node[k] = ssd.cdist(phasespace_led_v3[0:1], tracker_xyz_1n3[0]).argmin()
            tracker_xyz_v3 = interpnd(valid_times, tracker_stamp_s, tracker_xyz_sn3[:,led2node[k],:])

            marker2info[k]["times"] = valid_times
            marker2info[k]["position_relative"] = phasespace_led_v3 - tracker_xyz_v3
            marker2info[k]["position_prediction"] = tracker_xyz_v3
            marker2info[k]["position_actual"] = phasespace_led_v3

        else:
            marker2info[k]["times"] = np.array([])
            marker2info[k]["position_relative"] = np.zeros((0,3))
            marker2info[k]["position_prediction"] = np.zeros((0,3))
            marker2info[k]["position_actual"] = np.zeros((0,3))

    return marker2info
    #litf_inpainted_qk3 = interpnd(np.arange(q), good_times, litf_demeaned_qk3[good_times])
    #error_qk = norms(litf_inpainted_qk3,2)
    
    #return error_qk, new_times
            
            
def test_calculate_cloth_tracking_score():
    # fake data for cloth that has 2 triangles and 4 leds
    
    S = T = 1000
    f = 100.
    noise_size = .1
    
    tracker_xyz_sn3 = np.array([
        [[0,0,0],
         [0,1,0],
         [1,0,0],
         [1,1,0]]
        for _ in xrange(S)])
    tracker_triangles_m3 = np.array([[0,1,2], [3,1,2]])
    led0_pos = np.array([.1, .1, .1])
    led1_pos = np.array([.1, .2, .3])
    led2_pos = np.array([.1, .1, -.1])
    led3_pos = np.array([.1, .2, -.3])
    tracker_stamp_s = np.arange(0,S/f,1/f)
    phasespace_stamp_t = np.arange(0,S/f,1/f)
    phasespace_led_tk3 = np.empty((T,4,3))
    phasespace_led_tk3[:,0,:] = np.array([
        get_tri_transform(tracker_xyz_sn3[s,0],tracker_xyz_sn3[s,1],tracker_xyz_sn3[s,2]).dot(np.r_[led0_pos,1])[:3] for s in xrange(S)])
    phasespace_led_tk3[:,1,:] = np.array([
        get_tri_transform(tracker_xyz_sn3[s,3],tracker_xyz_sn3[s,1],tracker_xyz_sn3[s,2]).dot(np.r_[led1_pos,1])[:3] for s in xrange(S)])
    phasespace_led_tk3[:,2,:] = np.array([
        get_tri_transform(tracker_xyz_sn3[s,0],tracker_xyz_sn3[s,1],tracker_xyz_sn3[s,2]).dot(np.r_[led2_pos,1])[:3] for s in xrange(S)])
    phasespace_led_tk3[:,3,:] = np.array([
        get_tri_transform(tracker_xyz_sn3[s,3],tracker_xyz_sn3[s,1],tracker_xyz_sn3[s,2]).dot(np.r_[led3_pos,1])[:3] for s in xrange(S)])
               

    phasespace_led_tk3 += np.random.randn(*phasespace_led_tk3.shape)*noise_size
    #phasespace_led_tk3 += np.ones(phasespace_led_tk3.shape)*.1
    marker2info =  calculate_cloth_tracking_score1(tracker_xyz_sn3,
        tracker_stamp_s,
        tracker_triangles_m3,
        phasespace_led_tk3,
        phasespace_stamp_t, ((0,2), (1,3)))
    print "should be almost equal:"
    print "evaluated rms error:",[np.sqrt((info["position_relative"]**2).mean()) for info in marker2info]
    print "true rms error:", noise_size
if __name__ == "__main__":
    test_calculate_cloth_tracking_score()