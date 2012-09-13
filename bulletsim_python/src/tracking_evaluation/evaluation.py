from __future__ import division
import numpy as np
norm = np.linalg.norm

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

def calculate_cloth_tracking_score(tracker_xyz_sn3, tracker_stamp_s, tracker_triangles_m3, 
                                   phasespace_led_tk3, phasespace_stamp_t):
    S,N,_ = tracker_xyz_sn3.shape
    T,K,_ = phasespace_led_tk3.shape
    M = tracker_triangles_m3.shape[0]
    
    led2tri = np.zeros(K, int)
    
    first_verts_n3 = tracker_xyz_sn3[0]
    first_led_k3 = phasespace_led_tk3[0]
    # find which triangle to associate each LED with
    for (k,led) in enumerate(first_led_k3):
        dist_to_triangles = np.empty(M)
        for (m,(i0, i1, i2)) in enumerate(tracker_triangles_m3):
            pt0, pt1, pt2 = first_verts_n3[[i0, i1, i2],:]
            sum_of_dists = norm(pt0 - led) + norm(pt1 - led) + norm(pt2 - led)
            dist_to_triangles[m] = sum_of_dists
        led2tri[k] = dist_to_triangles.argmin(axis=0)
        
    # resample everything at 100hz
    new_times = np.arange(phasespace_stamp_t[0], phasespace_stamp_t[-1], .01)
    Q = new_times.size
    
    tr_qn3 = interpnd(new_times, tracker_stamp_s, tracker_xyz_sn3) # tracker
    ps_qk3 = interpnd(new_times, phasespace_stamp_t, phasespace_led_tk3) # phasespace
    
    led_in_tri_frame_qk3 = np.empty((Q,K,3))
    
    for q in xrange(Q):
        for k in xrange(K):
            i0, i1, i2 = tracker_triangles_m3[led2tri[k]]
            pt0, pt1, pt2 = tr_qn3[q,i0], tr_qn3[q,i1], tr_qn3[q,i2]
            tri_transform = get_tri_transform(pt0, pt1, pt2)
            led_in_tri_frame_qk3[q,k] = np.linalg.solve(tri_transform, np.r_[ps_qk3[q,k],1])[:3]
            

    led_error_in_tri_frame_qk3 = led_in_tri_frame_qk3 - led_in_tri_frame_qk3.mean(axis=0)[None,:,:]
    rms_error_k3 = np.sqrt((led_error_in_tri_frame_qk3**2).mean())
    return rms_error_k3.mean()
            
            
def test_calculate_cloth_tracking_score():
    # fake data for cloth that has 2 triangles and 2 leds
    
    S = T = 10000
    f = 100.
    noise_size = .03
    
    tracker_xyz_sn3 = np.array([
        [[0,0,0],
         [0,1,0],
         [1,0,0],
         [1,1,0]]
        for _ in xrange(S)])
    tracker_triangles_m3 = np.array([[0,1,2], [3,1,2]])
    led0_pos = np.array([.1, .1, .1])
    led1_pos = np.array([.1, .2, .3])
    tracker_stamp_s = np.linspace(0,S/f,S)
    phasespace_stamp_t = np.linspace(0,T/f,T)
    phasespace_led_tk3 = np.empty((T,2,3))
    phasespace_led_tk3[:,0,:] = np.array([
        get_tri_transform(tracker_xyz_sn3[s,0],tracker_xyz_sn3[s,1],tracker_xyz_sn3[s,2]).dot(np.r_[led0_pos,1])[:3] for s in xrange(S)])
    phasespace_led_tk3[:,1,:] = np.array([
        get_tri_transform(tracker_xyz_sn3[s,3],tracker_xyz_sn3[s,1],tracker_xyz_sn3[s,2]).dot(np.r_[led1_pos,1])[:3] for s in xrange(S)])
               
    phasespace_led_tk3 += np.random.randn(*phasespace_led_tk3.shape)*noise_size
    rms_error =  calculate_cloth_tracking_score(tracker_xyz_sn3,
                                                tracker_stamp_s,
                                                tracker_triangles_m3,
                                                phasespace_led_tk3,
                                               phasespace_stamp_t)
    print "should be almost equal:"
    print "evaluated rms error:",rms_error
    print "true rms error:", noise_size
if __name__ == "__main__":
    test_calculate_cloth_tracking_score()