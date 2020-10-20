#coding=utf-8
import numpy as np
import cv2

def stereo_triangulation(xL, xR, 
                         om,T,
                         fc_left,  cc_left,  kc_left,  alpha_c_left,
                         fc_right, cc_right, kc_right, alpha_c_right):
                         
    assert xL.shape == xR.shape
    assert xL.shape[1] == 2
                         
    N = xL.shape[0] #点的个数
    
    left_intrinsic = np.array(  [[fc_left[0], 0,          cc_left[0]],
                                [0,          fc_left[1], cc_left[1]],
                                [0,          0,          1         ]])
                      
    xt = cv2.undistortPoints(xL, left_intrinsic, kc_left).reshape([-1,2])
    
    right_intrinsic = np.array(  [[fc_right[0], 0,           cc_right[0]],
                                 [0,            fc_right[1], cc_right[1]],
                                 [0,            0,           1         ]])
    
    xtt = cv2.undistortPoints(xR, right_intrinsic, kc_right).reshape([-1,2])
    
    # 转换为齐次坐标
    xt = np.hstack((xt,np.ones([N, 1]))).T
    xtt = np.hstack((xtt,np.ones([N, 1]))).T
    
    R, _ = cv2.Rodrigues(om)
    
    u = np.matrix(R) * np.matrix(xt);
    
    n_xt2 = np.sum(np.power(xt,2), 0)
    n_xtt2 = np.sum(np.power(xtt,2), 0)
    
    #DD = n_xt2 .* n_xtt2 - dot(u,xtt).^2;
    DD = np.multiply(n_xt2,n_xtt2) - np.power(np.sum(np.multiply(u,xtt)),2)
    
    print(DD)
    print(DD.shape)
    assert 0
    
    return 0,0,0


if __name__ == '__main__':
    xL = np.array([[510.6646, 357.2370], [510.6646, 357.2370]])
    xR = np.array([[629.7592, 169.4815], [629.7592, 169.4815]])
    
    om = np.array([[0.0925],
                   [0.5269],
                   [0.0275]])
                   
    T = np.array( [[-230.7309],
                    [-52.8380],
                    [46.5424]])
                    
    fc_left = np.array([967.5319, 968.5227])

    cc_left = np.array([646.8668, 348.3653])

    #                     k1       k2       p1       p2      k3(not used)
    kc_left = np.array([0.1604, -0.5732, -0.0015, -0.0006, 0.7387])
    
    alpha_c_left = 0

    fc_right = np.array([930.4610, 930.2560])

    cc_right = np.array([628.7786, 320.8216])

    kc_right = np.array([-0.4524, 0.2999, 0.0003, -0.0004, -0.1418])
    
    alpha_c_right = 0
                    
    XL_target = np.array([-85.0024, 5.5794, 605.6999])
    
    XL, XR, ERR = stereo_triangulation( xL, xR, 
                                        om,T,
                                        fc_left,  cc_left,  kc_left,  alpha_c_left,
                                        fc_right, cc_right, kc_right, alpha_c_right)
    print(XL)
    print(XL_target)
    error = np.linalg.norm(XL-XL_target)
    print(error)
    assert error< 0.01
    