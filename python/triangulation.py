#coding=utf-8
import numpy as np
import cv2

def triangulation(xL, xR, 
                         R,T,
                         left_intrinsic,  kc_left,  alpha_c_left,
                         right_intrinsic, kc_right, alpha_c_right):
                         
    assert xL.shape == xR.shape
    assert xL.shape[1] == 2
                         
    N = xL.shape[0] #点的个数
    
    #left_intrinsic = np.array(  [[fc_left[0], 0,          cc_left[0]],
    #                            [0,          fc_left[1], cc_left[1]],
    #                            [0,          0,          1         ]])
                      
    xt = cv2.undistortPoints(xL, left_intrinsic, kc_left).reshape([-1,2])
    

    
    xtt = cv2.undistortPoints(xR, right_intrinsic, kc_right).reshape([-1,2])
    
    # 转换为齐次坐标
    xt = np.hstack((xt,np.ones([N, 1]))).T
    xtt = np.hstack((xtt,np.ones([N, 1]))).T
    
    u = np.matrix(R) * np.matrix(xt);
    
    n_xt2 = np.sum(np.power(xt,2), 0)
    n_xtt2 = np.sum(np.power(xtt,2), 0)
    
    DD = np.multiply(n_xt2,n_xtt2) - np.power(np.sum(np.multiply(u,xtt),0),2)
    
    dot_uT = np.sum(np.multiply(u,T),0);
    dot_xttT = np.sum(np.multiply(xtt,T),0);
    dot_xttu = np.sum(np.multiply(u,xtt),0);
    
    NN1 = np.multiply(dot_xttu,dot_xttT) - np.multiply(n_xtt2,dot_uT);
    NN2 = np.multiply(n_xt2,dot_xttT) - np.multiply(dot_uT,dot_xttu);
    
    Zt = np.divide(NN1,DD);
    Ztt = np.divide(NN2,DD);
    
    X1 = np.multiply(xt, Zt)
    X2 = R.T * (np.multiply(xtt,Ztt) - T);
    
    XL = (X1 + X2) / 2.0;
    
    XR = R*XL + T; #这一句解释了左右摄像机之间的关系

    Error = np.mean(np.sqrt(np.sum(np.power(X1-X2, 2),0)))
    
    return XL.T, XR.T, Error


if __name__ == '__main__':
    xL = np.array([[510.6646, 357.2370], [510.6646, 357.2370]])
    xR = np.array([[629.7592, 169.4815], [629.7592, 169.4815]])
    
    om = np.array([[0.0925],
                   [0.5269],
                   [0.0275]])
                   
    R, _ = cv2.Rodrigues(om)
                   
    T = np.array( [[-230.7309],
                    [-52.8380],
                    [46.5424]])
                    
    fc_left = np.array([967.5319, 968.5227])

    cc_left = np.array([646.8668, 348.3653])
    
    left_intrinsic = np.array(  [[fc_left[0], 0,          cc_left[0]],
                                [0,          fc_left[1], cc_left[1]],
                                [0,          0,          1         ]])

    #                     k1       k2       p1       p2      k3(not used)
    kc_left = np.array([0.1604, -0.5732, -0.0015, -0.0006, 0.7387])
    
    alpha_c_left = 0

    fc_right = np.array([930.4610, 930.2560])

    cc_right = np.array([628.7786, 320.8216])
    
    right_intrinsic = np.array(  [[fc_right[0], 0,           cc_right[0]],
                                 [0,            fc_right[1], cc_right[1]],
                                 [0,            0,           1         ]])

    kc_right = np.array([-0.4524, 0.2999, 0.0003, -0.0004, -0.1418])
    
    alpha_c_right = 0
                    
    XL_target = np.array([[-85.0024, 5.5794, 605.6999],[-85.0024, 5.5794, 605.6999]])
    
    XL, XR, ERR = triangulation( xL, xR, 
                                        R,T,
                                        left_intrinsic,  kc_left,  alpha_c_left,
                                        right_intrinsic, kc_right, alpha_c_right)

    error = np.linalg.norm(XL-XL_target)
    assert error< 0.05
    print('Unit test passed!')
    