import numpy as np
import cv2
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*10,3), np.float32)
objp[:,:2] = np.mgrid[0:200:20,0:140:20].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
 # 3d point in real world space
imgpoints = {'cam1':[], 'cam2':[]} # 2d points in image plane.
mtx = {}
dist = {}
rvecs = {}
tvecs = {}

for folder in ['cam1', 'cam2']:
    objpoints = []
    images = glob.glob('../' + folder + '/*.jpg')

    print(images)

    for fname in images:
        print(fname)
        img = cv2.imread(fname)
        img_shape = img.shape
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (10,7),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints[folder].append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (10,7), corners2,ret)
            cv2.imshow('img',img)
            cv2.waitKey(1)

    cv2.destroyAllWindows()

    ret, mtx[folder], dist[folder], rvecs[folder], tvecs[folder] = cv2.calibrateCamera(objpoints, imgpoints[folder], gray.shape[::-1],None,None)

    imgpoints_np = np.array(imgpoints[folder]).reshape(15, 70*2)
    np.savetxt(folder + '_points.txt', imgpoints_np)
    np.savetxt(folder + '_intrisic_mtx.txt', mtx[folder])
    np.savetxt(folder + '_distortion_param.txt', dist[folder])
    np.savetxt(folder + '_chessboard_r.txt', np.squeeze(np.stack(rvecs[folder])))
    np.savetxt(folder + '_chessboard_t.txt', np.squeeze(np.stack(tvecs[folder])))


stereocalib_criteria = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
ret, M1, d1, M2, d2, R, T, E, F = cv2.stereoCalibrate( objpoints, imgpoints['cam1'], imgpoints['cam2'], mtx['cam1'], dist['cam1'], mtx['cam2'], dist['cam2'], imageSize=(22,33))

np.savetxt('extrinsic_r.txt', R)
np.savetxt('extrinsic_t.txt', T) 
                                                                           
print('cam1 intrinsic: \n', M1)
print('cam2 intrinsic: \n', M2)
print('cam1 distort: \n', d1)
print('cam2 distort: \n', d2)
print('R: \n', R)
print('om: \n',cv2.Rodrigues(R)[0])
print('T: \n', T)

