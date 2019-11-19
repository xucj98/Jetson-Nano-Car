'''
This is a python script to calibrate binocular stereo camera.
Written by Xu Cuijie, 2019.11.18
Before running this script, you should prepare the folloing: 
    Prepare the config file and set its path (CAM_CFG_PATH) in scrpit.
    Set image source (USE_VIDEO_CAP), either video capture (USE_VIDEO_CAP = True) or image file (USE_VIDEO_CAP = False). If image source is video capture, press 'space' to snapshot, only valid image, both left and right image have all chessboard corners, will be used. 
    Set camera data dir (CAM_DATA_DIR), where read image file for calibrating or save image from video capture.     
    Set result file path (RESULT_PATH).
'''

import cv2
import sys
import yaml
import os
import numpy as np

if __name__ == '__main__':

    CAM_CFG_PATH = '../cfg/stereo_camera_calib.yaml'    
    USE_VIDEO_CAP = True
    CAM_DATA_DIR = '../data/stereo_1'
    RESULT_PATH = './result.yaml'
    
    # read config file
    assert os.path.exists(CAM_CFG_PATH), \
        'Calibrate configure file does not exist in the following path: {}.'.format(CAM_CFG_PATH)    
    calib_cfg_file = open(CAM_CFG_PATH, 'r')
    calib_cfg = yaml.load(calib_cfg_file.read())

    # use video capture
    if USE_VIDEO_CAP:
        # config video caputre
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, calib_cfg['SPLICED_IMAGE']['WIDTH'])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, calib_cfg['SPLICED_IMAGE']['HEIGHT'])
        assert cap.isOpened(), 'Camera is not available!'

    # stop criteria when optimize camera matrix    
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, calib_cfg['CRITERIA']['MAX_ITER'], calib_cfg['CRITERIA']['EPS'])
    # use chessboard with 9 * 6 corners
    w = calib_cfg['CHESSBOARD']['WIDTH']
    h = calib_cfg['CHESSBOARD']['HEIGHT']
    objp = np.zeros((w*h,3), np.float32)
    objp[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2) * calib_cfg['CHESSBOARD']['SIZE']
    objpoints = []
    imgpoints_l = []
    imgpoints_r = []
    
    width = calib_cfg['IMAGE']['WIDTH'];
    height = calib_cfg['IMAGE']['HEIGHT'];

    cnt = 0
    while cnt < calib_cfg['IMAGE_NUMBER']:
        img_path_l = os.path.join(CAM_DATA_DIR, 'left_' + str(cnt) + '.jpg')
        img_path_r = os.path.join(CAM_DATA_DIR, 'right_' + str(cnt) + '.jpg')
        
        key = None        
        if USE_VIDEO_CAP:
            _, frame = cap.read()
            left = frame[0:height, 0:width, :]
            right = frame[0:height, width:width*2, :]
            cv2.imshow('raw', frame)
            key = cv2.waitKey(1)
        else: 
            left = cv2.imread(img_path_l)
            right = cv2.imread(img_path_r)        

        ret_l = False
        ret_r = False
        if key == ord(' ') or not USE_VIDEO_CAP:
            gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
            ret_l, corners_l = cv2.findChessboardCorners(gray_l, (w,h), None)
            ret_r, corners_r = cv2.findChessboardCorners(gray_r, (w,h), None) 

        if ret_l and ret_r:
            cv2.cornerSubPix(gray_l,corners_l, (11,11), (-1,-1), criteria)
            cv2.cornerSubPix(gray_r,corners_r, (11,11), (-1,-1), criteria)
            objpoints.append(objp)
            imgpoints_l.append(corners_l)
            imgpoints_r.append(corners_r)
          
            if USE_VIDEO_CAP:
                cv2.imwrite(img_path_l, left)            
                cv2.imwrite(img_path_r, right)            
                
            cv2.drawChessboardCorners(left, (w,h), corners_l, ret_l)
            cv2.imshow('corners_l', left)
            cv2.imwrite(CAM_DATA_DIR + '/corners_l_' + str(cnt) + '.jpg', left)
            cv2.drawChessboardCorners(right, (w,h), corners_r, ret_r)
            cv2.imshow('corners_r', right)
            cv2.imwrite(CAM_DATA_DIR + '/corners_r_' + str(cnt) + '.jpg', right)
            if not USE_VIDEO_CAP:
                cv2.waitKey(500)
            cnt += 1

    cv2.destroyAllWindows()
    rmse_l, intrinsics_l, distortion_l, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints_l, (height, width), None, None)
    print('rmse_l =', rmse_l)
    print('intrinsics_l:\n', intrinsics_l)
    print('distortion_l:\n', distortion_l)
    rmse_r, intrinsics_r, distortion_r, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints_r, (height, width), None, None)
    print('rmse_r =', rmse_r)
    print('intrinsics_r:\n', intrinsics_r)
    print('distortion_r:\n', distortion_r)
    
    rmse, intrinsics_l, distortion_l, intrinsics_r, distortion_r, R, T, E, F = cv2.stereoCalibrate(objpoints, imgpoints_l, imgpoints_r, intrinsics_l, distortion_l, intrinsics_r, distortion_r, imageSize=(height, width))
    print('rmse =', rmse)
    print('intrinsics_l:\n', intrinsics_l)
    print('distortion_l:\n', distortion_l)
    print('intrinsics_r:\n', intrinsics_r)
    print('distortion_r:\n', distortion_r)
    print('R:\n', R)
    print('T:\n', T)
    print('E:\n', E)
    print('F:\n', F)

    result_file = open(RESULT_PATH, 'w')
    result = {'INTRINSICS_L': intrinsics_l.tolist(),
              'DISTORTION_L': distortion_l.tolist(),
              'INTRINSICS_R': intrinsics_r.tolist(),
              'DISTORTION_R': distortion_r.tolist(),
              'ROTATION': R.tolist(),
              'TRANSLATION': T.tolist()}
    yaml.dump(result, result_file)
    result_file.close()
