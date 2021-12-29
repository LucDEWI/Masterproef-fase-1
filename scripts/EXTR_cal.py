#bibliotheken
import numpy as np
import cv2 as cv
import os
import sys
from src import Functions as func

# In deze functie wordt de extrinsieke transformatie gekalibreed en kan men bepalen via een dambordpatroon waar het
# referentiestelsel wordt gelegd

#eigenschappen patroon
h=14
b=9
size=17.6 #aantal mm zijde van een vierkant op het schaakbord

#laden van intrinsieke gegevens uit data
datapath = os.path.abspath(os.path.join(os.getcwd(), os.pardir))+'\data'
mtx = np.load(datapath+'\intrinsics.npy')
dist = np.load(datapath+'\distortion.npy')

#venster instellingen
window_name = 'Extrinsic calibration'
window_b, window_h = 1920, 1080

#camera instellingen
color_resolution = (1920, 1080)
depth_resolution = (1280, 720)
frames_per_second = 30

#stel venster en camera in
func.setwindow(window_name, window_b, window_h)
pipeline, align_object = func.start(color_resolution, depth_resolution, frames_per_second)

#functie die niets doet, nodig voor sliders
def nothing(*args):
    pass

#aanmaken van slider
cv.createTrackbar('save',window_name,0,1,nothing)

def draw(img, corners, imgpts):
    corners = corners.astype('int32')
    imgpts = imgpts.astype('int32')
    corner = tuple(corners[0].ravel())

    img = cv.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((b*h,3), np.float32)
objp[:,:2] = np.mgrid[0:b,0:h].T.reshape(-1,2)
objp = 17.6*objp
axis = np.float32([[60,0,0], [0,60,0], [0,0,-60]]).reshape(-1,3)
amount = 10
k= 1
while k < amount:
    cv.waitKey(1000)


    save = cv.getTrackbarPos('save', window_name)
    if(save):
        break

    img, depth_img = func.getframes(pipeline, align_object)

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    ret, corners = cv.findChessboardCornersSB(img, (b, h), cv.CALIB_CB_MARKER)

    if ret == True:

        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        ret, rvecs, tvecs, inliers = cv.solvePnPRansac(objp, corners2, mtx, dist)

        cv.drawChessboardCorners(img, (b, h), corners2, ret)

        imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, mtx, dist)

        beeld = draw(img, corners2, imgpts)
        cv.waitKey(1000)
        cv.imshow('Extrinsic calibration', beeld)
        k = k+1
        print(k, "\n")

cv.destroyAllWindows()



rvecs_matrix=cv.Rodrigues(rvecs)[0]
extrinsics = np.hstack((rvecs_matrix, tvecs))
extrinsics = np.vstack((extrinsics, [0.0, 0.0, 0.0, 1.0]))
#print de extrinsieke matrix
print(extrinsics)
#sla de extrinsieke gegevens op in numpy arrays
np.save(datapath+'\extrinsics.npy',extrinsics)
np.save(datapath+'\extrinsic_rvecs.npy',rvecs)
np.save(datapath+'\extrinsic_tvecs.npy',tvecs)
print('done')
if k == amount:
    exit()