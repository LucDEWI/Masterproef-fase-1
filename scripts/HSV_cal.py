import numpy as np
import cv2 as cv
import pyrealsense2 as rs
from src import Functions as func
import os

# In dit script kan de HSV grenzen ingesteld worden voor de functies pixel_HSV, XYZ_ HSV, robot_sturen

#aanroepen van eventuele data of nieuwe data file aanmaken
datapath = os.path.abspath(os.path.join(os.getcwd(), os.pardir))+ '\data'
try:
    hsvfile = np.load(datapath+'\hsv.npy')
except FileNotFoundError:
    hsvfile = np.save(datapath+'\hsv.npy', [0, 0, 0, 0, 0, 0])
    print(hsvfile)
print(hsvfile)

#resolutie en fps  van de beelden ingeven
cres = (1920, 1080)
dres = (1280, 720)
fps = 30
# named window maken
cv.namedWindow('trackbar')
# connectie met camera starten
connection, align = func.start(cres, dres, fps)

#nutteloze functie voor trackbars
def nothing(x):
    pass
#trackbar aanmaken
cv.createTrackbar('H max', 'trackbar', hsvfile[0], 179, nothing)
cv.createTrackbar('S max', 'trackbar', hsvfile[1], 255, nothing)
cv.createTrackbar('V max', 'trackbar', hsvfile[2], 255, nothing)
cv.createTrackbar('H min', 'trackbar', hsvfile[3], 179, nothing)
cv.createTrackbar('S min', 'trackbar', hsvfile[4], 255, nothing)
cv.createTrackbar('V min', 'trackbar', hsvfile[5], 255, nothing)
cv.createTrackbar('save', 'trackbar', 0, 1, nothing)

#loop
while True:
    #frames ophalen
    color, depth = func.getframes(connection, align)

    # trackbarpositie ophalen
    H_max = cv.getTrackbarPos('H max', 'trackbar')
    S_max = cv.getTrackbarPos('S max', 'trackbar')
    V_max = cv.getTrackbarPos('V max', 'trackbar')
    H_min = cv.getTrackbarPos('H min', 'trackbar')
    S_min = cv.getTrackbarPos('S min', 'trackbar')
    V_min = cv.getTrackbarPos('V min', 'trackbar')
    save = cv.getTrackbarPos('save', 'trackbar')

    lower = np.array([H_min, S_min, V_min])
    upper = np.array([H_max, S_max, V_max])

    #masker zetten
    mask = func.setmaskHSV(color, lower, upper)

    #masker over kleurbeeld leggen
    res = cv.bitwise_and(color, color, mask= mask)

    func.showimage(res, '')

    #als save geactiveerd worden de waarden opgeslagen en de tool gsloten
    if (save):
        break

    if cv.waitKey(1) == ord('q'):
        break

hsvarray = np.array([H_max, S_max, V_max, H_min, S_min, V_min])
print(hsvarray)
np.save(datapath+'\hsv.npy', hsvarray)