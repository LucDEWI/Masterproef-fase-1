import numpy as np
import cv2 as cv
import pyrealsense2 as rs
from src import Functions as func
import os

# In dit script kan de grijswaarde ingesteld worden voor het script pixel_gray

#aanroepen van eventuele data of nieuwe data file aanmaken
datapath = os.path.abspath(os.path.join(os.getcwd(), os.pardir))+ '\data'
try:
    grayfile = np.load(datapath+'\gray.npy')
except FileNotFoundError:
    grayfile = np.save(datapath+'\gray.npy', [0, 0])


#resolutie en fps  van de beelden ingeven
cres = (640, 480)
dres = (640, 480)
fps = 30
# named window maken
cv.namedWindow('trackbar')
# connectie met camera starten
connection, align = func.start(cres, dres, fps)

#nutteloze functie voor trackbars
def nothing(x):
    pass
#trackbar aanmaken
cv.createTrackbar('gray max', 'trackbar', grayfile[1], 255, nothing)
cv.createTrackbar('gray min', 'trackbar', grayfile[0], 255, nothing)
cv.createTrackbar('save', 'trackbar', 0, 1, nothing)

#loop
while True:
    #frames ophalen
    color, depth = func.getframes(connection, align)

    # trackbarpositie ophalen
    gray_max = cv.getTrackbarPos('gray max', 'trackbar')
    gray_min = cv.getTrackbarPos('gray min', 'trackbar')
    save = cv.getTrackbarPos('save', 'trackbar')

    lower = np.array(gray_min)
    upper = np.array(gray_max)

    #masker zetten
    mask = func.setmask(color, lower, upper)

    #masker over kleurbeeld leggen
    res = cv.bitwise_and(color, color, mask= mask)

    func.showimage(res, '')

    #als save geactiveerd worden de waarden opgeslagen en de tool gsloten
    if (save):
        break

    if cv.waitKey(1) == ord('q'):
        break

grayarray = np.array([gray_min, gray_max])
print(grayarray)
np.save(datapath+'\gray.npy', grayarray)