import numpy as np
import cv2 as cv
import pyrealsense2 as rs
from src import Functions as func
import os

# In dit script kunnen de BGR grenzen van het masker voor functie pixel-BGR ingesteld worden

#aanroepen van eventuele data of nieuwe data file aanmaken
datapath = os.path.abspath(os.path.join(os.getcwd(), os.pardir))+ '\data'
try:
    BGRfile = np.load(datapath+'\BGR.npy')
except FileNotFoundError:
    BGRfile = np.save(datapath+'\BGR.npy', [0, 0, 0, 0, 0, 0])


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
cv.createTrackbar('B max', 'trackbar', BGRfile[0], 255, nothing)
cv.createTrackbar('G max', 'trackbar', BGRfile[1], 255, nothing)
cv.createTrackbar('R max', 'trackbar', BGRfile[2], 255, nothing)
cv.createTrackbar('B min', 'trackbar', BGRfile[3], 255, nothing)
cv.createTrackbar('G min', 'trackbar', BGRfile[4], 255, nothing)
cv.createTrackbar('R min', 'trackbar', BGRfile[5], 255, nothing)
cv.createTrackbar('save', 'trackbar', 0, 1, nothing)

#loop
while True:
    #frames ophalen
    color, depth = func.getframes(connection, align)

    # trackbarpositie ophalen
    B_max = cv.getTrackbarPos('B max', 'trackbar')
    B_min = cv.getTrackbarPos('B min', 'trackbar')
    G_max = cv.getTrackbarPos('G max', 'trackbar')
    G_min = cv.getTrackbarPos('G min', 'trackbar')
    R_max = cv.getTrackbarPos('R max', 'trackbar')
    R_min = cv.getTrackbarPos('R min', 'trackbar')
    save = cv.getTrackbarPos('save', 'trackbar')

    lower = np.array([B_min, G_min, R_min])
    upper = np.array([B_max, G_max, R_max])

    #masker zetten
    mask = func.setmaskBGR(color, lower, upper)

    #masker over kleurbeeld leggen
    res = cv.bitwise_and(color, color, mask= mask)

    func.showimage(res, '')

    #als save geactiveerd worden de waarden opgeslagen en de tool gsloten
    if (save):
        break

    if cv.waitKey(1) == ord('q'):
        break

BGRarray = np.array([B_max, G_max, R_max, B_min, G_min, R_min])
print(BGRarray)
np.save(datapath+'\BGR.npy', BGRarray)