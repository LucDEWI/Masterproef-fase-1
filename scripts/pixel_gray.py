import numpy as np
import cv2 as cv
import pyrealsense2 as rs
from src import Functions as func
import os

#Toont de pixelcoördinaten aan de hand van de grijs grens. Er moet eerst een grens opgesteld worden via gray_cal

#laden van grayscale calibratie
datapath = os.path.abspath(os.path.join(os.getcwd(), os.pardir))+ '\data'
try:
    grayfile = np.load(datapath + '\gray.npy')
    print(grayfile)
except FileNotFoundError:
    print('Er is nog geen calibratie uitgevoerd. Gelieve dit eerst te doen, om dit doen run het script gray_Threshold_cal.py')
    exit()
if grayfile.any() == 0:
    print('Er is nog geen calibratie uitgevoerd. Gelieve dit eerst te doen, om dit doen run het script gray_Threshold_cal.py')
    exit()
else:
    #instellen van de Threshold waardes uit de calibratie
    lower = int(grayfile[0])
    upper = int(grayfile[1])

    #camera instellen en starten
    cres = (640, 480)
    dres = (640, 480)
    fps = 30

    connection, align = func.start(cres, dres, fps)

    #referentie oppervlakte (berekenen van de opppervlakte van het met enkel de grootste straal, gat niet in rekening brengen)
    area = 6705.5

    #loop
    while True:
        try:
            #frames ophalen
            color, depth = func.getframes(connection, align)
            #masker opzetten


            mask = func.setmask(color, lower, upper)


            pixel, radius = func.getwheelpixel(mask, area)
            radius = int(radius)
            if pixel:
                depth_pixel = func.getdepthpixel(depth, pixel)
            else:
                depth_pixel = None

            img = func.showwheelpixel(color, pixel, depth_pixel, None, None, None, radius)

            cv.imshow('', color)
            if cv.waitKey(1) == ord('q'):
                break
        except:
            continue




