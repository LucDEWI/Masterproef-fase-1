#bibliotheken
import numpy
from src import Functions as func
import os
import cv2 as cv

#toont de pixelcoördinaten aan de hand van de HSV grens. Er moet eerst een grens opgesteld worden via HSV_cal

#gegevens van hsv kalibratie importeren uit de datafolder
datapath=os.path.abspath(os.path.join(os.getcwd(), os.pardir))+'\data'
hsvfile=numpy.load(datapath+'\hsv.npy')
# onderste en bovenste kleur definieren
lower_color = numpy.array([hsvfile[3],hsvfile[4],hsvfile[5]])
upper_color = numpy.array([hsvfile[0],hsvfile[1],hsvfile[2]])

#venster instellingen


#camera instellingen
color_resolution = (1920, 1080)
depth_resolution = (1280, 720)
frames_per_second = 30

#minimum straal van de bal contour
area= 6705.5

#stel venster en camera in

connection, align = func.start(color_resolution,depth_resolution,frames_per_second)

#loop
while True:
    try:
        # frames ophalen
        color, depth = func.getframes(connection, align)
        # masker opzetten

        mask = func.setmaskHSV(color, lower_color, upper_color)

        pixel, radius = func.getwheelpixel(mask, area)
        radius = int(radius)
        if pixel:
            depth_pixel = func.getdepthpixel(depth, pixel)
        else:
            depth_pixel = None

        img = func.showwheelpixel(color, pixel, depth_pixel, None, None, None, radius)

        cv.imshow('', img)
        if cv.waitKey(1) == ord('q'):
            break
    except:
        continue



