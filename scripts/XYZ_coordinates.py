#bibliotheken
import cv2
import numpy
from src import Functions as func
import os

#Dit script zal de ruimtecoördinaten van het object bepalen tegenover een referentiestelsel.
#Er dient eerst een extrisieke-en intrinsieke kalibratie toegepast te worden via voorgaande scripts


#gegevens van hsv kalibratie importeren uit de datafolder
datapath=os.path.abspath(os.path.join(os.getcwd(), os.pardir))+'\data'
hsvfile=numpy.load(datapath+'\hsv.npy')
# onderste en bovenste kleur definieren
lower_color = numpy.array([hsvfile[3],hsvfile[4],hsvfile[5]])
upper_color = numpy.array([hsvfile[0],hsvfile[1],hsvfile[2]])

#intrinsieke en extrinsieke gegevens importeren
mtx=numpy.load(datapath+'\intrinsics.npy')
dist=numpy.load(datapath+'\distortion.npy')
ext=numpy.load(datapath+'\extrinsics.npy')
rvecs=numpy.load(datapath+'\extrinsic_rvecs.npy')
tvecs=numpy.load(datapath+'\extrinsic_tvecs.npy')




#camera instellingen
color_resolution = (1920,1080)
depth_resolution = (1280, 720)
frames_per_second = 30

#minimum straal van de bal contour
area = 6705.5

#stel venster en camera in

pipeline, align_object = func.start(color_resolution,depth_resolution,frames_per_second)



#loop
while True:
    try:


        cv2.waitKey(10)
        #beelden ophalen
        color_image, depth_image = func.getframes(pipeline, align_object)
        #masker van de bal opvragen
        mask=func.setmaskHSV(color_image,lower_color,upper_color)
        #pixel van het zwaartepunt van de bal zoekekn
        ball_pixel, radius = func.getwheelpixel(mask, area)
        #wanneer er een bal gevonden is is er een zwaartepunt (pixel), bepaal hier de diepte
        if ball_pixel:
            depth_pixel = func.getdepthpixel(depth_image, ball_pixel)
        else:
            depth_pixel = None
        #intrinsiek transformeren naar cameracoördinaten
        xcam, ycam, zcam = func.intrinsictrans(ball_pixel, depth_pixel, mtx)
        #extrinsiek transformeren naar ruimtecoördinaten
        xworld, yworld, zworld = func.extrinsictrans(depth_pixel, xcam, ycam, zcam, ext)
        #visualiseer de gedetecteerde bal en de UV en Z waarde op het kleurenbeeld
        img = func.showwheelpixel(color_image, ball_pixel, depth_pixel, xworld, yworld, zworld, radius)
        img = func.drawaxes(img, mtx, dist, rvecs, tvecs, 60)
        #beelden laten zien
        cv2.imshow('', img)
    except:
        color_image, depth_image = func.getframes(pipeline, align_object)
        cv2.imshow('', color_image)
        continue
    if cv2.waitKey(10) == ord('q'):
        break

