#bibliotheken
import cv2
import numpy
from src import Functions as func
import os
from urx import Robot
import urx
import math3d as md
import time

import math3d as m3d

#Dit script zal de locatie van het object bepalen en dan de robotarm naar de locatie sturen.
#De robot zal het object nemen en terugkeren naar de home positie.
## WORK IN PROGRESS! WERKT NOG NIET, toepassen op robot niet aangeraden tot volgende update

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

#oppervlak van het object
area = 6705.5

#stel venster en camera in

pipeline, align_object = func.start(color_resolution,depth_resolution,frames_per_second)

#connectie naar de robot

if __name__ == "__main__":
    rob = urx.robot("hier IP robot invullen")
    rob.set_tcp((0, 0, 0.15, 0, 0, 0)) #opmeten op de robot afstand van endeffector naar laatste joint zie: https://docs.pickit3d.com/en/3.0/faq/robot-programming/how-to-define-the-tcp-on-a-universal-robots.html
    rob.set_payload(0.5, (0, 0, 0)) #zie of dat dit nodig is bepaalt het geicht aan de end effector en richting
    j = rob.getj()
    print("initiële joint waarden: ", j)
    t = rob.get_pose()
    print("init transformatiematrix van base naar tcp: ", t)

    time.sleep(2)
    #loop starten
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
            pos_obj = rob.set_pos((t.pose[0] + xworld, t.pose[1] + yworld, t.pose[2] + zworld), vel= 0.2, acc= 0.8)
            time.sleep(5)

            while True:
                if rob.get_pose() == pos_obj:
                    break
                else:
                    continue
            rob.movej(j, acc= 0.8, vel=0.2)
            while True:
                if rob.get_pose() == t:
                    break
                else:
                    continue

        except:
            color_image, depth_image = func.getframes(pipeline, align_object)
            cv2.imshow('', color_image)
            continue
        if cv2.waitKey(10) == ord('q'):
            break





