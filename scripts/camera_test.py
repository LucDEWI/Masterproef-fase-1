import numpy as np
import cv2 as cv
import pyrealsense2 as rs
from src import Functions as func

# deze functie toont de beelden van de kleur en diepte camera

#resolutie en fps van de beelden
cres = (1920, 1080)
dres = (1280, 720)
fps = 30
name = 'camera viewer'
func.setwindow(name, 1920, 1080)

#connectie starten
connection, align = func.start(cres, dres, fps)

#loop voor beelden
while True:
    #frames ophalen
    color, depth = func.getframes(connection, align)

    #dieptebeeld inkleuren
    depth_color = cv.applyColorMap(cv.convertScaleAbs(depth, alpha=0.03), cv.COLORMAP_JET)
    gray = cv.cvtColor(color, cv.COLOR_BGR2GRAY)

    #beelden naast elkaar zetten
    img = np.hstack((color, depth_color))
    func.showimage(img, name)
    if cv.waitKey(1) == ord('q'):
        break
