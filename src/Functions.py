import numpy as np
import cv2 as cv
import pyrealsense2 as rs

# In deze source file worden de functies die in de scripts worden gebruikt gedefiëneerd.
# Hierbij wordt een functie wordt aangemaakt en in inputs meegegeven en bij return wordt de outputs van de functie gezet

# Deze functie wordt gebruikt om de verbinding met de realsense camera op te starten
# Om de verbinding met de camera te starten moet de resolutie gegeven en fps moet ingegeven worden
def start(cres, dres, fps):
    #connectie met camera aanmaken
    connection = rs.pipeline()
    #camera configureren
    configuration = rs.config()
    configuration.enable_stream(rs.stream.color, cres[0], cres[1], rs.format.bgr8, fps)
    configuration.enable_stream(rs.stream.depth, dres[0], dres[1], rs.format.z16, fps)
    #camera starten
    connection.start(configuration)
    #beelden aligneren
    align = rs.align(rs.stream.color)

    return connection, align

#Deze fucntie haalt de frames op van de camera en converteert ze naar een formaat waar opencv mee kan werken
def getframes(connection, align):
    # wachten op frames
    frames = connection.wait_for_frames()
    #aligneren beeld
    aligned = align.process(frames)
    #kleur en diepte frame ophalen
    color_frame = aligned.get_color_frame()
    depth_frame = aligned.get_depth_frame()
    #omzetten naar array voor opencv
    color = np.asanyarray(color_frame.get_data())
    depth = np.asanyarray((depth_frame.get_data()))

    return color, depth

# in volgende functie worden de image processing stappen uitgevoerd
# hierbij wordt het beeld geblurd, grayscale, threshold ingesteld en gemasked
def setmask(image, min, max):
    #beeld blurren
    blur = cv.GaussianBlur(image, (11, 11), 0)
    #grayscalen
    gray = cv.cvtColor(blur, cv.COLOR_BGR2GRAY)
    #masken via threshold
    mask = cv.inRange(gray, min, max)
    #eroden en dilaten
    mask = cv.erode(mask, None, iterations= 2)
    mask = cv.dilate(mask, None, iterations= 2)

    return mask

def setmaskHSV(image, min, max):
    #beeld blurren
    blur = cv.GaussianBlur(image, (11, 11), 0)
    #grayscalen
    gray = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
    #masken via threshold
    mask = cv.inRange(gray, min, max)
    #eroden en dilaten
    mask = cv.erode(mask, None, iterations= 2)
    mask = cv.dilate(mask, None, iterations= 2)

    return mask

def setmaskBGR(image, min, max):
    #beeld blurren
    blur = cv.GaussianBlur(image, (11, 11), 0)
    #grayscalen

    #masken via threshold
    mask = cv.inRange(blur, min, max)
    #eroden en dilaten
    mask = cv.erode(mask, None, iterations= 2)
    mask = cv.dilate(mask, None, iterations= 2)

    return mask

def getwheelpixel(mask, area):
    residue = 10000
    #zoeken naar contours
    contour, hierarchy =cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    #kijken of er contouren zijn

    if len(contour) > 0:
        #lijst van contouren aanmaken
        contour_list = list(range(0, 3))
        # oppervlakte van verschillende contouren
        for i in range(3):
            contour_area = cv.contourArea(contour[i])
            contour_list[i] = contour_area
        contour_array = np.asanyarray(contour_list)
        for i in range(3):
            sub = abs(area - contour_array[i])
            if sub < residue:
                residue = sub
                wheel = contour[i]
        #center van het wiel berekenen
        moments = cv.moments(wheel)
        center = (int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"]))
        #cirkel rond contour tekenen en straal berekenen (tweede check)
        ((x, y), radius) = cv.minEnclosingCircle(wheel)

        return center, radius





#zal de diepte van een gekozen pixel geven(dus het center van een wiel)
def getdepthpixel(depth_image, pixel):
    depth = depth_image[pixel[1], pixel[0]]

    return depth

def showwheelpixel(color, pixel, depth, x, y,z, radius):
    #als er een pixel(center van een wiel) bepaalt is die coordinaten tonen
    radius = int(radius)
    if pixel:
        cv.circle(color, pixel, 5, (0, 0, 255), -1)
        cv.circle(color, pixel, radius, (255, 0, 0), 5)
    center_string = ''.join(str(pixel))
    #object te dicht geef waarschuwing
    if depth == 0:
        depth_string = 'to close'
    else:
        depth_string = str(depth)
    #pixelcoördinaten projecteren op het scherm
    cv.putText(color, 'pixelco:'+center_string, (10, 300), cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 2, cv.LINE_AA)
    cv.putText(color, 'depth:' + depth_string, (10, 400), cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 2, cv.LINE_AA)
    #ruimtecoördinaten projecteren op het scherm
    if depth:
        if x or y or z:
            cv.putText(color, "X: " + str(int(round(x)))+' mm', (10, 500), cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0),2, cv.LINE_AA)
            cv.putText(color, "Y: " + str(int(round(y)))+' mm', (10, 600), cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0),2, cv.LINE_AA)
            cv.putText(color, "Z: " + str(int(round(z)))+' mm', (10, 700), cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0),2, cv.LINE_AA)
    return color

def setwindow(windowname,width, height):
    #vensternaam instellen
    cv.namedWindow(windowname, cv.WINDOW_NORMAL)
    #venstergrootte instellen
    cv.resizeWindow(windowname, width, height)

def showimage(img, windowname):
    cv.imshow(windowname,img)

def intrinsictrans(pixel,z,mtx):
    if(z):
        x=(pixel[0]-mtx[0,2])/mtx[0,0]*z
        y=((pixel[1]-mtx[1,2])/mtx[1,1]*z)
        return x,y,z
    else:
        return None,None,None

def extrinsictrans(depth, x,y,z,ext):
    if(depth):
        mat = np.array([[x],[y],[z],[1]])

        inv = np.linalg.inv(ext)
        world = np.dot(inv, mat)
        xw, yw, zw = world[0,0], world[1,0], world[2,0],
        newx = yw
        newy = xw
        newz = -zw
        return newx, newy, newz
    else:
        return None, None, None

#teken de assen van het referentiestelsel
def drawaxes(img,mtx,dist,rvecs,tvecs,length):
    #vectors van de assen volgens 3D
    axis = np.float32([[0,0,0],[length, 0, 0], [0, length, 0], [0, 0, -length]]).reshape(-1, 3)
    #deze punten afbeelden volgens intrinsieke en extrinsieke eigenschappen op het 2d beeld
    imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, mtx, dist)

    #projectie van de lijnen
    corner = tuple(imgpts[0].ravel())
    img = cv.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    img = cv.line(img, corner, tuple(imgpts[3].ravel()), (255, 0, 0), 5)
    return img

def multiply_transforms(trans_A, trans_B):
    return np.dot(trans_A, trans_B)

def invert_transform(transform):
    return np.linalg.inv(transform)




