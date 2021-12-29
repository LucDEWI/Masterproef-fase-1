import urx
from IPython import embed
import logging

#Dit script script zal proberen een connectie te maken met de robot en zal een confirmatie geven als het een verbinding kan maken.
#Indien dit niet lukt zal er niets gemeld worden. Bekijk de netwerkinstellingen van de ethernetpoort van de pc

if __name__ == "__main__":
    try:
        logging.basicConfig(level=logging.INFO)
        #robot = urx.Robot("192.168.1.6")
        robot = urx.Robot("192.168.1.100")
        #robot = urx.Robot("localhost")
        r = robot
        print("Robot object is available as robot or r")
        embed()
    finally:
        robot.close()