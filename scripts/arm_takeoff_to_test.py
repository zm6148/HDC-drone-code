"""

NOTE: be sure to be using the latest dronekit. 
sudo pip uninstall dronekit
sudo pip uninstall pymavlink

cd dronekit-python
git pull

sudo python setup.py build
sudo python setup.py install

Be sure the RASPI CAMERA driver is correctly acivated -> type the following
modprobe bcm2835-v4l2 


"""
from os import sys, path
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import time
import math
import argparse


from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
from opencv.lib_aruco_pose import *

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default = '')
args = parser.parse_args()
    
#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------
#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)
           
#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
#-- Connect to the vehicle
print('Connecting...')

connection_string = "/dev/ttyS0"
baud_rate = 921600
vehicle = connect(connection_string, baud = baud_rate, wait_ready=True)  
print('connected!')

#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
arm_and_takeoff(1.5)

time.sleep(3)

print (" --> Commanding to LAND")
vehicle.mode = "LAND"

