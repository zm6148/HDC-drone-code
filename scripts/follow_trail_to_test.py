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
from gpiozero import Servo
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
from opencv.lib_aruco_pose import *

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default = '')
args = parser.parse_args()
    
#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------    

def open_clamp():
    
    from gpiozero import Servo
    from time import sleep
    
    myGPIO=17

    # Min and Max pulse widths converted into milliseconds
    # To increase range of movement:
    #   increase maxPW from default of 2.0
    #   decrease minPW from default of 1.0
    # Change myCorrection using increments of 0.05 and
    # check the value works with your servo.
    myCorrection=0.45
    maxPW=(2.0+myCorrection)/1000
    minPW=(1.0-myCorrection)/1000

    myServo = Servo(myGPIO,min_pulse_width=minPW,max_pulse_width=maxPW)

    myServo.mid()
    print("Set to middle position")
    sleep(1)
    
def close_clamp():
    
    from gpiozero import Servo
    from time import sleep
    
    myGPIO=17

    # Min and Max pulse widths converted into milliseconds
    # To increase range of movement:
    #   increase maxPW from default of 2.0
    #   decrease minPW from default of 1.0
    # Change myCorrection using increments of 0.05 and
    # check the value works with your servo.
    myCorrection=0.45
    maxPW=(2.0+myCorrection)/1000
    minPW=(1.0-myCorrection)/1000

    myServo = Servo(myGPIO,min_pulse_width=minPW,max_pulse_width=maxPW)


    myServo.max()
    print("Set to max position")
    sleep(1)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
        
def distance_from_lat_lon(lat1, lat2, lon1, lon2):
    # calcualte distance based on lat and lon
    dlon = abs(math.radians(lon2)  - math.radians(lon1))
    dlat = abs(math.radians(lat2)  - math.radians(lat1))
    a = (math.sin(dlat/2))**2 + math.cos(lat1) * math.cos(lat2) * (math.sin(dlon/2))**2 
    c = 2 * math.atan2( math.sqrt(a), math.sqrt(1-a) ) 
    earth_radius=6378137.0 #Radius of "spherical" earth
    d = earth_radius * c
    return d

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt and `is_relative` values
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    
    print ("dlat, dlon", dLat, dLon)

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return(newlat, newlon)

def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)
    
def camera_to_uav(x_cam, y_cam):
    x_uav =- (y_cam + 15)
    y_uav = x_cam
    return(x_uav, y_uav)
    
def uav_to_ne(x_uav, y_uav, yaw_rad):
    c       = math.cos(yaw_rad)
    s       = math.sin(yaw_rad)
    
    north   = x_uav*c - y_uav*s
    east    = x_uav*s + y_uav*c 
    return(north, east)
    
def check_angle_descend(angle_x, angle_y, angle_desc):
    return(math.sqrt(angle_x**2 + angle_y**2) <= angle_desc)
       
# make sure clamp open
open_clamp()

#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------  
#-- Connect to the vehicle
print('Connecting...')

connection_string = "/dev/ttyS0"
baud_rate = 921600
vehicle = connect(connection_string, baud = baud_rate, wait_ready=True)  

print("connected")

#--------------------------------------------------
#-------------- PARAMETERS  
#-------------------------------------------------- 
rad_2_deg   = 180.0/math.pi
deg_2_rad   = 1.0/rad_2_deg 

#--------------------------------------------------
#-------------- LANDING MARKER  
#--------------------------------------------------    
#--- Define Tag
marker_number_start      = 74
marker_number_land       = 76
marker_size     = 10 #- [cm]
freq_send       = 1 #- Hz

land_alt_cm         = 50.0
angle_descend       = 20*deg_2_rad
land_speed_cms      = 30.0

gnd_speed = 8 # [m/s]
radius    = 80
max_lat_speed = 4
k_err_vel   = 0.2
n_turns     = 3
direction   = 1 # 1 for cw, -1 ccw

#--------------------------------------------------
#--- Get the camera calibration path
# Find full directory path of this script, used for loading config and other files
#--------------------------------------------------
cwd                 = path.dirname(path.abspath(__file__))
calib_path          = cwd+"/../opencv/"
camera_matrix       = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')    
                                           
time_0 = time.time()

#--------------------------------------------------
# take off and set ot Guided
#--------------------------------------------------
arm_and_takeoff(1.5)

#--------------------------------------------------
# follow marker trail
#--------------------------------------------------
while True:    
    
    aruco_tracker = ArucoSingleTracker(id_to_find=marker_number_start, marker_size=marker_size, show_video=False, 
                camera_matrix=camera_matrix, camera_distortion=camera_distortion)            

    marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
    
    if marker_found:
        x_cm, y_cm          = camera_to_uav(x_cm, y_cm)
        uav_location        = vehicle.location.global_relative_frame
        
        #-- If high altitude, use baro rather than visual
        if uav_location.alt >= 5.0:
            print 
            z_cm = uav_location.alt*100.0
            
        angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)

        # update every 1 s
        if time.time() >= time_0 + 1.0/freq_send:
            time_0 = time.time()
            # print ""
            print " "
            print "Altitude = %.0fcm"%z_cm
            print "Marker found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f"%(x_cm, y_cm, angle_x*rad_2_deg, angle_y*rad_2_deg)
            
            north, east             = uav_to_ne(x_cm, y_cm, vehicle.attitude.yaw)
            
            print "Marker N = %5.0f cm   E = %5.0f cm   Yaw = %.0f deg"%(north, east, vehicle.attitude.yaw*rad_2_deg)
            
            marker_lat, marker_lon  = get_location_metres(uav_location, north*0.01, east*0.01)  
            
            # go to this marker
            location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)
            
            # check whether go to next marker
            distance_to_target = distance_from_lat_lon(marker_lat, uav_location.lat, marker_lon, uav_location.lon)
            
            if  distance_to_target < 0.1 and marker_number_start < marker_number_land :
                
                print ("at marker, move to the next")
                marker_number_start = mmarker_number_start + 1
                
            else:
                
                vehicle.simple_goto(location_marker)
                print "going to marker %.0fcm"%marker_number
                print "UAV Location    Lat = %.7f  Lon = %.7f"%(uav_location.lat, uav_location.lon)
                print "Commanding to   Lat = %.7f  Lon = %.7f"%(location_marker.lat, location_marker.lon)
                
            
            if marker_number_start == marker_number_land :
                
                print ("at landing marker, prepare for landing")
                
                #-- If angle is good, descend
                if check_angle_descend(angle_x, angle_y, angle_descend):
                    print "Low error: descending"
                    location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt-(land_speed_cms*0.01/freq_send))
                else:
                    location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)
                
                vehicle.simple_goto(location_marker)
                
                
                #--- Command to land
                if z_cm <= land_alt_cm:
                    if vehicle.mode == "GUIDED":
                        print (" -->>COMMANDING TO LAND<<")
                        vehicle.mode = "LAND"
                        break
                        
# close clamp
time.sleep(4)
close_clamp()

# takeoff again and set mode to position
                        


            
