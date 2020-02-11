from gpiozero import Servo
from time import sleep

myGPIO=17

myServo = Servo(myGPIO)

print("Using GPIO17")
print("Using Gpiozero defaults for the servo class")

while True:
  myServo.mid()
  print("Set to middle position")
  sleep(1)
  myServo.min()
  print("Set to minimum position")
  sleep(1)
  myServo.mid()
  print("Set to middle position")
  sleep(1)
  myServo.max()
  print("Set to maximum position")
  sleep(1)
