from zenmav.core import *
from zenmav.zenpoint import wp
drone = Zenmav(ip = 'tcp:127.0.0.1:5763', gps_thresh=1.6)

drone.set_mode('GUIDED')


drone.arm()
drone.takeoff(altitude = 10)



drone.local_target((50, 60, -10))

home = drone.home

height  = input('Enter height above home')
drone.global_target([home.lat, home.lon, home.alt + height])

local_point = wp(50, 60, -10, frame = "local")

drone.global_target(local_point)

input("Press Enter to scan...")
drone.rectilinear_scan(rayon_scan=40)
input("Press Enter to return to home...")
drone.RTL()