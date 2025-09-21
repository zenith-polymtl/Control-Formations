from zenmav.core import Zenmav
from zenmav.zenpoint import wp
drone = Zenmav(ip = 'tcp:127.0.0.1:5763', gps_thresh=1.6)

drone.set_mode('GUIDED')


drone.arm()
drone.takeoff(altitude = 10)



drone.local_target((50, 60, -10))

home = drone.home
print(f"Home frame : {home.frame}")

height  = float(input('Enter height above home : '))
drone.global_target([home.lat, home.lon, height])

local_point = wp(25, 30, -10, frame = "local")

drone.local_target(local_point)

local_point.show()
print(local_point.coordinates)

input("Press Enter to scan...")
drone.rectilinear_scan(scan_radius=40, center = local_point.coordinates)
drone.orbit(center = local_point, radius = 40, speed = 5)
input("Press Enter to return to home...")
drone.RTL()