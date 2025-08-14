from Zenmav.core import *

mav = pymav(ip = 'tcp:127.0.0.1:5763', gps_thresh=1.6)

mav.set_mode('GUIDED')

if 0:
    mav.arm()
    mav.takeoff(altitude = 10)

i = 0
while i<8000 :
    target = mav.convert_to_global((100,50))
    target.append(10)  # Altitude in meters
    print(f"Target: {target}")
    mav.global_target(target )
    i += 1


input("Press Enter to scan...")
mav.rectilinear_scan(rayon_scan=40)
input("Press Enter to return to home...")
mav.RTL()