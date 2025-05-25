from Zenmav.helper_func import *

mav = pymav(ip = 'tcp:127.0.0.1:5763')

mav.set_mode('GUIDED')
mav.arm()
mav.takeoff(altitude = 10)


mav.local_target([10,10, -5])
input("Press Enter to continue...")
mav.local_target([-0, 10,  -5])
mav.local_target([50, -30,  -5])
mav.local_target([100, 10, - 5])



mav.rectilinear_scan(rayon_scan=40)
input("Press Enter to return to home...")
mav.RTL()