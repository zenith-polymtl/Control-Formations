from zenmav.core import *

mav = Zenmav(ip = 'tcp:127.0.0.1:5763')

mav.set_mode('GUIDED')
mav.arm()
mav.takeoff(altitude = 10)

#*** ATTENTION ***
# Ne jamais armer le drone dans une situation réelle
# On laisse tourjours le pilote arm le drone
#En simulant vos scripts destoinés au vol, vous pouvez armer le drone via Mission Planner

