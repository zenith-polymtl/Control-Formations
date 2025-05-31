from Zenmav.core import *

mav = pymav(ip = 'tcp:127.0.0.1:5763')

while True:
    local_pos = mav.get_local_pos()
    global_pos = mav.get_global_pos()
    print("Local Position: ", local_pos," / Global Position: ", global_pos)
    time.sleep(0.1)

