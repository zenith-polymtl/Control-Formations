from zenmav.core import *

drone = Zenmav(ip = 'tcp:127.0.0.1:5763')

while True:
    local_pos = drone.get_local_pos()
    global_pos = drone.get_global_pos()
    print("Local Position: ", local_pos," / Global Position: ", global_pos)
    time.sleep(0.1)

