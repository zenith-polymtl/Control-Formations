from zenmav.core import Zenmav
import time

drone = Zenmav(ip = 'tcp:127.0.0.1:5763')

while True:
    local_pos = drone.get_local_pos()
    global_pos = drone.get_global_pos()
    print("Local Position: ", local_pos.coordinates," / Global Position: ", global_pos.coordinates)
    time.sleep(0.1)

