from zenmav.core import *

drone = Zenmav(ip = 'tcp:127.0.0.1:5763')

radius = drone.get_param('WPNAV_RADIUS')
drone.set_param('WPNAV_RADIUS', radius*1.2)
answer = input('enter y to download all parameters : ').lower()
if answer == 'y':
    drone.download_all_params('my_first_params.param')

global_pos = drone.get_global_pos(heading= True)
lat = global_pos.lat
lon = global_pos.lon
alt = global_pos.alt
hdg = global_pos.hdg

print('Global position : ')
print(lat, lon, alt, hdg)

local_pos = drone.get_local_pos()
north = local_pos.N
east = local_pos.E
down = local_pos.D

print('Local position : ')
print(north, east, down)




