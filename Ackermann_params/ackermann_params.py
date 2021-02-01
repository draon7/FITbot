import math
import matplotlib.pyplot as plt 
import numpy as np

## Calculates ideal ackermann angles
#  @param wb wheelbase
#  @param track track
#  @param fw_to_joint forward wheel to joint distance
#  @param id_w_angle angle of the ideal wheel situated in the center of the front axel determining turning radius
#  @return tuple of left and right angle of the wheel to vehicle main axis
def ideal_ackermann_angles(wb, track, fw_to_joint, id_w_angle):
    # we say that vehicle is turning right
    # pp = pivot point, tc = track center, lfw = left fornt wheel, rfw = right front wheel
    tc_to_pp = math.tan(math.radians(90.0 - id_w_angle)) * wb 
    tc_to_joint = (track/2 - fw_to_joint)
    pp_to_lfw_joint = tc_to_pp + tc_to_joint
    pp_to_rfw_joint = tc_to_pp - tc_to_joint
    lfw_angle = 90 - math.degrees(math.atan(pp_to_lfw_joint/wb))
    rfw_angle = 90 - math.degrees(math.atan(pp_to_rfw_joint/wb))

    return lfw_angle, rfw_angle

## Calculates ideal ackermann angle for second wheel
#  steering turns right
#  @param wb wheelbase
#  @param track track
#  @param fw_to_joint forward wheel to joint distance
#  @return left wheel ideal angle, 0 deg means straight
def ideal_ackermann_second_wheel(wb, track, fw_to_joint, rw_angle):
    tc_to_pp = math.tan(math.radians(90 - rw_angle)) * wb + track/2 - fw_to_joint
    tc_to_joint = (track/2 - fw_to_joint)
    pp_to_lfw_joint = tc_to_pp + tc_to_joint
    lfw_angle = 90 - math.degrees(math.atan(pp_to_lfw_joint/wb))
    return lfw_angle

## Calculates rack and pinion wheel angle
#  gets right wheel angle from left wheel angle
#  @param wb wheelbase
#  @param track track
#  @param fw_to_joint forward wheel to joint distance
#  @param wj_angle wheeljoint angle
#  @param wjlen lenght of wheeljoint part
#  @param racklen lenght of the rack
#  @param jointlen lenght of the joint 
def rack_pinion_angles(wb, track, fw_to_joint, wj_angle, wjlen, racklen, jointlen):
    return


wb = 200.0
track = 100.0
fw_to_joint = 15.0

angles = np.arange(0, 50)

lw_angles = np.array([])

for angle in angles:
    lw_angle = ideal_ackermann_second_wheel(wb, track, fw_to_joint, angle)
    lw_angles = np.append(lw_angles, lw_angle)

plt.plot(lw_angles, angles)
plt.show()
