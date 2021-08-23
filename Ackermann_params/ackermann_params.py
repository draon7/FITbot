# Autor: Deingruber Ondrej
import math
import matplotlib.pyplot as plt 
import numpy as np
from scipy.optimize import minimize, shgo, brute, NonlinearConstraint, BFGS
import sys
from multiprocessing import Pool

# Pro spusteni nastavte spravne prepinace, 
# graph - zobrazuje graficke vysledky
# test - prochazi ruzne geometrie (pro testovani)
# optimize - spusti optimalizaci s parametry params
# get_wheel_angles - spocita pozice hrebene pro natoceni virtualniho stredniho kola (pro ovladani) v pripade ze se
#   neziskaji nove parametry pomoci optimalizace je spusteno s parametry input_wj_angle, input_wjlen, input_wjlink

GRAPH = True
TEST = False
OPTIMIZE = False
GET_WHEEL_ANGLES = True

# method
SLSQP = False
SHGO = True
BRUTE = False

# params
wb = 160.0
track = 170
fw_to_joint = 20.0
rack_dist = 30.0
max_angle = 45.0
step = 0.5

# input for rack movement to archieve steering angle - for run without optimize
input_wj_angle = 114.95055602
input_wjlen = 62.82669785
input_wjlink = 32.91865791

#-----------------------------------------------------------------------------------------------------------------------

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

## Calculates rack lenght for given parameters
#  gets right wheel angle from left wheel angle
#  @param track track of the vehicle
#  @param fw_to_joint forward wheel to joint distance
#  @param wj_angle wheeljoint angle
#  @param wjlen lenght of wheeljoint part
#  @param wjlink lenght of the link between wheeljoint part and steering rack
#  @param racklen lenght of the rack
def rack_len(track, fw_to_joint, wj_angle, wjlen, wjlink, rack_dist):
    a = math.cos(math.radians(180 - wj_angle)) * wjlen
    ad = math.sqrt(wjlen**2 - a**2)
    bd = rack_dist - ad
    b = math.sqrt(wjlink**2 - bd**2)
    racklen = track - (2 * a) - (2 * b) - (2 * fw_to_joint)
    return racklen

# calculates second wheel for rack and pinion steering with given parameters
def rack_pinion_second_wheel(track, fw_to_joint, wj_angle, wjlen, wjlink, rack_dist, racklen, rw_angle):
    a = math.cos(math.radians(180 - wj_angle - rw_angle)) * wjlen
    if (180 - wj_angle - rw_angle) < 0 :    # angle over 180, change direction
        a = -a
    ad = math.sqrt(wjlen**2 - a**2)
    bd = rack_dist - ad
    b = math.sqrt(wjlink**2 - bd**2)
    cd = track - 2*fw_to_joint - a - b - racklen
    joint_rack = math.sqrt(cd**2 + rack_dist**2)
    if (cd == 0):
        beta = 90
    else:
        beta = math.degrees(math.atan(rack_dist/cd))
    if wjlink + wjlen == joint_rack or abs(wjlink - wjlen) == joint_rack:
        gamma = 0
    else:
        gamma = math.degrees(math.acos((wjlink**2 - joint_rack**2 - wjlen**2)/(-2*joint_rack*wjlen)))
    second_wheel = wj_angle + beta + gamma - 180
    return second_wheel

# calculates position of rack for given wheel angle
def rack_pinion_rack_position(track, fw_to_joint, wj_angle, wjlen, wjlink, rack_dist, racklen, rw_angle):
    a = math.cos(math.radians(180 - wj_angle - rw_angle)) * wjlen
    if (180 - wj_angle - rw_angle) < 0 :    # angle over 180, change direction
        a = -a
    ad = math.sqrt(wjlen**2 - a**2)
    bd = rack_dist - ad
    b = math.sqrt(wjlink**2 - bd**2)
    rack_pos = a + b
    return rack_pos

# calculates position of virtual middle wheel angle for two wheel angles and given parameters
def middle_wheel_angle(wb, track, fw_to_joint, lw_angle, rw_angle):
    if (lw_angle == 0 or rw_angle == 0):
        return 0
    joint_dist = track - 2*fw_to_joint
    wj_to_rot_center = (math.cos(math.radians(rw_angle)) * joint_dist * math.sin(math.radians(lw_angle))) / (math.sin(math.radians(rw_angle - lw_angle)))
    middle_wheel = 90 - math.degrees(math.atan((wj_to_rot_center + joint_dist/2)/wb))
    return middle_wheel
    
# error functionm returns square error of angle of second wheel for given geometry, works only for valid geometry
def se_rack_vs_ideal(wb, track, fw_to_joint, wj_angle, wjlen, wjlink, rack_dist, racklen, angle):
    ideal = ideal_ackermann_second_wheel(wb, track, fw_to_joint, angle)
    rack = rack_pinion_second_wheel(track, fw_to_joint, wj_angle, wjlen, wjlink, rack_dist, racklen, angle)
    error = (ideal - rack)**2
    return error

# mean square error for whole rack steering with given max angle and step, works only for valid geometry
def mse_rack_vs_ideal(wb, track, fw_to_joint, wj_angle, wjlen, wjlink, rack_dist, max_angle, step):
    angles = np.arange(0, max_angle, step)
    error = 0.0
    racklen = rack_len(track, fw_to_joint, wj_angle, wjlen, wjlink, rack_dist)
    for angle in angles:
        ideal = ideal_ackermann_second_wheel(wb, track, fw_to_joint, angle)
        rack = rack_pinion_second_wheel(track, fw_to_joint, wj_angle, wjlen, wjlink, rack_dist, racklen, angle)
        error += (ideal - rack)**2

    error /= angles.shape[0]
    return error

# vectorised version of mse calculation
def mse_vectorised_rack_vs_ideal(wb, track, fw_to_joint, wj_angle, wjlen, wjlink, rack_dist, max_angle, step):
    angles = np.arange(0, max_angle, step)
    racklen = rack_len(track, fw_to_joint, wj_angle, wjlen, wjlink, rack_dist)
    mse_vectorised = np.vectorize(se_rack_vs_ideal, excluded = ['wb', 'track', 'fw_to_joint', 'wj_angle', 'wjlen', 'wjlink', 'rack_dist', 'racklen'])
    errors = mse_vectorised(wb, track, fw_to_joint, wj_angle, wjlen, wjlink, rack_dist, racklen, angles)
    return np.average(errors)

## adapter for minimalization funcion
# @param x array of wj_angle, wjlen, wjlink
# @param wb wheelbase
# @param track track of the vehicle
# @param fw_to_joint distance from wheel to joint
# @param max_angle maximum angle for wheel to turn
# @param step steps of the angle to test
# @param rack_dist distance of the rack from the axel
def mse_rack_vs_ideal_minfmt(x, wb, track, fw_to_joint, rack_dist, max_angle, step):
    return (mse_vectorised_rack_vs_ideal(wb, track, fw_to_joint, x[0], x[1], x[2], rack_dist, max_angle, step))

# constrained mse function, works even for non valid geometry, returns max number for error geometry
def mse_rack_vs_ideal_minfmt_constrained(x, wb, track, fw_to_joint, rack_dist, max_angle, step):

    if ((x[1] + x[2]) - (rack_dist/math.cos(math.radians(abs(x[0]-90)))) < 0):
        return sys.float_info.max
    elif (triangle_ineq_ackermann_rack_right(x[0], x[1], x[2], rack_dist, max_angle) < 0):
        return sys.float_info.max
    elif (rack_len(track, fw_to_joint, x[0], x[1], x[2], rack_dist) < 0):
        return sys.float_info.max
    elif (triangle_ineq_ackermann_rack(track, fw_to_joint, x[0], x[1], x[2], rack_dist, max_angle) < 0):
        return sys.float_info.max
    else:
        return (mse_vectorised_rack_vs_ideal(wb, track, fw_to_joint, x[0], x[1], x[2], rack_dist, max_angle, step))

# chack of right side triangle equation for given geometry
def triangle_ineq_ackermann_rack_right(wj_angle, wjlen, wjlink, rack_dist, max_angle):

    a = math.cos(math.radians(180 - wj_angle - 0)) * wjlen
    ad = math.sqrt(wjlen**2 - a**2)
    remaining_dist_straight = abs(rack_dist - ad)

    a = math.cos(math.radians(180 - wj_angle - max_angle)) * wjlen
    if (180 - wj_angle - max_angle) < 0 :    # angle over 180, change direction
        a = -a
    ad = math.sqrt(wjlen**2 - a**2)
    remaining_dist_max = abs(rack_dist - ad)

    max_rem_dist = 0
    min_rem_dist = 0
    if (wj_angle < 90 and wj_angle + max_angle > 90):
        dist = abs(rack_dist - wjlen)
        max_rem_dist = max([remaining_dist_straight, remaining_dist_max, dist])
        min_rem_dist = min([remaining_dist_straight, remaining_dist_max, dist])
    else:
        max_rem_dist = max(remaining_dist_straight, remaining_dist_max)
        min_rem_dist = min(remaining_dist_straight, remaining_dist_max)

    if (max_rem_dist > wjlink):
        return wjlink - max_rem_dist

    return 0


# chack if triangle inequality is allright for left part of the steering according to rightwheel max angle and link sizes
def triangle_ineq_ackermann_rack(track, fw_to_joint, wj_angle, wjlen, wjlink, rack_dist, max_angle):
    racklen = rack_len(track, fw_to_joint, wj_angle, wjlen, wjlink, rack_dist)

    # wjlen and wjlink are in line - change of direction
    angle_list = []
    angle_list.append(math.degrees(math.asin(rack_dist/(wjlen+wjlink))) - wj_angle)
    angle_list.append(180 - (angle_list[-1] + wj_angle) - wj_angle)
    if ((wjlen != wjlink) and (abs(wjlen-wjlink) > rack_dist)):
        angle_list.append(math.degrees(math.asin(rack_dist/(abs(wjlen-wjlink)))) - wj_angle)
        angle_list.append(180 - angle_list[-1])

    for angle in list(angle_list): # iterate on new list
        if (angle > max_angle or angle < 0):
            angle_list.remove(angle)

    angle_list.append(max_angle)
    angle_list.append(0)

    distances_list = []
    left_angle_list = []

    for angle in angle_list:
        a = math.cos(math.radians(180 - wj_angle - angle)) * wjlen
        if (180 - wj_angle - angle) < 0 :    # angle over 180, change direction
            a = -a
        ad = math.sqrt(wjlen**2 - a**2)
        bd = rack_dist - ad
        b = math.sqrt(wjlink**2 - bd**2)
        cd = track - 2*fw_to_joint - a - b - racklen
        left_angle_list.append(math.sqrt(cd**2 + rack_dist**2))
        distances_list.append(a + b + racklen)

    joint_rack_pos_min = min(distances_list)
    joint_rack_pos_max = max(distances_list)

    joint_rack_zero_deg = min(left_angle_list)
    joint_rack_max_deg = max(left_angle_list)

    joint_rack_pos_shortest = track - 2 * fw_to_joint

    shortest = 0
    longest = 0
    if ((joint_rack_pos_min < joint_rack_pos_shortest and joint_rack_pos_max > joint_rack_pos_shortest) or 
        (joint_rack_pos_min > joint_rack_pos_shortest and joint_rack_pos_max < joint_rack_pos_shortest)):
        shortest = rack_dist
        longest = max(joint_rack_max_deg, joint_rack_zero_deg)
    else:
        shortest = min(joint_rack_zero_deg, joint_rack_max_deg)
        longest = max(joint_rack_zero_deg, joint_rack_max_deg)
    
    if (wjlen > wjlink):
        if (wjlen + wjlink < longest or wjlen - wjlink > shortest):
            if wjlen + wjlink < longest:
                return wjlen + wjlink - longest
            else:
                return shortest - (wjlen - wjlink)
    else:
        if (wjlen + wjlink < longest or wjlink - wjlen > shortest):
            if wjlen + wjlink < longest:
                return wjlen + wjlink - longest
            else:
                return shortest - (wjlink - wjlen)
    return 0

# constraints
def constr(x, track, fw_to_joint, rack_dist, max_angle):
    ret = (x[1] + x[2]) - (rack_dist/math.cos(math.radians(abs(x[0]-90))))
    if (ret < 0):
        return ret
    ret = triangle_ineq_ackermann_rack_right(x[0], x[1], x[2], rack_dist, max_angle)
    if (ret < 0):
        return ret
    ret = rack_len(track, fw_to_joint, x[0], x[1], x[2], rack_dist)
    if (ret < 0):
        return ret
    ret = triangle_ineq_ackermann_rack(track, fw_to_joint, x[0], x[1], x[2], rack_dist, max_angle)
    if (ret < 0):
        return ret
    return 0

def main():
    if (OPTIMIZE):
        con = lambda x: constr(x, track, fw_to_joint, rack_dist, max_angle)
        nlc = NonlinearConstraint(con, -track, 0, jac = '2-point', hess = BFGS())
        constraint = ({'type': 'ineq',
                    'fun': lambda x: constr(x, track, fw_to_joint, rack_dist, max_angle)
                    #    'jac': '2-point',
                    #    'hess': BFGS()
                    })
        bnds = ((10.0, 180.0), (5.0, track), (5.0, track))

        if SLSQP:
            print('SLSQP --------------------------------------------------')
            res = minimize(mse_rack_vs_ideal_minfmt,
                np.array([120.0, 30.0, 30.0]),
                args = (wb, track, fw_to_joint, rack_dist, max_angle, step),
                constraints = constraint,
                bounds = bnds,
                options = {'disp': True,
                        'maxiter': 500},
                method = 'SLSQP',
            )
            print('SLSQP result:')
            print(res)

        if SHGO:
            print('Shgo ---------------------------------------------------')
            res2 = shgo(mse_rack_vs_ideal_minfmt_constrained,
                bounds = bnds,
                args = (wb, track, fw_to_joint, rack_dist, max_angle, step),
                constraints = constraint,
                options = {'disp': True},
                iters = 9,
                # minimizer_kwargs={'method':'trust-constr',
                #                   'jac': '3-point',
                #                   'hess': BFGS
                # }
            )
            print('Shgo result:')
            print(res2)
        if BRUTE:
            print('Brute --------------------------------------------------')
            rranges = (slice(10.0, 180.0, 0.2), slice(5.0, track/2, 0.2), slice(5.0, track/2, 0.2))

            res3 = brute(mse_rack_vs_ideal_minfmt_constrained, 
                ranges = rranges,
                args = (wb, track, fw_to_joint, rack_dist, max_angle, step),
                full_output = True,
                workers = 4
            )

            print('Brute result:')
            print(res3)

    # Tests of functions
    if TEST:
        racklen = 0.0
        racklen = rack_len(200.0, 20.0, 100.0, 20.0, 30.0, 40.0)
        print(racklen)
        new_racklen = rack_len(100.0, 15.0, 73.49598963561695, 0.0060509520483643725, 38.713832675579795, 20.0)

        new_racklen2 = rack_len(100.0, 15.0, 31.0, 9.0, 49.5, 30.0)
        print(new_racklen2)
        #rack_pinion_second_wheel(100.0, 15.0, 31.0, 9.0, 49.5, 30.0, new_racklen2, 44.5) bad params

        ret = triangle_ineq_ackermann_rack_right(28.55800, 5.0, 33.302529, 30.0, 45.0)
        print(ret)
        ret = triangle_ineq_ackermann_rack(100.0, 15.0, 28.55800, 5.0, 33.302529, 30.0, 45.0) # 1
        ret = triangle_ineq_ackermann_rack(100.0, 15.0, 34.4, 42.0, 11.4, 30.0, 45.0) # -1
        ret = triangle_ineq_ackermann_rack(100.0, 15.0, 68.80000000000001, 16.200000000000003, 16.0, 30.0, 45.0) # -1
        ret = triangle_ineq_ackermann_rack(170.0, 20.0, 137.5, 25.625, 46.25, 30.0, 45.0) # -1

        rack_pinion_second_wheel(100.0, 15.0, 73.49598963561695, 0.0060509520483643725, 38.713832675579795, 20.0, new_racklen, 0)
        print(new_racklen)
        print(rack_pinion_second_wheel(200.0, 20.0, 100.0, 20.0, 30.0, 40.0, racklen, 23.0)) 

        print (rack_pinion_second_wheel(170.0, 20.0, 90.0, 50.2, 20.200000000000003, 30.0, 130.0, 0.0))
        
        ret = constr([95.00000026519103, 99.99999944188494, 64.37500051917687], track, fw_to_joint, rack_dist, max_angle)
        print('constr test:')
        print(ret)


    # Ploting results
    if GRAPH:
        # prepare angles array
        angles = np.arange(0, max_angle, step)
        # Plot ideal ackermann angles
        lw_angles = np.array([])

        for angle in angles:
            lw_angle = ideal_ackermann_second_wheel(wb, track, fw_to_joint, angle)
            lw_angles = np.append(lw_angles, lw_angle)

        plt.plot(lw_angles, angles)
        plt.show()

        if TEST:
            # Plot MSE of specified geometry
            lw_angles_rack = np.array([])

            for angle in angles:
                racklen = rack_len(track, fw_to_joint, 47.89726272, 5.80392606, 34.63040274, rack_dist)
                lw_angle = rack_pinion_second_wheel(track, fw_to_joint, 47.89726272, 5.80392606, 34.63040274, rack_dist, racklen, angle)
                lw_angles_rack = np.append(lw_angles_rack, lw_angle)

            diff = lw_angles - lw_angles_rack

            plt.plot(angles, diff)
            plt.show()

        if OPTIMIZE:
            # Plot MSE of geometry of optimalization result
            res = res2 # SHGO result 

            lw_angles_rack = np.array([])

            for angle in angles:
                racklen = rack_len(track, fw_to_joint, res.x[0], res.x[1], res.x[2], rack_dist)
                lw_angle = rack_pinion_second_wheel(track, fw_to_joint, res.x[0], res.x[1], res.x[2], rack_dist, racklen, angle)
                lw_angles_rack = np.append(lw_angles_rack, lw_angle)

            diff = lw_angles - lw_angles_rack

            plt.plot(angles, diff)
            plt.show()

    if GET_WHEEL_ANGLES:
        # prepare angles array
        angles = np.arange(0, max_angle, 1)
        global input_wj_angle
        global input_wjlen
        global input_wjlink

        if OPTIMIZE:
            input_wj_angle = res.x[0]
            input_wjlen = res.x[1]
            input_wjlink = res.x[2]

        racklen = rack_len(track, fw_to_joint, input_wj_angle, input_wjlen, input_wjlink, rack_dist)
        
        steering_angles = np.array([])
        rack_positions = np.array([])
        rack_neutral = rack_pinion_rack_position(track, fw_to_joint, input_wj_angle, input_wjlen, input_wjlink, rack_dist, racklen, 0)
        for angle in angles:
            lw_angle = rack_pinion_second_wheel(track, fw_to_joint, input_wj_angle, input_wjlen, input_wjlink, rack_dist, racklen, angle)
            avg_angle = (angle + lw_angle)/2
            middle_angle = middle_wheel_angle(wb, track, fw_to_joint, lw_angle, angle)
            print (avg_angle - middle_angle)
            steering_angles = np.append(steering_angles, middle_angle)
            rack_from_neutral = rack_pinion_rack_position(track, fw_to_joint, input_wj_angle, input_wjlen, input_wjlink, rack_dist, racklen, angle) - rack_neutral
            rack_positions = np.append(rack_positions, rack_from_neutral)

        if GRAPH:
            # plot difference vs ideal
            
            # prepare angles array
            angles = np.arange(0, max_angle, step)
            # Plot ideal ackermann angles
            lw_angles = np.array([])

            for angle in angles:
                lw_angle = ideal_ackermann_second_wheel(wb, track, fw_to_joint, angle)
                lw_angles = np.append(lw_angles, lw_angle)
                lw_angles_rack = np.array([])

            for angle in angles:
                racklen = rack_len(track, fw_to_joint, input_wj_angle, input_wjlen, input_wjlink, rack_dist)
                lw_angle = rack_pinion_second_wheel(track, fw_to_joint, input_wj_angle, input_wjlen, input_wjlink, rack_dist, racklen, angle)
                lw_angles_rack = np.append(lw_angles_rack, lw_angle)

            diff = lw_angles - lw_angles_rack

            plt.plot(angles, diff)
            plt.ylabel('odchylka od ideálního úhlu levého kola [deg]')
            plt.xlabel('úhel pravého kola [deg]')
            plt.show()
            
            # plot rack possitions
            plt.plot(steering_angles, rack_positions)
            plt.show()

        # print results
        print("steering angles:")
        print(steering_angles)

        print("rack position:")
        print(rack_positions)
            

if __name__ == "__main__":
    main()
