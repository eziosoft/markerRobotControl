import math
import time
from simple_pid import PID

wp_radius = 8
# if the heading to target is grater than this value robot will rotate in place
max_forward_heading = 45

pid_left_right = PID(1, 0, 0, 0)  # PID left right
pid_speed = PID(1, 0, 0, 0)  # PID Forward Backward

robot_distance_to_target = 0

start_time = 0

ch1 = 100
ch2 = 100


def distance(robot, target):
    return math.sqrt(math.fabs(robot[0] - target[0]) ** 2 + math.fabs(robot[1] - target[1]) ** 2)


# returns ch1, ch2 for driving the robot and if the WP is reached, rx - robot x position, tx- target x position, rh -robot hading
def navigate(rx, ry, rh, tx, ty):
    global start_time
    global robot_distance_to_target
    global ch1
    global ch2

    current_time = time. time()
    elapsed_time = current_time - start_time
    start_time = time.time()

    robot_heading = rh
    robot_distance_to_target = distance((rx, ry), (tx, ty))

    if robot_distance_to_target > wp_radius:  # waypoint reached then stop
        reached = False
        robot_heading_to_target = math.degrees(math.atan2(
            (tx - rx), (ty - ry)))

        
        heading_difference = robot_heading - robot_heading_to_target
        a=0
        if heading_difference>180 or heading_difference<-180:
            heading_difference = -heading_difference
            a=1

        end_heading = robot_heading + heading_difference

        # print("rh=%d rt=%d rd=%d eh=%d %d" % (robot_heading, robot_heading_to_target, heading_difference, end_heading, a))

        # tuning depends on the robot configuration, vision delay, etc
        pid_left_right.tunings = (0.01, 0.001, 0.0001)
        if math.fabs(robot_heading_to_target - robot_heading) > 45:  # I-term anti windup
            pid_left_right.Ki = 0
        # tuning depends on the robot configuration, vision delay, etc
        pid_left_right.output_limits = (-0.4, 0.4)
        pid_left_right.sample_time = elapsed_time  # update every 0.01 seconds
        pid_left_right.setpoint = 0
        pid_stear_out = pid_left_right(heading_difference)

        # depends on the robot configuration
        pid_speed.tunings = (0.007, 0.0005, 0)
        if robot_distance_to_target > 50:  # I-term anti windup
            pid_speed.Ki = 0
        # depends on the robot configuration
        pid_speed.output_limits = (-0.2, 0.2)
        pid_speed.sample_time = elapsed_time  # update every 0.01 seconds
        pid_speed.setpoint = 0
        pid_speed_out = pid_speed(robot_distance_to_target)

        # print("T=%.3f   d=%d  hR=%d  hT=%d   stear=%.3f  speed=%.3f  a=%d" % (elapsed_time, robot_distance_to_target,
        #       robot_heading, robot_heading_to_target, pid_stear_out, pid_speed_out, ( robot_heading-robot_heading_to_target)))

        # if robot_heading-robot_heading_to_target  > 180:
        #     pid_stear_out = -pid_stear_out
        # else:
        #     pid_stear_out = pid_stear_out

        ch1 = pid_stear_out * 100 + 100  # steering

        # # don't drive forward until error in heading is less than max_forward_heading
        if math.fabs(robot_heading_to_target - robot_heading) < (max_forward_heading):
            ch2 = -pid_speed_out * 100 + 100
        else:
            ch2 = 100
    else:
        reached = True
        ch1 = 100
        ch2 = 100

    return (ch1, ch2, reached, robot_distance_to_target)
