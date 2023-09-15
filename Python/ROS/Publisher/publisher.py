###Code inspired from github.com/Aayli
#!/usr/bin/python3
import rospy
import math
from dv_interfaces.msg import Control

RIDE_DURATION_s = 10
MAX_DISTANCE_m = 10
PUBLIC_SEND_PERIOD_s = 0.02
MAX_SPEED = 3

def drive_straight():
    time = 0
    mov = 1
    distance = 0

    while not rospy.is_shutdown() and time < RIDE_DURATION_s and distance < MAX_DISTANCE_m:
        mov = MAX_SPEED * abs(math.sin(time))
        
        msg = Control(move_type = Control.SPEED_KMH, movement=mov, steeringAngle_rad=0.0)
        pub.publish(msg)

        distance += mov * PUBLIC_SEND_PERIOD_s

        time += PUBLIC_SEND_PERIOD_s
        rospy.sleep(PUBLIC_SEND_PERIOD_s)
        print(f"{distance}\t{mov}")

    msg = Control(move_type = 1, movement=0, steeringAngle_rad=0, serviceBrake=255, finished=True)
    pub.publish(msg)


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('dv_pid_regulator_request', anonymous=True)
    pub = rospy.Publisher('/dv_board/control', Control, queue_size=10)

    print("Gooo!!")
    drive_straight()
