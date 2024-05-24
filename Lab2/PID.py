#Lisa Godwin 2437980
#Ashlea Smith 2455744
#Brendan Griffiths 2426285
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
import math as m
import sys


def get_state():
    rospy.wait_for_service("/gazebo/get_model_state")
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model_state = get_model_state("quadrotor", "")
        coordinates = model_state.pose.position
        rotation = model_state.pose.orientation
        return (coordinates, rotation)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None


class Vec3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)


def PID():
    if len(sys.argv) != 4:
        return

    takeoff_flag = True
    land_flag = False
    travel_flag = False

    target = Vec3(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))

    rospy.init_node('drone_controller', anonymous=True)
    takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
    land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    Kp = 3
    Ki = 0.0001 # really really small because its stable at 0
    Kd = 0.0001 # really really small because stable at 0 

    rate = rospy.Rate(10)

    sum_error = Vec3(0, 0, 0)
    prev_error = Vec3(0, 0, 0)

    current_tick = 0

    while not rospy.is_shutdown():
        current_tick += 1
        if takeoff_flag:
            takeoff_pub.publish(Empty())

            if current_tick >= 30:
                takeoff_flag = False
                travel_flag = True

        if travel_flag:
            coords, rotation = get_state()
            current = Vec3(coords.x, coords.y, coords.z)
            error = target - current

            rospy.loginfo(
                "Pos x = %f, y = %f, z = %f",
                current.x,
                current.y,
                current.z
            )
            rospy.loginfo(
                "Error x = %f, y = %f, z = %f",
                error.x,
                error.y,
                error.z
            )
            # print(f"error x: {error.x} y: {error.y} z: {error.z}")


            sum_error += error
            diff_error = error - prev_error
            prev_error = error

            vel = Vec3(
                Kp * error.x + Ki * sum_error.x + Kd * diff_error.x,
                Kp * error.y + Ki * sum_error.y + Kd * diff_error.y,
                Kp * error.z + Ki * sum_error.z + Kd * diff_error.z
            )

            # adjusting height, but don't really care if we are in the range of height because we land immediately after
            if m.sqrt(error.x**2 + error.y**2) < 0.5 and m.sqrt(vel.x**2 + vel.y**2) < 0.1:
                travel_flag = False
                land_flag = True


            # x -> roll
            # y -> pitch
            # z -> yaw 
            euler = Vec3(
                m.atan2(2 * ( rotation.w * rotation.x + rotation.y * rotation.z ), 1 - 2 * ( rotation.x**2 + rotation.y**2 ) ),
                -m.pi / 2 + 2 * m.atan2( m.sqrt( 1 + 2 * ( rotation.w * rotation.y + rotation.x * rotation.z ) ), m.sqrt( 1 - 2 * ( rotation.w * rotation.y + rotation.x * rotation.z ) ) ),
                m.atan2( 2 * ( rotation.w * rotation.z + rotation.x * rotation.y ), 1 - 2 * ( rotation.y**2 + rotation.z**2 ) )
            )

            vel.x = vel.x * m.cos(euler.z) * m.cos(euler.y)
            vel.y = vel.y * m.sin(euler.z) * m.cos(euler.y)
            vel.z = vel.z * m.sin(euler.y)

            msg = Twist()
            msg.linear.x = vel.x
            msg.linear.y = vel.y
            msg.linear.z = vel.z
            rospy.loginfo("Angle : x = %f, y = %f, z = %f", euler.x * 180 / m.pi, euler.y * 180 / m.pi, euler.z * 180 / m.pi)
            rospy.loginfo("Velocity: x = %f, y = %f, z = %f", vel.x, vel.y, vel.z)
            vel_pub.publish(msg)

        if land_flag:
            land_pub.publish(Empty())
            land_flag = False
            break


        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('drone_controller', anonymous=True)
        PID()
    except rospy.ROSInterruptException:
        pass