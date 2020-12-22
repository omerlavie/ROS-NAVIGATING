#!/usr/bin/env python

import math
import sys
import roslib; roslib.load_manifest('bugs')
import rospy
import tf.transformations as transform
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from location import Location, necessary_heading
from dist import Dist


current_location = Location()
current_dists = Dist()
delta = .1
WALL_PADDING = 0.75

STRAIGHT = 0
LEFT = 1
RIGHT = 2
MSG_STOP = 3


def init_listener():
    rospy.Subscriber('odom', Odometry, location_callback)
    rospy.Subscriber('scan', LaserScan, sensor_callback)


def location_callback(data):
    p = data.pose.pose.position
    q = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
    t = transform.euler_from_quaternion(q)[2] # in [-pi, pi]
    current_location.update_location(p.x, p.y, t)


def sensor_callback(data):
    current_dists.update(data)


class Bug:
    def __init__(self, tx, ty, forward_speed, spin_speed):
        self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.tx = tx
        self.ty = ty
        self.forward_speed = forward_speed
        self.spin_speed = spin_speed
        rospy.sleep(1)
        self.m, self.b = self.get_m_line(0, 0, tx, ty)

    def go(self, direction):
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = self.forward_speed
        elif direction == LEFT:
            cmd.angular.z = self.spin_speed
        elif direction == RIGHT:
            cmd.angular.z = -self.spin_speed
        elif direction == MSG_STOP:
            pass
        # print(cmd)
        self.pub.publish(cmd)

    def go_until_obstacle(self):
        print("Going until destination or obstacle")
        while current_location.distance(self.tx, self.ty) > delta:
            (front_dist, _) = current_dists.get()
            if front_dist <= WALL_PADDING:
                # detected obstacle
                return True
            current_heading = self.convert_to_360d(current_location.current_location()[2])
            heading_to_target = self.convert_to_360d(necessary_heading(current_location.current_location()[0],
                                                                       current_location.current_location()[1],
                                                                       self.tx, self.ty))
            need_to_turn_d = current_heading - heading_to_target
            print("neceseery to turn: " + str(need_to_turn_d))
            if abs(need_to_turn_d) > 10:
                self.spin_degrees(need_to_turn_d)
                rospy.sleep(.1)
            else:
                self.go(STRAIGHT)
                rospy.sleep(.1)
        return False

    def follow_wall(self):
        twist = Twist()
        twist.angular.z = 0.72
        twist.linear.x = 0.25
        while not self.should_leave():
            while current_dists.get()[0] <= WALL_PADDING + .25:
                # print("aligning with wall")
                self.go(RIGHT)
                rospy.sleep(.5)
            self.spin_degrees(90)
            rospy.sleep(.5)
            while current_dists.get()[0] > WALL_PADDING + .25 and not self.should_leave():
                self.pub.publish(twist)
                rospy.sleep(1)

    def should_leave(self):
        (cx, cy, _) = current_location.current_location()
        print("current x is: " + str(cx) + " and current y is: " + str(cy))
        if None in self.encountered_wall_at:
            self.encountered_wall_at = (cx, cy)
            print("==== updated to encountered wall at: ", self.encountered_wall_at)
            return False
        (ox, oy) = self.encountered_wall_at
        if self.on_m_line(cx, cy, self.m, self.b) and not near(cx, cy, ox, oy):
            return True
        return False

    def get_m_line(self, x1, y1, x2, y2):
        print(x1,y1,x2,y2)
        delta_x = x2 - x1
        delta_y = y2 - y1
        if x1 == x2:
            m = float('inf')
            b = x1
        else:
            m = delta_y / delta_x
            b = y1 - m * x1
        print("\n")
        print("======================= m is: " + str(m) + " and b is: " + str(b) + " =======================")
        print("\n")

        return m, b

    def on_m_line(self, x, y, m, b):
        if m == float('inf'):
            if math.fabs(x - self.tx) < .3:
                return True
            else:
                return False
        else:
            calculated_y = m * x + b
            if math.fabs(calculated_y - y) < .3:
                return True
            else:
                return False

    def should_leave_wall(self):
        sys.exit(1)

    def sign(self, number):
        if number < 0:
            return -1
        if number > 0:
            return 1
        else:
            return 0

    def convert_to_ros_z(self, degrees):
        if degrees > 180:
            degrees = -180 + (degrees % 180)
        elif degrees < -180:
            degrees = 180 - (degrees % 180)
        return degrees

    def convert_to_360d(self, z):
        degrees = math.degrees(z)
        if self.sign(degrees) == -1:
            degrees = 360 + degrees
        return degrees

    def crossed_zero(self, h_old, h_new, direction):
        if direction == "left":
            if h_new < h_old:
                return True
        elif direction == "right":
            if h_new > h_old:
                return True
        return False

    def spin_degrees(self, degrees):
        dirct = None
        crossed_zero = False
        if self.sign(degrees) == -1:
            dirct = "left"
        else:
            dirct = "right"
        # print("Turning " + str(degrees) + " degrees to the " + dirct)
        twist = Twist()
        twist.angular.z = -1.2*self.spin_speed * self.sign(degrees)
        (_, _, starting_heading) = current_location.current_location()
        starting_heading = self.convert_to_360d(starting_heading)
        desired_heading = (starting_heading - degrees) % 360
        # print("starting heading is: " + str(starting_heading))
        # print("desired heading is: " + str(desired_heading))
        last_heading = current_heading = self.convert_to_360d(current_location.current_location()[2])
        enter_anyway = False
        # If I'm walking left.
        if self.sign(degrees) == -1:
            # If I need to cross 0.
            if starting_heading > desired_heading:
                # It means I ned to cross 0.
                while not self.crossed_zero(last_heading, current_heading, "left"):
                    # print("1", self.convert_to_360d(current_location.current_location()[2]))
                    self.pub.publish(twist)
                    rospy.sleep(.25)
                    last_heading = current_heading
                    current_heading = self.convert_to_360d(current_location.current_location()[2])
                    enter_anyway = False
                    # If the scanning data was bad.
                    if last_heading > current_heading:
                        enter_anyway = True
                crossed_zero = True
                enter_anyway = False
            while self.convert_to_360d(current_location.current_location()[2]) <= desired_heading:
                if crossed_zero and self.convert_to_360d(current_location.current_location()[2]) > desired_heading:
                    break
                # print("2", self.convert_to_360d(current_location.current_location()[2]))
                self.pub.publish(twist)
                rospy.sleep(.25)
                last_heading = current_heading
                current_heading = self.convert_to_360d(current_location.current_location()[2])

        # If I'm walking right.
        elif self.sign(degrees) == 1:
            # It means I need to cross 0.
            if starting_heading < desired_heading:
                while not self.crossed_zero(last_heading, current_heading, "right"):
                    # print("3", self.convert_to_360d(current_location.current_location()[2]))
                    self.pub.publish(twist)
                    rospy.sleep(.25)
                    last_heading = current_heading
                    current_heading = self.convert_to_360d(current_location.current_location()[2])
                    # print("turning right headings: (last, current)", last_heading, current_heading)
                crossed_zero = True
            while self.convert_to_360d(current_location.current_location()[2]) >= desired_heading:
                if crossed_zero and self.convert_to_360d(current_location.current_location()[2]) < desired_heading:
                    break
                # print("4", self.convert_to_360d(current_location.current_location()[2]))
                self.pub.publish(twist)
                rospy.sleep(.25)
                last_heading = current_heading
                current_heading = self.convert_to_360d(current_location.current_location()[2])
        # print("finished turning " + str(degrees) + " degrees")
        print('\n')


class Bug2(Bug):
    def __init__(self, tx, ty, forward_speed, spin_speed):
        Bug.__init__(self, tx, ty, forward_speed, spin_speed)
        self.lh = None
        self.encountered_wall_at = (None, None)

    def face_goal(self):
        while not current_location.facing_point(self.tx, self.ty):
            self.go(RIGHT)
            rospy.sleep(.01)

    def follow_wall(self):
        Bug.follow_wall(self)
        self.face_goal()



def near(cx, cy, x, y):
    nearx = x - .6 <= cx <= x + .6
    neary = y - .6 <= cy <= y + .6
    return nearx and neary


def bug_algorithm(bug):
    init_listener()
    print("Calibrating sensors...")
    # This actually just lets the sensor readings propagate into the system
    rospy.sleep(1)
    print("Calibrated")

    while current_location.distance(bug.tx, bug.ty) > delta:
        print(current_location.distance(bug.tx, bug.ty))
        hit_wall = bug.go_until_obstacle()
        print(hit_wall)
        if hit_wall:
            bug.follow_wall()
    print("Arrived at", (bug.tx, bug.ty))


class Bug2node:
    def __init__(self, tx, ty, forward_speed=1, spin_speed=.25):
        self.bug2 = Bug2(tx, ty, forward_speed, spin_speed)

    def start(self):
        bug_algorithm(self.bug2)
