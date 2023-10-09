# position_hold_py
# Importing the required libraries
from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from pid_tune.msg import PidTune
import rospy

class QuadcopterController:
    def __init__(self):
        rospy.init_node('drone_control')  # Initializing ROS node with the name 'drone_control'

        # Current position of the drone [x, y, z]
        self.drone_position = [0.0, 0.0, 0.0]

        # Desired setpoint position [x, y, z]
        self.setpoint = [2.0, 2.0, 2.0]  # Modify as per your desired setpoint

        # Declaring a cmd message of type swift_msgs and initializing values
        self.cmd = swift_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

        # PID gains for [roll, pitch, throttle]
        self.Kp = [1.0, 1.0, 1.0]  # You should tune these gains
        self.Ki = [0.0, 0.0, 0.0]
        self.Kd = [0.0, 0.0, 0.0]

        # Previous errors for each axis
        self.prev_error = [0.0, 0.0, 0.0]

        # Max and min values for roll, pitch, and throttle
        self.max_values = [2000, 2000, 2000]
        self.min_values = [1000, 1000, 1000]

        # Sample time for PID control
        self.sample_time = 0.05  # 50ms

        # Publishing /drone_command
        self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)

        # Subscribing to /whycon/poses and /pid_tuning_altitude
        rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)

        # Arm the drone
        self.arm()

    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    def arm(self):
        self.disarm()
        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    def whycon_callback(self, msg):
        # Update the current position of the drone from the callback
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    def altitude_set_pid(self, alt):
        # Update PID gains for altitude control from the callback
        self.Kp[2] = alt.Kp
        self.Ki[2] = alt.Ki
        self.Kd[2] = alt.Kd

    def pid(self):
        # Compute errors in each axis
        errors = [self.setpoint[i] - self.drone_position[i] for i in range(3)]

        # Calculate PID output for each axis
        pid_output = [0.0, 0.0, 0.0]
        for i in range(3):
            pid_output[i] = (
                self.Kp[i] * errors[i] +
                self.Ki[i] * (errors[i] + self.prev_error[i] * self.sample_time) +
                self.Kd[i] * ((errors[i] - self.prev_error[i]) / self.sample_time)
            )

            # Update previous error
            self.prev_error[i] = errors[i]

            # Limit the output values
            pid_output[i] = max(min(pid_output[i], self.max_values[i]), self.min_values[i])

            # Update the corresponding command value
            if i == 0:
                self.cmd.rcRoll = int(1500 + pid_output[i])
            elif i == 1:
                self.cmd.rcPitch = int(1500 + pid_output[i])
            elif i == 2:
                self.cmd.rcThrottle = int(1500 + pid_output[i])

        # Publish the command
        self.command_pub.publish(self.cmd)

if __name__ == '__main__':
    quad_controller = QuadcopterController()
    rate = rospy.Rate(20)  # Specify the desired rate (e.g., 20 Hz)

    while not rospy.is_shutdown():
        quad_controller.pid()
        rate.sleep()
