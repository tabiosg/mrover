import math  # needed for math.nan

import moteus
import moteus_pi3hat
import rospy
from mrover.msg import ZedGimbalPosition


class MoteusBridge:
    """This will control the behavior of the Moteus"""

    def __init__(self) -> None:
        """
        Initialize the components.
        Start with a Default State
        """
        # assume connection to JC1
        # If there were multiple motors connected, then
        # f would need to be modified
        f = moteus_pi3hat.Pi3HatRouter(
                servo_bus_map={1: [1]},
        )

        # This holds the controller object
        self.c = moteus.Controller(transport=f, id=1)

        self.desired_rev = 0
        self.pub = rospy.Publisher(
            'zed_gimbal_data', ZedGimbalPosition, queue_size=1)
        self.sleep = 0.5

    def restrict_angle(self, input_angle: float) -> float:
        lower_bound = -180
        upper_bound = 180
        if input_angle > upper_bound:
            return upper_bound
        elif input_angle < lower_bound:
            return lower_bound
        return input_angle

    def publish_zed_gimbal_position(self) -> None:
        state = self.c.make_position(
            position=math.nan, velocity=0.2, maximum_torque=0.3,
            stop_position=self.desired_rev, watchdog_timeout=70, query=True)

        fault_value = state.values[moteus.Register.FAULT]
        error = fault_value != 0
        if error:
            print("Error #" + str(fault_value))
            self.c.set_stop()
            rospy.sleep(self.sleep)
            return

        rev = state.values[moteus.Register.POSITION]
        degrees = 360.0 * rev
        degrees = self.restrict_angle(degrees)

        zed_struct = ZedGimbalPosition()
        zed_struct.angle = degrees
        self.pub.publish(zed_struct)

        rospy.sleep(self.sleep)

    def zed_gimbal_position_callback(self, ros_msg: ZedGimbalPosition) -> None:
        degrees = ros_msg.angle
        degrees = self.restrict_angle(degrees)
        self.desired_rev = degrees / 360.0


async def main():
    rospy.init_node("zed_gimbal")

    bridge = MoteusBridge()

    rospy.Subscriber("zed_gimbal_cmd", ZedGimbalPosition,
                     bridge.zed_gimbal_position_callback)
    while not rospy.is_shutdown():
        bridge.publish_zed_gimbal_position()


if __name__ == "__main__":
    main()
