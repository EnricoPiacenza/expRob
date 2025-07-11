#! /usr/bin/env python3

import rclpy
from rclpy.parameter import Parameter
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
import lifecycle_msgs.msg


class MoveAction(ActionExecutorClient):
    def __init__(self):
        super().__init__("move", 0.5)
        self.test = 0

    def do_work(self):
        self.logger.info("Executing work" + str(self.test))
        self.test += 1


def main(args=None):
    rclpy.init(args=args)
    node_a = MoveAction()

    node_a.set_parameter(rclpy.Parameter('action_name', 'move'))
    node_a.trigger_transition(lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE)

    try:
        rclpy.spin(node_a)
    except KeyboardInterrupt:
        pass
    node_a.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
