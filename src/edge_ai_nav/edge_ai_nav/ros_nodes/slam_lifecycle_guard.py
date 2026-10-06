"""Retry SLAM Toolbox lifecycle activation after busy Gazebo startup."""

import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


class SlamLifecycleGuard(Node):
    def __init__(self):
        super().__init__('slam_lifecycle_guard')
        self.state_client = self.create_client(GetState, '/slam_toolbox/get_state')
        self.change_client = self.create_client(ChangeState, '/slam_toolbox/change_state')
        self.pending = None
        self.pending_kind = None
        self.pending_since = 0.0
        self.active_reported = False
        self.create_timer(2.0, self.tick)

    def request(self, client, request, kind):
        self.pending = client.call_async(request)
        self.pending_kind = kind
        self.pending_since = time.monotonic()

    def tick(self):
        if self.pending is not None:
            if not self.pending.done():
                if time.monotonic() - self.pending_since > 10.0:
                    self.get_logger().warn(f'SLAM {self.pending_kind} response timed out; retrying')
                    self.pending = None
                return
            kind = self.pending_kind
            try:
                response = self.pending.result()
            except Exception as exc:
                self.get_logger().warn(f'SLAM {kind} call failed: {exc}')
                self.pending = None
                return
            self.pending = None
            if kind == 'state':
                state = response.current_state.id
                if state == State.PRIMARY_STATE_ACTIVE:
                    if not self.active_reported:
                        self.get_logger().info('SLAM Toolbox ACTIVE; live map can update')
                    self.active_reported = True
                    return
                self.active_reported = False
                transition = {
                    State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_CONFIGURE,
                    State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_ACTIVATE,
                }.get(state)
                if transition is not None and self.change_client.service_is_ready():
                    self.request(self.change_client,
                                 ChangeState.Request(transition=Transition(id=transition)),
                                 'configure' if transition == Transition.TRANSITION_CONFIGURE
                                 else 'activate')
                return
            if not response.success:
                self.get_logger().warn(f'SLAM {kind} rejected; checking state before retry')
            return
        if self.state_client.service_is_ready():
            self.request(self.state_client, GetState.Request(), 'state')


def main():
    rclpy.init()
    node = SlamLifecycleGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
