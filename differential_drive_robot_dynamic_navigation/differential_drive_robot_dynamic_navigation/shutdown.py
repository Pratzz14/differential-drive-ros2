"""Bounded lifecycle teardown while Gazebo clock and sensor bridges still exist."""
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger


def orderly_shutdown(environment):
    ros_context = Context()
    rclpy.init(args=[], context=ros_context, domain_id=int(environment.get('ROS_DOMAIN_ID', '0')),
               signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('dynamic_navigation_shutdown', context=ros_context)
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(node)
    logger = node.get_logger()

    def call(service_type, name, request, timeout, accepted=lambda response: response.success):
        client = node.create_client(service_type, name)
        try:
            # This is a fresh DDS participant; allow discovery of the first
            # patrol service even while both GUIs are busy.
            if not client.wait_for_service(timeout_sec=3.0):
                logger.info(f'{name} unavailable; continuing bounded shutdown')
                return False
            future = client.call_async(request)
            executor.spin_until_future_complete(future, timeout_sec=timeout)
            if not future.done() or future.exception() or not accepted(future.result()):
                logger.warning(f'{name} did not complete; process shutdown will be the fallback')
                return False
            logger.info(f'{name}: complete')
            return future.result()
        except (RuntimeError, rclpy.executors.ExternalShutdownException) as error:
            logger.warning(f'{name}: {error}')
            return False
        finally:
            node.destroy_client(client)

    try:
        logger.info('Orderly shutdown: stop patrols, cancel navigation, shut down Nav2, then stop Gazebo')
        stop_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        goal_states = {}

        def status(message):
            for item in message.status_list:
                goal_states[bytes(item.goal_info.goal_id.uuid)] = item.status

        node.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        call(Trigger, '/dynamic_obstacles/stop', Trigger.Request(), 2.0)
        # Empty goal ID/time means all goals on this session's navigation
        # server. Wait for cancellation before deactivating its lifecycle.
        canceled = call(CancelGoal, '/navigate_to_pose/_action/cancel_goal',
                        CancelGoal.Request(), 2.0,
                        accepted=lambda response: response.return_code == CancelGoal.Response.ERROR_NONE)
        if canceled and canceled.goals_canceling:
            pending = {bytes(goal.goal_id.uuid) for goal in canceled.goals_canceling}
            deadline = time.monotonic() + 2.0
            terminal = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)
            while any(goal_states.get(goal) not in terminal for goal in pending):
                if time.monotonic() >= deadline:
                    logger.warning('Goal cancellation still pending; continuing bounded lifecycle shutdown')
                    break
                executor.spin_once(timeout_sec=0.05)
        request = ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.SHUTDOWN)
        call(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes', request, 8.0)
        call(ManageLifecycleNodes, '/lifecycle_manager_localization/manage_nodes', request, 4.0)
        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            stop_pub.publish(Twist())
            executor.spin_once(timeout_sec=0.05)
        # Partition is supplied by this launch's session guard: never address
        # another user's simulator through a global/default Gazebo partition.
        if environment.get('GZ_PARTITION', '').startswith('differential_drive_'):
            try:
                result = subprocess.run(
                    ['gz', 'service', '-s', '/server_control', '--reqtype', 'gz.msgs.ServerControl',
                     '--reptype', 'gz.msgs.Boolean', '--timeout', '1500', '--req', 'stop: true'],
                    env=dict(environment), capture_output=True, text=True, timeout=3.0)
                if result.returncode == 0 and 'true' in result.stdout:
                    logger.info('Gazebo server accepted orderly stop')
                else:
                    logger.warning('Gazebo stop service unavailable; falling back to process signals')
            except (OSError, subprocess.TimeoutExpired):
                logger.warning('Gazebo stop request timed out; falling back to process signals')
    finally:
        executor.shutdown()
        node.destroy_node()
        ros_context.try_shutdown()
