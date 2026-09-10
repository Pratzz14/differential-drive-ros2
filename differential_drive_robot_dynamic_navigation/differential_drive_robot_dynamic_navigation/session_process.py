"""Keep terminal Ctrl+C out of children until lifecycle shutdown has finished."""
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


class _IsolatedShutdown:
    def __init__(self, **kwargs):
        # Linux setsid execs each command in its own session. Otherwise terminal
        # Ctrl+C reaches every ROS process before the ordered shutdown hook.
        kwargs['prefix'] = ['setsid']
        kwargs.setdefault('sigterm_timeout', '15.0')
        super().__init__(**kwargs)

    def _shutdown_process(self, context, *, send_sigint):
        # Jazzy launch normally skips this signal for interactive Ctrl+C,
        # assuming the terminal already delivered it. Our isolated children
        # deliberately did not receive that signal. Keep standard escalation.
        return super()._shutdown_process(context, send_sigint=True)


class SessionNode(_IsolatedShutdown, Node):
    def __init__(self, **kwargs):
        # Keep launch diagnostics identifiable instead of labeling every ROS
        # process "setsid" after adding the session-isolation prefix.
        kwargs.setdefault('exec_name', kwargs.get('name', kwargs['executable']))
        super().__init__(**kwargs)


class SessionProcess(_IsolatedShutdown, ExecuteProcess):
    pass
