"""ROS initialization that keeps cleanup under the application's control."""


def init_preserving_python_signals():
    import rclpy
    try:
        from rclpy.signals import SignalHandlerOptions
    except ImportError:
        # Foxy has no signal_handler_options argument. Its SIGINT guard wakes
        # the executor without closing the context, so cleanup can still use
        # services before the explicit shutdown. Keep the native Foxy path.
        rclpy.init()
    else:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
