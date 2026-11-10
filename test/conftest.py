"""Keep host (ROS-free) pytest away from the launch tests, which need rclpy."""

try:
    import rclpy  # noqa: F401
except ImportError:
    collect_ignore_glob = ["launch/*"]
