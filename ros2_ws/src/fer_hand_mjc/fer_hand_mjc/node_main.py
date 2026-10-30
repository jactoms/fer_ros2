"""rclpy main shared by the fer_hand_mjc and fer_track_hand nodes (no MuJoCo)."""
from __future__ import annotations

import signal
from typing import Callable, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def run_node(name: str, factory: Callable[[Node], object],
             args: Optional[list[str]] = None) -> None:
    """Create node `name`, build factory(node) (which may have close()), spin until shutdown."""
    rclpy.init(args=args)

    node = Node(name)
    app = None

    try:
        app = factory(node)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C on a launch sends SIGINT twice (terminal and launch): shut down only once.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if app is not None and hasattr(app, 'close'):
            app.close()
        node.destroy_node()
        rclpy.try_shutdown()
