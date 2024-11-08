#! /usr/bin/env python3

import functools
import json
import os
from typing import Any, Callable, List, Optional

from ament_index_python.packages import get_package_share_directory
import yaml

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import watchdog.observers
import watchdog.events

from task_generator.task_generator_node  import TaskGenerator

def observe(file: str, callback: watchdog.events.FileSystemEventHandler):
    observer = watchdog.observers.Observer()
    observer.schedule(callback, path=file, recursive=False)
    observer.start()
    return observer

def safe_callback(fn: Callable, logger: Callable):
    def wrapper(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except KeyboardInterrupt as e:
            raise e
        except Exception as e:
            logger.warn(f"Exception in callback: {e}")

    return wrapper

def recursive_get(obj: Any, property: List[str], fallback: Any = None) -> Any:
    if not len(property):
        return fallback if obj is None else obj
    try:
        return recursive_get(dict(obj).get(property[0]), property[1:])
    except:
        return fallback

def encode(var: Any):
    if isinstance(var, list):
        return ";".join(map(str, var))
    if isinstance(var, dict):
        return json.dumps(var)
    return var

def get_or_ignore(obj: dict, key: str) -> dict:
    return {key: obj.get(key)} if key in obj else {}

class TaskGeneratorFileWatcher(Node):
    def __init__(self):
        super().__init__('task_generator_filewatcher')
        
        # Get the path to the config file
        self.FILE_TASK_CONFIG = os.path.join(
            get_package_share_directory("arena_bringup"), #should replace rospkg.RosPack().get_path("arena_bringup") - https://docs.ros.org/en/iron/p/ament_index_python/ament_index_python.packages.html
            "configs",
            "task_generator.yaml"
        )

        self.observers = []
        self.setup_file_watcher()

    def set_ros_params(self, params: dict, prefix: str = ""):
        for key, value in params.items():
            param_name = f"{prefix}{key}"
            if isinstance(value, dict):
                self.set_ros_params(value, f"{param_name}.")
            else:
                param = Parameter(
                    param_name,
                    value=value
                )
                self.set_parameters([param])

    def setup_file_watcher(self):
        class TaskConfigHandler(watchdog.events.FileSystemEventHandler):
            def __init__(self, node):
                super().__init__()
                self.node = node
                self.reconfigure()

            def reconfigure(self):
                with open(self.node.FILE_TASK_CONFIG) as f:
                    content = yaml.safe_load(f)

                self.node.get_logger().debug("SENSING CHANGE OF TASK_MODE PARAMS")

                if 'ros__parameters' in content:
                    self.node.set_ros_params(content['ros__parameters'])

            def on_modified(self, event):
                callback = safe_callback(self.reconfigure, self.node.get_logger())
                callback()

        self.observers.append(
            observe(self.FILE_TASK_CONFIG, TaskConfigHandler(self))
        )

    def cleanup(self):
        for observer in self.observers:
            observer.stop()
        for observer in self.observers:
            observer.join()

def main(args=None):
    rclpy.init(args=args)
    
    node = TaskGeneratorFileWatcher()
    
    try:
        while rclpy.ok() and any(observer.is_alive() for observer in node.observers):
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()
