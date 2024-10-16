from rclpy.node import Node
import rclpy, sys, os, ruamel.yaml
from whill_navi2.modules.ros2_launch_utils import DataPath

class OdometryBenchmark(Node):
    def __init__(self):
        super().__init__("odometry_benchmark")
        data_path = DataPath()
        
        # File source
        self.glim_odometry = {{}}
        self.hdl_odometry = {{}}
        with open(os.path.join(data_path.waypoint_base_path, "glilm_waypoint.yaml"), "w+") as f:
            documents = ruamel.yaml.safe_load_all(f)
            index = 0
            for document in documents:
                self.glim_odometry[index] = {}
                self.glim_odometry[index]["position_x"] = document["position_x"]
                self.glim_odometry[index]["position_y"] = document["position_y"]
                self.glim_odometry[index]["position_z"] = document["position_z"]
                self.glim_odometry[index]["quaternion_x"] = document["quaternion_x"]
                self.glim_odometry[index]["quaternion_y"] = document["quaternion_y"]
                self.glim_odometry[index]["quaternion_z"] = document["quaternion_z"]
                self.glim_odometry[index]["quaternion_w"] = document["quaternion_w"]
                self.glim_odometry[index]["roll"] = document["roll"]
                self.glim_odometry[index]["pitch"] = document["pitch"]
                self.glim_odometry[index]["yaw"] = document["yaw"]
                self.glim_odometry[index]["longitude"] = document["longitude"]
                self.glim_odometry[index]["latitude"] = document["latitude"]
                self.glim_odometry[index]["mode"] = document["mode"]
        
        with open(os.path.join(data_path.waypoint_base_path, "waypoint.yaml"), "w+") as f:
            documents = ruamel.yaml.safe_load_all(f)
            index = 0
            for document in documents:
                self.hdl_odometry[index] = {}
                self.hdl_odometry[index]["position_x"] = document["position_x"]
                self.hdl_odometry[index]["position_y"] = document["position_y"]
                self.hdl_odometry[index]["position_z"] = document["position_z"]
                self.hdl_odometry[index]["quaternion_x"] = document["quaternion_x"]
                self.hdl_odometry[index]["quaternion_y"] = document["quaternion_y"]
                self.hdl_odometry[index]["quaternion_z"] = document["quaternion_z"]
                self.hdl_odometry[index]["quaternion_w"] = document["quaternion_w"]
                self.hdl_odometry[index]["roll"] = document["roll"]
                self.hdl_odometry[index]["pitch"] = document["pitch"]
                self.hdl_odometry[index]["yaw"] = document["yaw"]
                self.hdl_odometry[index]["longitude"] = document["longitude"]
                self.hdl_odometry[index]["latitude"] = document["latitude"]
                self.hdl_odometry[index]["mode"] = document["mode"]
        