import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

ros_initialized = False

def ros_init():
    global ros_initialized
    if not rclpy.ok():
        rclpy.init()
    if not ros_initialized:
        ros_initialized = True

def fetch_remote_param(node_name: str, param_name: str) -> str:
        """
        Blocking retrieval of a single string parameter from `node_name`.

        Returns
        -------
        str
            Parameter string value (URDF/SRDF). Raises RuntimeError on failure.
        """
        tmp_node = Node("tracik_param_client")

        # --- connect to GetParameters service ---------------------------- #
        cli = tmp_node.create_client(GetParameters, f"{node_name}/get_parameters")
        if not cli.wait_for_service(timeout_sec=5.0):
            tmp_node.destroy_node()
            raise RuntimeError(f"Service '{node_name}/get_parameters' unavailable")

        # --- build and send request -------------------------------------- #
        req = GetParameters.Request()
        req.names = [param_name]
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(tmp_node, future, timeout_sec=5.0)

        # --- evaluate response ------------------------------------------ #
        if future.result() is None:
            tmp_node.destroy_node()
            raise RuntimeError("Parameter response timeout")

        result = future.result()
        if not result.values or result.values[0].type != 4:  # 4 ⇒ string
            tmp_node.destroy_node()
            raise RuntimeError(f"Parameter '{param_name}' missing or not a string")

        value = result.values[0].string_value
        if not value:
            tmp_node.destroy_node()
            raise RuntimeError(f"Parameter '{param_name}' on '{node_name}' is empty")

        tmp_node.destroy_node()
        return value