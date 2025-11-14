from langchain_anthropic import ChatAnthropic
from langchain_ollama import ChatOllama
from langchain.agents import tool
from rosa import ROSA
from rosa.prompts import RobotSystemPrompts
import os
import pathlib
import time
import subprocess
import math
from typing import Tuple
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

node = None
vel_publisher = None
explore_publisher = None
navigate_to_pose_action_client = None


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple:
    """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees."""

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def execute_ros_command(command: str) -> Tuple[bool, str]:
    """
    Execute a ROS2 command.

    :param command: The ROS2 command to execute.
    :return: A tuple containing a boolean indicating success and the output of the command.
    """

    # Validate the command is a proper ROS2 command
    cmd = command.split(" ")

    if len(cmd) < 2:
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    if cmd[0] != "ros2":
        raise ValueError(f"'{command}' is not a valid ROS2 command.")

    try:
        output = subprocess.check_output(command, shell=True).decode()
        return True, output
    except Exception as e:
        return False, str(e)


# Helper function to get the maps directory
def _get_maps_dir() -> str:
    """Gets the absolute path to the 'maps' directory in the rosa_summit package, creates it if it doesn't exist."""

    maps_dir = "/home/ros/rap/Gruppe2/maps"
    pathlib.Path(maps_dir).mkdir(parents=True, exist_ok=True)
    return maps_dir


# Hardcoded locations
LOCATIONS = {
    "gym": {
        "position": {"x": 1.9517535073729964, "y": 4.359393291484201, "z": 0.0},
        "orientation": {
            "x": 1.6302566310137402e-08,
            "y": 2.9703213238180324e-08,
            "z": -0.07945176214102775,
            "w": 0.9968387118750377,
        },
    },
    "kitchen": {
        "position": {"x": 7.353217566768062, "y": -3.458078519447155, "z": 0.0},
        "orientation": {
            "x": 1.611930208234276e-08,
            "y": 2.980589390984495e-08,
            "z": -0.07325043342926793,
            "w": 0.997313578571165,
        },
    },
    "living room": {
        "position": {"x": 1.084137940689, "y": -0.383112079564818, "z": 0.0},
        "orientation": {
            "x": 3.316520260505064e-08,
            "y": 6.931688679143018e-09,
            "z": -0.8089064668855616,
            "w": 0.5879373502599718,
        },
    },
    "office": {
        "position": {"x": -4.9521764504716765, "y": -3.573205806403106, "z": 0.0},
        "orientation": {
            "x": -3.238093524948138e-08,
            "y": 9.961584542476143e-09,
            "z": 0.9923116132365216,
            "w": -0.1237645435329962,
        },
    },
    "bedroom": {
        "position": {"x": -4.002267652240865, "y": -0.060121871401907084, "z": 0.0},
        "orientation": {
            "x": -2.1636165143756515e-08,
            "y": 2.6069771799291994e-08,
            "z": 0.8980250477792399,
            "w": 0.43994433007039957,
        },
    },
}


@tool
def send_vel(velocity: float) -> str:
    """
    Sets the forward velocity of the robot.

    :param velocity: the velocity at which the robot should move
    """
    print(f"[TOOL CALL] send_vel(velocity={velocity})")
    global vel_publisher
    twist = Twist()
    twist.linear.x = velocity
    vel_publisher.publish(twist)

    result = "Velocity set to %s" % velocity
    print(f"[TOOL RESULT] {result}")
    return result


@tool
def stop() -> str:
    """
    Stops or halts the robot by setting its velocity to zero

    """
    print(f"[TOOL CALL] stop()")
    global vel_publisher
    twist = Twist()
    vel_publisher.publish(twist)
    result = "Robot stopped"
    print(f"[TOOL RESULT] {result}")
    return result


@tool
def toggle_auto_exploration(resume_exploration: bool) -> str:
    """
    Starts or stops the autonomous exploration.

    :param resume_exploration: True to start/resume exploration, False to stop/pause exploration.
    """
    print(f"[TOOL CALL] toggle_auto_exploration(resume_exploration={resume_exploration})")
    global explore_publisher
    msg = Bool()
    msg.data = resume_exploration
    explore_publisher.publish(msg)

    if resume_exploration:
        result = "Autonomous exploration started/resumed."
    else:
        result = "Autonomous exploration stopped/paused."
    print(f"[TOOL RESULT] {result}")
    return result


@tool
def navigate_to_pose(
    x: float, y: float, z_orientation: float, w_orientation: float
) -> str:
    """
    Moves the robot to an absolute position on the map.

    :param x: The x coordinate of the target position.
    :param y: The y coordinate of the target position.
    :param z_orientation: The z component of the target orientation (quaternion).
    :param w_orientation: The w component of the target orientation (quaternion).
    """
    print(f"[TOOL CALL] navigate_to_pose(x={x}, y={y}, z_orientation={z_orientation}, w_orientation={w_orientation})")
    global navigate_to_pose_action_client, node

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    goal_msg.pose.pose.orientation.z = z_orientation
    goal_msg.pose.pose.orientation.w = w_orientation

    navigate_to_pose_action_client.send_goal_async(goal_msg)
    roll, pitch, yaw = quaternion_to_euler(0, 0, z_orientation, w_orientation)
    result = f"Navigation goal sent to (x={x:.2f}, y={y:.2f}, yaw={yaw:.1f}°)."
    print(f"[TOOL RESULT] {result}")
    return result


@tool
def navigate_relative(
    x: float, y: float, z_orientation: float, w_orientation: float
) -> str:
    """
    Moves the robot relative to its current position using the robot's local coordinate frame.

    In the robot's coordinate system (base_link):
    - Positive X: Move forward
    - Negative X: Move backward
    - Positive Y: Move to the left
    - Negative Y: Move to the right

    :param x: The x coordinate of the target position relative to the robot (forward/backward).
    :param y: The y coordinate of the target position relative to the robot (left/right).
    :param z_orientation: The z component of the target orientation (quaternion) relative to the robot.
    :param w_orientation: The w component of the target orientation (quaternion) relative to the robot.
    """
    print(f"[TOOL CALL] navigate_relative(x={x}, y={y}, z_orientation={z_orientation}, w_orientation={w_orientation})")
    global navigate_to_pose_action_client, node

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "base_link"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    goal_msg.pose.pose.orientation.z = z_orientation
    goal_msg.pose.pose.orientation.w = w_orientation

    navigate_to_pose_action_client.send_goal_async(goal_msg)
    roll, pitch, yaw = quaternion_to_euler(0, 0, z_orientation, w_orientation)
    result = f"Relative navigation goal sent to (x={x:.2f}, y={y:.2f}, yaw={yaw:.1f}°)."
    print(f"[TOOL RESULT] {result}")
    return result


@tool
def save_map(map_name: str) -> str:
    """
    Saves the current map from the /summit/map topic to .yaml and .pgm files
    in the 'maps' directory of the 'rosa_summit' package.

    :param map_name: The name for the map (e.g., 'my_lab_map'). Do not include file extensions.
    """
    maps_dir = _get_maps_dir()
    if not os.path.isdir(
        maps_dir
    ):  # Should be created by _get_maps_dir, but double check
        return f"Error: Maps directory {maps_dir} could not be accessed or created."

    filepath_prefix = os.path.join(maps_dir, map_name)

    # Added --ros-args -r map:=/summit/map to specify the topic
    cmd = f"ros2 run nav2_map_server map_saver_cli -f '{filepath_prefix}' --ros-args -r map:=/summit/map"
    success, output = execute_ros_command(cmd)
    if success:
        if "Map saved to" in output:
            return f"Map successfully saved as {map_name} in {maps_dir}"
        else:
            return f"Map saving process initiated for {map_name} in {maps_dir}. Output: {output}"
    else:
        return f"Failed to save map {map_name} in {maps_dir}. Error: {output}"


@tool
def list_saved_maps() -> str:
    """
    Lists all saved maps in the 'maps' directory of the 'rosa_summit' package.
    Returns a list of map names (without .yaml extension).
    """
    maps_dir = _get_maps_dir()
    if not os.path.isdir(maps_dir):
        return "Maps directory not found or is not a directory."

    try:
        files = os.listdir(maps_dir)
        map_files = [
            f[:-5]
            for f in files
            if f.endswith(".yaml") and os.path.isfile(os.path.join(maps_dir, f))
        ]
        if not map_files:
            return "No saved maps found in the maps directory."
        return f"Available maps: {', '.join(map_files)}"
    except Exception as e:
        return f"Error listing maps: {e}"


@tool
def get_location_names() -> str:
    """
    Returns a list of available location names.
    """
    return f"Available locations: {', '.join(LOCATIONS.keys())}"


@tool
def navigate_to_location_by_name(location_name: str) -> str:
    """
    Moves the robot to a predefined location by its name.

    :param location_name: The name of the location to navigate to (e.g., 'kitchen', 'gym').
    """
    print(f"[TOOL CALL] navigate_to_location_by_name(location_name='{location_name}')")
    global navigate_to_pose_action_client, node
    location_name_lower = location_name.lower()
    if location_name_lower not in LOCATIONS:
        result = f"Location '{location_name}' not found. Available locations are: {', '.join(LOCATIONS.keys())}"
        print(f"[TOOL RESULT] {result}")
        return result

    loc_data = LOCATIONS[location_name_lower]
    pos = loc_data["position"]
    orient = loc_data["orientation"]

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = pos["x"]
    goal_msg.pose.pose.position.y = pos["y"]
    goal_msg.pose.pose.orientation.z = orient["z"]
    goal_msg.pose.pose.orientation.w = orient["w"]

    navigate_to_pose_action_client.send_goal_async(goal_msg)

    # Convert orientation to yaw for display
    roll, pitch, yaw = quaternion_to_euler(0, 0, orient["z"], orient["w"])
    result = f"Navigation goal sent to '{location_name}' (x={pos['x']:.2f}, y={pos['y']:.2f}, yaw={yaw:.1f}°)."
    print(f"[TOOL RESULT] {result}")
    return result


def main():
    global node, vel_publisher, explore_publisher, navigate_to_pose_action_client
    print("Hi from rosa_summit.")

    # init rclpy
    rclpy.init()
    sim_time_param = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
    node = rclpy.create_node("rosa_summit_node", parameter_overrides=[sim_time_param])

    vel_publisher = node.create_publisher(Twist, "/summit/cmd_vel", 10)
    explore_publisher = node.create_publisher(Bool, "/summit/explore/resume", 10)
    navigate_to_pose_action_client = ActionClient(
        node, NavigateToPose, "/summit/navigate_to_pose"
    )

    # Get the current username
    user_name = os.getenv("USER")

    try:
        if user_name == "ros":
            print("Using remote Ollama instance")
            llm = ChatOllama(
                model="qwen2.5:7b",  # Fast text-only model for better performance
                temperature=0,
                num_ctx=2048,  # Reduced context for faster inference
                base_url="http://host.docker.internal:11434",
            )
        else:
            print("Using Anthropic API with Claude Sonnet 3.5")
            # Read API key from file
            try:
                with open("/home/ros/rap/Gruppe2/api-key.txt", "r") as f:
                    # Skip the comment line if it exists
                    api_key = f.read().strip().split("\n")[-1]
            except Exception as e:
                print(f"Error reading API key: {e}")
                return

            llm = ChatAnthropic(
                model="claude-3-5-sonnet-20240620",
                temperature=0,
                anthropic_api_key=api_key,
                max_tokens=4096,
            )
    except Exception as e:
        print(f"Error initializing LLM: {e}")
        return

    prompt = RobotSystemPrompts()
    prompt.embodiment = "You are an helpful robot named Summit, designed to assist users in a simulated environment. You can navigate, explore, and interact with the environment using various tools."

    # Pass the LLM to ROSA with both tools available
    agent = ROSA(
        ros_version=2,
        llm=llm,
        tools=[
            send_vel,
            stop,
            toggle_auto_exploration,
            navigate_to_pose,
            navigate_relative,
            save_map,
            list_saved_maps,
            get_location_names,
            navigate_to_location_by_name,
        ],
        prompts=prompt,
        verbose=True,
    )

    print("Type 'exit' or 'quit' to end the program")

    try:
        while True:
            request_start_time = time.time()
            msg = input("\n" + "="*60 + "\nEnter your request: ")
            if msg.lower() in ["exit", "quit"]:
                break

            try:
                print("="*60)
                print(f"[USER INPUT] {msg}")
                print("[LLM] Processing request...")
                print("="*60)

                llm_start_time = time.time()
                response = agent.invoke(msg)
                llm_end_time = time.time()

                print("="*60)
                print(f"[LLM] Response received in {llm_end_time - llm_start_time:.2f} seconds")
                print(f"[DEBUG] Response type: {type(response)}")

                # Handle different response formats
                if isinstance(response, list) and len(response) > 0:
                    res = response[0]
                    print(f"[DEBUG] Extracted first element from list response")
                elif isinstance(response, dict):
                    res = response
                    print(f"[DEBUG] Response is a dictionary with keys: {list(res.keys())}")
                else:
                    res = response
                    print(f"[DEBUG] Using raw response")

                print("="*60)
                print("[AGENT RESPONSE]")
                if isinstance(res, dict) and "text" in res:
                    print(res["text"])
                else:
                    print(res)
                print("="*60)

                request_end_time = time.time()
                print(f"[TOTAL] Complete request cycle took {request_end_time - request_start_time:.2f} seconds")
                print("="*60)
            except Exception as e:
                import traceback
                print("="*60)
                print(f"[ERROR] An error occurred: {e}")
                print(f"[ERROR] Full traceback:\n{traceback.format_exc()}")
                print("="*60)
    except KeyboardInterrupt:
        print("\n" + "="*60)
        print("Program terminated by user")
        print("="*60)

    agent.shutdown()
    print("Bye from rosa_summit.")


if __name__ == "__main__":
    main()
