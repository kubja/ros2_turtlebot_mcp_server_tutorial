import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
import uuid
import math
from flask import Flask, request, jsonify
import threading
import time
import sys
import queue
import json

# === ROS 2 Setup ===

controller_node = None
executor = None
command_processor_node = None
command_queue = queue.Queue()

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_mcp_server')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.current_pose = None
        self.get_logger().info('TurtleBot MCP Server Node started.')

    def pose_callback(self, msg):
        self.current_pose = msg

    def send_twist_command(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing Twist: linear.x={linear_x}, angular.z={angular_z}')

class CommandProcessorNode:
    def __init__(self):
        super().__init__('command_processor_node')
        self.active_move_command = None
        self.timer = self.create_timer(0.1, self.process_queue_callback)
        self.response_queue = queue.Queue()
        self.get_logger().info('Command Processor Node started.')

    def process_queue_callback(self):
        global command_queue, controller_node
        
        # Check for a new command from the queue and update the active command
        try:
            command = command_queue.get_nowait()
            if command['type'] == 'move':
                if 'duration_s' in command and command['duration_s'] is not None:
                    duration = float(command['duration_s'])
                    command['end_time'] = self.get_clock().now().nanoseconds / 1e9 + duration
                else:
                    # This is a continuous move, so no end_time
                    command.pop('end_time', None)
                self.active_move_command = command
            elif command['type'] == 'stop':
                self.active_move_command = None
                # Send stop command immediately
                controller_node.send_twist_command(0.0, 0.0)
            elif command['type'] == 'move_distance':
                self.active_move_command = command
                self.active_move_command['start_pose'] = controller_node.current_pose
                if self.active_move_command['start_pose'] is None:
                    self.get_logger().error("Could not get robot pose for move_distance command.")
                    self.active_move_command = None # Cancel command if pose not available
            elif command['type'] == 'rotate_angle':
                self.active_move_command = command
                self.active_move_command['start_yaw'] = self.get_robot_yaw()
                if self.active_move_command['start_yaw'] is None:
                    self.get_logger().error("Could not get robot yaw for rotate_angle command.")
                    self.active_move_command = None # Cancel command if yaw not available
        except queue.Empty:
            pass

        # Process the active move command
        if self.active_move_command:
            now = self.get_clock().now().nanoseconds / 1e9
            
            if self.active_move_command['type'] == 'move':
                if 'end_time' in self.active_move_command and self.active_move_command['end_time'] is not None:
                    if now >= self.active_move_command['end_time']:
                        # Duration is over, stop the robot
                        command_id = self.active_move_command.get('command_id')
                        if command_id:
                            self.response_queue.put({'command_id': command_id, 'status': 'completed'})
                        self.active_move_command = None
                        controller_node.send_twist_command(0.0, 0.0)
                    else:
                        # Duration is not over, send the move command
                        linear_x = self.active_move_command['linear_x']
                        angular_z = self.active_move_command['angular_z']
                        controller_node.send_twist_command(linear_x, angular_z)
                else:
                    # No duration, so just send the move command continuously
                    linear_x = self.active_move_command['linear_x']
                    angular_z = self.active_move_command['angular_z']
                    controller_node.send_twist_command(linear_x, angular_z)
            elif self.active_move_command['type'] == 'move_distance':
                self.execute_move_distance()
            elif self.active_move_command['type'] == 'rotate_angle':
                self.execute_rotate_angle()

    def get_robot_yaw(self):
        if controller_node.current_pose is None:
            return None
        return controller_node.current_pose.theta

    def execute_move_distance(self):
        if self.active_move_command is None or 'start_pose' not in self.active_move_command:
            return

        current_pose = controller_node.current_pose
        if current_pose is None:
            controller_node.send_twist_command(0.0, 0.0) # Stop if we lose tracking
            self.active_move_command = None
            self.get_logger().warn("Lost robot pose during move_distance. Stopping.")
            return

        start_pose = self.active_move_command['start_pose']
        distance_to_travel = self.active_move_command['distance']
        linear_velocity = self.active_move_command['linear_velocity']

        # Calculate distance traveled
        dx = current_pose.x - start_pose.x
        dy = current_pose.y - start_pose.y
        distance_traveled = math.sqrt(dx*dx + dy*dy)

        self.get_logger().info(f"Distance traveled: {distance_traveled:.2f}/{distance_to_travel:.2f}")

        if distance_traveled >= distance_to_travel:
            command_id = self.active_move_command.get('command_id')
            if command_id:
                self.response_queue.put({'command_id': command_id, 'status': 'completed'})
            controller_node.send_twist_command(0.0, 0.0)
            self.active_move_command = None
            self.get_logger().info("Move distance completed.")
        else:
            controller_node.send_twist_command(linear_velocity, 0.0)

    def execute_rotate_angle(self):
        if self.active_move_command is None or 'start_yaw' not in self.active_move_command:
            return

        current_yaw = self.get_robot_yaw()
        if current_yaw is None:
            controller_node.send_twist_command(0.0, 0.0) # Stop if we lose tracking
            self.active_move_command = None
            self.get_logger().warn("Lost robot yaw during rotate_angle. Stopping.")
            return

        start_yaw = self.active_move_command['start_yaw']
        angle_to_rotate = self.active_move_command['angle']
        angular_velocity = self.active_move_command['angular_velocity']

        # Calculate angle rotated, handling wrap-around from -pi to pi
        angle_diff = current_yaw - start_yaw
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Determine direction of rotation and adjust angular velocity if needed
        # This simple approach assumes positive angular_velocity for positive angle_to_rotate
        # and negative angular_velocity for negative angle_to_rotate.
        # For more robust control, a PID controller would be ideal.
        if angle_to_rotate < 0:
            angular_velocity = -abs(angular_velocity)
        else:
            angular_velocity = abs(angular_velocity)

        self.get_logger().info(f"Angle rotated: {angle_diff:.2f}/{angle_to_rotate:.2f}")

        # Check if target angle is reached. Consider tolerance for floating point comparisons.
        if (angle_to_rotate > 0 and angle_diff >= angle_to_rotate) or \
           (angle_to_rotate < 0 and angle_diff <= angle_to_rotate):
            command_id = self.active_move_command.get('command_id')
            if command_id:
                self.response_queue.put({'command_id': command_id, 'status': 'completed'})
            controller_node.send_twist_command(0.0, 0.0)
            self.active_move_command = None
            self.get_logger().info("Rotate angle completed.")
        else:
            controller_node.send_twist_command(0.0, angular_velocity)

def ros_thread_entrypoint():
    global controller_node, executor, command_processor_node
    rclpy.init(args=sys.argv)
    executor = rclpy.executors.MultiThreadedExecutor()
    controller_node = TurtleBotController()
    command_processor_node = CommandProcessorNode()
    executor.add_node(controller_node)
    executor.add_node(command_processor_node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        controller_node.destroy_node()
        command_processor_node.destroy_node()
        rclpy.shutdown()

# === Flask MCP Server ===

app = Flask(__name__)

def log_debug(prefix, obj):
    print(f"\n--- {prefix} ---")
    try:
        print(json.dumps(obj, indent=2))
    except Exception:
        print(obj)
    print("--- End ---\n")

def jsonrpc_error(id, code, message, data=None):
    error_response = {"jsonrpc": "2.0", "error": {"code": code, "message": message}}
    if id is not None:
        error_response["id"] = id
    if data is not None:
        error_response["error"]["data"] = data
    log_debug("Sending JSON-RPC Error", error_response)
    return jsonify(error_response), 200

def jsonrpc_response(id, result):
    resp = {"jsonrpc": "2.0", "result": result, "id": id}
    log_debug("Sending JSON-RPC Response", resp)
    return jsonify(resp), 200

def _move(linear_x, angular_z, duration_s=None):
    if controller_node and executor:
        command_id = str(uuid.uuid4())
        if duration_s is not None:
            if angular_z == 0 and linear_x != 0: # Straight move
                distance = linear_x * duration_s
                command_queue.put({'type': 'move_distance', 'distance': distance, 'linear_velocity': linear_x, 'command_id': command_id})
            elif linear_x == 0 and angular_z != 0: # Rotation
                angle = angular_z * duration_s
                command_queue.put({'type': 'rotate_angle', 'angle': angle, 'angular_velocity': angular_z, 'command_id': command_id})
            else: # Curved path or both zero, use original move
                command_queue.put({'type': 'move', 'linear_x': linear_x, 'angular_z': angular_z, 'duration_s': duration_s, 'command_id': command_id})
            
            # Wait for completion if a blocking command was issued
            if (angular_z == 0 and linear_x != 0) or (linear_x == 0 and angular_z != 0):
                while True:
                    try:
                        response = command_processor_node.response_queue.get(timeout=30) # Increased timeout
                        if response.get('command_id') == command_id and response.get('status') == 'completed':
                            return {'status': 'success', 'linear_x': linear_x, 'angular_z': angular_z, 'duration_s': duration_s}
                    except queue.Empty:
                        # If timeout, assume command failed or got stuck.
                        # Log an error and return.
                        controller_node.get_logger().error(f"Timeout waiting for command {command_id} to complete.")
                        return {'status': 'error', 'message': 'Command timed out.'}
        else: # No duration, continuous move
            command_queue.put({'type': 'move', 'linear_x': linear_x, 'angular_z': angular_z, 'duration_s': duration_s, 'command_id': command_id})
            return {'status': 'success', 'linear_x': linear_x, 'angular_z': angular_z, 'duration_s': duration_s}
    else:
        return None

def _stop():
    if controller_node and executor:
        command_queue.put({'type': 'stop'})
        return {'status': 'success', 'message': 'Robot stopped.'}
    else:
        return None

def _move_distance(distance, linear_velocity):
    if controller_node and executor:
        command_id = str(uuid.uuid4())
        command_queue.put({'type': 'move_distance', 'distance': distance, 'linear_velocity': linear_velocity, 'command_id': command_id})
        while True:
            try:
                response = command_processor_node.response_queue.get(timeout=30)
                if response.get('command_id') == command_id and response.get('status') == 'completed':
                    return {'status': 'success', 'distance': distance, 'linear_velocity': linear_velocity}
            except queue.Empty:
                controller_node.get_logger().error(f"Timeout waiting for move_distance command {command_id} to complete.")
                return {'status': 'error', 'message': 'Command timed out.'}
    else:
        return None

def _rotate_angle(angle, angular_velocity):
    if controller_node and executor:
        command_id = str(uuid.uuid4())
        command_queue.put({'type': 'rotate_angle', 'angle': angle, 'angular_velocity': angular_velocity, 'command_id': command_id})
        while True:
            try:
                response = command_processor_node.response_queue.get(timeout=30)
                if response.get('command_id') == command_id and response.get('status') == 'completed':
                    return {'status': 'success', 'angle': angle, 'angular_velocity': angular_velocity}
            except queue.Empty:
                controller_node.get_logger().error(f"Timeout waiting for rotate_angle command {command_id} to complete.")
                return {'status': 'error', 'message': 'Command timed out.'}
    else:
        return None

# === MCP Description ===

MCP_DESCRIPTION = {
    "name": "turtlebot_controller",
    "description": "TurtleBot MCP Server",
    "version": "1.0.0",
    "actions": [
        {
            "name": "move",
            "description": "Moves the TurtleBot with specified linear and angular velocities.",
            "params": {
                "linear_x": {
                    "type": "number",
                    "description": "Linear velocity in the x-direction.",
                    "required": True
                },
                "angular_z": {
                    "type": "number",
                    "description": "Angular velocity around the z-axis.",
                    "required": True
                },
                "duration_s": {
                    "type": "number",
                    "description": "Duration in seconds for which the command should be active before stopping.",
                    "required": False
                }
            }
        },
        {
            "name": "stop",
            "description": "Stops the TurtleBot.",
            "params": {}
        },
        {
            "name": "move_distance",
            "description": "Moves the TurtleBot a specified distance.",
            "params": {
                "distance": {
                    "type": "number",
                    "description": "Distance to move in meters.",
                    "required": True
                },
                "linear_velocity": {
                    "type": "number",
                    "description": "Linear velocity in m/s.",
                    "required": True
                }
            }
        },
        {
            "name": "rotate_angle",
            "description": "Rotates the TurtleBot by a specified angle.",
            "params": {
                "angle": {
                    "type": "number",
                    "description": "Angle to rotate in radians.",
                    "required": True
                },
                "angular_velocity": {
                    "type": "number",
                    "description": "Angular velocity in rad/s.",
                    "required": True
                }
            }
        }
    ]
}

def build_tools_capability():
    tools = []

    # move tool
    tools.append({
        "name": "move",
        "description": "Moves the TurtleBot with specified linear and angular velocities.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "linear_x": {"type": "number", "description": "Linear velocity in the x-direction."},
                "angular_z": {"type": "number", "description": "Angular velocity around the z-axis."},
                "duration_s": {"type": "number", "description": "Duration in seconds for which the command should be active before stopping."}
            },
            "required": ["linear_x", "angular_z"],
            "additionalProperties": False
        }
    })

    # stop tool
    tools.append({
        "name": "stop",
        "description": "Stops the TurtleBot.",
        "inputSchema": {
            "type": "object",
            "properties": {},
            "required": [],
            "additionalProperties": False
        }
    })

    # move_distance tool
    tools.append({
        "name": "move_distance",
        "description": "Moves the TurtleBot a specified distance.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "distance": {"type": "number", "description": "Distance to move in meters."},
                "linear_velocity": {"type": "number", "description": "Linear velocity in m/s."}
            },
            "required": ["distance", "linear_velocity"],
            "additionalProperties": False
        }
    })

    # rotate_angle tool
    tools.append({
        "name": "rotate_angle",
        "description": "Rotates the TurtleBot by a specified angle.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "angle": {"type": "number", "description": "Angle to rotate in radians."},
                "angular_velocity": {"type": "number", "description": "Angular velocity in rad/s."}
            },
            "required": ["angle", "angular_velocity"],
            "additionalProperties": False
        }
    })

    return {
        "tools.v1": {
            "tools": tools
        }
    }

@app.route('/', methods=['GET', 'POST'])
def mcp_entrypoint():
    if request.method == 'GET':
        log_debug("HTTP GET / Request", {"headers": dict(request.headers)})
        return jsonify(MCP_DESCRIPTION)

    if not request.is_json:
        log_debug("Non-JSON Request Received", {"headers": dict(request.headers), "data": request.data.decode()})
        return jsonify(MCP_DESCRIPTION)

    req = request.get_json()
    log_debug("Received JSON-RPC Request", req)

    if "jsonrpc" not in req or req["jsonrpc"] != "2.0":
        return jsonrpc_error(req.get("id", None), -32600, "Invalid JSON-RPC version")

    req_id = req.get("id")
    method = req.get("method")
    params = req.get("params", {})
    is_notification = req_id is None

    if method == "initialize":
        capabilities = build_tools_capability()
        result = {
            "protocolVersion": "2025-06-18",
            "serverInfo": {
                "name": "TurtleBot MCP Server",
                "version": "1.0.0"
            },
            "capabilities": capabilities
        }
        return jsonrpc_response(req_id, result)

    elif method == "tools/list":
        capabilities = build_tools_capability()
        return jsonrpc_response(req_id, capabilities["tools.v1"])

    elif method == "tools/call":
        tool_name = params.get("name")
        arguments = params.get("arguments", {})

        if tool_name == "move":
            linear_x = arguments.get("linear_x")
            angular_z = arguments.get("angular_z")
            duration_s = arguments.get("duration_s")
            result = _move(linear_x, angular_z, duration_s)
            if result is None:
                return jsonrpc_error(req_id, -32000, "ROS 2 node not ready.")
            return jsonrpc_response(req_id, result)

        elif tool_name == "stop":
            result = _stop()
            if result is None:
                return jsonrpc_error(req_id, -32000, "ROS 2 node not ready.")
            return jsonrpc_response(req_id, result)

        elif tool_name == "move_distance":
            distance = arguments.get("distance")
            linear_velocity = arguments.get("linear_velocity")
            result = _move_distance(distance, linear_velocity)
            if result is None:
                return jsonrpc_error(req_id, -32000, "ROS 2 node not ready.")
            return jsonrpc_response(req_id, result)

        elif tool_name == "rotate_angle":
            angle = arguments.get("angle")
            angular_velocity = arguments.get("angular_velocity")
            result = _rotate_angle(angle, angular_velocity)
            if result is None:
                return jsonrpc_error(req_id, -32000, "ROS 2 node not ready.")
            return jsonrpc_response(req_id, result)

        else:
            return jsonrpc_error(req_id, -32601, f"Tool not found: {tool_name}")

    else:
        if is_notification:
            return '', 204
        return jsonrpc_error(req_id, -32601, f"Method not found: {method}")

@app.route('/health', methods=['GET'])
def health():
    return jsonify({"status": "ok"}), 200

if __name__ == '__main__':
    ros_thread = threading.Thread(target=ros_thread_entrypoint, daemon=True)
    ros_thread.start()

    print("Starting Flask MCP Server on http://0.0.0.0:5001")
    app.run(host='0.0.0.0', port=5001)