import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from flask import Flask, request, jsonify
import threading
import time
import sys
import queue
import json

# === ROS 2 Setup ===

controller_node = None
executor = None
command_queue = queue.Queue()

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_mcp_server')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('TurtleBot MCP Server Node started.')

    def send_twist_command(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing Twist: linear.x={linear_x}, angular.z={angular_z}')

class CommandProcessorNode(Node):
    def __init__(self):
        super().__init__('command_processor_node')
        self.active_move_command = None
        self.timer = self.create_timer(0.1, self.process_queue_callback)
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
        except queue.Empty:
            pass

        # Process the active move command
        if self.active_move_command:
            now = self.get_clock().now().nanoseconds / 1e9
            
            if 'end_time' in self.active_move_command and self.active_move_command['end_time'] is not None:
                if now >= self.active_move_command['end_time']:
                    # Duration is over, stop the robot
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

def ros_thread_entrypoint():
    global controller_node, executor
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
        command_queue.put({'type': 'move', 'linear_x': linear_x, 'angular_z': angular_z, 'duration_s': duration_s})
        return {'status': 'success', 'linear_x': linear_x, 'angular_z': angular_z, 'duration_s': duration_s}
    else:
        return None

def _stop():
    if controller_node and executor:
        command_queue.put({'type': 'stop'})
        return {'status': 'success', 'message': 'Robot stopped.'}
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