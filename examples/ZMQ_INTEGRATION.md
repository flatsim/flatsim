# ZMQ Robot Controller Integration Guide

## Overview

This document describes how to implement a robot controller that communicates with the Multi-Agent RL evaluation system via ZeroMQ (ZMQ). The controller receives target positions for robots and sends back arrival confirmations.

## Communication Protocol

**Pattern:** Request-Reply (REQ/REP)  
**Transport:** IPC (Inter-Process Communication)  
**Default Address:** `ipc:///tmp/robot_control.ipc`  
**Message Format:** JSON

## Message Flow

```
Evaluation System (REQ) → Robot Controller (REP)
    1. Sends initialization message with grid size and agent info
    
Robot Controller (REP) → Evaluation System (REQ)
    2. Sends initialization acknowledgment
    
Evaluation System (REQ) → Robot Controller (REP)
    3. Sends move command with target positions
    
Robot Controller (REP) → Evaluation System (REQ)
    4. Sends arrival confirmation after robots reach targets
    
[Repeat steps 3-4 for each simulation step]
```

## Message Formats

### 1. Initialization Message (Received by Controller)

Sent once at the start of evaluation.

```json
{
  "type": "init",
  "timestamp": 1700000000.123,
  "grid_size": 10,
  "num_agents": 2,
  "agents": ["agent_0", "agent_1"]
}
```

### 2. Initialization Acknowledgment (Sent by Controller)

```json
{
  "type": "init_ack",
  "timestamp": 1700000000.456
}
```

### 3. Move Command (Received by Controller)

```json
{
  "type": "move_command",
  "timestamp": 1700000000.123,
  "episode": 0,
  "step": 5,
  "current_poses": {
    "agent_0": {
      "x": 3.0,
      "y": 2.0
    },
    "agent_1": {
      "x": 7.0,
      "y": 4.0
    }
  },
  "target_poses": {
    "agent_0": {
      "x": 4.0,
      "y": 2.0
    },
    "agent_1": {
      "x": 7.0,
      "y": 5.0
    }
  }
}
```

### 4. Arrival Confirmation (Sent by Controller)

```json
{
  "type": "arrived",
  "timestamp": 1700000000.456,
  "agents": {
    "agent_0": true,
    "agent_1": true
  }
}
```

## Implementation Requirements

Your robot controller must:

1. **Connect** to the ZMQ REP socket at the specified IPC address
2. **Receive** move commands from the evaluation system
3. **Command real robots** to move to target positions
4. **Wait** for physical robots to arrive at targets
5. **Send** arrival confirmation back to evaluation system
6. **Repeat** for each step until evaluation completes

## C++ Implementation Example

### Dependencies

```bash
# Install ZeroMQ C++ bindings
sudo apt-get install libzmq3-dev libcppzmq-dev

# Install JSON library (nlohmann/json)
sudo apt-get install nlohmann-json3-dev
```

### Sample Code

```cpp
#include <zmq.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <string>

using json = nlohmann::json;

class RobotController {
private:
    zmq::context_t context;
    zmq::socket_t socket;
    
public:
    RobotController(const std::string& ipc_address) 
        : context(1), socket(context, zmq::socket_type::rep) {
        socket.connect(ipc_address);
        std::cout << "✓ Connected to " << ipc_address << std::endl;
    }
    
    void run() {
        int grid_size = 0;
        int num_robots = 0;
        std::vector<std::string> agent_names;
        
        while (true) {
            // Receive message
            zmq::message_t request;
            socket.recv(request, zmq::recv_flags::none);
            
            // Parse JSON
            std::string msg_str(static_cast<char*>(request.data()), request.size());
            json message = json::parse(msg_str);
            
            std::string msg_type = message["type"];
            
            if (msg_type == "init") {
                // Handle initialization
                grid_size = message["grid_size"];
                num_robots = message["num_agents"];
                agent_names = message["agents"].get<std::vector<std::string>>();
                
                std::cout << "\n✓ Received initialization:" << std::endl;
                std::cout << "  Grid size: " << grid_size << "x" << grid_size << std::endl;
                std::cout << "  Number of robots: " << num_robots << std::endl;
                std::cout << "  Agent names: ";
                for (const auto& name : agent_names) {
                    std::cout << name << " ";
                }
                std::cout << "\n" << std::endl;
                
                // TODO: Initialize your robot controllers here
                initializeRobots(grid_size, num_robots, agent_names);
                
                // Send acknowledgment
                json ack = {
                    {"type", "init_ack"},
                    {"timestamp", std::time(nullptr)}
                };
                std::string ack_str = ack.dump();
                zmq::message_t reply(ack_str.size());
                memcpy(reply.data(), ack_str.c_str(), ack_str.size());
                socket.send(reply, zmq::send_flags::none);
                
            } else if (msg_type == "move_command") {
                handleMoveCommand(message);
            }
        }
    }
    
private:
    void initializeRobots(int grid_size, int num_robots, 
                         const std::vector<std::string>& agent_names) {
        // TODO: Initialize your robot system here
        // Example: Allocate robot controllers, set grid boundaries, etc.
    }
    
    void handleMoveCommand(const json& message) {
        int episode = message["episode"];
        int step = message["step"];
        auto current_poses = message["current_poses"];
        auto target_poses = message["target_poses"];
        
        std::cout << "\n[Step " << step << "] Episode " << episode << std::endl;
        std::cout << "Target positions:" << std::endl;
        
        // Display targets
        for (auto& [agent_name, target] : target_poses.items()) {
            auto current = current_poses[agent_name];
            std::cout << "  " << agent_name << ": "
                      << "(" << current["x"] << "," << current["y"] << ") -> "
                      << "(" << target["x"] << "," << target["y"] << ")" << std::endl;
        }
        
        // TODO: Command your real robots to move to target positions
        moveRobotsToTargets(target_poses);
        
        // TODO: Wait for robots to arrive at targets
        waitForRobotsArrival();
        
        // Send arrival confirmation
        json response = {
            {"type", "arrived"},
            {"timestamp", std::time(nullptr)},
            {"agents", {}}
        };
        
        for (auto& [agent_name, _] : target_poses.items()) {
            response["agents"][agent_name] = true;
        }
        
        std::string response_str = response.dump();
        zmq::message_t reply(response_str.size());
        memcpy(reply.data(), response_str.c_str(), response_str.size());
        socket.send(reply, zmq::send_flags::none);
        
        std::cout << "  ✓ Sent arrival confirmation" << std::endl;
    }
    
    void moveRobotsToTargets(const json& target_poses) {
        // TODO: Implement your robot control logic here
        // Example: Send commands to robot actuators/motors
        // 
        // for (auto& [agent_name, target] : target_poses.items()) {
        //     double x = target["x"];
        //     double y = target["y"];
        //     sendCommandToRobot(agent_name, x, y);
        // }
    }
    
    void waitForRobotsArrival() {
        // TODO: Implement logic to wait for physical robots
        // Example: Poll robot sensors/encoders until positions match targets
        //
        // while (!allRobotsArrived()) {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //     checkRobotPositions();
        // }
    }
};

int main(int argc, char* argv[]) {
    std::string ipc_address = "ipc:///tmp/robot_control.ipc";
    
    if (argc > 1) {
        ipc_address = argv[1];
    }
    
    try {
        RobotController controller(ipc_address);
        std::cout << "Waiting for move commands..." << std::endl;
        controller.run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
```

### Compilation

```bash
g++ -std=c++17 robot_controller.cpp -o robot_controller -lzmq -lpthread
```

### Running

```bash
# Terminal 1: Run your robot controller
./robot_controller

# Terminal 2: Run the evaluation system
cd MARL_Pytorch
python ppo_crop_eval_zmq_sync.py --episodes 1 --max_steps 20 --zmq --device cuda
```

## Python Reference Implementation

The Python reference implementation is provided in `zmq_pose_subscriber.py`:

```python
#!/usr/bin/env python3
import zmq
import json
import time

# Setup ZMQ REP socket
context = zmq.Context()
socket = context.socket(zmq.REP)
socket.connect('ipc:///tmp/robot_control.ipc')

grid_size = 0
num_robots = 0
agent_names = []

while True:
    # Receive message
    message = socket.recv_json()
    
    if message['type'] == 'init':
        # Handle initialization
        grid_size = message['grid_size']
        num_robots = message['num_agents']
        agent_names = message['agents']
        
        print(f"✓ Grid size: {grid_size}x{grid_size}")
        print(f"✓ Number of robots: {num_robots}")
        print(f"✓ Agents: {agent_names}")
        
        # TODO: Initialize your robot controllers here
        
        # Send acknowledgment
        socket.send_json({
            'type': 'init_ack',
            'timestamp': time.time()
        })
        
    elif message['type'] == 'move_command':
        target_poses = message['target_poses']
        
        # TODO: Move real robots to targets
        # ... your robot control code here ...
        
        # Wait for user confirmation (replace with actual robot arrival check)
        input("Press ENTER when robots arrived >>> ")
        
        # Send arrival confirmation
        response = {
            'type': 'arrived',
            'timestamp': time.time(),
            'agents': {agent: True for agent in target_poses.keys()}
        }
        socket.send_json(response)
```

## Key Points

1. **REP Socket Type**: Your controller MUST use a REP (reply) socket, not REQ or any other type
2. **Connect, Don't Bind**: Use `socket.connect()`, not `socket.bind()` - the evaluation system binds
3. **Blocking Receive**: The evaluation system will wait indefinitely for your response - no timeout
4. **JSON Format**: All messages are JSON - parse incoming, serialize outgoing
5. **Synchronous**: One command → one response, strict alternation

## Grid Coordinate System

- **Grid size**: 10x10 (configurable)
- **Origin**: (0, 0) at top-left
- **X-axis**: Left to right (0 to 9)
- **Y-axis**: Top to bottom (0 to 9)
- **Units**: Grid cells (integers, but sent as floats)

## Testing

Use the provided Python reference implementation first to verify the integration works:

```bash
# Terminal 1
python zmq_pose_subscriber.py

# Terminal 2
python ppo_crop_eval_zmq_sync.py --episodes 1 --max_steps 10 --zmq
```

Once working, replace the Python controller with your C++ implementation.

## Troubleshooting

**"Address already in use"**
- Another process is bound to the IPC address
- Run: `rm /tmp/robot_control.ipc`

**"Connection refused"**
- Evaluation system not running
- Start evaluation system first

**"Operation cannot be accomplished in current state"**
- REQ/REP pattern violated (sent two requests without reply)
- Ensure strict alternation: receive → send → receive → send

**Messages not received**
- Check socket type (must be REP)
- Check IPC address matches
- Verify both processes running

## Contact

For questions or issues, refer to the evaluation system documentation or contact the simulation team.
