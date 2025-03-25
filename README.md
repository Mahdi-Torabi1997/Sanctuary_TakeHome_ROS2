## Bonus Task: ROS 2 Integration

This bonus task demonstrates a ROS 2 integration where two nodes communicate over a topic to compute and verify both forward and inverse kinematics in real-time.

---

### Running the Project with Docker

You can reproduce the exact environment using the prebuilt Docker image.

1. Pull the image:

```bash
docker pull mahditorabi/ros2_takehome:latest
```

2. Run the container:

```bash
docker run -it mahditorabi/ros2_takehome:latest
```

3. Inside the container, source ROS 2 and workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

4. Launch the ROS 2 nodes:

```bash
ros2 launch cpp_pkg cpp_pkg_launch.py
```

---

### Running Locally (Without Docker)

If you're using your local ROS 2 Humble setup:

1. Build the workspace:

```bash
colcon build --packages-select cpp_pkg
source install/setup.bash
```

2. Launch the nodes:

```bash
ros2 launch cpp_pkg cpp_pkg_launch.py
```

---

### Architecture

- **publisher_node**
  - Reads `L1`, `L2`, `L3`, and joint angles `θ₁`, `θ₂`, `θ₃` as parameters.
  - Computes forward kinematics using `getEndEffectorPose()`.
  - Publishes the end-effector pose `(x, y, φ)` on the topic `/ee_pose`.

- **subscriber_node**
  - Subscribes to `/ee_pose`.
  - Uses the inverse kinematics solver to recover joint angles `θ₁`, `θ₂`, `θ₃`.
  - Prints recovered angles to the terminal.

**Communication**  
- Topic: `/ee_pose`  
- Message Type: `example_interfaces/msg/Float64MultiArray`  
- Payload: `[x, y, phi]` (3 elements)

---

### Launch File

The launch file `cpp_pkg_launch.py` starts both nodes with the specified parameters:

```bash
ros2 launch cpp_pkg cpp_pkg_launch.py
```

---

### Package Structure

```text
cpp_pkg/
├── src/
│   ├── publisher_node.cpp       # FK computation and publishing
│   └── subscriber_node.cpp      # IK computation and logging
├── launch/
│   └── cpp_pkg_launch.py
├── CMakeLists.txt
└── package.xml
```

---

### Example Output

Here’s how the output appears when running the nodes:

```
[publisher_node]: FK Publisher Node started
[subscriber_node]: IK Subscriber Node started
[publisher_node]: Published FK: x=0.555, y=0.415, orientation=0.600
[subscriber_node]: Received FK: x=0.555, y=0.415, phi=0.600
[subscriber_node]: IK solution: theta1=0.500, theta2=0.300, theta3=-0.200
```

This verifies that the forward kinematics from the given joint angles and the inverse kinematics from the pose match correctly.
