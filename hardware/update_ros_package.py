from pathlib import Path
import shutil

URDF_EXTRA_ROS2_CONTROL = '''
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>

    <joint name="left_steering_joint">
      <command_interface name="position">
        <param name="min">-10</param>
        <param name="max">10</param>
      </command_interface>
      <state_interface name="position">
        <param name="initial_value">1.0</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="right_steering_joint">
      <command_interface name="position">
        <param name="min">-10</param>
        <param name="max">10</param>
      </command_interface>
      <state_interface name="position">
        <param name="initial_value">1.0</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <joint name="left_rear_axle">
      <command_interface name="velocity">
        <param name="min">-10</param>
        <param name="max">10</param>
      </command_interface>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="effort"/>
    </joint>

    <joint name="right_rear_axle">
      <command_interface name="velocity">
        <param name="min">-10</param>
        <param name="max">10</param>
      </command_interface>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>

  <gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <robot_param>robot_description</robot_param>
      <robot_param_node>robot_state_publisher</robot_param_node>
      <parameters>/truck/packages/truck_description/config/controller.yaml</parameters>
    </plugin>
  </gazebo>
'''

URDF_SRC = Path("hardware/model/model.urdf")
XACRO_DST = Path("packages/truck_description/urdf/truck.xacro")
MESHES_SRC = Path("hardware/model/meshes")
MESHES_DST = Path("/opt/gzweb/http/client/assets/hardware/model/meshes")

with open(URDF_SRC) as file:
    content = file.read()
    content = content.replace("</robot>", URDF_EXTRA_ROS2_CONTROL)
    content += "</robot>"

with open(XACRO_DST, "w") as file:
    print(f"Writing URDF to {XACRO_DST}")
    file.write(content)

MESHES_DST.mkdir(parents=True, exist_ok=True)
for path in MESHES_SRC.glob("*"):
    src = MESHES_SRC / path.name
    dst = MESHES_DST / path.name
    print(f"Copying {src} -> {dst}")
    shutil.copy(src, dst)
