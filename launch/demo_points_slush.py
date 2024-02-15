<!--
  <!-- Start MoveIt -->
  <include file="$(find fetch_moveit_config)/launch/move_group.launch" >
    <arg name="info" value="true"/><!-- publish grasp markers -->
    <arg name="srdf" value="$(find fetch_gazebo)/robots/fetch.gazebo.srdf.xacro"/>
  </include>

  <!-- Put a robot in gazebo, make it look pretty
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
        args="-urdf -x $(arg x) -y $(arg y) -z $(arg z) -Y $(arg yaw) -model fetch -param robot_description"/>
  <node name="prepare_robot" pkg="fetch_gazebo" type="prepare_simulated_robot.py" />
  -->
-->

  <!-- Navigation parameter files 
  <arg name="move_base_include" default="$(find fetch_navigation)/launch/include/move_base.launch.xml" />
  <arg name="amcl_include" default="$(find fetch_navigation)/launch/include/amcl.launch.xml" />
  -->

  <!-- Give this robot a serial number and version -->
  <param name="robot/serial" value="ABCDEFGHIJKLMNOPQRSTUVWX" />
  <param name="robot/version" value="0.0.1" />

  <!-- Head Camera Pipeline -->
  <include file="$(find fetch_gazebo)/launch/include/head_camera.launch.xml" />

  <!-- Publish base_scan_raw if anything subscribes to it -->
  <node name="publish_base_scan_raw" pkg="topic_tools" type="relay" args="base_scan base_scan_raw" >
    <param name="lazy" type="bool" value="True"/>
  </node>

  <!-- Start a mux between application and teleop -->
  <node pkg="topic_tools" type="mux" name="cmd_vel_mux" respawn="true" args="base_controller/command /cmd_vel /teleop/cmd_vel">
    <remap from="mux" to="cmd_vel_mux" />
  </node>

  <!-- Start Perception 
  <node name="basic_grasping_perception" pkg="simple_grasping" type="basic_grasping_perception" >
    <rosparam command="load" file="$(find fetch_gazebo_demo)/config/simple_grasping.yaml" />
  </node>
  -->

    <!-- Setup controllers -->
  <rosparam file="$(find fetch_gazebo)/config/default_controllers.yaml" command="load" />

  <!-- Load the URDF, SRDF and other .yaml configuration files on the param server -->
  <include file="$(find fetch_moveit_config)/launch/planning_context.launch">
    <arg name="load_robot_description" value="true"/>
  </include>

  <!-- URDF and TF support 
  <param name="robot_description" command="$(find xacro)/xacro.py $(find fetch_gazebo)/robots/fetch.gazebo.xacro" />
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" >
    <param name="publish_frequency" value="100.0"/>
  </node>
  -->

<!-- <include file="$(find fetch_gazebo_demo)/launch/fetch_nav.launch" >
    <arg name="map_file" value="$(find fetch_maps)/maps/aims-ada.yaml" />
    <arg name="map_keepout_file" value="$(find fetch_maps)/maps/aims-keepout.yaml" />
    <arg name="use_keepout" value="false" />
  </include> -->

