<?xml version="1.0"?>
<launch>
  <!-- Velodyne lidar-->
  <!--
  <group ns="velodyne">
      <include file="$(find velodyne_pointcloud)/launch/VLP16_points.launch"/>
  </group>
  -->
    
  <!-- MCU and IMU-->
  <node pkg="mcu_interface" type="mcu" name="mcu" args="/dev/ttyUSB0"/>
</launch>
