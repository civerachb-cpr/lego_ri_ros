Lego Spike Description
=======================

This package contains description files for the Lego Spike and Mindstorms
Robot Inventor sets.  Not all Lego parts have descriptions, but sensors
and motors do.

Motors are configured to use continuous joints that rotate around the Z axis.

Distance and Color sensors have their sensing apparatus pointed along the positive X axis.


Usage
------

You must include the `lego_materials.urdf.xacro` file in addition to any specific components your robot needs.
For convenience, `lego_dimensions.urdf.xacro` contains additional properties to help align URDF components with
Lego's standard plate & stud dimensions.  The origins of all sensors and motors are set such that they will align
with each other in XYZ offsets of `${lego_stud_spacing}` multiplied by increments of `0.5`.  For example, the following
will move the origin along the X axis by 3 studs, the Y axis by -1 stud, and the Z axis by 1.5 studs:
```
<origin xyz="${lego_stud_spacing * 3} -${lego_stud_spacing} ${lego_stud_spacing * 1.5}" />
```

For more complete examples, showing rotations and adding multiple components see `basic_lego_robot.urdf.xacro` and/or
`simple_arm.urdf.xacro`.

You can use Stud.io to design a complete lego model and export some/all of it as a Collada mesh (.DAE) and import that
mesh into your URDF:
```
<link name="custom_lego_model">
  <visual>
    <geometry>
      <!--
        Make sure your exported DAE is included in a package in a sourced workspace.
      -->
      <mesh filename="package://my_lego_design/meshes/my_lego_model.dae" scale="0.001 0.001 0.001" />

      <!--
        Adjust the XYZ and RPY offset as needed to align your model
      -->
      <origin xyz="..." rpy="..." />
    </geometry>
    <
  </visual>
</link>
```
