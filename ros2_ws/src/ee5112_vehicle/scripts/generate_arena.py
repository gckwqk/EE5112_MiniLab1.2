#!/usr/bin/env python3

import json
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parent.parent

JSON_FILE = (
    PACKAGE_DIR
    / "config"
    / "MiniLab1.2_platform_specs_5112.json"
)

OUTPUT_FILE = (
    PACKAGE_DIR
    / "worlds"
    / "arena.world"
)


def wall_model(name, x, y, w, h, wall_height):
    """
    JSON wall coordinates give the south-west floor corner.
    Gazebo box pose uses its centre.
    """

    cx = x + w / 2.0
    cy = y + h / 2.0
    cz = wall_height / 2.0

    return f"""
    <model name="{name}">
      <static>true</static>

      <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>

      <link name="link">

        <collision name="collision">
          <geometry>
            <box>
              <size>{w:.4f} {h:.4f} {wall_height:.4f}</size>
            </box>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <box>
              <size>{w:.4f} {h:.4f} {wall_height:.4f}</size>
            </box>
          </geometry>

          <material>
            <ambient>0.70 0.70 0.70 1.00</ambient>
            <diffuse>0.70 0.70 0.70 1.00</diffuse>
            <specular>0.10 0.10 0.10 1.00</specular>
          </material>
        </visual>

      </link>
    </model>
"""


def block_model(name, x, y, size, rgba):
    """
    JSON block x/y values are block-centre coordinates.

    A cube of height 0.08 m therefore has centre z=0.04 m.
    """

    z = size / 2.0

    return f"""
    <model name="block_{name.lower()}">
      <static>true</static>

      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>

      <link name="link">

        <collision name="collision">
          <geometry>
            <box>
              <size>{size:.4f} {size:.4f} {size:.4f}</size>
            </box>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <box>
              <size>{size:.4f} {size:.4f} {size:.4f}</size>
            </box>
          </geometry>

          <material>
            <ambient>{rgba}</ambient>
            <diffuse>{rgba}</diffuse>
            <specular>0.10 0.10 0.10 1.00</specular>
            <emissive>0 0 0 0</emissive>
          </material>
        </visual>

      </link>
    </model>
"""


def main():

    with JSON_FILE.open("r", encoding="utf-8") as f:
        specs = json.load(f)

    arena = specs["map"]

    wall_height = arena["wall_height_m"]
    block_size = arena["block_size_m"]

    outer_walls = arena["outer_walls_axis_aligned"]
    inner_walls = arena["inner_walls_axis_aligned"]

    block_positions = arena["block_positions_xy_m"]
    block_colours = arena["block_visual_rgba"]

    models = []

    # ----------------------------------------------------------
    # Outer walls
    # ----------------------------------------------------------

    for index, wall in enumerate(outer_walls, start=1):

        models.append(
            wall_model(
                name=f"outer_wall_{index}",
                x=wall["x"],
                y=wall["y"],
                w=wall["w"],
                h=wall["h"],
                wall_height=wall_height,
            )
        )

    # ----------------------------------------------------------
    # Inner walls
    # ----------------------------------------------------------

    for wall in inner_walls:

        models.append(
            wall_model(
                name=wall["name"],
                x=wall["x"],
                y=wall["y"],
                w=wall["w"],
                h=wall["h"],
                wall_height=wall_height,
            )
        )

    # ----------------------------------------------------------
    # Coloured blocks
    # ----------------------------------------------------------

    for colour, position in block_positions.items():

        rgba = block_colours[colour]["gazebo_rgba"]

        models.append(
            block_model(
                name=colour,
                x=position[0],
                y=position[1],
                size=block_size,
                rgba=rgba,
            )
        )

    # ----------------------------------------------------------
    # Complete SDF world
    # ----------------------------------------------------------

    world = f"""<?xml version="1.0" ?>

<sdf version="1.6">

  <world name="ee5112_arena">

    <!-- ====================================================== -->
    <!-- PHYSICS                                                -->
    <!-- ====================================================== -->

    <physics name="default_physics"
             default="true"
             type="ode">

      <gravity>0 0 -9.81</gravity>

      <ode>

        <solver>
          <type>quick</type>
          <iters>100</iters>
          <sor>1.3</sor>
        </solver>

        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>
            100.0
          </contact_max_correcting_vel>
          <contact_surface_layer>
            0.001
          </contact_surface_layer>
        </constraints>

      </ode>

      <max_step_size>0.001</max_step_size>

      <real_time_update_rate>
        1000
      </real_time_update_rate>

    </physics>


    <!-- ====================================================== -->
    <!-- LIGHTING                                               -->
    <!-- ====================================================== -->

    <include>
      <uri>model://sun</uri>
    </include>


    <!-- ====================================================== -->
    <!-- GROUND                                                 -->
    <!-- ====================================================== -->

    <include>
      <uri>model://ground_plane</uri>
    </include>


    <!-- ====================================================== -->
    <!-- ARENA + BLOCKS                                         -->
    <!-- ====================================================== -->

    {''.join(models)}


    <!-- ====================================================== -->
    <!-- GUI CAMERA                                             -->
    <!-- ====================================================== -->

    <gui fullscreen="0">

      <camera name="user_camera">

        <pose>
          2.10 -2.00 4.50
          0 0.85 1.57
        </pose>

      </camera>

    </gui>

  </world>

</sdf>
"""

    OUTPUT_FILE.write_text(
        world,
        encoding="utf-8"
    )

    print(f"Generated: {OUTPUT_FILE}")

    print(
        f"Walls: {len(outer_walls) + len(inner_walls)}"
    )

    print(
        f"Blocks: {len(block_positions)}"
    )


if __name__ == "__main__":
    main()
