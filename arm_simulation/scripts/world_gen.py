#!/usr/bin/env python3

import random
import os
from pathlib import Path
from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom
import math
import json

# Where to save the generated world files
OUTPUT_DIR = "generated_worlds"
NUM_WORLDS = 100
OBJECTS_PER_WORLD = 5

# Object types and corresponding model URIs
OBJECTS = {
    "cube_s": "model://objects/small/cube",
    "cube_m": "model://objects/medium/cube",
    "cube_l": "model://objects/large/cube",
    "sphere_s": "model://objects/small/sphere",
    "sphere_m": "model://objects/medium/sphere",
    "sphere_l": "model://objects/large/sphere",
    "cylinder_s": "model://objects/small/cylinder",
    "cylinder_m": "model://objects/medium/cylinder",
    "cylinder_l": "model://objects/large/cylinder"
}

# Table dimensions (approx)
TABLE_X_MIN = 0.1
TABLE_X_MAX = 0.5
TABLE_Y_MIN = -0.3
TABLE_Y_MAX = 0.3
TABLE_Z = 1.05  # Table height

OBJECT_GEOMETRY = {
    "cube_s": ("box", "0.025 0.025 0.025"),
    "cube_m": ("box", "0.05 0.05 0.05"),
    "cube_l": ("box", "0.1 0.1 0.1"),
    "sphere_s": ("sphere", "0.0125"),
    "sphere_m": ("sphere", "0.025"),
    "sphere_l": ("sphere", "0.05"),
    "cylinder_s": ("cylinder", "0.0125 0.0125"),
    "cylinder_m": ("cylinder", "0.025 0.025"),
    "cylinder_l": ("cylinder", "0.05 0.05"),
}

OBJECT_RADII = {
    "cube_s": 0.025,
    "cube_m": 0.05,
    "cube_l": 0.1,
    "sphere_s": 0.0125,
    "sphere_m": 0.025,
    "sphere_l": 0.05,
    "cylinder_s": 0.0125,
    "cylinder_m": 0.025,
    "cylinder_l": 0.05
}

MIN_DISTANCE = 0.08  # minimum center-to-center distance between objects


def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def is_valid_position(new_pos, existing_positions, new_radius):
    for pos, radius in existing_positions:
        if distance(new_pos, pos) < (radius + new_radius + MIN_DISTANCE):
            return False
    return True


def random_position(existing_positions, obj_radius, max_attempts=100):
    for _ in range(max_attempts):
        x = round(random.uniform(TABLE_X_MIN, TABLE_X_MAX), 3)
        y = round(random.uniform(TABLE_Y_MIN, TABLE_Y_MAX), 3)
        if is_valid_position((x, y), existing_positions, obj_radius):
            return (x, y)
    raise RuntimeError("Couldn't find non-overlapping position for object.")


def random_pose():
    x = round(random.uniform(TABLE_X_MIN, TABLE_X_MAX), 3)
    y = round(random.uniform(TABLE_Y_MIN, TABLE_Y_MAX), 3)
    yaw = round(random.uniform(0, 6.28), 2)
    return f"{x} {y} {TABLE_Z} 0 0 {yaw}"


def random_color():
    r = round(random.uniform(0.1, 1.0), 2)
    g = round(random.uniform(0.1, 1.0), 2)
    b = round(random.uniform(0.1, 1.0), 2)
    return f"{r} {g} {b} 1"


def prettify(elem):
    rough_string = tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def create_world(world_index):
    sdf = Element('sdf', version="1.6")
    world = SubElement(sdf, 'world', name="arm_world")

    # Plugins, physics, lights, table
    for elem in _include_basic_plugins():
        world.append(elem)

    world.append(_include_sun())
    world.append(_include_table())
    world.append(_include_ground())

    object_data = []

    for i in range(OBJECTS_PER_WORLD):
        obj_type = random.choice(list(OBJECTS.keys()))
        pose_str = random_pose()
        color = random_color()

        model_name = f"{obj_type}_{i}"
        _add_custom_object(world, obj_type, model_name, pose_str, color)
        # Save object info to JSON
        x, y, z, roll, pitch, yaw = map(float, pose_str.split())
        r, g, b, a = map(float, color.split())
        object_data.append({
            "name": model_name,
            "type": obj_type,
            "position": {"x": x, "y": y, "z": z},
            "rotation": {"roll": roll, "pitch": pitch, "yaw": yaw},
            "color": {"r": r, "g": g, "b": b, "a": a}
        })

    # Save .sdf file
    sdf_pretty = prettify(sdf)
    sdf_path = os.path.join(OUTPUT_DIR, f"world_{world_index}.sdf")
    with open(sdf_path, "w") as f:
        f.write(sdf_pretty)

    # Save .json metadata
    json_path = os.path.join(OUTPUT_DIR, f"world_{world_index}.json")
    with open(json_path, "w") as jf:
        json.dump(object_data, jf, indent=2)

    print(f"Generated: {sdf_path} and {json_path}")


def _include_basic_plugins():
    elements = []

    plugin_physics = Element('plugin', filename="gz-sim-physics-system", name="gz::sim::systems::Physics")
    engine = SubElement(plugin_physics, 'engine')
    filename = SubElement(engine, 'filename')
    filename.text = "libgz-physics-bullet-featherstone-plugin.so"
    elements.append(plugin_physics)

    for name, filename in [
        ("gz::sim::systems::UserCommands", "gz-sim-user-commands-system"),
        ("gz::sim::systems::SceneBroadcaster", "gz-sim-scene-broadcaster-system")
    ]:
        plugin = Element('plugin', filename=filename, name=name)
        elements.append(plugin)

    plugin_sensors = Element('plugin', filename="gz-sim-sensors-system", name="gz::sim::systems::Sensors")
    render = SubElement(plugin_sensors, 'render_engine')
    render.text = "ogre2"
    elements.append(plugin_sensors)

    gravity = Element('gravity')
    gravity.text = "0.0 0.0 -9.8"
    elements.append(gravity)

    scene = Element('scene')
    shadows = SubElement(scene, 'shadows')
    shadows.text = "false"
    elements.append(scene)

    gui = Element('gui')
    camera = SubElement(gui, 'camera', name="user_camera")
    pose = SubElement(camera, 'pose')
    pose.text = "0.75 -0.75 1.4 0 0.29 2.21"
    elements.append(gui)

    return elements


def _include_sun():
    include = Element('include')
    uri = SubElement(include, 'uri')
    uri.text = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun"
    return include


def _include_table():
    model = Element('model', name="table")
    static = SubElement(model, 'static')
    static.text = "true"
    include = SubElement(model, 'include')
    uri = SubElement(include, 'uri')
    uri.text = "model://table"
    pose = SubElement(model, 'pose')
    pose.text = "0.29 0 0 0 0 1.5708"
    return model



def _add_custom_object(world, obj_type, name, pose_str, color_str):
    shape_type, size = OBJECT_GEOMETRY[obj_type]

    model = SubElement(world, 'model', name=name)
    pose = SubElement(model, 'pose')
    pose.text = pose_str

    link = SubElement(model, 'link', name=f"{name}_link")

    # Collision
    collision = SubElement(link, 'collision', name=f"{name}_collision")
    geometry = SubElement(collision, 'geometry')
    shape = SubElement(geometry, shape_type)
    if shape_type == "box":
        size_el = SubElement(shape, 'size')
        size_el.text = size
    elif shape_type == "sphere":
        radius_el = SubElement(shape, 'radius')
        radius_el.text = size
    elif shape_type == "cylinder":
        radius, length = size.split()
        radius_el = SubElement(shape, 'radius')
        radius_el.text = radius
        length_el = SubElement(shape, 'length')
        length_el.text = length

    # Visual
    visual = SubElement(link, 'visual', name=f"{name}_visual")
    geometry_v = SubElement(visual, 'geometry')
    shape_v = SubElement(geometry_v, shape_type)
    if shape_type == "box":
        size_el = SubElement(shape_v, 'size')
        size_el.text = size
    elif shape_type == "sphere":
        radius_el = SubElement(shape_v, 'radius')
        radius_el.text = size
    elif shape_type == "cylinder":
        radius, length = size.split()
        radius_el = SubElement(shape_v, 'radius')
        radius_el.text = radius
        length_el = SubElement(shape_v, 'length')
        length_el.text = length

    # Damping
    velocity_decay = SubElement(link, 'velocity_decay')
    linear = SubElement(velocity_decay, 'linear')
    linear.text = "1"  # tweak as needed
    angular = SubElement(velocity_decay, 'angular')
    angular.text = "1"  # tweak as needed

    material = SubElement(visual, 'material')
    ambient = SubElement(material, 'ambient')
    ambient.text = color_str
    diffuse = SubElement(material, 'diffuse')
    diffuse.text = color_str


def _include_ground():
    model = Element('model', name="ground_plane")
    static = SubElement(model, 'static')
    static.text = "true"
    link = SubElement(model, 'link', name="link")
    collision = SubElement(link, 'collision', name="collision")
    geometry = SubElement(collision, 'geometry')
    plane = SubElement(geometry, 'plane')
    normal = SubElement(plane, 'normal')
    normal.text = "0 0 1"

    surface = SubElement(collision, 'surface')
    friction = SubElement(surface, 'friction')
    ode = SubElement(friction, 'ode')
    mu = SubElement(ode, 'mu')
    mu.text = "1.0"
    mu2 = SubElement(ode, 'mu2')
    mu2.text = "1.0"

    visual = SubElement(link, 'visual', name="visual")
    geometry_v = SubElement(visual, 'geometry')
    plane_v = SubElement(geometry_v, 'plane')
    normal_v = SubElement(plane_v, 'normal')
    normal_v.text = "0 0 1"
    size = SubElement(plane_v, 'size')
    size.text = "100 100"
    material = SubElement(visual, 'material')
    for tag in ["ambient", "diffuse", "specular"]:
        el = SubElement(material, tag)
        el.text = "0.8 0.8 0.8 1"

    return model


def generate_worlds():
    Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)

    for i in range(NUM_WORLDS):
        create_world(i)


if __name__ == "__main__":
    generate_worlds()
