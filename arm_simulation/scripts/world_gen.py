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


def _shape_tag_from_obj_type(obj_type):
    if "cube" in obj_type:
        return "box"
    elif "sphere" in obj_type:
        return "sphere"
    elif "cylinder" in obj_type:
        return "cylinder"
    else:
        raise ValueError(f"Unknown object type: {obj_type}")


def _get_dimensions(obj_type):
    # Assign consistent sizes
    if "cube" in obj_type:
        return 0.05, 0.05  # radius (for box: half-size), height
    elif "sphere" in obj_type:
        return 0.025, 0.025  # radius, dummy height
    elif "cylinder" in obj_type:
        return 0.025, 0.025  # radius, length
    else:
        raise ValueError(f"Unknown object type: {obj_type}")


def _add_custom_object(world, obj_type, name, pose_str, color_str):
    geometry_type, size_str = OBJECT_GEOMETRY[obj_type]
    size_vals = list(map(float, size_str.split()))

    model = SubElement(world, 'model', name=name)
    pose = SubElement(model, 'pose')
    pose.text = pose_str

    link = SubElement(model, 'link', name=f"{name}_link")

    # === Inertial ===
    inertial = SubElement(link, 'inertial')
    mass = SubElement(inertial, 'mass')
    mass.text = "0.05"  # Fixed for now or calculate from size if you want
    inertia = SubElement(inertial, 'inertia')
    SubElement(inertia, 'ixx').text = "2.04e-7"
    SubElement(inertia, 'iyy').text = "2.04e-7"
    SubElement(inertia, 'izz').text = "3.9e-8"
    SubElement(inertia, 'ixy').text = "0"
    SubElement(inertia, 'ixz').text = "0"
    SubElement(inertia, 'iyz').text = "0"

    # === Collision ===
    collision = SubElement(link, 'collision', name=f"{name}_collision")
    geometry = SubElement(collision, 'geometry')
    shape = SubElement(geometry, geometry_type)

    if geometry_type == "box":
        SubElement(shape, 'size').text = size_str
    elif geometry_type == "sphere":
        SubElement(shape, 'radius').text = size_str
    elif geometry_type == "cylinder":
        radius, length = size_vals
        SubElement(shape, 'radius').text = str(radius)
        SubElement(shape, 'length').text = str(length)

    # Surface/friction/contact/bounce
    surface = SubElement(collision, 'surface')
    friction = SubElement(surface, 'friction')
    ode = SubElement(friction, 'ode')
    SubElement(ode, 'mu').text = "0.8"
    SubElement(ode, 'mu2').text = "0.8"
    SubElement(ode, 'fdir1').text = "0 0 1"
    SubElement(ode, 'slip1').text = "0.001"
    SubElement(ode, 'slip2').text = "0.001"

    torsional = SubElement(friction, 'torsional')
    SubElement(torsional, 'coefficient').text = "0.1"
    SubElement(torsional, 'surface_radius').text = "0.015"
    SubElement(torsional, 'use_patch_radius').text = "true"

    contact = SubElement(surface, 'contact')
    contact_ode = SubElement(contact, 'ode')
    SubElement(contact_ode, 'kp').text = "1e5"
    SubElement(contact_ode, 'kd').text = "10"
    SubElement(contact_ode, 'max_vel').text = "0.1"
    SubElement(contact_ode, 'min_depth').text = "0.001"
    SubElement(contact_ode, 'soft_cfm').text = "0.01"
    SubElement(contact_ode, 'soft_erp').text = "0.2"

    bounce = SubElement(surface, 'bounce')
    SubElement(bounce, 'restitution_coefficient').text = "0.2"
    SubElement(bounce, 'threshold').text = "0.01"

    # === Visual ===
    visual = SubElement(link, 'visual', name=f"{name}_visual")
    geometry_v = SubElement(visual, 'geometry')
    shape_v = SubElement(geometry_v, geometry_type)

    if geometry_type == "box":
        SubElement(shape_v, 'size').text = size_str
    elif geometry_type == "sphere":
        SubElement(shape_v, 'radius').text = size_str
    elif geometry_type == "cylinder":
        SubElement(shape_v, 'radius').text = str(radius)
        SubElement(shape_v, 'length').text = str(length)

    material = SubElement(visual, 'material')
    SubElement(material, 'ambient').text = color_str
    SubElement(material, 'diffuse').text = color_str

    # === Damping ===
    velocity_decay = SubElement(link, 'velocity_decay')
    SubElement(velocity_decay, 'linear').text = "1"
    SubElement(velocity_decay, 'angular').text = "1"

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
