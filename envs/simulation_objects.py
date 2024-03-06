'''ToolBox for robot urdf test'''
import sys
sys.path.append('./')

import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data
from robots import CR5
import math
import numpy as np
from collections import namedtuple
from omegaconf import OmegaConf
import xml.etree.ElementTree as ET
from stl import mesh
import glob
import os


def get_object_info(dataset_root, object_name):
    if object_name == "Cube":
        urdf_path = os.path.join(dataset_root, object_name, "cube_small.urdf")
        force_range, deformation = FT_data_dict["RubiksCube"]
        return urdf_path, 1., 0.05, force_range, deformation
    
    urdf_path = os.path.join(dataset_root, object_name, "model.urdf")
    tree = ET.parse(urdf_path)
    mass_node = next(tree.iter('mass'))
    if mass_node is None:
        raise KeyError("No mass in the urdf file.")
    mass = float(mass_node.attrib["value"])

    friction_node = next(tree.iter('lateral_friction'))
    if friction_node is None:
        raise KeyError("No friction in the urdf file.")
    friction = float(friction_node.attrib["value"])

    obj_mesh_list = glob.glob(os.path.join(dataset_root, object_name, "*.obj"))
    if len(obj_mesh_list) > 0:
        obj_path = obj_mesh_list[0]
        height = _get_height(obj_path)
    else:
        mesh_file_name = os.path.join(dataset_root, object_name, "*.stl")
        mesh_file_list = glob.glob(mesh_file_name)
        stl_path = mesh_file_list[0]
        stl_file = mesh.Mesh.from_file(stl_path)
        height = np.max(stl_file.z) - np.min(stl_file.z)

    force_range = np.array(
        [0.013903, 0.08583400000000001, 0.18635599999999997, 0.301228, 0.44313, 0.6062639999999999,
        0.7980979999999996,
        1.006655, 1.255222, 1.498395, 1.791708, 2.10153, 2.4639089999999997, 2.873739, 3.3301070000000004,
        3.8420690000000004, 4.392766999999999, 4.958345, 5.535276, 6.171562, 6.803239000000002,
        7.445841000000001,
        8.154558, 8.841395, 9.530221, 10.181313, 10.924032999999998, 11.770725, 12.293285000000001,
        13.091981999999998])
    deformation = np.arange(0.0001, 0.0031, 0.0001)

    return urdf_path, mass, height, force_range, deformation, friction

def _get_height(obj_path):
        lcut = hcut = None
        with open(obj_path) as file:
            while 1:
                line = file.readline()
                if not line:
                    break
                strs = line.split(" ")
                if strs[0] == "v":
                    z_cor = float(strs[3])
                    if lcut is None or z_cor < lcut:
                        lcut = z_cor
                    if hcut is None or z_cor > hcut:
                        hcut = z_cor
                if strs[0] == "vt":
                    break
        return hcut - lcut


clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
    p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraPitch=-20,
    cameraYaw=180,
    cameraTargetPosition=[0, 0, 0.6],
)
p.loadURDF("plane.urdf", [0, 0, -0.0])
p.loadURDF("table/table.urdf", [0, 0.35, 0], [0, 0, 0, 1])
# load objects
p.loadURDF("models/plastic_cup/model.urdf", [0.05, 0.5, 0.675], globalScaling=1, useFixedBase=False)
p.loadURDF("models/apple/model.urdf", [0.15, 0.52, 0.675], globalScaling=0.0025, useFixedBase=False)
p.loadURDF("models/waterbottle/model.urdf", [-0.1, 0.3, 0.715], globalScaling=0.001, useFixedBase=True)
p.loadURDF("models/bagged_water/model.urdf", [0.35, 0.5, 0.69], globalScaling=0.9, useFixedBase=True)
p.loadURDF("models/banana/model.urdf", [0.15, 0.5, 0.675], globalScaling=0.001, useFixedBase=True)
p.loadURDF("models/bread/model.urdf", [-0.05, 0.7, 0.675], globalScaling=0.01, useFixedBase=False)
p.loadURDF("models/cherry/model.urdf", [-0.15, 0.7, 0.675], globalScaling=0.0002, useFixedBase=False)
p.loadURDF("models/cookies/model.urdf", [-0.25, 0.3, 0.675], globalScaling=0.9, useFixedBase=False)
p.loadURDF("models/egg/model.urdf", [-0.35, 0.7, 0.675], globalScaling=0.00002, useFixedBase=False)
p.loadURDF("models/insole/model.urdf", [-0.25, 0.4, 0.675], globalScaling=0.0008, useFixedBase=False)
p.loadURDF("models/plastic_jar/model.urdf", [0.05, 0.7, 0.675], globalScaling=1, useFixedBase=False)
p.loadURDF("models/potato_chip_bag/model.urdf", [0.15, 0.3, 0.675], globalScaling=1, useFixedBase=False)
p.loadURDF("models/tennis_ball/model.urdf", [0.25, 0.7, 0.675], globalScaling=0.0008, useFixedBase=False)
p.loadURDF("models/fabric/model.urdf", [0.35, 0.3, 0.675], globalScaling=0.45, useFixedBase=False)
p.loadURDF("models/orange/model.urdf", [0.45, 0.7, 0.675], globalScaling=0.8, useFixedBase=False)

    
while True:
    p.stepSimulation()