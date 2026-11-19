"""Tiny forward kinematics for the hand URDF (numpy only): link poses from joint values."""
import xml.etree.ElementTree as ET

import numpy as np


def _rpy(rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def _axis_angle(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * k + (1 - np.cos(q)) * k @ k


def _vec(text, default="0 0 0"):
    return np.array([float(v) for v in (text or default).split()])


class Hand:
    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        self.joints = {}
        for j in root.findall("joint"):
            origin = j.find("origin")
            axis = j.find("axis")
            self.joints[j.get("name")] = dict(
                parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                xyz=_vec(origin.get("xyz") if origin is not None else None),
                rpy=_vec(origin.get("rpy") if origin is not None else None),
                axis=_vec(axis.get("xyz")) if axis is not None else None,
                type=j.get("type"), limit=(float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
                if j.find("limit") is not None else None)

    def poses(self, q):
        """link -> (R, p) in the world frame for joint values q (dict, default 0)."""
        poses = {"world": (np.eye(3), np.zeros(3))}
        pending = dict(self.joints)
        while pending:
            for name, j in list(pending.items()):
                if j["parent"] in poses:
                    R, p = poses[j["parent"]]
                    Rj = _rpy(j["rpy"])
                    if j["type"] == "revolute":
                        Rj = Rj @ _axis_angle(j["axis"], q.get(name, 0.0))
                    poses[j["child"]] = (R @ Rj, p + R @ j["xyz"])
                    del pending[name]
        return poses
