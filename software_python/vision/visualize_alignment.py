"""Small matplotlib 3D diagnostic view for camera/world alignment."""

from __future__ import annotations

import numpy as np

from .coordinate_transform import quaternion_wxyz_to_rotation


def show_alignment(camera_points, world_points, ekf_position=None, ekf_quaternion=None,
                   title="Gluvn alignment", source_label="source", target_label="transformed"):
    import matplotlib.pyplot as plt

    camera = np.asarray(camera_points, dtype=float)
    world = np.asarray(world_points, dtype=float)
    figure = plt.figure(title)
    camera_axis = figure.add_subplot(121, projection="3d")
    world_axis = figure.add_subplot(122, projection="3d")
    for axis, points, label, color in ((camera_axis, camera, source_label, "tab:orange"),
                                        (world_axis, world, target_label, "tab:cyan")):
        if len(points):
            axis.scatter(points[:, 0], points[:, 1], points[:, 2], c=color)
            axis.scatter(points[0, 0], points[0, 1], points[0, 2], c="black", marker="x")
        axis.set_title(label)
        axis.set_xlabel("X")
        axis.set_ylabel("Y")
        axis.set_zlabel("Z")
    if ekf_position is not None and ekf_quaternion is not None:
        origin = np.asarray(ekf_position, dtype=float)
        rotation = quaternion_wxyz_to_rotation(ekf_quaternion)
        for index, color in enumerate(("r", "g", "b")):
            world_axis.quiver(*origin, *rotation[:, index], length=0.1, color=color)
    figure.suptitle(title)
    plt.show()
