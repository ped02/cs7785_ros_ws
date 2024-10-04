import numpy as np
from numbers import Number
from typing import Tuple


def to_homogeneous(vector: np.ndarray) -> np.ndarray:
    """Add 1 to last dimension"""
    return np.concatenate([vector, np.ones((len(vector), 1))], axis=-1)


def get_camera_intrinsic(
    camera_f: Number,
    sensor_width: Number,
    sensor_height: Number,
    image_width: int,
    image_height: int,
) -> np.ndarray:
    """
    Return
    -------
    Camera Intrinsics Matrix 3 x 3"""
    image_center_x = image_width / 2
    image_center_y = image_height / 2

    intrinsics = np.array(
        [
            [camera_f * image_width / sensor_width, 0.0, image_center_x],
            [0.0, camera_f * image_height / sensor_height, image_center_y],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )

    return intrinsics


def pixels_to_world_point(
    camera_extrinsics: np.ndarray,
    camera_intrinsics_inv: np.ndarray,
    pixel_points: np.ndarray,
    projection_distance: Number,
) -> np.ndarray:
    """Transform pixels to world coordinate (homogeneous coordinate)

    Parameters
    --------
    camera_extrinsics: 4 x 4

    camera_intrinsics: 3 x 3

    pixel_points: N x 3 pixels in homogeneous coordinate

    projection_distance: number, projecting plane distance (ambiguous due to inverse projection)

    Returns
    -------
    world coordinate N x 4 (homogeneous coordinate)
    """
    # pixel_points in homogeneous coordinate

    # Pixel to Image Plane
    image_plane_points = pixel_points @ camera_intrinsics_inv.T
    image_plane_points *= projection_distance

    # Image Plane to World
    image_plane_points_homogeneous = to_homogeneous(image_plane_points)
    world_points = image_plane_points_homogeneous @ camera_extrinsics.T

    world_points /= world_points[..., -1]

    return world_points


def get_image_plane_polygon(
    camera_extrinsics: np.ndarray,
    image_width: int,
    image_height: int,
    camera_intrinsics_inv: np.ndarray,
    camera_f: Number,
):
    """Get image plane polygon (corners) in world coordinate

    Order: Top left, Top right, Bottom right, bottom left (clockwise)"""
    # Left Top first then cw
    pixel_vectors = np.array(
        [
            [0, 0, 1],
            [image_width, 0, 1],
            [image_width, image_height, 1],
            [0, image_height, 1],
        ],
        dtype=np.float32,
    )
    world_vectors = pixels_to_world_point(
        camera_extrinsics, camera_intrinsics_inv, pixel_vectors, camera_f
    )

    return world_vectors


def bounding_polygon_to_inward_planes(
    bounding_polygon: np.ndarray,
    camera_intrinsics_inv: np.ndarray,
    camera_f: Number,
    camera_extrinsics: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Parameters
    ----------
    bounding_polygon: array N x 2 of polygon pixels

    Returns
    -------
    planes_normal_unit: planes normal unit vector, pointing inward to the bounding area
    planes_d: d value of plane

    Note: ax + by + cz = d
    """
    # List of points [[x0, y0], [x1, y1], ...]
    # N x 2

    # N x 4
    bounding_point_vectors_homogeneous = pixels_to_world_point(
        camera_extrinsics,
        camera_intrinsics_inv,
        to_homogeneous(bounding_polygon),
        camera_f,
    )

    # N x 3
    bounding_point_vectors = np.divide(
        bounding_point_vectors_homogeneous[:, :-1],
        bounding_point_vectors_homogeneous[:, [-1]],
    )

    # Transform to planes
    camera_center = camera_extrinsics[:3, 3]

    edge_vector = (
        np.roll(bounding_point_vectors, -1, axis=0) - bounding_point_vectors
    )
    camera_center_to_corner = bounding_point_vectors - camera_center

    inward_planes_normal = np.cross(edge_vector, camera_center_to_corner)

    # N x 3
    inward_planes_normal_unit = np.divide(
        inward_planes_normal,
        np.linalg.norm(inward_planes_normal, axis=-1, keepdims=True),
    )

    plane_d = np.sum(
        np.multiply(inward_planes_normal_unit, bounding_point_vectors), axis=-1
    )

    return inward_planes_normal_unit, plane_d


def get_plane_side_mask(
    points,
    planes_n,
    planes_d,
    same_side_as_normal: bool = True,
    on_plane_valid: bool = True,
) -> np.ndarray:
    """Get mask for points on a side of the given planes

    Parameters
    ----------
    points: np.ndarray[N x 3]
        array of points

    planes_n: np.ndarray[M x 3]
        plane normal unit vector

    planes_d: np.ndarray[M]
        plane d value. Note: ax + by + cz = d

    same_side_as_normal: bool
        whether to take the same or different side of normal as valid

    on_plane_valid: bool
        whether to take the points on the plane as valid

    Returns
    -------
    mask: np.ndarry[bool, N]
        Array of mask

    Note: Use loop to reduce computation on filtered components. Vectorize use a lot of memory and extra compute
    """

    candidate_indices = np.arange(len(points))
    candidate_points = points

    for n, d in zip(planes_n, planes_d):
        normal_dot = (
            np.sum(np.multiply(candidate_points, np.expand_dims(n, 0)), axis=-1)
            - d
        )

        candidate_mask = (
            normal_dot > 0 if same_side_as_normal else normal_dot < 0
        )
        candidate_mask = (
            candidate_mask
            if not on_plane_valid
            else np.logical_or(normal_dot == 0, candidate_mask)
        )

        candidate_points = candidate_points[candidate_mask]
        candidate_indices = candidate_indices[candidate_mask]

    mask = np.zeros(len(points), dtype=bool)
    mask[candidate_indices] = True
    return mask


def plane_to_polygon(
    planes_n: np.ndarray,
    planes_d: np.ndarray,
    x_bounds: np.ndarray,
    y_bounds: np.ndarray,
    z_bounds: np.ndarray,
) -> np.ndarray:
    """Approximate plane using polygon within a specified bounds

    Parameters
    ----------
    planes_n: np.ndarray[N x 3]
        plane normal unit vector

    planes_d: np.ndarray[N]
        plane d value. Note: ax + by + cz = d

    x_bounds: np.ndarray[2]
        x bounds to generate the polygon

    y_bounds: np.ndarray[2]
        y bounds to generate the polygon

    z_bounds: np.ndarray[2]
        z bounds to generate the polygon

    Returns
    -------
    plane_polygon: np.ndarry[N, 4, 3]
        Array of polygon. 4 corner points for each plane, in 3 dimension.

    """
    planes_d_expanded = np.expand_dims(planes_d, -1)

    max_index = np.argmax(np.abs(planes_n), axis=-1)

    corner_pair_orient = np.array([0, 1, 3, 2])

    # 1 x (2*2) x 2
    xy_pair = np.stack(np.meshgrid(x_bounds, y_bounds), axis=-1).reshape(
        1, -1, 2
    )[:, corner_pair_orient]
    yz_pair = np.stack(np.meshgrid(y_bounds, z_bounds), axis=-1).reshape(
        1, -1, 2
    )[:, corner_pair_orient]
    zx_pair = np.stack(np.meshgrid(z_bounds, x_bounds), axis=-1).reshape(
        1, -1, 2
    )[:, corner_pair_orient]

    # N x (2x2)
    solve_z = np.divide(
        planes_d_expanded
        - np.sum(
            np.multiply(np.expand_dims(planes_n[:, [0, 1]], 1), xy_pair),
            axis=-1,
        ),
        np.expand_dims(planes_n[:, 2], -1),
    )
    solve_x = np.divide(
        planes_d_expanded
        - np.sum(
            np.multiply(np.expand_dims(planes_n[:, [1, 2]], 1), yz_pair),
            axis=-1,
        ),
        np.expand_dims(planes_n[:, 0], -1),
    )
    solve_y = np.divide(
        planes_d_expanded
        - np.sum(
            np.multiply(np.expand_dims(planes_n[:, [2, 0]], 1), zx_pair),
            axis=-1,
        ),
        np.expand_dims(planes_n[:, 1], -1),
    )

    # N x (2x2) x 3
    solve_z_corner_points = np.concatenate(
        [np.tile(xy_pair, (len(planes_n), 1, 1)), np.expand_dims(solve_z, -1)],
        -1,
    )
    solve_x_corner_points = np.roll(
        np.concatenate(
            [
                np.tile(yz_pair, (len(planes_n), 1, 1)),
                np.expand_dims(solve_x, -1),
            ],
            -1,
        ),
        1,
        -1,
    )
    solve_y_corner_points = np.roll(
        np.concatenate(
            [
                np.tile(zx_pair, (len(planes_n), 1, 1)),
                np.expand_dims(solve_y, -1),
            ],
            -1,
        ),
        -1,
        -1,
    )

    solve_corner_points = np.stack(
        [solve_x_corner_points, solve_y_corner_points, solve_z_corner_points],
        axis=1,
    )

    return solve_corner_points[np.arange(len(planes_n)), max_index, ...]
