import math
from numbers import Number
from typing import Tuple, List, Union

import numpy as np

from .union_find import UnionFind


def cluster_scan(
    scan_r, scan_theta, scan_points, max_neighbour_distance=0.03, min_points=5
):
    """USE NP.TAKE mode wrap to query"""

    # Sort point by angle
    scan_sorted_index = np.argsort(scan_theta)

    # scan_r_sorted = scan_r[scan_sorted_index]
    # scan_theta_sorted = scan_theta[scan_sorted_index]

    scan_points_sorted = scan_points[scan_sorted_index]

    scan_points_diff = (
        np.roll(scan_points_sorted, -1, axis=0) - scan_points_sorted
    )

    scan_points_diff_distance = np.sqrt(
        np.sum(np.power(scan_points_diff, 2), axis=-1)
    )

    cluster_member_mask = scan_points_diff_distance <= max_neighbour_distance
    bound_mask = np.diff(cluster_member_mask.astype(int), axis=0, prepend=0)
    bound_start_mask = bound_mask > 0
    bound_end_mask = bound_mask < 0

    derivative_count = 1

    cluster_start = np.where(bound_start_mask)[0]
    cluster_end = np.where(bound_end_mask)[0] + derivative_count

    if len(cluster_start) != len(cluster_end):
        cluster_end = np.concatenate(
            [cluster_end, [len(cluster_member_mask)]], axis=0
        )

    # Loop back
    cluster_bound_pair = np.stack(
        [
            cluster_start,
            cluster_end,
        ],
        axis=-1,
    )

    if len(cluster_bound_pair) > 1:
        if cluster_member_mask[-1] and cluster_member_mask[0]:
            # Fuse first and last cluster
            cluster_bound_pair[0, 0] = cluster_bound_pair[-1, 0] - len(
                cluster_member_mask
            )
            cluster_bound_pair = cluster_bound_pair[:-1, :]

    cluster_size = np.diff(cluster_bound_pair, axis=-1)[..., 0]

    valid_cluster_mask = cluster_size > min_points

    filtered_cluster_bound_pair = cluster_bound_pair[valid_cluster_mask]

    return scan_sorted_index, filtered_cluster_bound_pair


def points_to_distance_angle(points):
    r = np.sqrt(np.sum(np.power(points, 2), axis=-1))
    theta = np.arctan2(points[:, 1], points[:, 0])

    return r, theta


def fit_line(points: np.ndarray) -> np.ndarray:
    """Fit line to points. Require Length at least 2 distint points.

    Parameters
    ---------
    points: float np.ndarray[N,2]
        points to fit the line. Require at least 2 distinct points

    Returns
    --------
    slope_intercept: float np.ndarray[2, ]
        where slope_intercept = [a b] where y = ax + b

    Raise
    -------
    ValueError
        Not enough distinct points (<2) or singular A.T @ A

    """
    EPSILON = 1e-5

    n = len(points)
    if n < 2:
        raise ValueError(
            f'Required at least 2 points. Received {n} points. {points}'
        )

    A = np.stack([points[:, 0], np.ones((n,))], axis=-1)

    ata = A.T @ A

    det = np.linalg.det(ata)
    if np.abs(det) < EPSILON:
        raise ValueError(f'Singular. Received {n} points. {points}')

    ata_inverse = np.linalg.inv(ata)

    # slope_intercept = [a b].T where y=ax+b
    slope_intercept = ata_inverse @ A.T @ np.expand_dims(points[:, 1], -1)

    return np.squeeze(slope_intercept)


def fit_line_uniform(
    points: np.ndarray, max_variance_threshold: float
) -> np.ndarray:
    """Fit line to points. Require Length at least 2 distint points.

    Parameters
    ---------
    points: float np.ndarray[N,2]
        points to fit the line. Require at least 2 distinct points

    max_variance_threshold: float
        max variance accepted for point uniformity on the line. Threshold for varianceo of difference between consecutive points projected on the line

    Returns
    --------
    slope_intercept: float np.ndarray[2, ]
        where slope_intercept = [a b] where y = ax + b

    Raise
    -------
    ValueError
        Not enough distinct points (<2) or singular A.T @ A

    """
    EPSILON = 1e-5

    n = len(points)
    if n < 2:
        raise ValueError(
            f'Required at least 2 points. Received {n} points. {points}'
        )

    A = np.stack([points[:, 0], np.ones((n,))], axis=-1)

    ata = A.T @ A

    det = np.linalg.det(ata)
    if np.abs(det) < EPSILON:
        raise ValueError(f'Singular. Received {n} points. {points}')

    ata_inverse = np.linalg.inv(ata)

    # slope_intercept = [a b].T where y=ax+b
    slope_intercept = ata_inverse @ A.T @ np.expand_dims(points[:, 1], -1)

    slope_intercept = np.squeeze(slope_intercept)

    parallel_distance, _ = get_point_line_parallel_perpendicular(
        points, *slope_intercept_to_vector_form(slope_intercept)
    )

    parallel_distance_sorted = np.sort(parallel_distance)
    parallel_distance_sorted_diff = np.diff(parallel_distance_sorted)
    parallel_distance_sorted_diff_var = np.var(parallel_distance_sorted_diff)

    if np.any(parallel_distance_sorted_diff_var > max_variance_threshold):
        raise ValueError(
            f'Line not uniform enough. Max Variance: {np.max(parallel_distance_sorted_diff_var)}. Threshold: {max_variance_threshold}'
        )

    return np.squeeze(slope_intercept)


def fit_line_ransac(
    points: np.ndarray,
    fit_line_max_variance_threshold: float,
    min_inlier_points: int,
    sample_size: int,
    inlier_distance_threshold: float,
    inlier_proability: float,
    success_probability: float = 0.95,
    max_iterations: int = 10000,
) -> np.ndarray:
    iterations = int(
        math.ceil(
            math.log(1.0 - success_probability)
            / math.log(1.0 - inlier_proability**sample_size)
        )
    )
    iterations = min(max_iterations, iterations)

    data_n = len(points)

    best_inlier_count = min_inlier_points
    best_line_fit = None

    for i in range(iterations):
        # Sample random subset
        sample_index = np.random.choice(data_n, size=sample_size)
        sample_points = points[sample_index]

        # Fit line
        try:
            # line_slope_intercept = fit_line_uniform(sample_points, fit_line_max_variance_treshold)
            line_slope_intercept = fit_line(sample_points)
        except ValueError:
            # Error in line selection
            continue

        line_p, line_v_hat = slope_intercept_to_vector_form(
            line_slope_intercept
        )

        # Evaluate inliers
        (
            parallel_distance,
            perpendicular_distance,
        ) = get_point_line_parallel_perpendicular(points, line_p, line_v_hat)

        inlier_mask = perpendicular_distance < inlier_distance_threshold
        inlier_points = points[inlier_mask]

        inlier_count = np.sum(inlier_mask.astype(int))

        if inlier_count < best_inlier_count:
            continue

        inlier_parallel_distance = parallel_distance[inlier_mask]

        parallel_distance_sort_index = np.argsort(inlier_parallel_distance)
        parallel_distance_sorted = inlier_parallel_distance[
            parallel_distance_sort_index
        ]

        parallel_distance_sorted_diff = np.diff(parallel_distance_sorted)
        parallel_distance_sorted_diff_var = np.var(
            parallel_distance_sorted_diff
        )

        if np.any(
            parallel_distance_sorted_diff_var > fit_line_max_variance_threshold
        ):
            # Discard line, not uniform enough
            print(
                f'Not uniform enough. {np.max(parallel_distance_sorted_diff_var)} - {fit_line_max_variance_threshold}'
            )
            continue

        if inlier_count >= best_inlier_count:
            # Refit line
            best_line_fit = fit_line(inlier_points)
            best_inlier_count = inlier_count

    # Return best
    return best_line_fit


def slope_intercept_to_vector_form(
    slope_intercept: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Convert line slope intercept form to vector form

    Parameters
    ----------
    slope_intercept: float np.ndarray[2, ]
        [slope, intercept]

    Returns
    -------
        p: float np.ndarray[1, 2]
            position vector of point on line, orthogonal to direction
        v_hat: float np.ndarray[1, 2]
            normalized direction vector of line
    """

    # y = ax + b

    slope, intercept = slope_intercept

    v = np.array([[1.0, slope]])
    v_hat = v / np.expand_dims(np.linalg.norm(v), axis=-1)

    x = 0.0
    y = slope * x + intercept
    point_on_line = np.array([[x, y]])

    perpendicular_point = (
        point_on_line - (np.sum(np.multiply(v_hat, point_on_line))) * v_hat
    )

    return perpendicular_point, v_hat


def line_vector_form_from_line_segment(
    x_0: np.ndarray, x_1: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Return line in vectorm form

    Parameters
    ----------
    x_0: float np.ndarray[1,2]
    x_1: float np.ndarray[1,2]

    Returns
    -------
    p: float np.ndarray[1,2]
        vector from origin to point on line. Perpedicular to v

    v: float np.ndarray[1,2]
        unit direction vector of line

    """

    EPSILON = 1e-6
    if x_0.shape != (1, 2) or x_1.shape != (1, 2):
        raise ValueError(
            f'Expected shaped (1,2) but received {x_0.shape=} and {x_1.shape=}'
        )

    v = x_1 - x_0
    v_norm = np.linalg.norm(v)

    if v_norm < EPSILON:
        # Same point
        raise ValueError(f'Two points are the same/. {x_0=} {x_1=}')

    v_hat = v / v_norm

    perpendicular_point = x_0 - (np.sum(np.multiply(v_hat, x_0))) * v_hat

    return perpendicular_point, v_hat


def line_vector_to_hessian_form(p, v_hat):
    """Assuming p is already perpendicular point

    Returns
    -------
    line_hessian_form: float np.array[2,]
        Line in hessian normal form. [r, alpha] where r is distance from origin, alpha is angle from x [-pi, pi]

    """

    r = np.linalg.norm(p)
    alpha = np.arctan2(p[0, 1], p[0, 0])

    return np.array([r, alpha])


def get_point_line_parallel_perpendicular(
    points: np.ndarray, p: np.ndarray, v_hat: np.ndarray
) -> np.ndarray:
    """Get point parallel distance from p and (perpendicular) distance from line defined by position p and direction v_hat

    Parameters
    -----------
        points: float np.ndarray[N, 2]
            points to contain in line

        p: float np.ndarray[1, 2]
            position vector of point on line, orthogonal to direction

        v_hat: float np.ndarray[1, 2]
            normalized direction vector of line

    Returns
    -----------
        points_parallel_distance: float np.ndarray[N, ]
            Parallel distance of points to p

        points_perpendicular_distance: float np.ndarray[N, ]
            Perpendicular distance of points to line

    """

    line_to_points = points - p

    points_parallel_distance = np.sum(
        np.multiply(line_to_points, v_hat), axis=-1
    )

    line_to_point_project_normal = line_to_points - np.multiply(
        np.expand_dims(points_parallel_distance, -1), v_hat
    )

    points_perpendicular_distance = np.linalg.norm(
        line_to_point_project_normal, axis=-1
    )

    return points_parallel_distance, points_perpendicular_distance


def split_and_merge_points(
    scan_points: np.ndarray,
    inlier_distance_threshold: Number,
    merge_distance_threshold: Number,
    min_line_points: int = 10,
) -> List[np.ndarray]:
    """Split and merge algorithm

    1) Fit line to set
    2)

    """

    def split(
        scan_points: np.ndarray, inlier_distance_threshold: Number
    ) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Split subroutine.


        Returns
        --------
            lines: List[Tuple[float np.ndarray[2, ], np.ndarray[M, 2]]]
                list of tuple containing (line parameter, line points)

                line parameter: float np.ndarray[2, ]
                    Contains R, perpendicular distance from origin, and alpha, angle of normal of line to x axis

                line points: float np.ndarray[M, 2]
                    Points used to form the line

        """

        n = len(scan_points)
        if n < min_line_points:
            return []

        try:
            slope_intercept = fit_line(scan_points)
        except ValueError:
            # No line can be formed
            return []
        p1, v_hat1 = slope_intercept_to_vector_form(slope_intercept)

        (
            points_parallel_distance_1,
            points_perpendicular_distance_1,
        ) = get_point_line_parallel_perpendicular(scan_points, p1, v_hat1)

        furthest_distance_index = np.argmax(points_perpendicular_distance_1)
        furthest_distance = points_perpendicular_distance_1[
            furthest_distance_index
        ]

        if furthest_distance < inlier_distance_threshold:
            # Return line
            return [(p1, v_hat1, scan_points)]

        # Sort points
        point_sort_index = np.argsort(points_parallel_distance_1)
        scan_points_sorted = scan_points[point_sort_index]

        # Assume sorted points
        # Points can be sorted by arranging in order of increasing parallel distance
        first_point = np.expand_dims(scan_points_sorted[0], axis=0)
        last_point = np.expand_dims(scan_points_sorted[-1], axis=0)

        p2, v_hat2 = line_vector_form_from_line_segment(first_point, last_point)

        (
            points_parallel_distance_2,
            points_perpendicular_distance_2,
        ) = get_point_line_parallel_perpendicular(
            scan_points_sorted, p2, v_hat2
        )

        furthest_distance_2_index = np.argmax(points_perpendicular_distance_2)

        lines = []

        try:
            first_split = split(
                scan_points_sorted[:furthest_distance_2_index],
                inlier_distance_threshold,
            )
            lines.extend(first_split)
        except ValueError:
            pass

        try:
            second_split = split(
                scan_points_sorted[furthest_distance_2_index:],
                inlier_distance_threshold,
            )
            lines.extend(second_split)
        except ValueError:
            pass

        return lines

    def merge(
        line_list,
        merge_distance_threshold: Number,
        inlier_distance_threshold: Number,
    ):
        """

        merge_distance_threshold: Number
            threshold to consider 2 line similar

        inlier_distance_threshold: Number
            threshold to consider for line segment forming
        """

        n = len(line_list)

        if n < 2:
            return []

        # N x 2
        lines_hessian = np.stack(
            [
                line_vector_to_hessian_form(p, v_hat)
                for p, v_hat, _ in line_list
            ],
            axis=0,
        )

        # Symmetric

        line_hessian_difference = np.expand_dims(
            lines_hessian, axis=1
        ) - np.expand_dims(lines_hessian, axis=0)

        # TODO: Get min angle instead
        angle_difference = line_hessian_difference[..., -1]
        angle_difference_abs = np.abs(angle_difference)
        line_hessian_difference[..., -1] = np.minimum(
            angle_difference_abs, 2 * math.pi - angle_difference_abs
        )  # Since will be squared anyways

        line_hessian_difference_2 = np.power(line_hessian_difference, 2)

        similar_line_matrix = (
            np.sum(line_hessian_difference_2, axis=-1)
            <= merge_distance_threshold
        )

        # Form N sets
        disjoint_sets = UnionFind()
        for i in range(n):
            disjoint_sets.make_set(i)

        for i in range(n):
            for j in range(n):
                if j >= i:
                    # Skip half triangle
                    break

                # print(f'[{i=},{j=}] {similar_line_matrix[j,i]=}')
                if similar_line_matrix[j, i]:
                    # Combine sets
                    # print(f'[{i=},{j=}] {similar_line_matrix[j,i]=} Pass')
                    disjoint_sets.union(i, j)

        set_roots = disjoint_sets.get_roots()

        lines = []
        for root in set_roots:
            point_set = []
            set_size = 0
            for i in range(n):
                if similar_line_matrix[root][i]:
                    point_set.append(line_list[i][-1])
                    set_size += 1

            point_set = np.concatenate(point_set, axis=0)

            # print(f'{root=} {set_size}')

            p, v_hat = line_list[root][:-1]

            if set_size > 1:
                # Fit line to point set
                line_slope_intercept = fit_line(point_set)
                p, v_hat = slope_intercept_to_vector_form(line_slope_intercept)
            line_segment = line_to_segment(
                point_set, p, v_hat, inlier_distance_threshold
            )

            if line_segment is None:
                continue

            lines.append((line_segment, point_set))

        return lines

    line_list = split(scan_points, inlier_distance_threshold)

    refined_line_list = merge(
        line_list, merge_distance_threshold, inlier_distance_threshold
    )

    return refined_line_list


def split_and_merge_points_ransac(
    scan_points: np.ndarray,
    inlier_distance_threshold: Number,
    merge_distance_threshold: Number,
    fit_line_max_variance_thresold: Number,
    ransac_inlier_distance_threshold: Number,
    min_line_points: int = 10,
    ransac_inlier_probability: Number = 0.8,
    ransac_success_probability: Number = 0.95,
    ransac_sample_size: int = 5,
    ransac_max_iterations: int = 10000,
) -> List[np.ndarray]:
    """Split and merge algorithm

    1) Fit line to set
    2)

    """

    def split(
        scan_points: np.ndarray, inlier_distance_threshold: Number
    ) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Split subroutine.


        Returns
        --------
            lines: List[Tuple[float np.ndarray[2, ], np.ndarray[M, 2]]]
                list of tuple containing (line parameter, line points)

                line parameter: float np.ndarray[2, ]
                    Contains R, perpendicular distance from origin, and alpha, angle of normal of line to x axis

                line points: float np.ndarray[M, 2]
                    Points used to form the line

        """

        n = len(scan_points)
        if n < min_line_points:
            return []

        try:
            # slope_intercept = fit_line(scan_points)
            slope_intercept = fit_line_ransac(
                scan_points,
                fit_line_max_variance_threshold=fit_line_max_variance_thresold,
                min_inlier_points=min_line_points,
                sample_size=ransac_sample_size,
                inlier_distance_threshold=ransac_inlier_distance_threshold,
                inlier_proability=ransac_inlier_probability,
                success_probability=ransac_success_probability,
                max_iterations=ransac_max_iterations,
            )
        except ValueError:
            # No line can be formed
            return []

        if slope_intercept is None:
            # No line found

            return []
        p1, v_hat1 = slope_intercept_to_vector_form(slope_intercept)

        (
            points_parallel_distance_1,
            points_perpendicular_distance_1,
        ) = get_point_line_parallel_perpendicular(scan_points, p1, v_hat1)

        furthest_distance_index = np.argmax(points_perpendicular_distance_1)
        furthest_distance = points_perpendicular_distance_1[
            furthest_distance_index
        ]

        if furthest_distance < inlier_distance_threshold:
            # Return line
            return [(p1, v_hat1, scan_points)]

        # Sort points
        point_sort_index = np.argsort(points_parallel_distance_1)
        scan_points_sorted = scan_points[point_sort_index]

        # Assume sorted points
        # Points can be sorted by arranging in order of increasing parallel distance
        first_point = np.expand_dims(scan_points_sorted[0], axis=0)
        last_point = np.expand_dims(scan_points_sorted[-1], axis=0)

        p2, v_hat2 = line_vector_form_from_line_segment(first_point, last_point)

        (
            points_parallel_distance_2,
            points_perpendicular_distance_2,
        ) = get_point_line_parallel_perpendicular(
            scan_points_sorted, p2, v_hat2
        )

        furthest_distance_2_index = np.argmax(points_perpendicular_distance_2)

        lines = []

        try:
            first_split = split(
                scan_points_sorted[:furthest_distance_2_index],
                inlier_distance_threshold,
            )
            lines.extend(first_split)
        except ValueError:
            pass

        try:
            second_split = split(
                scan_points_sorted[furthest_distance_2_index:],
                inlier_distance_threshold,
            )
            lines.extend(second_split)
        except ValueError:
            pass

        return lines

    def merge(
        line_list,
        merge_distance_threshold: Number,
        inlier_distance_threshold: Number,
    ):
        """

        merge_distance_threshold: Number
            threshold to consider 2 line similar

        inlier_distance_threshold: Number
            threshold to consider for line segment forming
        """

        n = len(line_list)

        if n < 2:
            return []

        # N x 2
        lines_hessian = np.stack(
            [
                line_vector_to_hessian_form(p, v_hat)
                for p, v_hat, _ in line_list
            ],
            axis=0,
        )

        # Symmetric

        line_hessian_difference = np.expand_dims(
            lines_hessian, axis=1
        ) - np.expand_dims(lines_hessian, axis=0)

        # TODO: Get min angle instead
        angle_difference = line_hessian_difference[..., -1]
        angle_difference_abs = np.abs(angle_difference)
        line_hessian_difference[..., -1] = np.minimum(
            angle_difference_abs, 2 * math.pi - angle_difference_abs
        )  # Since will be squared anyways

        line_hessian_difference_2 = np.power(line_hessian_difference, 2)

        similar_line_matrix = (
            np.sum(line_hessian_difference_2, axis=-1)
            <= merge_distance_threshold
        )

        # Form N sets
        disjoint_sets = UnionFind()
        for i in range(n):
            disjoint_sets.make_set(i)

        for i in range(n):
            for j in range(n):
                if j >= i:
                    # Skip half triangle
                    break

                # print(f'[{i=},{j=}] {similar_line_matrix[j,i]=}')
                if similar_line_matrix[j, i]:
                    # Combine sets
                    # print(f'[{i=},{j=}] {similar_line_matrix[j,i]=} Pass')
                    disjoint_sets.union(i, j)

        set_roots = disjoint_sets.get_roots()

        lines = []
        for root in set_roots:
            point_set = []
            set_size = 0
            for i in range(n):
                if similar_line_matrix[root][i]:
                    point_set.append(line_list[i][-1])
                    set_size += 1

            point_set = np.concatenate(point_set, axis=0)

            # print(f'{root=} {set_size}')

            p, v_hat = line_list[root][:-1]

            if set_size > 1:
                # Fit line to point set
                # line_slope_intercept = fit_line(point_set)

                line_slope_intercept = fit_line_ransac(
                    scan_points,
                    fit_line_max_variance_threshold=fit_line_max_variance_thresold,
                    min_inlier_points=min_line_points,
                    sample_size=ransac_sample_size,
                    inlier_distance_threshold=ransac_inlier_distance_threshold,
                    inlier_proability=ransac_inlier_probability,
                    success_probability=ransac_success_probability,
                    max_iterations=ransac_max_iterations,
                )

                if line_slope_intercept is None:
                    # Failed to merge
                    print('Failed to merge set.')
                    continue

                p, v_hat = slope_intercept_to_vector_form(line_slope_intercept)
            line_segment = line_to_segment(
                point_set, p, v_hat, inlier_distance_threshold
            )

            if line_segment is None:
                continue

            lines.append((line_segment, point_set))

        return lines

    line_list = split(scan_points, inlier_distance_threshold)

    refined_line_list = merge(
        line_list, merge_distance_threshold, inlier_distance_threshold
    )

    return refined_line_list


def line_to_segment(
    points: np.ndarray,
    p: np.ndarray,
    v_hat: np.ndarray,
    inlier_distance_threshold: Number,
) -> Union[None, np.ndarray]:
    """Convert line to line segments given points and slope_intercept parameters. Convert to (x1, y1) , (x2, y2)

    Parameters
    ---------
    points: float np.ndarray[N, 2]
        points to contain in line

    p: float np.ndarray[1, 2]
            position vector of point on line, orthogonal to direction

    v_hat: float np.ndarray[1, 2]
        normalized direction vector of line

    inlier_distance_threshold: Number
        shortest distance to line to consider inliers

    Returns
    --------
    bounds: float np.ndarray[2, 2]
        bounds array. bounds = [[x1, y1], [x2, y2]].
        Return None if no inlier found

    """

    # Len of points must be at least 2
    n = len(points)

    if n < 2:
        raise ValueError(
            f'Required at least 2 points. Received {n} points. {points}'
        )

    (
        points_parallel_distance,
        points_perpendicular_distance,
    ) = get_point_line_parallel_perpendicular(points, p, v_hat)

    inlier_masks = points_perpendicular_distance < inlier_distance_threshold

    n_inliers = np.sum(inlier_masks.astype(int))
    if n_inliers < 2:
        return None

    inlier_line_project_magnitude = points_parallel_distance[inlier_masks]

    sorted_points_index = np.argsort(inlier_line_project_magnitude)

    first_point_magnitude = inlier_line_project_magnitude[
        sorted_points_index[0]
    ]
    last_point_magnitude = inlier_line_project_magnitude[
        sorted_points_index[-1]
    ]

    first_line_end = p + first_point_magnitude * v_hat
    last_line_end = p + last_point_magnitude * v_hat

    return np.concatenate([first_line_end, last_line_end], axis=0)


def line_line_intersection(
    p1: np.ndarray,
    v1: np.ndarray,
    p2: np.ndarray,
    v2: np.ndarray,
    angle_limit_radians: float = math.pi * 2 / 180,
    epsilon=1e-5,
):
    """Get line-line intersection

    Parameters
    ---------
    p1: float np.ndarray[1, 2]
        position vector to point on line 1
    v1: float np.ndarray[1, 2]
        direction vector of line 1

    p2: float np.ndarray[1, 2]
        position vector to point on line 2
    v2: float np.ndarray[1, 2]
        direction vector of line 2

    angle_limit_radians: float
        Max angle different in randians between v1 and v2 to consider them to not intersect


    epsilon: float
        use to check errors between line 1 and line 2 calculations for consistency

    Returns
    ---------
    intersection point: float np.ndarray[1, 2]
        Intersection point. Return None if line is parallel (not intersecting) or result inconsistent
    """

    v1_norm = np.linalg.norm(v1)
    if v1_norm < epsilon:
        raise ValueError(f'v1 has zero norm - {v1_norm}. v1: {v1}')
    v1_unit = v1 / v1_norm

    v2_norm = np.linalg.norm(v2)
    if v2_norm < epsilon:
        raise ValueError(f'v2 has zero norm - {v2_norm}. v2: {v2}')
    v2_unit = v2 / v2_norm

    direction_cross = np.cross(v1_unit, v2_unit)
    direction_dot = np.sum(np.multiply(v1_unit, v2_unit))

    angle_difference = np.arctan2(direction_cross, direction_dot)

    if abs(angle_difference) < angle_limit_radians:
        # Line too similar
        return None

    # 2 x 1
    b = p1.T - p2.T

    # 2 x 2
    A = np.concatenate([-v1.T, v2.T], axis=1)

    x = np.matmul(np.linalg.inv(A), b)

    # Check
    k1 = x[0, 0]
    k2 = x[1, 0]

    point_line1 = k1 * v1 + p1
    point_line2 = k2 * v2 + p2

    if np.linalg.norm(point_line1 - point_line2) > epsilon:
        return None

    return (point_line1 + point_line2) / 2


def get_corner_points_lines(line_list, distance_threshold):
    """
    Parameters
    ---------
    lines:  List[Tuple[float np.ndarray[2, 2], float np.ndarray[N, 2]]]
        list of tuple of line segment (2 end points) and point set defining the line segment
    """

    # M lines
    # Find minimum distance between endpoints
    # O(m^2)
    # 2 * 2 * M^2
    # Expensive but assuming # lines in line set is small

    M = len(line_list)
    if M < 2:
        return []
    lines, point_sets = list(zip(*line_list))

    # 2*M x 2
    lines_packed = np.concatenate(lines, axis=0)

    # 2*M x 2*M
    end_point_distances = np.sqrt(
        np.sum(
            np.power(
                np.expand_dims(lines_packed, axis=1)
                - np.expand_dims(lines_packed, axis=0),
                2,
            ),
            axis=-1,
        )
    )

    close_point_mask = end_point_distances <= distance_threshold
    # Disregard connection in same 2x2 block as its from the same line

    intersection_points = []
    for i in range(M):
        for j in range(M):
            if j >= i:
                # Skip half triangle
                break

            x_index = 2 * i
            y_index = 2 * j

            if np.any(
                close_point_mask[y_index : y_index + 2, x_index : x_index + 2]
            ):
                # Find intersection point

                line_1 = lines[i]
                line_2 = lines[j]

                line_1_p, line_1_v = line_vector_form_from_line_segment(
                    np.expand_dims(line_1[0], 0), np.expand_dims(line_1[1], 0)
                )
                line_2_p, line_2_v = line_vector_form_from_line_segment(
                    np.expand_dims(line_2[0], 0), np.expand_dims(line_2[1], 0)
                )

                intersection_point = line_line_intersection(
                    line_1_p, line_1_v, line_2_p, line_2_v
                )

                if intersection_point is not None:
                    intersection_points.append((intersection_point, (i, j)))

    return intersection_points


def get_cluster_lines(
    sorted_points,
    max_angle_difference_radians: Number,
    min_line_points: int,
    line_segment_inlier_threshold: Number,
    merge_endpoint_distance_threshold: Number = 0,
):
    rolled_points = np.roll(sorted_points, -1, axis=0)

    sorted_direction = rolled_points - sorted_points
    sorted_direction_unit = np.divide(
        sorted_direction,
        np.expand_dims(np.linalg.norm(sorted_direction, axis=-1), axis=-1),
    )

    rolled_sorted_direction_unit = np.roll(sorted_direction_unit, -1, axis=0)
    sorted_direction_cross = np.cross(
        rolled_sorted_direction_unit, sorted_direction_unit
    )
    sorted_direction_dot = np.sum(
        np.multiply(rolled_sorted_direction_unit, sorted_direction_unit),
        axis=-1,
    )

    sorted_angle_difference = np.arctan2(
        sorted_direction_cross, sorted_direction_dot
    )

    cluster_member_mask = (
        np.abs(sorted_angle_difference) <= max_angle_difference_radians
    )
    bound_mask = np.diff(cluster_member_mask.astype(int), axis=0, prepend=0)
    bound_start_mask = bound_mask > 0
    bound_end_mask = bound_mask < 0

    derivative_count = 2

    cluster_bound_pair = np.stack(
        [
            np.where(bound_start_mask)[0],
            np.where(bound_end_mask)[0] + derivative_count,
        ],
        axis=-1,
    )

    if len(cluster_bound_pair) > 1:
        if cluster_member_mask[-1] and cluster_member_mask[0]:
            # Fuse first and last cluster
            cluster_bound_pair[0, 0] = cluster_bound_pair[-1, 0] - len(
                sorted_angle_difference
            )
            cluster_bound_pair = cluster_bound_pair[:-1, :]

    cluster_size = np.diff(cluster_bound_pair, axis=-1)[..., 0]

    valid_cluster_mask = cluster_size >= min_line_points

    filtered_cluster_bound_pair = cluster_bound_pair[valid_cluster_mask]

    clusters = []

    for lower_bound, upper_bound in filtered_cluster_bound_pair:
        line_points = np.take(
            sorted_points, range(lower_bound, upper_bound), axis=0, mode='wrap'
        )
        try:
            line_slope_intercept = fit_line(line_points)
        except ValueError:
            continue
        line_p, line_v_hat = slope_intercept_to_vector_form(
            line_slope_intercept
        )
        line_segment = line_to_segment(
            line_points, line_p, line_v_hat, line_segment_inlier_threshold
        )
        # To segment
        if line_segment is not None:
            clusters.append((line_segment, line_points, line_v_hat))

    if merge_endpoint_distance_threshold != 0:
        # Merge consecutive cluster
        # If close enough in segment end points and angle

        cluster_count = len(clusters)
        if cluster_count == 0:
            return clusters

        cluster_lines, cluster_points, cluster_v_hat = list(zip(*clusters))

        cluster_v_hat_np = np.concatenate(cluster_v_hat, axis=0)

        cluster_angle_difference = np.arccos(
            np.abs(
                np.sum(
                    np.multiply(
                        np.expand_dims(cluster_v_hat_np, axis=0),
                        np.expand_dims(cluster_v_hat_np, axis=1),
                    ),
                    -1,
                )
            )
        )
        cluster_angle_merge_mask = (
            cluster_angle_difference <= max_angle_difference_radians
        )

        cluster_lines_packed = np.concatenate(cluster_lines, axis=0)

        cluster_end_point_distances = np.sqrt(
            np.sum(
                np.power(
                    np.expand_dims(cluster_lines_packed, axis=0)
                    - np.expand_dims(cluster_lines_packed, axis=1),
                    2,
                ),
                axis=-1,
            )
        )
        cluser_distance_merge_mask = (
            cluster_end_point_distances <= merge_endpoint_distance_threshold
        )

        # Collapse 2 x 2 into 1x1 block
        cluser_distance_merge_mask_collapse = np.logical_or(
            cluser_distance_merge_mask[::2], cluser_distance_merge_mask[1::2]
        )
        cluser_distance_merge_mask_collapse = np.logical_or(
            cluser_distance_merge_mask_collapse[:, ::2],
            cluser_distance_merge_mask_collapse[:, 1::2],
        )

        cluster_merge_mask = np.logical_and(
            cluster_angle_merge_mask, cluser_distance_merge_mask_collapse
        )

        disjoint_lines = UnionFind()

        for i in range(cluster_count):
            disjoint_lines.make_set(i)

        for i in range(cluster_count):
            for j in range(cluster_count):
                if j >= i:
                    # Skip half triangle
                    break

                if cluster_merge_mask[j, i]:
                    # Combine sets
                    disjoint_lines.union(i, j)

        set_roots = disjoint_lines.get_roots()

        clusters = []
        for root in set_roots:
            refit_points = []
            for i in range(cluster_count):
                if cluster_merge_mask[root][i]:
                    refit_points.append(cluster_points[i])

            if len(refit_points) <= 1:
                clusters.append((cluster_lines[root], cluster_points[root]))
                continue

            refit_points = np.concatenate(refit_points, axis=0)
            try:
                line_slope_intercept = fit_line(refit_points)
            except ValueError:
                continue
            line_p, line_v_hat = slope_intercept_to_vector_form(
                line_slope_intercept
            )
            line_segment = line_to_segment(
                refit_points, line_p, line_v_hat, line_segment_inlier_threshold
            )
            # To segment
            if line_segment is not None:
                clusters.append((line_segment, refit_points))
    else:
        clusters = [(line, points) for line, points, _ in clusters]

    return clusters


def get_corner_points_lines_consecutive(
    line_list: List[Tuple[np.ndarray, np.ndarray]],
    max_intersection_point_distance: Number,
):
    """Assume line in line_list are ordered consecutively (i.e intersection only tested on consecutive pairs)
    Parameters
    ---------
    line_list:  List[Tuple[float np.ndarray[2, 2], float np.ndarray[N, 2]]]
        list of tuple of line segment (2 end points) and point set defining the line segment

    max_intersection_point_distance: Number
        distance to accept intersection point. reject if intersection is too far away

    Returns
    -------
    intersection_list: List[Tuple[float np.ndarray[1,2], int, int]]
        list of (intersection_point, line_1 index, line_2 index)

    """

    # line_list is list of line segment and point set
    M = len(line_list)
    if M < 2:
        return []
    lines, point_sets = list(zip(*line_list))

    intersection_points = []
    for i in range(M):
        line_1 = lines[i]
        line_2 = lines[(i + 1) % M]

        line_1_p, line_1_v = line_vector_form_from_line_segment(
            np.expand_dims(line_1[0], 0), np.expand_dims(line_1[1], 0)
        )
        line_2_p, line_2_v = line_vector_form_from_line_segment(
            np.expand_dims(line_2[0], 0), np.expand_dims(line_2[1], 0)
        )

        intersection_point = line_line_intersection(
            line_1_p, line_1_v, line_2_p, line_2_v
        )

        if intersection_point is None:
            continue

        # intersection
        line_packed = np.concatenate([line_1, line_2], axis=0)

        intersection_point_distance = np.sqrt(
            np.sum(np.power(intersection_point - line_packed, 2), axis=-1)
        )
        if (
            np.min(intersection_point_distance)
            > max_intersection_point_distance
        ):
            continue

        intersection_points.append((intersection_point, (i, i + 1)))

    return intersection_points
