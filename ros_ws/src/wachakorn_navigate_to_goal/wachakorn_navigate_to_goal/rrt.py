import math
from numbers import Number
from typing import Tuple, List, Union, Optional

import numpy as np

import matplotlib
import matplotlib.collections

from .geometry_utils import get_point_line_parallel_perpendicular

PositionType = Tuple[Number, Number]
PointType = np.ndarray
IndexArrayType = np.ndarray
MapType = np.ndarray


class RRTStar:
    """RRT* for 2D path planning using straight movements"""

    def __init__(
        self,
        map: np.ndarray,
        map_x_bounds: np.ndarray,
        map_y_bounds: np.ndarray,
    ):
        self.map: MapType = map  # Bool, freespace = False

        self.map_x_bounds = map_x_bounds
        self.map_y_bounds = map_y_bounds

        if len(self.map_x_bounds) - 1 != self.map.shape[1]:
            raise ValueError(
                f'Received x bounds of length ({len(self.map_x_bounds)}) but map is {self.map.shape}. Expecting x bounds to be 1 more than axis 1 length'
            )

        if len(self.map_y_bounds) - 1 != self.map.shape[0]:
            raise ValueError(
                f'Received x bounds of length ({len(self.map_y_bounds)}) but map is {self.map.shape}. Expecting x bounds to be 1 more than axis 0 length'
            )

        self.map_x_cell_size = self.map_x_bounds[1] - self.map_x_bounds[0]
        self.map_y_cell_size = self.map_y_bounds[1] - self.map_y_bounds[0]

        self.map_corner_phi = np.arctan2(
            self.map_y_cell_size, self.map_x_cell_size
        )

        self.map_x_bounds_expanded = np.expand_dims(self.map_x_bounds, -1)
        self.map_y_bounds_expanded = np.expand_dims(self.map_y_bounds, -1)

        self.map_bounds = np.stack(
            np.meshgrid(self.map_x_bounds, self.map_y_bounds), -1
        )
        self.map_centers = self.map_bounds[:-1, :-1] + np.array(
            [[[self.map_x_cell_size / 2, self.map_y_cell_size / 2]]]
        )

        # Assuming uniform grid in x and y
        self.map_x_range = self.map_x_bounds[-1] - self.map_x_bounds[0]
        self.map_y_range = self.map_y_bounds[-1] - self.map_y_bounds[0]
        self.map_position_range = np.array(
            [[self.map_x_range, self.map_y_range]]
        )

        self.map_x_low = self.map_x_bounds[0]
        self.map_y_low = self.map_y_bounds[0]
        self.map_position_low = np.array([[self.map_x_low, self.map_y_low]])

        self.map_x_count = self.map.shape[1]
        self.map_y_count = self.map.shape[0]
        self.map_cell_count = np.array([[self.map_x_count, self.map_y_count]])

        self.reset()

    def reset(self):
        self.index_table: dict[PositionType, int] = dict()
        self.nodes: List[PositionType] = []

        self.node_child: dict[PositionType, List[PositionType]] = dict()
        self.node_parent: dict[PositionType, List[PositionType]] = dict()

        self.node_cost: List[Number] = []

        self.path = []

        self.start_position = None
        self.goal_position = None

        self.found = False

        # debug
        self._random_position = None
        self._extend_point = None
        self._new_position = None

    def get_map(self) -> MapType:
        return self.map

    def is_position_blocked(self, position: PositionType) -> bool:
        """Blocked considers outside too"""
        point = np.array(position).reshape(1, 2)
        grid_index, in_grid = self._get_grid_index(point)

        if not np.any(in_grid):
            return False

        return self.map[grid_index[0, 1], grid_index[0, 0]]

    def get_centers_distance(self, point: PointType) -> PointType:
        center_distances = np.linalg.norm(self.map_centers - point, axis=-1)
        return center_distances

    def set_goals(
        self, start_position: PositionType, goal_position: PositionType
    ) -> bool:
        """
        Returns
        -------
        Set goal successful: bool
            fails if start or goal is blocked

        """

        if self.is_position_blocked(start_position):
            # print('Start position blocked')
            return False

        if self.is_position_blocked(goal_position):
            # print('Goal position blocked')
            return False

        self.start_position = tuple(
            *self._round_position_to_nearest_center(start_position)
        )
        self.goal_position = tuple(
            *self._round_position_to_nearest_center(goal_position)
        )

        self.start_position_np: PointType = np.array(
            self.start_position
        ).reshape(1, 2)
        self.goal_position_np: PointType = np.array(self.goal_position).reshape(
            1, 2
        )

        print(f'{self.start_position=} {self.start_position_np=}')

        self._add_node(self.start_position, 0.0, None)

        return True

    def _add_node(
        self,
        position: PointType,
        node_cost: Number,
        parent: Union[PositionType, None],
    ):
        self.index_table[position] = len(self.nodes)
        self.nodes.append(position)

        self.node_cost.append(node_cost)

        self.node_child[position] = []
        self.node_parent[position] = parent

        if parent is not None:
            self.node_child[parent].append(position)

    def _clip_position_to_edge(self, points: PointType) -> PointType:
        """Clip the points to edge of map

        Parameters:
        points: float np.array[N, 2]
        """

        return np.clip(
            points,
            a_min=[self.map_x_bounds[0], self.map_y_bounds[0]],
            a_max=[self.map_x_bounds[-1], self.map_y_bounds[-1]],
        )

    def _round_position_to_nearest_center(self, points: PointType) -> PointType:
        """Round to nearest center"""
        grid_indices, _ = self._get_grid_index(points)

        grid_indices_clipped = self._clip_index_to_edge(grid_indices)

        return self.map_centers[
            grid_indices_clipped[:, 1], grid_indices_clipped[:, 0]
        ]

    def _clip_index_to_edge(self, indices: IndexArrayType) -> IndexArrayType:
        """Clip the indices to edge of map

        Parameters:
        indices: int np.array[N, 2]
        """

        return np.clip(indices, a_min=0, a_max=self.map_cell_count - 1)

    def _get_grid_index(
        self, points: PointType
    ) -> Tuple[IndexArrayType, np.ndarray]:
        """Fast implementation. Assume uniform grid.
        Parameters
        ----------
        points: float np.ndarray[N, 2]
            List of points to get grid index of

        Returns
        -------
        grid_index_array: float np.ndarray[N, 2]
            Grid indices of points.  If points lie outside the grid, then the index will be clipped
        in_grid_mask: bool np.ndarray[N, 2]
            Boolean mask desginating which points lies inside the map

        """

        grid_indices = np.floor(
            np.multiply(
                self.map_cell_count,
                np.divide(
                    points - self.map_position_low, self.map_position_range
                ),
            )
        ).astype(int)

        in_grid_mask = np.logical_not(
            np.any(
                np.logical_or(
                    grid_indices < 0, grid_indices >= self.map_cell_count
                ),
                axis=-1,
            )
        )

        return grid_indices, in_grid_mask

    def steer(
        self,
        p1: PointType,
        p2: PointType,
        dQ: Number,
        stop_goal_distance: Optional[Number] = None,
        epsilon: Number = 1e-6,
    ) -> Tuple[PointType, bool]:
        """
        Straight line steer between p1 and p2. Checks for collision, return the final point of steer. Raise value error if cant steer: p1, p2 is the same or collision too soon

        Parameters
        ----------
        p1: float np.ndarray[1, 2]
            start point of steer
        p2: float np.ndarray[1, 2]
            end point direction of steer
        dQ: Number
            max distancce to steer for
        stop_goal_distance: Number
            Consider this point is in goal if within this number. Should be less than smallest grid side to make sure not clipping through walls

        epsilon: Number
            numerical stability constant. Use to check equalities

        Returns
        -------
        new_configuration : float np.ndarray[1, 2]
            end point of the steer
        reached_goal: bool
            whether the new configuration is within the stop_goal_distance
        """

        # print(f'{p1.shape=} {p2.shape=}')

        direction_vector = p2 - p1
        direction_vector_norm = np.linalg.norm(direction_vector)
        if direction_vector_norm < epsilon:
            # Same point
            raise ValueError(
                f'Distance between p1 and p2 too low: {direction_vector_norm}. Same point'
            )
        direction_vector_unit = direction_vector / np.expand_dims(
            direction_vector_norm, -1
        )
        # print(f'{p1=} {p2=} {dQ=} {direction_vector=}')

        new_point_candidate = p1 + dQ * direction_vector_unit
        new_point = self._clip_position_to_edge(new_point_candidate)
        # new_point = self._round_position_to_nearest_center(new_point)
        # print(f'{new_point_candidate=} {new_point=}')

        # might be different after clip?
        p1_to_new_point_direction = new_point - p1
        p1_to_new_point_distance = np.linalg.norm(p1_to_new_point_direction)

        if p1_to_new_point_distance < epsilon:
            # Same point when clipped
            return ValueError(
                f'Distance between p1 and new configuration too low: {p1_to_new_point_distance}. Same point. p1: {p1} new_point: {new_point}'
            )

        p1_to_new_point_direction_unit = (
            p1_to_new_point_direction
            / np.expand_dims(p1_to_new_point_distance, -1)
        )

        # print(f'{p1=} {new_point=} {p1_to_new_point_direction_unit=}')

        # Clip to edge

        # Check for collision
        points_check = np.concatenate([p1, new_point], axis=0)

        grid_index_range, in_grid = self._get_grid_index(points_check)

        # in grid must be all true
        sorted_range = np.sort(grid_index_range, axis=0)
        sorted_range[1] += 1  # End range is exclusive when querying

        # print(f'{sorted_range=}')

        # Query only grids enclosing the 2 points
        block_centers = self.map_centers[
            sorted_range[0, 1] : sorted_range[1, 1],
            sorted_range[0, 0] : sorted_range[1, 0],
        ]
        block_values = self.map[
            sorted_range[0, 1] : sorted_range[1, 1],
            sorted_range[0, 0] : sorted_range[1, 0],
        ]

        if block_centers.size == 0:
            # Shouldnt ever happen?
            raise ValueError(f'Block centers is zero. {sorted_range=}')

        block_centers_flat = block_centers.reshape(-1, 2)
        block_values_flat = block_values.reshape(-1)

        line_perpendicular_angle = np.arctan2(
            abs(p1_to_new_point_direction_unit[0, 1]),
            abs(p1_to_new_point_direction_unit[0, 0]),
        )

        if line_perpendicular_angle < self.map_corner_phi:
            # left - right bound
            distance_threshold = self.map_x_cell_size / (
                2 * np.cos(line_perpendicular_angle)
            )
        else:
            # top - bottom bound
            distance_threshold = self.map_y_cell_size / (
                2 * np.sin(line_perpendicular_angle)
            )

        distance_threshold *= 1.5

        (
            block_centers_parallel_distance,
            block_centers_perpendicular_distance,
        ) = get_point_line_parallel_perpendicular(
            block_centers_flat, p1, p1_to_new_point_direction_unit
        )

        line_block_mask = (
            block_centers_perpendicular_distance <= distance_threshold
        )

        self._line_mask_pre_shuffled = line_block_mask

        # line_block_perpendicular_distance = block_centers_perpendicular_distance[line_block_mask]
        line_block_parallel_distance = block_centers_parallel_distance[
            line_block_mask
        ]
        line_block_centers_flat = block_centers_flat[line_block_mask]
        # print(f'{line_block_centers_flat=}')

        line_block_value_flat = block_values_flat[line_block_mask]
        # print(f'{line_block_value_flat=}')

        line_order_indices = np.argsort(line_block_parallel_distance)
        self._line_order_block_indices = line_order_indices

        line_block_centers_flat_sorted = line_block_centers_flat[
            line_order_indices
        ]
        # print(f'{line_block_centers_flat_sorted=}')

        line_block_value_flat_sorted = line_block_value_flat[line_order_indices]
        # print(f'{line_block_value_flat_sorted=}')

        block_not_free_indices = np.where(line_block_value_flat_sorted)[0]
        # print(f'{block_not_free_indices=}')

        # new_point = new_point
        if len(block_not_free_indices) != 0:
            # Collided
            first_hit_index = block_not_free_indices[0]
            if first_hit_index - 1 < 0:
                # Collided at start
                raise ValueError('Collided at start.')

            # Clip to poit on line nearest to block center
            # p1_to_new_point_distance = line_order_block_parallel_distance[first_hit_index - 1]
            # new_point = p1 + p1_to_new_point_distance * p1_to_new_point_direction_unit

            # Round to the center
            # new_point = block_centers_flat[first_hit_index - 1].reshape(1,2)
            new_point = line_block_centers_flat_sorted[
                first_hit_index - 1
            ].reshape(1, 2)

        # print(f'Pre Round: {new_point=}')
        new_point = self._round_position_to_nearest_center(new_point)
        # print(f'Post Round: {new_point=}')

        if stop_goal_distance is None:
            return new_point, False

        # Check new point goal distance

        (
            goal_parallel_distance,
            goal_perpendicular_distance,
        ) = get_point_line_parallel_perpendicular(
            self.goal_position_np, p1, p1_to_new_point_direction_unit
        )

        return new_point, np.all(
            goal_perpendicular_distance <= stop_goal_distance
        ) and np.all(
            goal_parallel_distance <= p1_to_new_point_distance
        ) and np.all(goal_parallel_distance >= 0)

    def obstacle_free(self, p1: PointType, p2: PointType, epsilon=1e-5) -> bool:
        p1_to_p2 = p2 - p1
        p1_to_p2_distance = np.linalg.norm(p1_to_p2)

        if p1_to_p2_distance < epsilon:
            return True

        try:
            end_point, _ = self.steer(p1, p2, p1_to_p2_distance * 1.05)
        except ValueError:
            return False

        p1_to_end_point = end_point - p1
        p1_to_end_point_distance = np.linalg.norm(p1_to_end_point)

        return p1_to_p2_distance < p1_to_end_point_distance

    def recalculate_cost(self, parent_node: PointType, vertex: PointType):
        """Recalulate cost and propagate cost update to all children of parent"""
        vertex_index = self.index_table[vertex]

        self.node_cost[vertex_index] = (
            math.sqrt(
                (vertex[0] - parent_node[0]) ** 2
                + (vertex[1] - parent_node[1]) ** 2
            )
            + self.node_cost[self.index_table[parent_node]]
        )

        for child in self.node_child[vertex]:
            self.recalculate_cost(vertex, child)

    def reparent_node(self, node: PointType, new_parent_node: PointType):
        current_parent_node = self.node_parent[node]

        self.node_child[current_parent_node].remove(node)

        self.node_parent[node] = new_parent_node
        self.node_child[new_parent_node].append(node)

        self.recalculate_cost(new_parent_node, node)

    def sample_random_position(self, greedy_bias: Number = 0.05):
        while True:
            if np.random.random() < (1.0 - greedy_bias):
                # Return random sample
                # sampled_position = self._clip_position_to_edge(
                # np.multiply((self.map.shape[1] - 1, self.map.shape[0] - 1), np.random.uniform(size=(1,2)))
                # np.multiply((self.map.shape[1] - 1, self.map.shape[0] - 1), np.random.uniform(size=(1,2)))
                # )
                grid_index = self._clip_index_to_edge(
                    np.multiply(
                        (self.map.shape[1] - 1, self.map.shape[0] - 1),
                        np.random.uniform(size=(1, 2)),
                    ).astype(int)
                    # np.multiply((self.map.shape[1] - 1, self.map.shape[0] - 1), np.random.uniform(size=(1,2)))
                )
                sampled_position = self.map_centers[
                    grid_index[0, 1], grid_index[0, 0]
                ]
            else:
                # Goal biased (goal)
                sampled_position = self.goal_position_np

                sampled_position = self._round_position_to_nearest_center(
                    sampled_position
                )

                # Check if blocked
                grid_index, in_grid = self._get_grid_index(sampled_position)
                # print(f'{sampled_position=} {grid_index=} {in_grid=}')

                if not np.all(in_grid):
                    # Shouldnt happen
                    # print('Not in grid')
                    continue

            # print('Grid Index: {}')
            if not self.map[grid_index[0, 1], grid_index[0, 0]]:
                break

        return sampled_position

    def step(self, dQ, greedy_bias=0.05, neighbour_radius=0):
        if self.start_position is None:
            raise ValueError('Goal and Start havent been set.')

        random_position = self.sample_random_position(greedy_bias=greedy_bias)

        # Find point on tree closest to random position
        nodes_vector = np.array(self.nodes)
        nodes_difference = random_position - nodes_vector
        nodes_distance = np.linalg.norm(nodes_difference, axis=-1)
        nodes_distance_sort_indices = np.argsort(nodes_distance)

        extend_node_index = nodes_distance_sort_indices[0]

        # extend_point = self.nodes[extend_node_index]
        extend_point_np = np.expand_dims(nodes_vector[extend_node_index], 0)

        self._random_position = random_position
        self._extend_point = extend_point_np

        # Steer
        try:
            new_position, pass_goal = self.steer(
                extend_point_np,
                random_position,
                dQ,
                stop_goal_distance=(self.map_x_cell_size + self.map_y_cell_size)
                / 4,
            )
        except ValueError:
            # Cant steer
            return self.found

        # See if passed goal
        if pass_goal:
            new_position = self.goal_position_np

        self._new_position = new_position

        # print(f'{new_position.shape=}')

        # Cost
        # new_position_cost = self.node_cost[extend_node_index] + nodes_distance[extend_node_index] WRONG
        nodes_costs_np = np.array(self.node_cost)
        extend_point_to_new_point = new_position - extend_point_np
        extend_point_to_new_point_distance = np.linalg.norm(
            extend_point_to_new_point
        )
        new_position_cost = (
            nodes_costs_np[extend_node_index]
            + extend_point_to_new_point_distance
        )

        # Find minimum cost neighbour to new point
        nodes_to_new_position = new_position - nodes_vector
        nodes_to_new_position_distance = np.linalg.norm(
            nodes_to_new_position, axis=-1
        )

        neighbour_mask = nodes_to_new_position_distance <= neighbour_radius

        # print(f'Neighbour Indices: {np.where(neighbour_mask)}')

        neighbour_costs = nodes_costs_np[neighbour_mask]
        new_position_through_neighbour_cost = (
            neighbour_costs + nodes_to_new_position_distance[neighbour_mask]
        )
        neighbour_candidate_mask = (
            new_position_through_neighbour_cost <= new_position_cost
        )

        # print(f'Neighbour Candidate Indices: {np.where(neighbour_candidate_mask)}')

        neighbour_candidates = nodes_vector[neighbour_mask][
            neighbour_candidate_mask
        ]
        neighbour_candidates_costs = new_position_through_neighbour_cost[
            neighbour_candidate_mask
        ]
        neighbour_candidates_indices = np.where(neighbour_mask)[0][
            neighbour_candidate_mask
        ]

        neighbour_costs_sorted_key = np.argsort(neighbour_candidates_costs)

        neighbour_candidates_sorted = neighbour_candidates[
            neighbour_costs_sorted_key
        ]
        neighbour_candidates_costs_sorted = neighbour_candidates_costs[
            neighbour_costs_sorted_key
        ]
        neighbour_candidates_indices_sorted = neighbour_candidates_indices[
            neighbour_costs_sorted_key
        ]

        minimum_index = extend_node_index
        minimum_cost = new_position_cost

        for (
            extend_candidate,
            extend_candidate_cost,
            extend_candidate_index,
        ) in zip(
            neighbour_candidates_sorted,
            neighbour_candidates_costs_sorted,
            neighbour_candidates_indices_sorted,
        ):
            # print(f'Checking Neighbour: {extend_candidate=} {extend_candidate_cost=} {extend_candidate_index=}')
            if self.obstacle_free(
                np.expand_dims(extend_candidate, axis=0), new_position
            ):
                # print(f'Obstacle Free')
                minimum_index = extend_candidate_index
                minimum_cost = extend_candidate_cost
                break

        minimum_cost_neighbour = self.nodes[minimum_index]
        # print(f'{minimum_cost_neighbour=} {minimum_index=} {minimum_cost=}')

        # Update
        # print(f'{new_position=}')
        new_position_tuple = tuple(*new_position)
        # print(f'{new_position_tuple=}')

        if new_position_tuple not in self.index_table:
            #  Adding node
            # print(f'Adding Node: {new_position_tuple=} {minimum_cost_neighbour=} {minimum_index=} {minimum_cost=}')
            self._add_node(
                new_position_tuple,
                node_cost=minimum_cost,
                parent=minimum_cost_neighbour,
            )
            # print(self.goal_position)
            if new_position_tuple == self.goal_position:
                self.found = True

        else:
            # Rewire to less  cost parent
            new_position_index = self.index_table[new_position_tuple]

            if self.node_cost[new_position_index] > minimum_cost:
                # Rewire to lesser cost parent
                self.node_cost[new_position_index] = minimum_cost
                self.reparent_node(new_position_tuple, minimum_cost_neighbour)

        # Attemp to rewire neighbours through new node
        cost_from_new = minimum_cost + nodes_to_new_position_distance
        new_neighbour_mask = neighbour_mask[:]  # copy
        new_neighbour_mask[minimum_index] = False

        rewire_mask = (
            cost_from_new[new_neighbour_mask]
            < nodes_costs_np[new_neighbour_mask]
        )
        rewire_indices = np.where(new_neighbour_mask)[0][rewire_mask]

        for rewire_index in rewire_indices:
            attempt_node = nodes_vector[rewire_index]
            attempt_node_tuple = tuple(attempt_node)

            if attempt_node_tuple != new_position_tuple and self.obstacle_free(
                new_position, np.expand_dims(attempt_node, axis=0)
            ):
                # Rewire
                self.reparent_node(attempt_node_tuple, new_position_tuple)

        return self.found

    def trace_path(self):
        path = []

        current_node = self.goal_position
        while current_node is not None:
            path.append(current_node)

            current_node = self.node_parent[current_node]

        return path[::-1]

    def plot(
        self,
        ax: matplotlib.axes.Axes,
        plot_start: bool = True,
        plot_goal: bool = True,
        plot_debug: bool = False,
        plot_path: bool = True,
    ):
        ax.imshow(
            self.map,
            extent=[
                self.map_centers[0, 0, 0],
                self.map_centers[-1, -1, 0],
                self.map_centers[-1, -1, 1],
                self.map_centers[0, 0, 1],
            ],
        )

        node_array = np.array(self.nodes)

        lines = []

        for node, parent in self.node_parent.items():
            if parent is None:
                continue

            lines.append([node, parent])

        line_collection = matplotlib.collections.LineCollection(lines)
        ax.add_collection(line_collection)

        if len(node_array) > 0:
            ax.scatter(node_array[:, 0], node_array[:, 1])

        if plot_start and self.start_position is not None:
            ax.scatter(
                self.start_position_np[:, 0],
                self.start_position_np[:, 1],
                color=(0, 1, 0, 1),
                label='Start',
            )

        if plot_goal and self.goal_position is not None:
            ax.scatter(
                self.goal_position_np[:, 0],
                self.goal_position_np[:, 1],
                color=(1, 0, 0, 1),
                label='Goal',
            )

        if plot_path:
            if self.found:
                path_color = (0.5, 0.0, 0.7, 1.0)

                path = self.trace_path()
                path_nodes = np.array(path)

                path_lines = []
                for i in range(len(path) - 1):
                    path_lines.append((path[i], path[i + 1]))

                path_line_collection = matplotlib.collections.LineCollection(
                    path_lines, colors=path_color
                )
                ax.add_collection(path_line_collection)

                ax.scatter(
                    path_nodes[:, 0],
                    path_nodes[:, 1],
                    color=path_color,
                    label='Path Points',
                )

        if plot_debug:
            if self._random_position is not None:
                ax.scatter(
                    self._random_position[:, 0],
                    self._random_position[:, 1],
                    color=(0.5, 0.5, 0, 1.0),
                    label='Random Sample',
                )
            if self._extend_point is not None:
                ax.scatter(
                    self._extend_point[:, 0],
                    self._extend_point[:, 1],
                    color=(0.5, 0, 0.7, 1.0),
                    label='Extend Point',
                )
            if self._new_position is not None:
                ax.scatter(
                    self._new_position[:, 0],
                    self._new_position[:, 1],
                    color=(0.3, 0.6, 0.0, 1.0),
                    label='Step Final Point',
                )

    def get_line_grid_indices(self, p1, p2, inflate_radius, epsilon=1e-5):
        """
        Assume p1 and p2 is on map

        Parameters
        ----------


        """

        # Form line direction
        direction_vector = p2 - p1

        direction_vector_norm = np.linalg.norm(direction_vector)
        if direction_vector_norm < epsilon:
            # Same point
            raise ValueError(
                f'Distance between p1 and p2 too low: {direction_vector_norm}. Same point'
            )

        direction_vector_unit = direction_vector / np.expand_dims(
            direction_vector_norm, -1
        )

        # Query related blocks
        points_check = np.concatenate([p1, p2], axis=0)

        grid_index_range, in_grid = self._get_grid_index(points_check)

        # in grid must be all true
        sorted_range = np.sort(grid_index_range, axis=0)
        sorted_range[1] += 1  # End range is exclusive when querying

        # TODO: Check if sorted_range is out of range -> if out of range then p1 or p2 is not in map

        clipped_sorted_range = self._clip_index_to_edge(sorted_range)
        block_size = np.squeeze(np.diff(clipped_sorted_range, axis=0))

        if np.any(block_size == 0):
            # Rectangle lies outside of grid
            raise np.zeros((0, 2), dtype=int)

        block_centers = self.map_centers[
            sorted_range[0, 1] : sorted_range[1, 1],
            sorted_range[0, 0] : sorted_range[1, 0],
        ]

        block_centers_flat = block_centers.reshape(-1, 2)

        line_perpendicular_angle = np.arctan2(
            abs(direction_vector_unit[0, 1]), abs(direction_vector_unit[0, 0])
        )

        if line_perpendicular_angle < self.map_corner_phi:
            # left - right bound
            distance_threshold = self.map_x_cell_size / (
                2 * np.cos(line_perpendicular_angle)
            )
        else:
            # top - bottom bound
            distance_threshold = self.map_y_cell_size / (
                2 * np.sin(line_perpendicular_angle)
            )

        distance_threshold *= max(1.0, 2.0 * (inflate_radius - 1) + 1)

        (
            block_centers_parallel_distance,
            block_centers_perpendicular_distance,
        ) = get_point_line_parallel_perpendicular(
            block_centers_flat, p1, direction_vector_unit
        )

        line_block_mask = (
            block_centers_perpendicular_distance <= distance_threshold
        )

        line_block_parallel_distance = block_centers_parallel_distance[
            line_block_mask
        ]

        line_order_indices = np.argsort(line_block_parallel_distance)

        # Convert to 2d index
        y_indices, x_indices = np.divmod(
            np.where(line_block_mask)[0][line_order_indices], block_size[0]
        )

        stacked_indices = (
            np.stack([x_indices, y_indices], axis=-1)
            + clipped_sorted_range[0, :]
        )

        return stacked_indices

    def get_path_mask_indices(
        self, path: PointType, inflate_radius: Number
    ) -> IndexArrayType:
        """
        Parameters
        ----------
        inflate_radius: Number
            radius to inflate to ( base line at 1 px )

        Return
        ------
        path_array_indices:
        """

        path_length = len(path)

        path_indices = []

        for i in range(path_length - 1):
            p1 = np.expand_dims(path[i], 0)
            p2 = np.expand_dims(path[i + 1], 0)

            path_indices.append(
                self.get_line_grid_indices(p1, p2, inflate_radius)
            )

        path_indices_np = np.concatenate(path_indices, axis=0)

        return path_indices_np

    def check_index_colide(self, path_indices: IndexArrayType) -> bool:
        return np.any(self.map[path_indices[:, 1], path_indices[:, 0]])
