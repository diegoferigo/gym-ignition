# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from itertools import count
from collections import OrderedDict
from typing import Dict, NamedTuple


class GridPoint(NamedTuple):
    x: float
    y: float


class Stride(NamedTuple):
    dx: float
    dy: float


# TODO: SpiralAllocator? SpiralXX?
class ModelsGrid:
    """
    Generator of points following a spiral trajectory in a 2D grid space.

    This class can be used to automatically compute the 2D position of constantly spaced
    models. The trajectory takes the form of a spiral that starts from the starting point
    and follows the grid defined with the stride option.

    Args:
        stride: The 2D grid space definition.
        starting_point: The starting point of the spiral.

    Example:

    .. code-block:: python

        # Create the object starting from (0, 0) with a (1, 1) stride
        grid = ModelsGrid(stride=Stride(dx=1,dy=1))

        # Get the iterator
        trajectory_generator = iter(grid)

        # Get the first three point
        p1 = next(trajectory_generator)  # ( 0,  0)
        p2 = next(trajectory_generator)  # ( 0, -1)
        p3 = next(trajectory_generator)  # (-1, -1)

        # Make p2 available again (it could happen after model deletion)
        grid.free(p2)

        assert not grid.is_free(p1)
        assert grid.is_free(p2)

        # Get the next available point
        p4 = next(trajectory_generator)  # () that is p2
    """

    def __init__(self,
                 stride: Stride,
                 starting_point=GridPoint(x=0, y=0)):

        self._generator = self.spiral_generator(
            stride=stride, starting_point=starting_point)

        # Point -> taken
        self._slots: Dict[GridPoint, bool] = OrderedDict()

    @staticmethod
    def _front(current_point: GridPoint, stride: Stride) -> GridPoint:
        new_x = current_point.x + stride.dx
        new_y = current_point.y
        return GridPoint(x=new_x, y=new_y)

    @staticmethod
    def _back(current_point: GridPoint, stride: Stride) -> GridPoint:
        new_x = current_point.x - stride.dx
        new_y = current_point.y
        return GridPoint(x=new_x, y=new_y)

    @staticmethod
    def _left(current_point: GridPoint, stride: Stride) -> GridPoint:
        new_x = current_point.x
        new_y = current_point.y + stride.dy
        return GridPoint(x=new_x, y=new_y)

    @staticmethod
    def _right(current_point: GridPoint, stride: Stride) -> GridPoint:
        new_x = current_point.x
        new_y = current_point.y - stride.dy
        return GridPoint(x=new_x, y=new_y)

    def __iter__(self):
        return self

    def is_free(self, point: GridPoint) -> bool:
        """
        Checks if a point of a trajectory is free.

        Args:
            point: A point of the trajectory.

        Raises:
            ValueError: If the point is not part of the trajectory.

        Returns:
            True if the point is free, false if it is taken.
        """

        if point not in self._slots.keys():
            raise ValueError("Failed to find point in spiral trajectory")

        return not self._slots[point]

    def free(self, point: GridPoint) -> None:
        """
        Make a trajectory point free.

        Args:
            point: A point of the trajectory.

        Raises:
            ValueError: If the point is not part of the trajectory.
            RuntimeError: If the point is already free.
        """

        if point not in self._slots.keys():
            raise ValueError("Failed to find point in spiral trajectory")

        if self._slots[point] is False:
            raise RuntimeError("The point is already free")

        self._slots[point] = False

    def __next__(self):

        for point, taken in self._slots.items():

            if not taken:
                self._slots[point] = True
                return point

        # Get a new spiral point
        new_point = next(self._generator)
        self._slots[new_point] = True

        return new_point

    @staticmethod
    def spiral_generator(stride: Stride,
                         starting_point: GridPoint = GridPoint(x=0, y=0)):
        """
        The spiral trajectory generator.

        This is the python generator used to compute the trajectory.

        Args:
            stride: the stride of the 2D grid.
            starting_point: the initial point of the spiral.

        Yields:
            :py:class:`GridPoint`: the last point of the trajectory.
        """

        current_point = starting_point
        yield current_point

        is_odd = lambda num: num % 2 != 0

        for num in count(start=1):

            if is_odd(num):
                current_point = ModelsGrid._right(current_point, stride)
                yield current_point

                for i in range(num):
                    current_point = ModelsGrid._back(current_point, stride)
                    yield current_point

                for i in range(num):
                    current_point = ModelsGrid._left(current_point, stride)
                    yield current_point
            else:
                current_point = ModelsGrid._left(current_point, stride)
                yield current_point

                for i in range(num):
                    current_point = ModelsGrid._front(current_point, stride)
                    yield current_point

                for i in range(num):
                    current_point = ModelsGrid._right(current_point, stride)
                    yield current_point
