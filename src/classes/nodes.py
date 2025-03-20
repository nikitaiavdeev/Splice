from functools import cached_property

import numpy as np
from numpy.typing import NDArray


class Node:
    """A 2D node in a structural analysis model with position, constraints, and loads."""

    def __init__(
        self,
        index: int,
        x: float,
        y: float,
        fixed_dof: NDArray[np.int_] | None = None,
        load_dof: NDArray[np.float64] | None = None,
    ):
        """
        Initialize a node with position, boundary conditions, and applied loads.

        Args:
            index: Unique identifier for the node (non-negative integer).
            x: X-coordinate of the node (m).
            y: Y-coordinate of the node (m).
            fixed_dof: Array of fixed degrees of freedom (0: u_x, 1: u_y, 2: θ_z).
                       Defaults to empty array (no constraints).
            load_dof: Array of applied loads [F_x, F_y, M_z] (N, N, N·m).
                      Defaults to zero loads.
        Raises:
            ValueError: If index is negative, fixed_dof contains invalid indices,
                        or load_dof has incorrect shape.
        """

        # Validate index
        if not isinstance(index, int) or index < 0:
            raise ValueError("Index must be a non-negative integer.")

        # Validate fixed_dof
        if fixed_dof is None:
            fixed_dof = np.array([], dtype=int)
        else:
            fixed_dof = np.asarray(fixed_dof, dtype=int)
        if not np.all((fixed_dof >= 0) & (fixed_dof < 3)):
            raise ValueError("Fixed DOF indices must be 0 (u_x), 1 (u_y), or 2 (θ_z).")

        # Validate load_dof
        if load_dof is None:
            load_dof = np.zeros(3, dtype=np.float64)
        else:
            load_dof = np.asarray(load_dof, dtype=np.float64)
        if load_dof.shape != (3,):
            raise ValueError("load_dof must be a 3-element array [F_x, F_y, M_z].")

        self.index: int = index
        self.coord: NDArray[np.float64] = np.array([x, y], dtype=np.float64)
        self.fixed_dof: NDArray[np.int_] = fixed_dof
        self.load_dof: NDArray[np.float64] = load_dof

        self._displ: NDArray[np.float64] | None = None

    @cached_property
    def x(self) -> np.float64:
        """X-coordinate of the node."""
        return self.coord[0]

    @cached_property
    def y(self) -> np.float64:
        """Y-coordinate of the node."""
        return self.coord[1]

    @cached_property
    def dof_indices(self) -> NDArray[np.int_]:
        """Global DOF indices for this node (assuming 3 DOF per node)."""
        return np.array(
            [self.index * 3, self.index * 3 + 1, self.index * 3 + 2], dtype=int
        )

    @cached_property
    def free_dof(self) -> NDArray[np.int_]:
        """Array of free degrees of freedom (not fixed)."""
        return np.array(
            [self.index * 3 + i for i in range(3) if i not in self.fixed_dof], dtype=int
        )

    @property
    def displ(self) -> NDArray[np.float64]:
        """Node displacements [u_x, u_y, θ_z]."""
        if self._displ is None:
            raise ValueError("Displacements not set - solve the model first")
        return self._displ

    @displ.setter
    def displ(self, value: NDArray[np.float64]) -> None:
        """Set node displacements."""
        self._displ = value

    def __repr__(self) -> str:
        """String representation of the node."""
        return (
            f"Node(index={self.index}, x={self.x:.3f}, y={self.y:.3f}, "
            f"fixed_dof={self.fixed_dof.tolist()}, load_dof={self.load_dof.tolist()})"
        )
