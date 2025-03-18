import numpy as np
from functools import cached_property

class Node:
    """A 2D node in a structural analysis model with position, constraints, and loads."""

    def __init__(
        self,
        index: int,
        x: float,
        y: float,
        fixed_dof: np.ndarray = np.array([], dtype=int),
        load_dof: np.ndarray = np.zeros(3, dtype=np.float64),
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
        fixed_dof = np.asarray(fixed_dof, dtype=int)
        if not np.all((fixed_dof >= 0) & (fixed_dof < 3)):
            raise ValueError("Fixed DOF indices must be 0 (u_x), 1 (u_y), or 2 (θ_z).")
        
        # Validate load_dof
        load_dof = np.asarray(load_dof, dtype=np.float64)
        if load_dof.shape != (3,):
            raise ValueError("load_dof must be a 3-element array [F_x, F_y, M_z].")
        
        self.index: int = index
        self.coord: np.ndarray = np.array([x, y], dtype=np.float64)
        self.fixed_dof: np.ndarray = fixed_dof
        self.load_dof: np.ndarray = load_dof

    @cached_property
    def x(self) -> float:
        """X-coordinate of the node."""
        return self.coord[0]

    @cached_property
    def y(self) -> float:
        """Y-coordinate of the node."""
        return self.coord[1]

    @cached_property
    def dof_indices(self) -> np.ndarray:
        """Global DOF indices for this node (assuming 3 DOF per node)."""
        return np.array([self.index * 3, self.index * 3 + 1, self.index * 3 + 2], dtype=int)
    
    @cached_property
    def free_dof(self) -> np.ndarray:
        """Array of free degrees of freedom (not fixed)."""
        return np.array([self.index * 3 + i for i in range(3) if i not in self.fixed_dof], dtype=int)

    def __repr__(self) -> str:
        """String representation of the node."""
        return (f"Node(index={self.index}, x={self.x:.3f}, y={self.y:.3f}, "
                f"fixed_dof={self.fixed_dof.tolist()}, load_dof={self.load_dof.tolist()})")