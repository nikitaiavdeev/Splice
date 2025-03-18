from functools import cached_property

import numpy as np

from classes.nodes import Node


class MPC(object):
    """A Multi-Point Constraint (MPC) enforcing kinematic relationships between nodes."""

    def __init__(self, master_node: Node, slave_node: Node, dofs: np.ndarray):
        """
        Initialize an MPC relationship between a master and slave node.

        Args:
            master_node: Node controlling the slave node's motion.
            slave_node: Node whose DOFs are constrained by the master.
            dofs: Array of DOF indices (0: u_x, 1: u_y, 2: θ_z) to constrain.

        Raises:
            ValueError: If master and slave nodes are the same, or dofs are invalid.
        """

        if master_node.index == slave_node.index:
            raise ValueError("Master and slave nodes must be distinct.")

        dofs = np.asarray(dofs, dtype=int)
        if not np.all((dofs >= 0) & (dofs < 3)):
            raise ValueError("DOF indices must be 0 (u_x), 1 (u_y), or 2 (θ_z).")
        if dofs.size == 0:
            raise ValueError("At least one DOF must be specified for constraint.")

        self.master_node = master_node
        self.slave_node = slave_node
        self.dofs = dofs

    @cached_property
    def constraints(self) -> list[tuple[int, list[int], np.ndarray]]:
        """
        Returns the constraint matrix for the MPC relationship between master and slave nodes.

        The constraints define how slave DOFs depend on master DOFs:
        - DOF 0: u_s = u_m - θ_m * (y_s - y_m)  # x-displacement
        - DOF 1: v_s = v_m + θ_m * (x_s - x_m)  # y-displacement
        - DOF 2: θ_s = θ_m                     # rotation

        Returns:
            List of tuples containing (slave_dof, master_dofs, coefficients),
            where each tuple represents one constraint equation.
        """

        # Pre-calculate indices and coordinates
        master_idx = self.master_node.index * 3
        slave_idx = self.slave_node.index * 3
        x_m, y_m = self.master_node.coord
        x_s, y_s = self.slave_node.coord
        master_dofs = [master_idx + i for i in range(3)]

        # Coordinate differences (calculated once)
        dx = x_s - x_m
        dy = y_s - y_m

        # Coefficient mapping for each DOF
        coeff_map = {
            0: np.array([1.0, 0.0, -dy], dtype=np.float64),  # u_s = u_m - θ_m * dy
            1: np.array([0.0, 1.0, dx], dtype=np.float64),  # v_s = v_m + θ_m * dx
            2: np.array([0.0, 0.0, 1.0], dtype=np.float64),  # θ_s = θ_m
        }

        # Generate constraints efficiently
        return [(slave_idx + dof, master_dofs, coeff_map[dof]) for dof in self.dofs]

    def __repr__(self) -> str:
        return f"MPC(master={self.master_node.index}, slave={self.slave_node.index}, dofs={self.dofs.tolist()})"
