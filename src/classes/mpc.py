from functools import cached_property
from typing import List, Tuple

import numpy as np
from numpy.typing import NDArray

from classes.nodes import Node


class MPC:
    """A Multi-Point Constraint (MPC) enforcing kinematic relationships between nodes."""

    def __init__(
        self, master_node: Node, slave_node: Node, dofs: NDArray[np.int_]
    ) -> None:
        """
        Initialize an MPC relationship between a master and slave node.

        Args:
            master_node: Node controlling the slave node's motion
            slave_node: Node whose DOFs are constrained by the master
            dofs: Array of DOF indices (0: u_x, 1: u_y, 2: θ_z) to constrain

        Raises:
            ValueError: If nodes are identical or coincident, or if DOFs are invalid
        """
        # Convert and validate DOFs
        dofs = np.asarray(dofs, dtype=int)
        if dofs.size == 0:
            raise ValueError("At least one DOF must be specified")
        if not np.all((dofs >= 0) & (dofs <= 2)):
            raise ValueError("DOF indices must be 0 (u_x), 1 (u_y), or 2 (θ_z)")
        if len(np.unique(dofs)) != len(dofs):
            raise ValueError("DOF indices must be unique")

        # Validate nodes
        if master_node is slave_node:
            raise ValueError("Master and slave nodes must be distinct objects")

        self.master_node = master_node
        self.slave_node = slave_node
        self.dofs = dofs

    @cached_property
    def delta_coord(self) -> NDArray[np.float64]:
        """Displacement vector from master to slave node."""
        return self.slave_node.coord - self.master_node.coord

    @cached_property
    def constraints(self) -> List[Tuple[int, List[int], NDArray[np.float64]]]:
        """
        Generate constraint equations for the MPC relationship.

        Constraints define slave DOF dependence on master DOFs:
        - DOF 0: u_s = u_m - θ_m * (y_s - y_m)  # x-displacement
        - DOF 1: v_s = v_m + θ_m * (x_s - x_m)  # y-displacement
        - DOF 2: θ_s = θ_m                     # rotation

        Returns:
            List of tuples (slave_dof, master_dofs, coefficients) where each tuple
            represents one constraint equation
        """
        master_idx = self.master_node.index * 3
        slave_idx = self.slave_node.index * 3
        master_dofs = [master_idx + i for i in range(3)]

        # Coordinate differences
        dx = self.delta_coord[0]
        dy = self.delta_coord[1]

        # Constraint coefficients for each DOF
        coeff_map = {
            0: np.array([1.0, 0.0, -dy], dtype=np.float64),  # u_s = u_m - θ_m * dy
            1: np.array([0.0, 1.0, dx], dtype=np.float64),  # v_s = v_m + θ_m * dx
            2: np.array([0.0, 0.0, 1.0], dtype=np.float64),  # θ_s = θ_m
        }

        return [(slave_idx + dof, master_dofs, coeff_map[dof]) for dof in self.dofs]

    def calc_slave_displacements(self) -> None:
        """
        Apply constrained displacements to the slave node based on master node.
        Assumes master node displacements are already set.
        """
        master_displ = self.master_node.displ
        dx = self.delta_coord[0]
        dy = self.delta_coord[1]

        slave_displ = np.zeros(3, dtype=np.float64)
        for dof in self.dofs:
            if dof == 0:  # u_x
                slave_displ[0] = master_displ[0] - master_displ[2] * dy
            elif dof == 1:  # u_y
                slave_displ[1] = master_displ[1] + master_displ[2] * dx
            elif dof == 2:  # θ_z
                slave_displ[2] = master_displ[2]

        self.slave_node.displ = slave_displ

    def __repr__(self) -> str:
        """String representation of the MPC."""
        return (
            f"MPC(master={self.master_node.index}, slave={self.slave_node.index}, "
            f"dofs={self.dofs.tolist()})"
        )
