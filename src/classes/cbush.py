from functools import cached_property

import numpy as np
from numpy.typing import NDArray

from classes.nodes import Node


class CBush:
    """A CBush element representing a spring-damper with axial and rotational stiffness."""

    def __init__(
        self,
        axial_stiffness: float,
        rotational_stiffness: float,
        node_1: Node,
        node_2: Node,
    ):
        """
        Initialize a CBush element with stiffness properties.

        Args:
            axial_stiffness: Axial stiffness of the element (N/m).
            rotational_stiffness: Rotational stiffness of the element (N·m/rad).
            node_1: First node of the element.
            node_2: Second node of the element.
        Raises:
            ValueError: If axial_stiffness or rotational_stiffness are negative.
            ValueError: If node_1 is the same as node_2.
        """

        # Validate node connection
        if node_1 == node_2:
            raise ValueError("CBush cannot connect a node to itself")

        # Validate stiffness values
        if axial_stiffness < 0 or rotational_stiffness < 0:
            raise ValueError("Stiffness values must be non-negative.")

        self.axial_stiffness = float(axial_stiffness)
        self.rotational_stiffness = float(rotational_stiffness)
        self.node_1 = node_1
        self.node_2 = node_2

    @cached_property
    def global_stiffness_matrix(self) -> NDArray[np.float64]:
        """
        Global stiffness matrix transformed to global coordinates.

        Returns:
            6x6 stiffness matrix in global coordinate system
        """

        ka = self.axial_stiffness
        kr = self.rotational_stiffness

        return np.array(
            [
                [ka, 0, 0, -ka, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, kr, 0, 0, -kr],
                [-ka, 0, 0, ka, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, -kr, 0, 0, kr],
            ],
            dtype=np.float64,
        )

    @property
    def internal_forces(self) -> NDArray[np.float64]:
        """
        Calculate internal forces based on current displacements.

        Returns:
            Array of forces [Fx1, Fy1, M1, Fx2, Fy2, M2] in global coordinates
        """
        displacements = np.concatenate((self.node_1.displ, self.node_2.displ))
        return self.global_stiffness_matrix @ displacements

    def __repr__(self) -> str:
        """String representation of the CBush element."""
        return (
            f"CBush(node_1={self.node_1.index}, node_2={self.node_2.index}, "
            f"ka={self.axial_stiffness:.3g}, kr={self.rotational_stiffness:.3g})"
        )
