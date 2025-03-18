import numpy as np
from classes.nodes import Node
from functools import cached_property



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
    def global_stiffness_matrix(self) -> np.ndarray:
        """Computes the global stiffness matrix for the CBush element."""
        ka = self.axial_stiffness
        kr = self.rotational_stiffness

        return np.array(
            [
                [ka,  0,  0, -ka,  0,  0],
                [ 0,  0,  0,  0,  0,  0],
                [ 0,  0, kr,  0,  0, -kr],
                [-ka, 0,  0,  ka,  0,  0],
                [ 0,  0,  0,  0,  0,  0],
                [ 0,  0, -kr, 0,  0,  kr],
            ],
            dtype=np.float64
        )

    def __repr__(self):
        return f"Cbush({self.node_1.index}, {self.node_2.index}, k={self.axial_stiffness}, kr={self.rotational_stiffness})"