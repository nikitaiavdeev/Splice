import numpy as np
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
        self.axial_stiffness = float(axial_stiffness)
        self.rotational_stiffness = float(rotational_stiffness)
        self.node_1 = node_1
        self.node_2 = node_2

    @property
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
            dtype=np.float64,
        )
