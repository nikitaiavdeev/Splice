import numpy as np
from classes.nodes import Node
from functools import cached_property


class Beam:
    """A 2D beam element for structural analysis with stiffness matrix computation."""
    
    def __init__(
        self,
        area: float,
        inertia: float,
        elastic_modulus: float,
        node_1: Node,
        node_2: Node,
    ) -> None:
        """
        Initialize a beam element with geometric and material properties.
        
        Args:
            area: Cross-sectional area of the beam (in²).
            inertia: Moment of inertia of the beam (in⁴).
            elastic_modulus: Young's modulus of the material (psi).
            node_1: Starting node of the beam.
            node_2: Ending node of the beam.

            Raises:
                ValueError: If area, inertia, or elastic_modulus are non-positive.
                ValueError: If the two nodes are the same (coincide).
                ValueError: If the beam length is zero.
        """

        # Validate node connection
        if node_1 == node_2:
            raise ValueError("Beam cannot connect a node to itself.")

        # Validate input values
        if not all(x > 0 for x in (area, inertia, elastic_modulus)):
            raise ValueError("Area, inertia, and elastic modulus must be positive.")
        
        self.area = float(area)
        self.inertia = float(inertia)
        self.elastic_modulus = float(elastic_modulus)
        self.node_1 = node_1
        self.node_2 = node_2

        # Validate beam length
        if self.length == 0:
            raise ValueError("Beam length cannot be zero (nodes coincide).")

    @cached_property
    def delta_coord(self) -> np.ndarray:
        """Displacement vector between the two nodes."""
        return self.node_2.coord - self.node_1.coord

    @cached_property
    def length(self) -> float:
        """Length of the beam."""
        length = np.linalg.norm(self.delta_coord)

        return float(length)

    @cached_property
    def angle(self) -> float:
        """Angle of the beam relative to the x-axis in radians."""
        return np.arctan2(self.delta_coord[1], self.delta_coord[0])

    @cached_property
    def cos_sin(self) -> tuple[float, float]:
        """Cosine and sine of the beam angle."""
        angle = self.angle  # Cache to avoid recomputation
        return float(np.cos(angle)), float(np.sin(angle))

    @cached_property
    def local_stiffness_matrix(self) -> np.ndarray:
        """Local stiffness matrix for the beam in its local coordinate system."""
        l = self.length
        one_over_l = 1.0 / l
        eal = self.elastic_modulus * self.area * one_over_l
        eil = self.elastic_modulus * self.inertia * one_over_l
        eil2 = eil * one_over_l
        eil3 = eil2 * one_over_l

        return np.array(
            [
                [eal, 0.0, 0.0, -eal, 0.0, 0.0],
                [0.0, 12.0 * eil3, 6.0 * eil2, 0.0, -12.0 * eil3, 6.0 * eil2],
                [0.0, 6.0 * eil2, 4.0 * eil, 0.0, -6.0 * eil2, 2.0 * eil],
                [-eal, 0.0, 0.0, eal, 0.0, 0.0],
                [0.0, -12.0 * eil3, -6.0 * eil2, 0.0, 12.0 * eil3, -6.0 * eil2],
                [0.0, 6.0 * eil2, 2.0 * eil, 0.0, -6.0 * eil2, 4.0 * eil],
            ]
        )

    @cached_property
    def global_stiffness_matrix(self) -> np.ndarray:
        """Global stiffness matrix transformed to the global coordinate system."""
        cos, sin = self.cos_sin
        t_matrix = np.array(
            [
                [cos, sin, 0.0, 0.0, 0.0, 0.0],
                [-sin, cos, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, cos, sin, 0.0],
                [0.0, 0.0, 0.0, -sin, cos, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        k_local = self.local_stiffness_matrix
        return t_matrix.T @ k_local @ t_matrix
    
    def __repr__(self) -> str:
        return f"Beam({self.node_1.index}, {self.node_2.index}, A={self.area}, I={self.inertia}, E={self.elastic_modulus})"
    