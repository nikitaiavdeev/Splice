import numpy as np
from numpy.typing import NDArray

from classes.nodes import Node


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

        # Update properties
        self.update_properties()

        # Validate beam length
        if self.length == 0:
            raise ValueError("Beam length cannot be zero (nodes coincide).")

    def update_properties(self) -> None:
        self.delta_coord = self.calc_delta_coord()
        self.length = self.calc_length()
        self.transformation_matrix = self.calc_transformation_matrix()
        self.local_stiffness_matrix = self.calc_local_stiffness_matrix()
        self.geometric_stiffness_matrix = self.calc_geometric_stiffness_matrix()
        self.global_stiffness_matrix = self.calc_global_stiffness_matrix()

    def calc_delta_coord(self) -> NDArray[np.float64]:
        """Displacement vector between the two nodes."""
        try:
            return (self.node_2.coord + self.node_2.displ[:2]) - (
                self.node_1.coord + self.node_1.displ[:2]
            )
        except Exception as _:
            return self.node_2.coord - self.node_1.coord

    def calc_length(self) -> float:
        """Length of the beam."""
        length = np.linalg.norm(self.delta_coord)

        if length == 0:
            raise ValueError("Beam length is zero after deformation.")

        return float(length)

    def calc_transformation_matrix(self) -> NDArray[np.float64]:
        """Transformation matrix from local to global coordinates."""

        c, s = self.delta_coord / self.length
        return np.array(
            [
                [c, s, 0.0, 0.0, 0.0, 0.0],
                [-s, c, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, c, s, 0.0],
                [0.0, 0.0, 0.0, -s, c, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def calc_local_stiffness_matrix(self) -> NDArray[np.float64]:
        """
        Local stiffness matrix in the beam's coordinate system.

        Returns:
            6x6 stiffness matrix for axial, flexural, and rotational DOFs
        """

        # Precompute common terms
        one_over_l = 1.0 / self.length
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
            ],
            dtype=np.float64,
        )

    def calc_geometric_stiffness_matrix(self) -> NDArray[np.float64]:
        """Compute geometric stiffness matrix accounting for axial force."""

        try:
            length = self.length
            axial_force = -(
                self.area
                * self.elastic_modulus
                * (self.node_2.displ[0] - self.node_1.displ[0])
                / length
            )

            return (
                axial_force
                / length
                * np.array(
                    [
                        [1, 0, 0, -1, 0, 0],
                        [0, 6 / 5, length / 10, 0, -6 / 5, length / 10],
                        [
                            0,
                            length / 10,
                            2 * length**2 / 15,
                            0,
                            -length / 10,
                            -(length**2) / 30,
                        ],
                        [-1, 0, 0, 1, 0, 0],
                        [0, -6 / 5, -length / 10, 0, 6 / 5, -length / 10],
                        [
                            0,
                            length / 10,
                            -(length**2) / 30,
                            0,
                            -length / 10,
                            2 * length**2 / 15,
                        ],
                    ]
                )
            )
        except Exception as _:
            return np.zeros((6, 6), dtype=np.float64)

    def calc_global_stiffness_matrix(self) -> NDArray[np.float64]:
        """
        Global stiffness matrix in the global coordinate system.

        Returns:
            6x6 stiffness matrix transformed to global coordinates
        """
        T = self.transformation_matrix
        k_local = self.local_stiffness_matrix + self.geometric_stiffness_matrix
        return T.T @ (k_local @ T)

    def calc_internal_forces(self):
        """
        Calculate internal forces based on current displacements.

        Returns:
            Array of forces [Fx1, Fy1, M1, Fx2, Fy2, M2] in global coordinates
        """
        T = self.transformation_matrix
        displacements_local = T @ np.concatenate((self.node_1.displ, self.node_2.displ))
        self.internal_forces = self.global_stiffness_matrix @ displacements_local

    def __repr__(self) -> str:
        """String representation of the beam."""

        return (
            f"Beam(node_1={self.node_1.index}, node_2={self.node_2.index}, "
            f"A={self.area:.3g}, I={self.inertia:.3g}, E={self.elastic_modulus:.3g})"
        )
