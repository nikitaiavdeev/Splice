import numpy as np
from classes.nodes import Node


class Beam:
    def __init__(
        self,
        area: float,
        inertia: float,
        elastic_modulus: float,
        node_1: Node,
        node_2: Node,
    ):
        self.area = float(area)
        self.inertia = float(inertia)
        self.elastic_modulus = float(elastic_modulus)
        self.node_1 = node_1
        self.node_2 = node_2

    @property
    def delta_coord(self) -> np.ndarray:
        return self.node_2.coord - self.node_1.coord

    @property
    def length(self) -> float:
        return np.linalg.norm(self.delta_coord).astype(np.float64)

    @property
    def angle(self) -> float:
        return np.arctan2(self.delta_coord[1], self.delta_coord[0])

    @property
    def cos_sin(self) -> tuple[float, float]:
        """Returns (cos, sin) values of the beam angle."""
        cos_theta = np.cos(self.angle)
        sin_theta = np.sin(self.angle)
        return cos_theta, sin_theta

    @property
    def local_stiffness_matrix(self) -> np.ndarray:
        """Computes the local stiffness matrix for a beam element."""
        one_over_l = 1 / self.length
        eal = self.elastic_modulus * self.area * one_over_l
        eil = self.elastic_modulus * self.inertia * one_over_l
        eil2 = eil * one_over_l
        eil3 = eil2 * one_over_l

        return np.array(
            [
                [eal, 0, 0, -eal, 0, 0],
                [0, 12 * eil3, 6 * eil2, 0, -12 * eil3, 6 * eil2],
                [0, 6 * eil2, 4 * eil, 0, -6 * eil2, 2 * eil],
                [-eal, 0, 0, eal, 0, 0],
                [0, -12 * eil3, -6 * eil2, 0, 12 * eil3, -6 * eil2],
                [0, 6 * eil2, 2 * eil, 0, -6 * eil2, 4 * eil],
            ],
            dtype=np.float64,
        )

    @property
    def global_stiffness_matrix(self) -> np.ndarray:
        """Computes the global stiffness matrix by transforming the local stiffness matrix."""
        cos, sin = self.cos_sin

        t_matrix = np.array(
            [
                [cos, sin, 0, 0, 0, 0],
                [-sin, cos, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, cos, sin, 0],
                [0, 0, 0, -sin, cos, 0],
                [0, 0, 0, 0, 0, 1],
            ],
            dtype=np.float64,
        )

        return t_matrix.T @ self.local_stiffness_matrix @ t_matrix
