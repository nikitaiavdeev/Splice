import numpy as np


class Node:
    def __init__(
        self,
        index: int,
        x: float,
        y: float,
        fixed_dof: np.ndarray = np.array([], dtype=int),
        load_dof: np.ndarray = np.zeros(3, dtype=np.float64),
    ):
        self.index: int = index
        self.coord: np.ndarray = np.array([x, y], dtype=np.float64)
        self.fixed_dof: np.ndarray = fixed_dof
        self.load_dof: np.ndarray = load_dof

    @property
    def x(self) -> float:
        return self.coord[0]

    @property
    def y(self) -> float:
        return self.coord[1]
