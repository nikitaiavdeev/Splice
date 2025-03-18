import numpy as np
from typing import Optional
from classes.nodes import Node
from classes.beam import Beam
from classes.cbush import CBush
from classes.mpc import MPC


class FEM:
    def __init__(self):
        self.nodes: list[Node] = []
        self.beams: list[Beam] = []
        self.cbush: list[CBush] = []
        self.mpc: list[MPC] = []

    def add_node(
        self, 
        x: float, 
        y: float, 
        fixed_dof: Optional[np.ndarray] = None,
        load_dof: Optional[np.ndarray] = None
    ) -> Node:
        """
        Creates and adds a node to the FEM system.
        
        Args:
            x: X-coordinate of the node
            y: Y-coordinate of the node
            fixed_dof: Array of fixed degrees of freedom (0:u, 1:v, 2:θ)
            load_dof: Array of applied loads [Fx, Fy, M]
            
        Returns:
            Created Node object
        """

        fixed_dof = np.array([]) if fixed_dof is None else np.asarray(fixed_dof, dtype=int)
        load_dof = np.zeros(3) if load_dof is None else np.asarray(load_dof)

        if load_dof.size != 3:
            raise ValueError("load_dof must have exactly 3 components [Fx, Fy, M]")
        if not np.all(np.isin(fixed_dof, [0, 1, 2])):
            raise ValueError("fixed_dof must contain only 0, 1, or 2")
        
        new_node = Node(index=len(self.nodes), x=x, y=y, fixed_dof=fixed_dof, load_dof=load_dof)
        self.nodes.append(new_node)
        return new_node

    def add_beam(self, area: float, inertia: float, elastic_modulus: float, node_1: Node, node_2: Node) -> Beam:
        """
        Creates and adds a beam element between two nodes.
        
        Args:
            area: Cross-sectional area of the beam
            inertia: Moment of inertia of the beam
            elastic_modulus: Elastic modulus of the beam material
            node_1: First node connected by the beam
            node_2: Second node connected by the beam
        
        Returns:
            Created Beam object
        """

        if node_1 == node_2:
            raise ValueError("Beam cannot connect a node to itself")
        if area <= 0 or inertia <= 0 or elastic_modulus <= 0:
            raise ValueError("Area, inertia, and elastic modulus must be positive")
    
        new_beam = Beam(area=area, inertia=inertia, elastic_modulus=elastic_modulus, node_1=node_1, node_2=node_2)
        self.beams.append(new_beam)
        return new_beam

    def add_cbush(self, axial_stiffness: float, rotational_stiffness: float, node_1: Node, node_2: Node) -> CBush:
        """
        Creates and adds a CBush element (spring/damper) between two nodes.
        
        Args:
            axial_stiffness: Axial stiffness of the CBush element
            rotational_stiffness: Rotational stiffness of the CBush element
            node_1: First node connected by the CBush element
            node_2: Second node connected by the CBush element
        
        Returns:
            Created CBush object
        """

        if node_1 == node_2:
            raise ValueError("CBush cannot connect a node to itself")
        if axial_stiffness <= 0 or rotational_stiffness <= 0:
            raise ValueError("Axial and rotational stiffness must be positive")

        new_cbush = CBush(axial_stiffness=axial_stiffness, rotational_stiffness=rotational_stiffness, node_1=node_1, node_2=node_2)
        self.cbush.append(new_cbush)
        return new_cbush

    def add_mpc(self, master_node: Node, slave_node: Node, dofs: np.ndarray) -> MPC:
        """
        Creates and adds an MPC constraint between two nodes.

        Args:
            master_node: Master node of the MPC constraint
            slave_node: Slave node of the MPC constraint
            dofs: Array of dependent degrees of freedom (0:u, 1:v, 2:θ) 
        
        Returns:
            Created MPC object
        """

        if master_node == slave_node:
            raise ValueError("MPC cannot connect a node to itself")
        if not np.all(np.isin(dofs, [0, 1, 2])):
            raise ValueError("MPC dofs must contain only 0, 1, or 2")
        
        new_mpc = MPC(master_node=master_node, slave_node=slave_node, dofs=dofs)
        self.mpc.append(new_mpc)
        return new_mpc

    @property
    def stiffness_matrix(self) -> np.ndarray:
        """Assembles the global stiffness matrix."""
        dof_count = len(self.nodes) * 3
        k_matrix = np.zeros((dof_count, dof_count))

        # Assemble stiffness matrices for beams and CBush elements
        for element in self.beams + self.cbush:
            n1_idx, n2_idx = element.node_1.index * 3, element.node_2.index * 3
            k_global = element.global_stiffness_matrix

            # Vectorized assembly
            indices = np.array([n1_idx, n1_idx+1, n1_idx+2, n2_idx, n2_idx+1, n2_idx+2])
            k_matrix[np.ix_(indices, indices)] += k_global

        return k_matrix
    
    @property
    def mpc_constraints(self) -> list[tuple[int, list[int], np.ndarray]]:
        """
        Returns all MPC constraints in the system.
        
        Returns:
            List of tuples containing (slave_dof, master_dofs, coefficients)
        """

        constraints = []
        for mpc in self.mpc:
            constraints.extend(mpc.constraints)
        return constraints

    def apply_loads(self) -> np.ndarray:
        """
        Applies nodal loads to the global load vector.
        
        Returns:
            Global load vector
        """
        dof_count = len(self.nodes) * 3
        load_vector = np.zeros(dof_count)

        for node in self.nodes:
            node_idx = node.index * 3
            load_vector[node_idx:node_idx+3] = node.load_dof

        return load_vector

    def get_free_dofs(self) -> np.ndarray:
        """
        Returns array of free DOFs considering boundary conditions.
        
        Returns:
            Array of free DOFs
        """
        free_dofs = np.arange(len(self.nodes) * 3)
        fixed_indices = np.concatenate([
            node.index * 3 + node.fixed_dof for node in self.nodes 
            if node.fixed_dof.size > 0
        ])
        return np.setdiff1d(free_dofs, fixed_indices)

    def apply_mpc_constraints(self, k_global: np.ndarray, 
                            load_vector: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Modifies stiffness matrix and load vector for MPC constraints.

        Args:
            k_global: Global stiffness matrix
            load_vector: Global load vector
        
        Returns:
            Tuple of free DOFs, modified stiffness matrix, and modified load vector
        """
        free_dofs = np.arange(len(self.nodes) * 3)
        for slave_dof, master_dofs, coeffs in self.mpc_constraints:
            free_dofs = free_dofs[free_dofs != slave_dof]
            k_global[master_dofs] += coeffs[:, np.newaxis] * k_global[slave_dof]
            k_global[:, master_dofs] += (coeffs[:, np.newaxis] * k_global[:, slave_dof]).T
            load_vector[master_dofs] += coeffs * load_vector[slave_dof]

        return free_dofs, k_global, load_vector

    def solve(self) -> np.ndarray:
        """
        Solves for nodal displacements considering all constraints.
        
        Returns:
            Array of displacements [u, v, θ] for each node
        """

        k_global = self.stiffness_matrix
        load_vector = self.apply_loads()
        
        # Apply constraints
        bc_free_dofs = self.get_free_dofs()
        mpc_free_dofs, k_global, load_vector = self.apply_mpc_constraints(k_global, load_vector)
        free_dofs = np.intersect1d(bc_free_dofs, mpc_free_dofs)

        # Solve reduced system
        k_reduced = k_global[np.ix_(free_dofs, free_dofs)]
        load_reduced = load_vector[free_dofs]
        
        try:
            disp_reduced = np.linalg.solve(k_reduced, load_reduced)
        except np.linalg.LinAlgError:
            raise ValueError("System is singular or ill-conditioned")

        # Reconstruct full displacement vector
        displacements = np.zeros(len(self.nodes) * 3)
        displacements[free_dofs] = disp_reduced

        # Compute dependent DOFs
        for slave_dof, master_dofs, coeffs in self.mpc_constraints:
            displacements[slave_dof] = np.dot(coeffs, displacements[master_dofs])

        return displacements


# Example usage
fem = FEM()
