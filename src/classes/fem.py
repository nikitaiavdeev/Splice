from functools import cached_property
from typing import List, Optional, Tuple

import numpy as np
from numpy.typing import NDArray

from classes.beam import Beam
from classes.cbush import CBush
from classes.mpc import MPC
from classes.nodes import Node


class FEM:
    """Finite Element Method solver for 2D structural analysis."""

    def __init__(self) -> None:
        self.nodes: List[Node] = []
        self.beams: List[Beam] = []
        self.cbush: List[CBush] = []
        self.mpc: List[MPC] = []

    def add_node(
        self,
        x: float,
        y: float,
        fixed_dof: Optional[NDArray[np.int_]] = None,
        load_dof: Optional[NDArray[np.float64]] = None,
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
        fixed_dof = (
            np.array([], dtype=int)
            if fixed_dof is None
            else np.asarray(fixed_dof, dtype=int)
        )
        load_dof = (
            np.zeros(3) if load_dof is None else np.asarray(load_dof, dtype=float)
        )

        new_node = Node(
            index=len(self.nodes), x=x, y=y, fixed_dof=fixed_dof, load_dof=load_dof
        )
        self.nodes.append(new_node)
        return new_node

    def add_beam(
        self,
        area: float,
        inertia: float,
        elastic_modulus: float,
        node_1: Node,
        node_2: Node,
    ) -> Beam:
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

        new_beam = Beam(
            area=area,
            inertia=inertia,
            elastic_modulus=elastic_modulus,
            node_1=node_1,
            node_2=node_2,
        )
        self.beams.append(new_beam)
        return new_beam

    def add_cbush(
        self,
        axial_stiffness: float,
        rotational_stiffness: float,
        node_1: Node,
        node_2: Node,
    ) -> CBush:
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

        new_cbush = CBush(
            axial_stiffness=axial_stiffness,
            rotational_stiffness=rotational_stiffness,
            node_1=node_1,
            node_2=node_2,
        )
        self.cbush.append(new_cbush)
        return new_cbush

    def add_mpc(
        self, master_node: Node, slave_node: Node, dofs: NDArray[np.int_]
    ) -> MPC:
        """
        Creates and adds an MPC constraint between two nodes.

        Args:
            master_node: Master node of the MPC constraint
            slave_node: Slave node of the MPC constraint
            dofs: Array of dependent degrees of freedom (0:u, 1:v, 2:θ)

        Returns:
            Created MPC object
        """

        new_mpc = MPC(master_node=master_node, slave_node=slave_node, dofs=dofs)
        self.mpc.append(new_mpc)
        return new_mpc

    @cached_property
    def dof_count(self) -> int:
        """Total number of degrees of freedom in the system."""
        return len(self.nodes) * 3

    @property
    def stiffness_matrix(self) -> NDArray[np.float64]:
        """Assembles the global stiffness matrix."""
        k_matrix = np.zeros((self.dof_count, self.dof_count))

        for element in self.beams + self.cbush:
            k_global = element.global_stiffness_matrix
            indices = np.concatenate(
                (element.node_1.dof_indices, element.node_2.dof_indices)
            )
            k_matrix[np.ix_(indices, indices)] += k_global

        return k_matrix

    @property
    def internal_force_vector(self) -> NDArray[np.float64]:
        """Assemble internal force vector."""
        f_int = np.zeros(self.dof_count)
        for beam in self.beams:
            f_element = beam.internal_forces
            indices = np.concatenate((beam.node_1.dof_indices, beam.node_2.dof_indices))
            f_int[indices] += f_element
        return f_int

    def apply_loads(self) -> NDArray[np.float64]:
        """Assembles the global load vector."""
        load_vector = np.zeros(self.dof_count)

        for node in self.nodes:
            load_vector[node.dof_indices] = node.load_dof

        return load_vector

    @property
    def mpc_constraints(self) -> List[Tuple[int, List[int], NDArray[np.float64]]]:
        """Returns all MPC constraints in the system."""
        return [constraint for mpc in self.mpc for constraint in mpc.constraints]

    def apply_mpc_constraints(
        self, k_global: NDArray[np.float64], load_vector: NDArray[np.float64]
    ) -> Tuple[NDArray[np.int_], NDArray[np.float64], NDArray[np.float64]]:
        """Modifies stiffness matrix and load vector for MPC constraints."""
        free_dofs = np.arange(self.dof_count)

        for slave_dof, master_dofs, coeffs in self.mpc_constraints:
            free_dofs = free_dofs[free_dofs != slave_dof]
            k_global[master_dofs] += coeffs[:, np.newaxis] * k_global[slave_dof]
            k_global[:, master_dofs] += (
                coeffs[:, np.newaxis] * k_global[:, slave_dof]
            ).T
            load_vector[master_dofs] += coeffs * load_vector[slave_dof]

        return free_dofs, k_global, load_vector

    def solve_linear(self) -> NDArray[np.float64]:
        """
        Solves for nodal displacements considering all constraints.

        Returns:
            Array of displacements [u, v, θ] for each node

        Raises:
            ValueError: If the system has no nodes or is singular
        """
        if not self.nodes:
            raise ValueError("No nodes defined in the system")

        k_global = self.stiffness_matrix
        load_vector = self.apply_loads()

        # Apply boundary conditions
        bc_free_dofs = np.concatenate([node.free_dof for node in self.nodes])
        mpc_free_dofs, k_global, load_vector = self.apply_mpc_constraints(
            k_global, load_vector
        )
        free_dofs = np.intersect1d(bc_free_dofs, mpc_free_dofs)

        if free_dofs.size == 0:
            raise ValueError("System is fully constrained")

        # Solve reduced system
        k_reduced = k_global[np.ix_(free_dofs, free_dofs)]
        load_reduced = load_vector[free_dofs]

        try:
            disp_reduced = np.linalg.solve(k_reduced, load_reduced)
        except np.linalg.LinAlgError as e:
            raise ValueError("System is singular or ill-conditioned") from e

        # Reconstruct full displacement vector
        displacements = np.zeros(self.dof_count)
        displacements[free_dofs] = disp_reduced

        # Update node displacements
        for node in self.nodes:
            node.displ = displacements[node.dof_indices]

        for beam in self.beams:
            beam.calc_internal_forces()

        # Compute dependent DOFs from MPC
        for mpc in self.mpc:
            mpc.calc_slave_displacements()

        return displacements

    def solve_nonlinear(
        self,
        max_iterations: int = 100,
        analysis_tolerance: float = 1e-6,
        debug: bool = False,
    ) -> NDArray[np.float64]:
        """
        Non-linear solver using Newton-Raphson method.

        Args:
            max_iterations: Maximum number of iterations.
            analysis_tolerance: Convergence threshold for residual norm.
            debug: If True, prints intermediate results for debugging.

        Returns:
            Array of displacements [u, v, θ] for each node.

        Raises:
            ValueError: If the system is singular or does not converge.
        """

        if not self.nodes:
            raise ValueError("No nodes defined in the system.")

        external_loads = self.apply_loads()

        # Step 1: Initial Linear Solution
        displacements = self.solve_linear()

        bc_free_dofs = np.concatenate([node.free_dof for node in self.nodes])
        free_dofs = bc_free_dofs

        for iteration in range(max_iterations):
            # Step 2: Update elements (geometry and stiffness)
            for beam in self.beams:
                beam.update_properties()
                beam.calc_internal_forces()

            # Step 3: Assemble Tangent Stiffness Matrix & Internal Forces
            k_tangent = self.stiffness_matrix
            f_int = self.internal_force_vector

            # Apply boundary conditions
            k_red = k_tangent[np.ix_(free_dofs, free_dofs)]
            residual = external_loads.copy()
            residual[free_dofs] -= f_int[free_dofs]

            # Convergence Check
            residual_norm = np.linalg.norm(residual)

            if residual_norm < analysis_tolerance:
                if debug:
                    print(f"Converged in {iteration} iterations!")
                break

            # Step 4: Solve for displacement increment
            try:
                delta_disp = np.linalg.solve(k_red, residual[free_dofs])
            except np.linalg.LinAlgError:
                raise ValueError(
                    f"System is singular at iteration {iteration}. Check constraints & stiffness matrix."
                )

            if debug:
                print(f"Iteration {iteration}: Residual Norm = {residual_norm:.6e}")
                print(
                    f"np.linalg.norm(delta_disp/displacements[free_dofs]): {np.linalg.norm(delta_disp / displacements[free_dofs])}"
                )

            # Step 5: Update Displacements
            displacements[free_dofs] += delta_disp / 2

            # Step 6: Check for Divergence
            if np.linalg.norm(delta_disp) < 1e-10:
                raise ValueError(
                    f"Displacement change is too small at iteration {iteration}. Possible rigid body motion."
                )

            # Step 7: Update Node Displacements
            for node in self.nodes:
                node.displ = displacements[node.dof_indices]
        else:
            raise ValueError(
                f"Nonlinear solver did not converge after {max_iterations} iterations."
            )

        return displacements


# Example usage
fem = FEM()
