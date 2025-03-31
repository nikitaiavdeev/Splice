import numpy as np

from classes.fem import fem

b = 0.7853
t1 = 0.082

plate_1_area = b * t1
plate_1_inertia = b * t1**3 / 12

aluminium_elastic_modulus = 10.3e6
aluminum_nu = 0.33


nodes = [
    # Plate 1
    fem.add_node(x=0, y=0, fixed_dof=np.array([0, 1, 2])),  # 0
    fem.add_node(x=1, y=0, load_dof=np.array([-100, -100, 0])),  # 1
]

# Plate 1
plate1_beam = [
    fem.add_beam(
        area=plate_1_area,
        inertia=plate_1_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[0],
        node_2=nodes[1],
    ),
]

displacement = fem.solve_linear()

for node in nodes:
    print(f"Node {node.index} displacement: {node.displ}")

for beam in plate1_beam:
    beam.calc_internal_forces()
    print(f"Beam forces {beam.internal_forces}")


displacement = fem.solve_nonlinear(500, 1e-4, True)

for node in nodes:
    print(f"Node {node.index} displacement: {node.displ}")
