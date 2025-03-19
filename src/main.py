import numpy as np

from classes.fem import fem

b = 0.5
t1 = 0.1
t2 = 0.2

plate_1_area = b * t1
plate_1_inertia = b * t1**3 / 12

plate_2_area = b * t2
plate_2_inertia = b * t2**3 / 12

bolt_dia = 0.156
bolt_area = np.pi * bolt_dia**2 / 4
bolt_inertia = np.pi * bolt_dia**4 / 64

aluminium_elastic_modulus = 10.3e6
aluminum_nu = 0.33
titanium_elastic_modulus = 16.9e6
titaniu_nu = 0.31

cbush_plate_1_coeff = (1 - aluminum_nu**2) / aluminium_elastic_modulus + (
    1 - titaniu_nu**2
) / titanium_elastic_modulus
cbush_axial_stiffness_plate_1 = t1 / cbush_plate_1_coeff
cbush_rotational_stiffness_plate_1 = t1**3 / 12 / cbush_plate_1_coeff

cbush_plate_2_coeff = (1 - aluminum_nu**2) / aluminium_elastic_modulus + (
    1 - titaniu_nu**2
) / titanium_elastic_modulus
cbush_axial_stiffness_plate_2 = t2 / cbush_plate_2_coeff
cbush_rotational_stiffness_plate_2 = t2**3 / 12 / cbush_plate_2_coeff


# nodes = [
#     # Plate 1
#     fem.add_node(x=0, y=0, load_dof=np.array([-1000, 0, 0])),
#     fem.add_node(x=1, y=0),
#     # Plate 2
#     fem.add_node(x=1, y=0.15),
#     fem.add_node(x=2, y=0.15, fixed_dof=np.array([0, 1, 2])),
#     # Plate 3
#     fem.add_node(x=1, y=-0.15),
#     fem.add_node(x=2, y=-0.15, fixed_dof=np.array([0, 1, 2])),
#     # Bolt
#     fem.add_node(x=1, y=-0.25),
#     fem.add_node(x=1, y=-0.15),
#     fem.add_node(x=1, y=0),
#     fem.add_node(x=1, y=0.15),
#     fem.add_node(x=1, y=0.25),
# ]

# # Plate 1
# fem.add_beam(
#     area=plate_1_area,
#     inertia=plate_1_inertia,
#     elastic_modulus=aluminium_elastic_modulus,
#     node_1=nodes[0],
#     node_2=nodes[1],
# )
# # Plate 2
# fem.add_beam(
#     area=plate_2_area,
#     inertia=plate_2_inertia,
#     elastic_modulus=aluminium_elastic_modulus,
#     node_1=nodes[2],
#     node_2=nodes[3],
# )
# # Plate 3
# fem.add_beam(
#     area=plate_2_area,
#     inertia=plate_2_inertia,
#     elastic_modulus=aluminium_elastic_modulus,
#     node_1=nodes[4],
#     node_2=nodes[5],
# )

# # Bolt
# fem.add_beam(
#     area=bolt_area,
#     inertia=bolt_inertia,
#     elastic_modulus=titanium_elastic_modulus,
#     node_1=nodes[6],
#     node_2=nodes[7],
# )
# fem.add_beam(
#     area=bolt_area,
#     inertia=bolt_inertia,
#     elastic_modulus=titanium_elastic_modulus,
#     node_1=nodes[7],
#     node_2=nodes[8],
# )
# fem.add_beam(
#     area=bolt_area,
#     inertia=bolt_inertia,
#     elastic_modulus=titanium_elastic_modulus,
#     node_1=nodes[8],
#     node_2=nodes[9],
# )
# fem.add_beam(
#     area=bolt_area,
#     inertia=bolt_inertia,
#     elastic_modulus=titanium_elastic_modulus,
#     node_1=nodes[9],
#     node_2=nodes[10],
# )

# # CBush Plate 1
# fem.add_cbush(
#     axial_stiffness=cbush_axial_stiffness_plate_1,
#     rotational_stiffness=cbush_rotational_stiffness_plate_1,
#     node_1=nodes[1],
#     node_2=nodes[8],
# )
# # CBush Plate 2
# fem.add_cbush(
#     axial_stiffness=cbush_axial_stiffness_plate_2,
#     rotational_stiffness=cbush_rotational_stiffness_plate_2,
#     node_1=nodes[2],
#     node_2=nodes[9],
# )
# fem.add_cbush(
#     axial_stiffness=cbush_axial_stiffness_plate_2,
#     rotational_stiffness=cbush_rotational_stiffness_plate_2,
#     node_1=nodes[4],
#     node_2=nodes[7],
# )

# # Add MPC constraints
# fem.add_mpc(master_node=nodes[6], slave_node=nodes[4], dofs=np.array([1, 2]))
# fem.add_mpc(master_node=nodes[6], slave_node=nodes[1], dofs=np.array([1, 2]))
# fem.add_mpc(master_node=nodes[6], slave_node=nodes[2], dofs=np.array([1, 2]))
# fem.add_mpc(master_node=nodes[6], slave_node=nodes[10], dofs=np.array([1, 2]))


nodes = [
    # Plate 1
    fem.add_node(x=0, y=0, load_dof=np.array([-1000, 0, 0])),
    fem.add_node(x=1, y=0),
    # Plate 2
    fem.add_node(x=1, y=0.15),
    fem.add_node(x=2, y=0.15, fixed_dof=np.array([0, 1, 2])),
    # Bolt
    fem.add_node(x=1, y=-0.05),
    fem.add_node(x=1, y=0),
    fem.add_node(x=1, y=0.15),
    fem.add_node(x=1, y=0.25),
]

# Plate 1
fem.add_beam(
    area=plate_1_area,
    inertia=plate_1_inertia,
    elastic_modulus=aluminium_elastic_modulus,
    node_1=nodes[0],
    node_2=nodes[1],
)
# Plate 2
fem.add_beam(
    area=plate_2_area,
    inertia=plate_2_inertia,
    elastic_modulus=aluminium_elastic_modulus,
    node_1=nodes[2],
    node_2=nodes[3],
)

# Bolt
fem.add_beam(
    area=bolt_area,
    inertia=bolt_inertia,
    elastic_modulus=titanium_elastic_modulus,
    node_1=nodes[4],
    node_2=nodes[5],
)
fem.add_beam(
    area=bolt_area,
    inertia=bolt_inertia,
    elastic_modulus=titanium_elastic_modulus,
    node_1=nodes[5],
    node_2=nodes[6],
)
fem.add_beam(
    area=bolt_area,
    inertia=bolt_inertia,
    elastic_modulus=titanium_elastic_modulus,
    node_1=nodes[6],
    node_2=nodes[7],
)

# CBush Plate 1
fem.add_cbush(
    axial_stiffness=cbush_axial_stiffness_plate_1,
    rotational_stiffness=cbush_rotational_stiffness_plate_1,
    node_1=nodes[1],
    node_2=nodes[5],
)
# CBush Plate 2
fem.add_cbush(
    axial_stiffness=cbush_axial_stiffness_plate_2,
    rotational_stiffness=cbush_rotational_stiffness_plate_2,
    node_1=nodes[2],
    node_2=nodes[6],
)

# Add MPC constraints
fem.add_mpc(master_node=nodes[4], slave_node=nodes[1], dofs=np.array([1, 2]))
fem.add_mpc(master_node=nodes[4], slave_node=nodes[2], dofs=np.array([1, 2]))
fem.add_mpc(master_node=nodes[4], slave_node=nodes[7], dofs=np.array([1, 2]))

displacement = fem.solve()

for node in nodes:
    print(f"Node {node.index} displacement: {node.displ}")

for beam in fem.beams:
    print(f"Beam forces {beam.internal_forces}")

for cbush in fem.cbush:
    print(f"CBush forces {cbush.internal_forces}")
