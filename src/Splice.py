import numpy as np

from classes.fem import fem

b = 0.7853
t1 = 0.082
t2 = 0.05
t3 = 0.06

plate_1_area = b * t1
plate_1_inertia = b * t1**3 / 12

plate_2_area = b * t2
plate_2_inertia = b * t2**3 / 12

plate_3_area = b * t3
plate_3_inertia = b * t3**3 / 12

bolt_dia = 0.157
bolt_area = np.pi * bolt_dia**2 / 4
bolt_inertia = np.pi * bolt_dia**4 / 64

aluminium_elastic_modulus = 10.3e6
aluminum_nu = 0.33

cbush_aluminum_coeff = (1 - aluminum_nu**2) / aluminium_elastic_modulus + (
    1 - aluminum_nu**2
) / aluminium_elastic_modulus
cbush_axial_stiffness_plate_1 = t1 / cbush_aluminum_coeff
cbush_rotational_stiffness_plate_1 = t1**3 / 12 / cbush_aluminum_coeff

cbush_axial_stiffness_plate_2 = t2 / cbush_aluminum_coeff
cbush_rotational_stiffness_plate_2 = t2**3 / 12 / cbush_aluminum_coeff

cbush_axial_stiffness_plate_3 = t3 / cbush_aluminum_coeff
cbush_rotational_stiffness_plate_3 = t3**3 / 12 / cbush_aluminum_coeff

nodes = [
    # Plate 1
    fem.add_node(x=-5, y=0, load_dof=np.array([-318.3, 0, 0])),  # 0
    fem.add_node(x=0, y=0),  # 1
    fem.add_node(x=0.7823, y=0),  # 2
    fem.add_node(x=1.559, y=0),  # 3
    # Plate 2
    fem.add_node(x=0, y=(t1 + t2) / 2),  # 4
    fem.add_node(x=0.7823, y=(t1 + t2) / 2),  # 5
    fem.add_node(x=1.559, y=(t1 + t2) / 2),  # 6
    fem.add_node(x=2.0776, y=(t1 + t2) / 2, fixed_dof=np.array([0, 1, 2])),  # 7
    # Plate 3
    fem.add_node(x=0.7823, y=t2 + (t1 + t3) / 2),  # 8
    fem.add_node(x=1.559, y=t2 + (t1 + t3) / 2),  # 9
    fem.add_node(x=2.0776, y=t2 + (t1 + t3) / 2, fixed_dof=np.array([0, 1, 2])),  # 10
    # Bolt #1
    fem.add_node(x=0, y=-t1 / 2),  # 11
    fem.add_node(x=0, y=0),  # 12
    fem.add_node(x=0, y=(t1 + t2) / 2),  # 13
    fem.add_node(x=0, y=t1 / 2 + t2),  # 14
    # Bolt #2
    fem.add_node(x=0.7823, y=-t1 / 2),  # 15
    fem.add_node(x=0.7823, y=0),  # 16
    fem.add_node(x=0.7823, y=(t1 + t2) / 2),  # 17
    fem.add_node(x=0.7823, y=t2 + (t1 + t3) / 2),  # 18
    fem.add_node(x=0.7823, y=t2 + t3 + t1 / 2),  # 19
    # Bolt #3
    fem.add_node(x=1.559, y=-t1 / 2),  # 20
    fem.add_node(x=1.559, y=0),  # 21
    fem.add_node(x=1.559, y=(t1 + t2) / 2),  # 22
    fem.add_node(x=1.559, y=t2 + (t1 + t3) / 2),  # 23
    fem.add_node(x=1.559, y=t2 + t3 + t1 / 2),  # 24
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
    fem.add_beam(
        area=plate_1_area,
        inertia=plate_1_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[1],
        node_2=nodes[2],
    ),
    fem.add_beam(
        area=plate_1_area,
        inertia=plate_1_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[2],
        node_2=nodes[3],
    ),
]

# Plate 2
plate2_beam = [
    fem.add_beam(
        area=plate_2_area,
        inertia=plate_2_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[4],
        node_2=nodes[5],
    ),
    fem.add_beam(
        area=plate_2_area,
        inertia=plate_2_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[5],
        node_2=nodes[6],
    ),
    fem.add_beam(
        area=plate_2_area,
        inertia=plate_2_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[6],
        node_2=nodes[7],
    ),
]

# Plate 3
plate3_beam = [
    fem.add_beam(
        area=plate_3_area,
        inertia=plate_3_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[8],
        node_2=nodes[9],
    ),
    fem.add_beam(
        area=plate_3_area,
        inertia=plate_3_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[9],
        node_2=nodes[10],
    ),
]

# Bolt #1
bolt1_beam = [
    fem.add_beam(
        area=bolt_area,
        inertia=bolt_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[11],
        node_2=nodes[12],
    ),
    fem.add_beam(
        area=bolt_area,
        inertia=bolt_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[12],
        node_2=nodes[13],
    ),
    fem.add_beam(
        area=bolt_area,
        inertia=bolt_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[13],
        node_2=nodes[14],
    ),
]
bolt1_cbush = [
    fem.add_cbush(
        axial_stiffness=cbush_axial_stiffness_plate_1,
        rotational_stiffness=cbush_rotational_stiffness_plate_1,
        node_1=nodes[1],
        node_2=nodes[12],
    ),
    fem.add_cbush(
        axial_stiffness=cbush_axial_stiffness_plate_2,
        rotational_stiffness=cbush_rotational_stiffness_plate_2,
        node_1=nodes[4],
        node_2=nodes[13],
    ),
]
bolt1_mpc = [
    fem.add_mpc(master_node=nodes[11], slave_node=nodes[1], dofs=np.array([1, 2])),
    fem.add_mpc(master_node=nodes[11], slave_node=nodes[4], dofs=np.array([1, 2])),
    fem.add_mpc(master_node=nodes[11], slave_node=nodes[14], dofs=np.array([1, 2])),
]

# Bolt #2
bolt2_beam = [
    fem.add_beam(
        area=bolt_area,
        inertia=bolt_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[15],
        node_2=nodes[16],
    ),
    fem.add_beam(
        area=bolt_area,
        inertia=bolt_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[16],
        node_2=nodes[17],
    ),
    fem.add_beam(
        area=bolt_area,
        inertia=bolt_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[17],
        node_2=nodes[18],
    ),
    fem.add_beam(
        area=bolt_area,
        inertia=bolt_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[18],
        node_2=nodes[19],
    ),
]
bolt2_cbush = [
    fem.add_cbush(
        axial_stiffness=cbush_axial_stiffness_plate_1,
        rotational_stiffness=cbush_rotational_stiffness_plate_1,
        node_1=nodes[2],
        node_2=nodes[16],
    ),
    fem.add_cbush(
        axial_stiffness=cbush_axial_stiffness_plate_2,
        rotational_stiffness=cbush_rotational_stiffness_plate_2,
        node_1=nodes[5],
        node_2=nodes[17],
    ),
    fem.add_cbush(
        axial_stiffness=cbush_axial_stiffness_plate_3,
        rotational_stiffness=cbush_rotational_stiffness_plate_3,
        node_1=nodes[8],
        node_2=nodes[18],
    ),
]
bolt2_mpc = [
    fem.add_mpc(master_node=nodes[15], slave_node=nodes[2], dofs=np.array([1, 2])),
    fem.add_mpc(master_node=nodes[15], slave_node=nodes[5], dofs=np.array([1, 2])),
    fem.add_mpc(master_node=nodes[15], slave_node=nodes[8], dofs=np.array([1, 2])),
    fem.add_mpc(master_node=nodes[15], slave_node=nodes[19], dofs=np.array([1, 2])),
]

# Bolt #3
bolt3_beam = [
    fem.add_beam(
        area=bolt_area,
        inertia=bolt_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[20],
        node_2=nodes[21],
    ),
    fem.add_beam(
        area=bolt_area,
        inertia=bolt_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[21],
        node_2=nodes[22],
    ),
    fem.add_beam(
        area=bolt_area,
        inertia=bolt_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[22],
        node_2=nodes[23],
    ),
    fem.add_beam(
        area=bolt_area,
        inertia=bolt_inertia,
        elastic_modulus=aluminium_elastic_modulus,
        node_1=nodes[23],
        node_2=nodes[24],
    ),
]
bolt3_cbush = [
    fem.add_cbush(
        axial_stiffness=cbush_axial_stiffness_plate_1,
        rotational_stiffness=cbush_rotational_stiffness_plate_1,
        node_1=nodes[3],
        node_2=nodes[21],
    ),
    fem.add_cbush(
        axial_stiffness=cbush_axial_stiffness_plate_2,
        rotational_stiffness=cbush_rotational_stiffness_plate_2,
        node_1=nodes[6],
        node_2=nodes[22],
    ),
    fem.add_cbush(
        axial_stiffness=cbush_axial_stiffness_plate_3,
        rotational_stiffness=cbush_rotational_stiffness_plate_3,
        node_1=nodes[9],
        node_2=nodes[23],
    ),
]
bolt3_mpc = [
    fem.add_mpc(master_node=nodes[20], slave_node=nodes[3], dofs=np.array([1, 2])),
    fem.add_mpc(master_node=nodes[20], slave_node=nodes[6], dofs=np.array([1, 2])),
    fem.add_mpc(master_node=nodes[20], slave_node=nodes[9], dofs=np.array([1, 2])),
    fem.add_mpc(master_node=nodes[20], slave_node=nodes[24], dofs=np.array([1, 2])),
]

displacement = fem.solve()

for node in nodes:
    print(f"Node {node.index} displacement: {node.displ}")

# for beam in fem.beams:
#     print(f"Beam forces {beam.internal_forces}")

print("Bolt 1 CBush")
for cbush in bolt1_cbush:
    print(f"CBush forces {cbush.internal_forces}")

print("Bolt 2 CBush")
for cbush in bolt2_cbush:
    print(f"CBush forces {cbush.internal_forces}")

print("Bolt 3 CBush")
for cbush in bolt3_cbush:
    print(f"CBush forces {cbush.internal_forces}")
