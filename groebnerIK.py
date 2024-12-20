import sympy as sp

# Geometric parameters for the legs (constants for a real robot, but slow computation if symbolic)
a0 = 0
a1 = 0.4
a2 = 0.5
a3 = 0.5
a4 = 0.2
d1 = 0.2

# Define symbols for the leg
px, py, pz, q1, q2, q3, q4, s1, c1, s2, c2, s23, c23, phi = sp.symbols(
    'px py pz q1 q2 q3 q4 s1 c1 s2 c2 s23 c23 phi'
)

# Polynomials (Forward Kinematics)
H = sp.sqrt((px + a0)**2 + py**2 - a1**2) - a2 * c2 - a3 * c23 - a4 * sp.sin(phi)
poly1 = -a0 - a1 * c1 + sp.sqrt((px + a0)**2 + py**2 - a1**2) * s1 - px
poly2 = a1 * s1 + sp.sqrt((px + a0)**2 + py**2 - a1**2) * c1 + py
poly3 = -d1 + a2 * s2 + a3 * s23 - a4 * sp.cos(phi) - pz

# Trigonometric identities
id2 = s2**2 + c2**2 - 1
id23 = s23**2 + c23**2 - 1

# Gröbner bases
# Basis 1: Variables [s1, c1]
polys1 = [poly1, poly2]
vars1 = [s1, c1]
gbase1 = sp.groebner(polys1, *vars1, order='grevlex')

# Basis 2: Variables [c2, s2, c23, s23]
polys2 = [H, poly3, id2, id23]
vars2 = [s2, c2, s23, c23]
gbase2 = sp.groebner(polys2, *vars2, order='grevlex')

# Simplify Gröbner bases
gbase1_simplified = [sp.simplify(poly) for poly in gbase1]
gbase2_simplified = [sp.simplify(poly) for poly in gbase2]

# Symbolic solutions for q1:

# Solve for s1 from the first polynomial in gbase1_simplified
s1_solution = sp.solve(gbase1_simplified[0], s1)

# Compute q1 using asin (arcsin) of s1 solutions
q1 = sp.asin(s1_solution[0])

# Symbolic solutions for q2 and q3:

# Solve for s23 from the first polynomial in gbase2_simplified
c23_solutions = sp.solve(gbase2_simplified[0], c23)

# Compute q2 and q3 based on c23_solutions[0]
s2_solutions_1 = sp.solve(gbase2_simplified[1].subs(c23, c23_solutions[0]), s2)
q2_1 = sp.asin(s2_solutions_1[0])
q23_1 = sp.acos(c23_solutions[0])
q3_1 = q23_1 - q2_1
q4_1 = -q3_1 - q2_1 + phi

# Compute q2 and q3 based on c23_solutions[1]
s2_solutions_2 = sp.solve(gbase2_simplified[1].subs(c23, c23_solutions[1]), s2)
q2_2 = sp.asin(s2_solutions_1[0])
q23_2 = sp.acos(c23_solutions[1])
q3_2 = q23_2 - q2_2
q4_2 = -q3_2 - q2_2 + phi