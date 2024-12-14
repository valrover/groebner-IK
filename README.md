# Inverse kinematics of a biped robot using Gröbner basis theory

This document briefly outlines the forward kinematics of the leg of a biped robot and provides the background on the polynomial equations used in the inverse kinematics computations. These polynomial equations are then processed by an algorithm that computes Gröbner bases using the graded reverse lexicographic order (grevlex) for monomial ordering. The inverse kinematics is solved based on the forward kinematics of the left leg, but the solutions can be reused for the right leg by accounting for the offset in the input due to symmetry.

### Homogeneous transformation matrices

Transformations between robot frames:

```math
\mathbf{T}_{0}^{1}(\theta_{1}) = \begin{bmatrix}
    \sin(\theta_{1})&0&-\cos(\theta_{1})&-a_0-a_1\cos(\theta_{1})\\
    -\cos(\theta_{1})&0&-\sin(\theta_{1})&-a_1\sin(\theta_{1})\\
    0&1&0&-d_1\\
    0&0&0&1
\end{bmatrix}
```

```math
\mathbf{T}_{1}^{2}(\theta_{2}) = \begin{bmatrix}
    \cos(\theta_{2})&-\sin(\theta_{2})&0&a_2\cos(\theta_{2})\\
    \sin(\theta_{2})&\cos(\theta_{2})&0&a_2\sin(\theta_{2})\\
    0&0&1&0\\
    0&0&0&1
\end{bmatrix}
```

```math
\mathbf{T}_{2}^{3}(\theta_{3}) = \begin{bmatrix}
    \sin(\theta_{3})&\cos(\theta_{3})&0&a_3\cos(\theta_{3})\\
    -\cos(\theta_{3})&\sin(\theta_{3})&0&a_3\sin(\theta_{3})\\
    0&0&1&0\\
    0&0&0&1
\end{bmatrix}
```

```math
\mathbf{T}_{3}^{4}(\theta_{4}) = \begin{bmatrix}
    \cos(\theta_{4})&-\sin(\theta_{4})&0&a_4\cos(\theta_{4})\\
    \sin(\theta_{4})&\cos(\theta_{4})&0&a_4\sin(\theta_{4})\\
    0&0&1&0\\
    0&0&0&1
\end{bmatrix}
```

### Position of the end effectors

The position of the end effector is represented by using the following trigonometric polynomials obtained by multiplying the previous transformation matrices and extracting the positions:

```math
p_{x} = -a_0 - a_1\cos(\theta_{1}) + [a_2\cos(\theta_{2}) + a_3\cos(\theta_{2}+\theta_{3}) + a_4\sin(\phi)]\sin(\theta_{1})
```

```math
p_{y} = -a_1\sin(\theta_{1}) - [a_2\cos(\theta_{2}) + a_3\cos(\theta_{2}+\theta_{3}) + a_4\sin(\phi)]\cos(\theta_{1})
```

```math
p_{z} = -d_1 + a_2\sin(\theta_{2}) + a_3\sin(\theta_{2}+\theta_{3}) - a_4\cos(\phi)
```

#### Additional Equations for computing the Gröbner bases

From the previous polynomials the following relationship can be obtained by simple algebraic manipulations without computers. This additional expression speeds up the computation of the gröbner bases, since it simplifies the polynomials by separating the first joint angles from the rest. This separation allows the algorithm to handle equations in smaller, decoupled sets, improving computational efficiency. This also makes sense because the first joint angles from each leg rotates about an axis that is not parallel to the rest of joint angles. The additional expression is the following:

```math
a_2\cos(\theta_{2}) + a_3\cos(\theta_{2}+\theta_{3}) + a_4\sin(\phi) = \sqrt{(p_{x}+a_0)^2+p_{y}^2-a_1^2}
```

### Trigonometric polynomial system of equations

With the previous expressions from the forward kinematics it is possible to arrange the polynomials as a nonlinear system of equations, which are used as the input for the algorithms that compute the Gröbner bases. The system of equations is complemented with trigonometric identities to account for the trigonometric nature of the polynomials. Otherwise, the algorithm would treat them as ordinary polynomials.

```math
\begin{split}
    I_{1}=\Bigl\{&- a_0 - a_1C_{1} + \sqrt{(p_{x}+a_0)^2+p_{y}^2-a_1^2)}S_{1} - p_{x}=0, \\
    &a_1S_{1} + \sqrt{(p_{x}+a_0)^2+p_{y}^2-a_1^2)}C_{1} + p_{y}=0\Bigr\}
\end{split}
```

```math
\begin{split}
    I_{2}=\Bigl\{&\sqrt{(p_{x}+a_0)^2+p_{y}^2-a_1^2)} - a_2C_{2} - a_3C_{23} - a_4\sin(\phi)=0, \\
    &- d_1 + a_2S_{2} + a_3S_{23} - a_4\cos(\phi) - p_{z}=0, \\
    &S_{2}^2 + C_{2}^2 - 1=0, \\
    &S_{23}^2 + C_{23}^2 - 1=0\Bigr\}
\end{split}
```

## References

1. M. Sarajchi and K. Sirlantzis, “Pediatric Robotic Lower-Limb Exoskeleton: An Innovative design and Kinematic analysis,” IEEE Access, vol. 11, pp. 115219–115230, Jan. 2023, doi: 10.1109/access.2023.3325211.
2. A. Meurer et al., “SymPy: symbolic computing in Python,” PeerJ Computer Science, vol. 3, p. e103, Jan. 2017, doi: 10.7717/peerj-cs.103.
