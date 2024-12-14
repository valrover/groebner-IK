# Inverse kinematics of a biped robot using Gröbner basis theory

This document briefly outlines the forward kinematics for the right and left legs of a biped robot, providing the background of the polynomial equations used in the inverse kinematics computations. The polynomial equations are then fed into an algorithm that computes Gröbner bases using the graded reverse lexicographic order (grevlex) for the monomial ordering.

### Homogeneous transformation matrices

Transformations between robot frames:

#### Right leg

```math
\mathbf{T}_{0}^{1R}(\theta_{1R}) = \begin{bmatrix}
    \sin(\theta_{1R})&0&\cos(\theta_{1R})&a_0+a_1\cos(\theta_{1R})\\
    -\cos(\theta_{1R})&0&\sin(\theta_{1R})&a_1\sin(\theta_{1R})\\
    0&-1&0&-d_1\\
    0&0&0&1
\end{bmatrix}
```

```math
\mathbf{T}_{1}^{2R}(\theta_{2R}) = \begin{bmatrix}
    \cos(\theta_{2R})&-\sin(\theta_{2R})&0&a_2\cos(\theta_{2R})\\
    \sin(\theta_{2R})&\cos(\theta_{2R})&0&a_2\sin(\theta_{2R})\\
    0&0&1&0\\
    0&0&0&1
\end{bmatrix}
```

```math
\mathbf{T}_{2}^{3R}(\theta_{3R}) = \begin{bmatrix}
    -\sin(\theta_{3R})&-\cos(\theta_{3R})&0&a_3\cos(\theta_{3R})\\
    \cos(\theta_{3R})&-\sin(\theta_{3R})&0&a_3\sin(\theta_{3R})\\
    0&0&1&0\\
    0&0&0&1
\end{bmatrix}
```

```math
\mathbf{T}_{3}^{4R}(\theta_{4R}) = \begin{bmatrix}
    \cos(\theta_{4R})&-\sin(\theta_{4R})&0&a_4\cos(\theta_{4R})\\
    \sin(\theta_{4R})&\cos(\theta_{4R})&0&a_4\sin(\theta_{4R})\\
    0&0&1&0\\
    0&0&0&1
\end{bmatrix}
```

#### Left leg

```math
\mathbf{T}_{0}^{1L}(\theta_{1L}) = \begin{bmatrix}
    \sin(\theta_{1L})&0&-\cos(\theta_{1L})&-a_0-a_1\cos(\theta_{1L})\\
    -\cos(\theta_{1L})&0&-\sin(\theta_{1L})&-a_1\sin(\theta_{1L})\\
    0&1&0&-d_1\\
    0&0&0&1
\end{bmatrix}
```

```math
\mathbf{T}_{1}^{2L}(\theta_{2L}) = \mathbf{T}_{1}^{2R}(\theta_{2L})
```

```math
\mathbf{T}_{2}^{3L}(\theta_{3L}) = \begin{bmatrix}
    \sin(\theta_{3L})&\cos(\theta_{3L})&0&a_3\cos(\theta_{3L})\\
    -\cos(\theta_{3L})&\sin(\theta_{3L})&0&a_3\sin(\theta_{3L})\\
    0&0&1&0\\
    0&0&0&1
\end{bmatrix}
```

```math
\mathbf{T}_{3}^{4L}(\theta_{4L}) = \mathbf{T}_{1}^{4R}(\theta_{4L})
```

### Position of the end effectors

The position of the end effector is represented by using the following trigonometric polynomials obtained by multiplying the previous transformation matrices and extracting the positions:

#### Right leg

```math
p_{xR} = a_0 + a_1\cos(\theta_{1R}) + [a_2\cos(\theta_{2R}) + a_3\cos(\theta_{2R}+\theta_{3R}) - a_4\sin(\phi_R)]\sin(\theta_{1R})
```

```math
p_{yR} = a_1\sin(\theta_{1R}) - [a_2\cos(\theta_{2R}) + a_3\cos(\theta_{2R}+\theta_{3R}) - a_4\sin(\phi_R)]\cos(\theta_{1R})
```

```math
p_{zR} = -d_1 - a_2\sin(\theta_{2R}) - a_3\sin(\theta_{2R}+\theta_{3R}) - a_4\cos(\phi_R)
```

#### Left leg

```math
p_{xL} = -a_0 - a_1\cos(\theta_{1L}) + [a_2\cos(\theta_{2L}) + a_3\cos(\theta_{2L}+\theta_{3L}) + a_4\sin(\phi_L)]\sin(\theta_{1L})
```

```math
p_{yL} = -a_1\sin(\theta_{1L}) - [a_2\cos(\theta_{2L}) + a_3\cos(\theta_{2L}+\theta_{3L}) + a_4\sin(\phi_L)]\cos(\theta_{1L})
```

```math
p_{zL} = -d_1 + a_2\sin(\theta_{2L}) + a_3\sin(\theta_{2L}+\theta_{3L}) - a_4\cos(\phi_L)
```

#### Additional Equations for computing the gröbner bases

From the previous polynomials the following relationships can be obtained by simple algebraic manipulations without computers. These additional polynomials speed up the computation of the gröbner bases, since they simplify the expressions by separating the first joint angles from the rest. This separation allows the algorithm to handle equations in smaller, decoupled sets, improving computational efficiency. This also makes sense because the first joint angles from each leg rotate about an axis that is not parallel to the rest of joint angles. The additional expressions are the following:

```math
a_2\cos(\theta_{2R}) + a_3\cos(\theta_{2R}+\theta_{3R}) - a_4\sin(\phi_R) = \sqrt{(p_{xR}-a_0)^2+p_{yR}^2-a_1^2}
```

```math
a_2\cos(\theta_{2L}) + a_3\cos(\theta_{2L}+\theta_{3L}) + a_4\sin(\phi_L) = \sqrt{(p_{xL}+a_0)^2+p_{yL}^2-a_1^2}
```

### Trigonometric polynomial system of equations

With the previous expressions from the forward kinematics it is possible to arrange the polynomials as a nonlinear system of equations, which are used a the input for the algorithms that compute the Gröbner bases. The system of equations is complemented with trigonometric identities to account for the trigonometric nature of the polynomials. Otherwise, the algorithm would treat them as ordinary polynomials.

#### Right leg

```math
\begin{split}
     I_{1R}=\Bigl\{&a_0 + a_1C_{1R} + \sqrt{(p_{xR}-a_0)^2+p_{yR}^2-a_1^2)}S_{1R} - p_{xR}=0, \\
     &a_1S_{1R} - \sqrt{(p_{xR}-a_0)^2+p_{yR}^2-a_1^2)}C_{1R} - p_{yR}=0\Bigr\}
\end{split}
```

```math
\begin{split}
    I_{2R}=\Bigl\{&\sqrt{(p_{xR}-a_0)^2+p_{yR}^2-a_1^2)} - a_2C_{2R} - a_3C_{23R} + a_4\sin(\phi_R)=0, \\
    &d_1 + a_2S_{2R} + a_3S_{23R} + a_4\cos(\phi_R) + p_{zR}=0, \\
    &S_{2R}^2 + C_{2R}^2 - 1=0, \\
    &S_{23R}^2 + C_{23R}^2 - 1=0\Bigr\}
\end{split}
```

#### Left leg

```math
\begin{split}
    I_{1L}=\Bigl\{&- a_0 - a_1C_{1L} + \sqrt{(p_{xL}+a_0)^2+p_{yL}^2-a_1^2)}S_{1L} - p_{xL}=0, \\
    &a_1S_{1L} + \sqrt{(p_{xL}+a_0)^2+p_{yL}^2-a_1^2)}C_{1L} + p_{yL}=0\Bigr\}
\end{split}
```

```math
\begin{split}
    I_{2L}=\Bigl\{&\sqrt{(p_{xL}+a_0)^2+p_{yL}^2-a_1^2)} - a_2C_{2L} - a_3C_{23L} - a_4\sin(\phi_L)=0, \\
    &- d_1 + a_2S_{2L} + a_3S_{23L} - a_4\cos(\phi_L) - p_{zL}=0, \\
    &S_{2L}^2 + C_{2L}^2 - 1=0, \\
    &S_{23L}^2 + C_{23L}^2 - 1=0\Bigr\}
\end{split}
```

## References

1. M. Sarajchi and K. Sirlantzis, “Pediatric Robotic Lower-Limb Exoskeleton: An Innovative design and Kinematic analysis,” IEEE Access, vol. 11, pp. 115219–115230, Jan. 2023, doi: 10.1109/access.2023.3325211.
2. A. Meurer et al., “SymPy: symbolic computing in Python,” PeerJ Computer Science, vol. 3, p. e103, Jan. 2017, doi: 10.7717/peerj-cs.103.
