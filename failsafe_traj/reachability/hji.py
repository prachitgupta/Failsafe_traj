import jax
import jax.numpy as jnp
from jax import grad, jit, vmap

# Define the Hamiltonian function
def hamiltonian(x, u, grad_v, hess_v):
    # Define your system dynamics and cost here
    # For example, consider a simple 2D system with quadratic cost
    Q = jnp.array([[1.0, 0.0], [0.0, 1.0]])
    R = 0.1
    dynamics = jnp.array([x[0] - u[0], x[1] - u[1]])  # Example dynamics
    cost = jnp.dot(dynamics.T @ Q @ dynamics) + R * jnp.dot(u.T, u)
    return cost

# Define the HJI PDE
def hji_pde(t, x, grad_v, hess_v):
    u_space = jnp.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])  # Control input space
    min_hamiltonian = jnp.inf
    for u in u_space:
        ham = hamiltonian(x, u, grad_v, hess_v)
        if ham < min_hamiltonian:
            min_hamiltonian = ham
    return -min_hamiltonian

# Define the initial condition and time grid
x0 = jnp.array([0.0, 0.0])  # Initial state
t_grid = jnp.linspace(0.0, 1.0, 101)  # Time grid

# Compute backward reachable sets using JAX
grad_v = grad(lambda x: 0.0)  # Initial gradient
hess_v = grad(grad(lambda x: 0.0))  # Initial Hessian

for t in t_grid[::-1]:
    grad_v = grad(hji_pde, (1, 2))(t, x0, grad_v, hess_v)
    hess_v = grad(grad(hji_pde, (1, 2)), (1, 2))(t, x0, grad_v, hess_v)

# The value function V is now stored in grad_v
