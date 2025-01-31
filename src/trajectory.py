import numpy as np
from .solver import solve_subproblem
import cvxpy as cp

def gfold_3d_successive_convexification(
    x0=0.0, y0=0.0, z0=100.0,
    vx0=0.0, vy0=0.0, vz0=0.0,
    m0=2000.0,
    xf=20.0, yf=0.0, zf=20.0,
    vxf=0.0, vyf=0.0, vzf=-2.0,
    mf=1500.0,
    N=50,
    dt=0.25,
    g=9.81,
    alpha=0.001,
    T_min=100.0,
    T_max=10000.0,
    theta_max_deg=180.0,
    obstacle_center=(10.0, 0.0, 50.0),
    obstacle_radius=5.0,
    max_iterations=50
):
    """
    Perform successive convexification to solve a 3D powered descent problem.
    """
    # Prepare an initial guess for the trajectory
    x_guess = np.linspace(x0, xf, N+1)
    y_guess = np.linspace(y0, yf, N+1)
    z_guess = np.linspace(z0, zf, N+1)
    vx_guess = np.gradient(x_guess, dt)
    vy_guess = np.gradient(y_guess, dt)
    vz_guess = np.gradient(z_guess, dt)
    m_guess  = np.linspace(m0, mf, N+1)

    # Initialize thrust values (simple vertical thrust guess)
    Tx_guess = np.zeros(N)
    Ty_guess = np.zeros(N)
    # Vertical thrust to approximately counter gravity
    Tz_guess = np.ones(N) * m_guess[:-1] * g  
    
    prev_solution = {
        "x": x_guess,
        "y": y_guess,
        "z": z_guess,
        "vx": vx_guess,
        "vy": vy_guess,
        "vz": vz_guess,
        "m": m_guess,
        "Tx": Tx_guess,
        "Ty": Ty_guess,
        "Tz": Tz_guess
    }

    solution = None

    for i in range(max_iterations):
        # Solve the convex subproblem around the last solution
        sol_i = solve_subproblem(
            x_init=x0, y_init=y0, z_init=z0,
            vx_init=vx0, vy_init=vy0, vz_init=vz0,
            m_init=m0,
            x_final=xf, y_final=yf, z_final=zf,
            vx_final=vxf, vy_final=vyf, vz_final=vzf,
            m_final=mf,
            N=N,
            dt=dt,
            g=g,
            alpha=alpha,
            T_min=T_min,
            T_max=T_max,
            theta_max_deg=theta_max_deg,
            obstacle_center=obstacle_center,
            obstacle_radius=obstacle_radius,
            prev_solution=prev_solution,
            solver=cp.SCS
        )
        
        # Check if solve failed
        if sol_i["x"] is None:
            print("Iteration", i, "failed to converge. Stopping.")
            break

        # Check how different the new solution is from the old one
        dx = np.linalg.norm(sol_i["x"] - prev_solution["x"]) + \
             np.linalg.norm(sol_i["y"] - prev_solution["y"]) + \
             np.linalg.norm(sol_i["z"] - prev_solution["z"])
        if dx < 1e-3:
            print("Converged at iteration =", i)
            solution = sol_i
            break

        # Otherwise, update the guess
        prev_solution = sol_i
        solution = sol_i

    return solution 