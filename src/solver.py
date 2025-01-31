import cvxpy as cp
import numpy as np

def solve_subproblem(
    x_init, y_init, z_init,
    vx_init, vy_init, vz_init,
    m_init,
    x_final, y_final, z_final,
    vx_final, vy_final, vz_final,
    m_final,
    N,
    dt,
    g,
    alpha,
    T_min,
    T_max,
    theta_max_deg,
    obstacle_center,
    obstacle_radius,
    prev_solution=None,
    weight_soft_constraints=1e5,
    solver=cp.SCS
):
    """
    Solve one convex subproblem for the 3D powered descent guidance.
    """
    # Decision Variables
    x = cp.Variable(N+1)
    y = cp.Variable(N+1)
    z = cp.Variable(N+1)
    vx = cp.Variable(N+1)
    vy = cp.Variable(N+1)
    vz = cp.Variable(N+1)
    m  = cp.Variable(N+1)

    # Control (thrust) at each step: (Tx, Ty, Tz)
    Tx = cp.Variable(N)
    Ty = cp.Variable(N)
    Tz = cp.Variable(N)

    # Keep an auxiliary variable for thrust magnitude
    T_mag = cp.Variable(N, nonneg=True)

    # Constraints collection
    constraints = []

    # Initial conditions
    constraints += [
        x[0] == x_init,
        y[0] == y_init,
        z[0] == z_init,
        vx[0] == vx_init,
        vy[0] == vy_init,
        vz[0] == vz_init,
        m[0] == m_init
    ]

    # Final conditions
    constraints += [
        x[N] == x_final,
        y[N] == y_final,
        z[N] == z_final,
        vx[N] == vx_final,
        vy[N] == vy_final,
        vz[N] == vz_final,
        m[N] == m_final
    ]

    # Thrust tilt constraint
    theta_max = np.deg2rad(theta_max_deg)
    cos_th = np.cos(theta_max)
    
    # Get previous iteration's mass for linearization
    if prev_solution is not None:
        m_prev = prev_solution["m"]
    else:
        m_prev = np.linspace(m_init, m_final, N+1)

    # Build step-wise constraints
    for k in range(N):
        # 1) Thrust magnitude bound
        constraints += [cp.SOC(T_max, cp.vstack([Tx[k], Ty[k], Tz[k]]))]  # Upper bound
        constraints += [T_mag[k] >= T_min]  # Lower bound
        constraints += [cp.SOC(T_mag[k], cp.vstack([Tx[k], Ty[k], Tz[k]]))]  # T_mag definition

        # 2) Thrust tilt constraint
        constraints += [Tz[k] >= T_mag[k] * cos_th]

        # 3) Dynamics with linearized 1/m terms
        if prev_solution is not None:
            Tx_prev = prev_solution["Tx"][k]
            Ty_prev = prev_solution["Ty"][k]
            Tz_prev = prev_solution["Tz"][k]
        else:
            Tx_prev = 0.0
            Ty_prev = 0.0
            Tz_prev = T_min

        # Linearized thrust/mass terms
        thrust_x_over_m = Tx[k]/m_prev[k] - (Tx_prev/(m_prev[k]**2))*(m[k] - m_prev[k])
        thrust_y_over_m = Ty[k]/m_prev[k] - (Ty_prev/(m_prev[k]**2))*(m[k] - m_prev[k])
        thrust_z_over_m = Tz[k]/m_prev[k] - (Tz_prev/(m_prev[k]**2))*(m[k] - m_prev[k])

        # Updated dynamics constraints
        constraints += [
            x[k+1] == x[k] + vx[k]*dt + 0.5*thrust_x_over_m*(dt**2),
            y[k+1] == y[k] + vy[k]*dt + 0.5*thrust_y_over_m*(dt**2),
            z[k+1] == z[k] + vz[k]*dt + 0.5*(thrust_z_over_m - g)*(dt**2),
            vx[k+1] == vx[k] + thrust_x_over_m*dt,
            vy[k+1] == vy[k] + thrust_y_over_m*dt,
            vz[k+1] == vz[k] + (thrust_z_over_m - g)*dt
        ]

        # 4) Mass depletion
        constraints += [m[k+1] == m[k] - alpha * T_mag[k] * dt]

        # 5) Non-negativity of mass
        constraints += [m[k+1] >= 0.0]

        # 6) Stay above ground
        constraints += [z[k] >= 0.0]

        # 7) Obstacle avoidance
        if prev_solution is not None:
            xk_prev = prev_solution["x"][k]
            yk_prev = prev_solution["y"][k]
            zk_prev = prev_solution["z"][k]

            (ox, oy, oz) = obstacle_center
            vx_p = xk_prev - ox
            vy_p = yk_prev - oy
            vz_p = zk_prev - oz
            dist_prev = np.sqrt(vx_p**2 + vy_p**2 + vz_p**2)

            if dist_prev > obstacle_radius + 1e-3:
                lhs = (vx_p)*(x[k] - ox) + (vy_p)*(y[k] - oy) + (vz_p)*(z[k] - oz)
                rhs = obstacle_radius * dist_prev
                constraints += [lhs >= rhs]

    # Solver settings for SCS
    solver_opts = {
        'max_iters': 10000,
        'eps': 1e-3
    }

    # Objective
    obj = cp.Minimize(cp.sum(T_mag))

    # Solve the problem
    prob = cp.Problem(obj, constraints)
    prob.solve(solver=solver, verbose=True, **solver_opts)

    if prob.status not in ["optimal", "optimal_inaccurate"]:
        print("Subproblem not solved optimally. Status =", prob.status)

    # Build solution dictionary
    sol = {
        "x": x.value,
        "y": y.value,
        "z": z.value,
        "vx": vx.value,
        "vy": vy.value,
        "vz": vz.value,
        "m": m.value,
        "Tx": Tx.value,
        "Ty": Ty.value,
        "Tz": Tz.value,
        "T_mag": T_mag.value,
        "objective": prob.value
    }
    return sol 