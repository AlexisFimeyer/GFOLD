import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def visualize_3d_solution(solution_dict, obstacle_center=None, obstacle_radius=None):
    """
    Rudimentary 3D plot of the trajectory and the obstacle.
    """
    x = solution_dict["x"]
    y = solution_dict["y"]
    z = solution_dict["z"]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(x, y, z, 'bo-', label="Trajectory")
    ax.scatter(x[0], y[0], z[0], color='green', s=50, label="Start")
    ax.scatter(x[-1], y[-1], z[-1], color='red', s=50, label="End")

    # Add obstacle visualization 
    if obstacle_center is not None and obstacle_radius is not None:
        # Create a wireframe sphere for the obstacle
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        ox, oy, oz = obstacle_center
        x_obs = ox + obstacle_radius * np.outer(np.cos(u), np.sin(v))
        y_obs = oy + obstacle_radius * np.outer(np.sin(u), np.sin(v))
        z_obs = oz + obstacle_radius * np.outer(np.ones(np.size(u)), np.cos(v))
        ax.plot_surface(x_obs, y_obs, z_obs, alpha=0.2, color='red')

    # Add fuel usage to title if available
    if "T_mag" in solution_dict:
        fuel_usage = np.sum(solution_dict["T_mag"])
        plt.title(f"3D Trajectory (Total Fuel Usage: {fuel_usage:.1f} N⋅s)")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    plt.show() 