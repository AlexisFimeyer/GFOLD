from src.trajectory import gfold_3d_successive_convexification
from src.visualization import visualize_3d_solution
import numpy as np

def main():
    # Solve the trajectory optimization problem
    sol = gfold_3d_successive_convexification()
    
    if sol is not None and sol["x"] is not None:
        print("Fuel usage (approx) =", np.sum(sol["T_mag"]))
        # Pass obstacle information to visualization
        visualize_3d_solution(
            sol, 
            obstacle_center=(10.0, 0.0, 50.0),
            obstacle_radius=5.0
        )
    else:
        print("No valid solution found.")

if __name__ == "__main__":
    main() 