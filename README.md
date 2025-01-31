
# GFOLD (Guidance for Fuel Optimal Large Diverts), 3D Powered Descent Guidance with Successive Convexification

This project implements a 3D powered descent guidance algorithm using successive convexification. It solves for an optimal trajectory that starts at a high altitude and lands at a lower altitude, considering an obstacle avoidance constraint.

## Project Structure

```
.
├── main.py
└── src
    ├── trajectory.py
    ├── solver.py
    └── visualization.py
```

- **main.py**  
  Entrypoint. Calls the solver (`gfold_3d_successive_convexification`) and plots the solution if one is found.

- **src/trajectory.py**  
  Contains the function `gfold_3d_successive_convexification` that performs successive convexification:
  1. Initializes a guess for the position, velocity, and mass profiles.  
  2. Iteratively calls the subproblem solver (`solve_subproblem` in `solver.py`).  
  3. Checks convergence by comparing the new solution to the previous one.  
  4. Returns the final solution if convergence is achieved (or if the iteration limit is reached).

- **src/solver.py**  
  Defines `solve_subproblem`, which sets up a single convex subproblem and solves it using CVXPY. Key points:
  - Enforces dynamics constraints through linearization around the previous trajectory.  
  - Constrains thrust magnitude, tilt, mass depletion, and obstacle avoidance.  
  - Minimizes total thrust usage (`sum(T_mag)`).  
  - Uses [SCS](https://www.cvxpy.org/tutorial/advanced/index.html#scs) as the solver by default.

- **src/visualization.py**  
  Contains `visualize_3d_solution`, which uses Matplotlib to produce a 3D plot of the trajectory. It also plots a wireframe sphere representing the obstacle, if provided.

## Getting Started

1. **Install Dependencies**  
   - Python 3.8+ recommended  
   - Required packages: `numpy`, `matplotlib`, `cvxpy`, `mpl_toolkits`  
     ```bash
     pip install numpy matplotlib cvxpy
     ```
   
2. **Run the Main Script**  
   From the project root, execute:
   ```bash
   python main.py
   ```
   - If a solution is found, it prints the approximate fuel usage and displays a 3D trajectory plot along with the obstacle.  
   - If no valid solution is found, it prints a message and exits.

## Key Features & Notes

- **Successive Convexification**:  
  The technique linearizes the non-convex parts around the previous iteration’s solution. After each iteration, it updates the reference trajectory and repeats until convergence or an iteration limit is reached.

- **Obstacle Avoidance**:  
  The algorithm includes a simple obstacle avoidance constraint by enforcing a half-space constraint around the obstacle for each time step, based on the previous trajectory’s relative distance to the obstacle.

- **Thrust Constraints**:  
  - Lower bound (`T_min`) prevents throttle from going too low.  
  - Upper bound (`T_max`) prevents excessive thrust.  
  - Tilt angle (`theta_max_deg`) ensures the thrust vector doesn’t tilt too far from the vertical.

- **Plotting**:  
  - Displays the start point in green, the end point in red, and the discrete points of the path in blue.  
  - The obstacle is represented by a translucent red sphere.

## License

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.


