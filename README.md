# Spatiotemporal Tubes for Differential Drive Robots with Model Uncertainty

This repository provides the implementation for synthesizing spatiotemporal tubes (STTs) and designing controllers to satisfy Temporal Reach-Avoid-Stay specifications for differential drive mobile robots. This has been presented in the above mentioned paper.

It includes two case studies.

- The folder "Office Environment" showcases the case where the robot is moving in an office environment with static obstacles, having multiple reach-avoid tasks.
- The folder "Dynamic Obstacle" showcases the case where the robot is moving in an environment with static and dynamic obstacles present.

In addition, we have compared our approach with existing approaches, which are available in the "Comparison" folder.

## Repository Structure 

### OFFICE ENVIRONMENT

- `test.py`: Generates STTs for the robot moving inside the office environment. (Python file)
- `coefficients_Final.csv`: Coefficients of the STT generated for the office environment. (CSV file)
- `test_office.m`: Synthesizes the closed-form controller and simulates the robot trajectory using the generated tube in the office environment. (MATLAB File)
- `arenaPlot.m`: Utility Script to plot the office environment. (MATLAB file)
- `augmentPolygon.m`: Utility script for augmenting Polygons inside the plot. (MATLAB file) 

### DYNAMIC OBSTACLE
- `dynamic_obs.py`: Generates STTs for the robot moving inside the dynamic obstacle environment. (Python file)
- `dynamic_tube.xlsx`: Coefficients of the STT generated for the dynamic environment. (Excel file)
- `dynamic_obs.m`: Synthesizes the closed-form controller and simulates the robot trajectory using the generated tube in the dynamic unsafe set environment. (MATLAB File)
- `traj_t=2.fig`, `traj_t=3.fig`, `traj_t=5.fig`: Visualization of the trajectories and the corresponding tube at different timesteps.

### COMPARISON
- `comparison.py`: Generates STT for the robot for comparing with other methods (Python file)
- `comparison.xlsx`: Coefficients of the STT generated for the comparison environment (Excel file)
- `comparison_new.m`: Generate and compare the trajectories using the STT-based closed-form controller with conventional CBF and MPC-based controller in the presence and absence of disturbance. (MATLAB File)
- `diff_traj_Comparison.fig`, `diff_traj_Comparison.eps`: Visualizes the trajectories of the robot obtained under different control policies.

## Requirements

- Python 3.9+ with `z3-solver` installed (`pip install z3-solver`)
- MATLAB R2024B or later (for running `.m` files)

## How to Run

1. **Generate STTs**:
   - Run the `.py` files in any of the folders in Python to generate STTs.

2. **Run Controller and Simulation**:
   - Open MATLAB.
   - Run `.m` files for the robot examples.
   - This will simulate the system within the tube and visualize the results.

## Citation

If you use this code in your research, please cite the following paper:
```bibtex
@article{das2025spatiotemporal,
  title={Spatiotemporal Tubes for Differential Drive Robots with Model Uncertainty},
  author={Das, Ratnangshu and Basu, Ahan and Verginis, Christos and Jagtap, Pushpak},
  journal={arXiv preprint arXiv:2512.05495},
  year={2025}
}
