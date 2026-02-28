# Active Defence for Robotic Manipulators

This repository accompanies the paper: 

> **"From Passive Monitoring to Active Defense: Resilient Control of Manipulators Under Cyberattacks"**
> *Gabriele Gualandi and Alessandro V. Papadopoulos*

accepted at IEEE International Conference on Robotics and Automation (ICRA), 2026.

The final publication DOI will be added once available (see file: NOTICE).

This code implements the active defense framework and generates the simulation results presented in the paper.

## Citation
If you use this code in academic work, please cite the associated paper.

### BibTeX
```bibtex
% Citation placeholder (Updated when published)
@inproceedings{Gualandi2026ActiveDefense,
  title     = {From Passive Monitoring to Active Defense: Resilient Control of Manipulators Under Cyberattacks},
  author    = {Gualandi, Gabriele and Papadopoulos, Alessandro V.},
  booktitle = {Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)},
  year      = {2026}
}
```

## 0. Prerequisites — Gurobi Solver
Gurobi must be installed and visible in the MATLAB path for the execution of the Active Defence and GreedyPD attacker algorithms. The code has been tested with **MATLAB 2025b** and **Gurobi 12.03**. 

Follow these steps to ensure Gurobi is set up correctly:

1. Install Gurobi and MATLAB interface.
2. Add Gurobi to your MATLAB path (e.g., `addpath('/Library/gurobi1203/macos_universal2/matlab')`).

If you wish to use a different solver, the optimization problem is formulated in standard form. You must modify the following function to integrate your solver:
   `core/+optim/+gurobi/solveQCQP.m`

> [NOTE]
> As of 2026, Gurobi offers free licenses for academic use.
> Researchers and students can obtain these licenses at
> [https://www.gurobi.com/academia/academic-program-and-licenses/](https://www.gurobi.com/academia/academic-program-and-licenses/).

## 1. Workflow & Configuration
In this streamlined project structure, the configurations for the simulations are stored directly as `.mat` files, rather than configuring them dynamically.

* The simulation structures and their parameters are serialized inside the `.mat` files located under the `params/` folder.
* The descriptions for the parameters are documented within the `par_info` field loaded from these `.mat` files.
* Robot dynamics and kinematics matrices are pre-rendered and exported into the `genFunctions/` folder to massively speed up simulations.

## 2. Running the Pipeline
The script `main.m` orchestrates the entire pipeline: it loops over all the simulations defined in `params/`, runs the physics kernel (`coreSimulator`), computes post-processing metrics, exports the results structurally, and finally triggers the plot generation module.

1. **Add a new simulation:** Drop the pre-generated `<simID>.mat` into `params/ICRA26/`
2. **Run one simulation:** Execute `main_singleSimulation.m`
3. **Run all & plot results:** Execute `main.m`

## 3. Folder Map
```text
.
├── main.m                     ← batch execution and plotting script
├── main_singleSimulation.m    ← single execution script
├── ICRA26plotResults.m        ← data visualization 
├── params/                    ← holding parameters (single source of truth)
├── genFunctions/              ← pre-rendered symbolic robot functions
├── results/                   ← generated output .mat logs
├── core/                      ← simulation engine
├── main_pipeline/             ← orchestration (main_2_runSimulation, etc.)
├── main_utils/                ← auxiliary tools (auto_runAllSimulations, etc.)
├── utils/                     ← general MATLAB helper libraries
└── gglib/                     ← other MATLAB helper libraries
```

## License
This repository is released under the **Apache License 2.0**. See the `LICENSE` file for details.

## Disclaimer
This code is provided for research and educational purposes and is released as-is, without warranties or guarantees of any kind. 
Users are responsible for ensuring that any use complies with applicable laws, regulations, institutional policies, and ethical standards.
