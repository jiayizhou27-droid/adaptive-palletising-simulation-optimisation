# Adaptive Palletising with Simulation Optimisation

This repository contains the MATLAB and Simulink implementation of an MSc project on mixed-case robotic palletising. The workflow connects constraint-aware pallet layout planning, finite-buffer execution planning, UR10e trajectory construction, and Simulation 3D verification.

- [MATLAB and Simulink Challenge Project Hub](https://github.com/mathworks/MATLAB-Simulink-Challenge-Project-Hub)
- [Adaptive Palletizing with Simulation Optimization project 254](https://github.com/mathworks/MATLAB-Simulink-Challenge-Project-Hub/tree/main/projects/Adaptive%20Palletizing%20with%20Simulation%20Optimization)
- [Project 254 submission form](https://www.mathworks.com/academia/student-challenge/mathworks-excellence-in-innovation-submission-form.html?tfa_1=Adaptive%20Palletizing%20with%20Simulation%20Optimization&tfa_2=254)

## What the project does

The implementation separates the problem into three stages:

1. Plan a mixed-case pallet layout using a deterministic Constructive method, Constructive-seeded Genetic Algorithm (CS-GA), or Constructive-seeded Simulated Annealing (CS-SA).
2. Convert the selected target layout into conveyor-to-pallet, conveyor-to-buffer, and buffer-to-pallet actions under a finite buffer capacity.
3. Construct top-down UR10e pick-and-place trajectories using inverse kinematics, trapezoidal joint trajectories, collision checking, and local Manipulator RRT fallback.

The repository is intended as a reproducible simulation demonstration. It is not a hardware safety certification or a validated industrial collision-clearance assessment.

## Main entry point

Run the small end-to-end demonstration from MATLAB:

```matlab
results = run_demo;
```

The default demonstration uses the Constructive layout method with 10 cartons and prepares both an execution schedule and robot trajectory. Alternative examples are:

```matlab
results = run_demo(Method="csga", NumCartons=10, MaxEvaluations=25);
results = run_demo(Method="cssa", NumCartons=10, MaxEvaluations=25);
results = run_demo(Method="csga", NumCartons=40, MaxEvaluations=120);
```

To open the Simulink model after preparing the workspace variables:

```matlab
results = run_demo(OpenModel=true);
```

The full Simulation 3D scene requires desktop MATLAB and the listed products. Runtime varies with the selected carton count, search budget, and trajectory configuration.

## Quick verification

Run the lightweight layout test without robot trajectory planning:

```matlab
results = runtests(fullfile("tests","smoke_test.m"));
assertSuccess(results);
```

Expected output: six cartons are placed, the envelope utilisation is positive, and all recorded target positions are finite.

## Repository contents

- `run_demo.m` - main configurable entry point.
- `PalletizeBoxesUsingCobot.slx` - Simulink and Simulation 3D workcell model.
- `PalletizeBoxesUsingCobotExample.mlx` - supporting Live Script demonstration.
- `buildLayoutPlanBatch3.m` - deterministic constructive physical layout decoder.
- `buildLayoutPlanCSGA.m` and `buildLayoutPlanCSSA.m` - seeded search methods.
- `buildBufferExecutionPlan.m` - finite-buffer execution scheduler.
- `planPalletizingTrajectory.m` - waypoint, inverse-kinematic, joint-trajectory, and collision-checking workflow.
- `palletTaskConfig.m` - common pallet, conveyor, buffer, and robot configuration.
- `data/sample/boxes.xlsx` - sample mixed-carton input data.
- `results/` - representative layout and validation figures.
- `experiments/` - archived scripts used for the broad layout, scale, type-complexity, buffer, and software fault-injection studies. This folder contains its own versions of several helper functions; run it in an isolated MATLAB session rather than adding both implementations recursively to the same path.
- `tests/smoke_test.m` - fast MATLAB verification test.

## Requirements

The project was developed with MATLAB R2025b-compatible files and uses:

- MATLAB
- Simulink
- Stateflow
- Robotics System Toolbox
- Navigation Toolbox functionality used by `manipulatorRRT`
- Simulink 3D Animation and the Simulation 3D interface

The optimisation implementations in this repository are custom MATLAB code and do not call the Global Optimization Toolbox `ga` or `simulannealbnd` solvers.

## Input data

The sample spreadsheet must contain the columns:

- `L` - carton length in metres
- `W` - carton width in metres
- `H` - carton height in metres
- `Mass` - carton mass
- `Type` - optional carton-type label

All paths used by the main entry point are relative to the repository root.

## Results summary

The accompanying dissertation evaluated three linked evidence levels:

- 150 paired layout instances, producing 450 complete method-case records.
- 2,400 finite-buffer execution conditions across multiple arrival patterns and capacities.
- A separate fixed 40-carton robot-integration case for each layout method, plus controlled software fault-injection scenarios.

Across the broad layout comparison, all three methods produced complete layouts under the implemented checks. Under the deliberately small broad-study search budget, the seeded searches provided occasional rather than uniform improvements over the Constructive baseline. The buffer study showed that execution feasibility and temporary-storage demand depend strongly on the relationship between arrival order and the target placement sequence.

Representative figures are provided in `results/`.

The aggregated CSV records used for the dissertation evidence chain are provided in `experiments/data/`. To rebuild the summary tables and figures from those completed experiment records:

```matlab
originalFolder = pwd;
cleanup = onCleanup(@() cd(originalFolder));
cd experiments
outputs = buildThesisEvidenceChain;
```

## Important scope notes

- The public implementation preserves explicit carton identities and a fixed offline target layout during execution scheduling.
- Geometric support and centre-of-mass checks are quasi-static proxies; carton deformation, friction, compression, and transport vibration are outside the model.
- Robot results are simulation evidence and do not replace hardware commissioning, safety-rated controls, or formal collision certification.
- Simulink build folders, caches, backups, and large intermediate result files are excluded from version control.

## Acknowledgements and provenance

This work was completed at Imperial College London in collaboration with MathWorks. The robot workcell was developed from the MathWorks `Palletize Boxes Using a Cobot` example and extended with mixed-case layout planning, finite-buffer scheduling, optimiser comparisons, trajectory validation, and supervisory verification. Files retaining MathWorks copyright notices remain attributed to The MathWorks, Inc.

## Author

Jiayi Zhou, MSc Control and Optimisation, Imperial College London.
