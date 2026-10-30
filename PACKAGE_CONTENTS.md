# Package contents

This public package was assembled from the final MSc project workspace. It intentionally excludes thesis drafts, CV files, temporary MATLAB/Simulink build products, editor backups, compiled MEX files, and superseded project copies.

## Robot integration implementation

The repository root contains the restored 40-carton robot-integration implementation:

- Constructive, CS-GA, and CS-SA layout entry functions
- finite-buffer action generation
- UR10e waypoint, inverse-kinematic, trapezoidal trajectory, collision-checking, and Manipulator RRT code
- Simulink and Simulation 3D model
- coordinate conversion, spawning, validation, and plotting helpers
- sample spreadsheet data

## Broad dissertation experiments

The `experiments` folder contains the archived code and completed aggregate records used for:

- paired layout-method comparisons
- order-size and carton-type studies
- arrival-pattern and buffer-capacity experiments
- offline optimiser comparison and budget calibration
- controlled software placement-fault verification
- generation of the thesis evidence-chain tables and figures

Several functions have both a robot-integration version at the repository root and an experimental version under `experiments`. They are kept in separate folders to preserve the exact workflows. Do not add both folders recursively to the MATLAB path in the same session.

## Verification status

- The package has been screened for hard-coded personal paths and common credential patterns.
- Generated caches and build directories are excluded by `.gitignore`.
- The completed aggregate experiment CSV files are included for reproducibility.
- A lightweight MATLAB test is provided under `tests`.
- Automated MATLAB execution was attempted during packaging but did not complete within the available headless-session time limit; reviewers should run the smoke test in an installed MATLAB desktop or licensed batch environment.
