# Trajectory calibration experiments

Data and experiments for motion-based extrinsic calibration using [trajectory_calibration].

The repository is organised as follows:

```
├── calib
│   └── *.csv  # The output calibration values
├── kitti_tests
│   └── ...    # The trajectories generated from KITTI data
├── matlab
│   └── ...    # Matlab files to generate simulation data
├── simulation_tests
│   └── ...    # The generated simulation data
├── *.py       # Scripts to run the calibration experiments
└── README.md
```

If you use the data in an academic context, please cite:

    @misc{https://doi.org/10.48550/arxiv.2303.03129,
      author = {Välimäki, Tuomas and Garigipati, Bharath and Ghabcheloo, Reza},
      title = {Motion-based extrinsic sensor-to-sensor calibration:
               Effect of reference frame selection for new and existing methods},
      publisher = {arXiv},
      year = {2023},
      doi = {10.48550/ARXIV.2303.03129},
      url = {https://arxiv.org/abs/2303.03129}
    }

[trajectory_calibration]: https://github.com/tau-alma/trajectory_calibration
