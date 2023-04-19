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

    @article{valimaki2023,
      author = {Välimäki, Tuomas and Garigipati, Bharath and Ghabcheloo, Reza},
      title = {Motion-Based Extrinsic Sensor-to-Sensor Calibration: Effect of Reference Frame Selection for New and Existing Methods},
      journal = {Sensors},
      volume = {23},
      year = {2023},
      number = {7},
      pages = {3740},
      url = {https://www.mdpi.com/1424-8220/23/7/3740},
      issn = {1424-8220},
      doi = {10.3390/s23073740}
    }

[trajectory_calibration]: https://github.com/tau-alma/trajectory_calibration
