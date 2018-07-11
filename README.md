# shape fitting using PCL

## Installing
```mkdir build && cd build && cmake .. && make```

## Generating test dataset (ground truth + point clouds) 
1. Go inside ```$SHAPE_FITTING_DIR``` folder

2. Generate dataset (dataset/ground_truth dataset/point_clouds): ```bash scripts/generate_clouds.sh``` (change the folder paths and desired parameters inside fitting_tests.sh file)

## Generating test results 
1. Go inside ```$SHAPE_FITTING_DIR``` folder

2. Generate test results (dataset/results): ```bash scripts/fitting_tests.sh``` (change the folder paths and desired parameters inside fitting_tests.sh file)


### Reference

In case you use our library in your research, please cite our work

```
@inproceedings{figueiredo2017robustCylinderFitting,
  title={Robust cylinder detection and pose estimation using 3D point cloud information},
  author={Figueiredo, Rui and Moreno, Plinio and Bernardino, Alexandre},
  booktitle={Autonomous Robot Systems and Competitions (ICARSC), 2017 IEEE International Conference on},
  pages={234--239},
  year={2017},
  organization={IEEE}
}

```
[paper]: http://vislab.isr.ist.utl.pt/wp-content/uploads/2017/09/rfigueiredo-icdlepirob2017.pdf

