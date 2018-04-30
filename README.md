# shape fitting using PCL

## Installing
```mkdir build && cd build && cmake .. && make```

## Generating test dataset (ground truth + point clouds) 
1. Go inside ```$SHAPE_FITTING_DIR``` folder

2. Generate dataset (dataset/ground_truth dataset/point_clouds): ```bash scripts/generate_clouds.sh``` (change the folder paths and desired parameters inside fitting_tests.sh file)

## Generating test results 
1. Go inside ```$SHAPE_FITTING_DIR``` folder

2. Generate test results (dataset/results): ```bash scripts/fitting_tests.sh``` (change the folder paths and desired parameters inside fitting_tests.sh file)



