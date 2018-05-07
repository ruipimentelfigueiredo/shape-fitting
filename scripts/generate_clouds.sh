#!/bin/bash
CYLINDER_GENERATION_EXEC="/home/rui/shape-fitting/build/cylinder_generation"

# Directory where dataset is stored
DATASET_DIR="/home/rui/shape-fitting/dataset/"

# Number of iterations per noise (for robustness against statistical outliers)
ITERATIONS=200

# Define considered cylinder heights
HEIGHTS="1.0,2.0,3.0,4.0,5.0,6.0"

# Define considered cylinder radii 
RADII="0.5"

# Define noise (std_dev) levels
NOISE_STD_DEVS="0.001,0.05,0.10,0.15,0.20,0.25,0.30,0.35,0.40,0.45,0.50"

#NOISE_STD_DEVS="0.001"

# Define outlier levels
OUTLIERS="0,0.5,1.0,1.5,2.0"

# Define height sampling density
HEIGHT_SAMPLES=30

# Define angle sampling density
ANGLE_SAMPLES=30


#mkdir $DATASET_DIR
$CYLINDER_GENERATION_EXEC $DATASET_DIR $ITERATIONS $HEIGHTS $RADII $NOISE_STD_DEVS $OUTLIERS $HEIGHT_SAMPLES $ANGLE_SAMPLES

