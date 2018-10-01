#!/bin/bash
CYLINDER_GENERATION_EXEC=$PWD"/build/cylinder_generation"

# Directory where dataset is stored
DATASET_DIR=$PWD"/dataset/cylinder/"

# Number of iterations per noise and parameter
ITERATIONS=1000

# Define considered cylinder heights
#HEIGHTS="1.0,2.0,3.0,4.0,5.0"
HEIGHTS="0.3"
# Define considered cylinder radii 
RADII="0.05"

# Define noise (std_dev) levels
#NOISE_STD_DEVS="0.0,0.05,0.10,0.15,0.20,0.25,0.30,0.35,0.40,0.45,0.50,0.60,0.70,0.80,0.90,1.0"
NOISE_STD_DEVS="0.0,0.10,0.20,0.30,0.40,0.50,0.60,0.70,0.80,0.90,1.0"
#NOISE_STD_DEVS="0.05"

# Define outlier levels
OUTLIERS="0.0"
#,0.25,0.5,0.75,1.0,1.25,1.5,1.75,2.0"

#OUTLIERS="0.05"
# Define occlusion levels
OCCLUSION_LEVELS="0.0,0.10,0.20,0.30,0.40,0.50,0.60,0.70,0.80,0.90"

# Define height sampling density per meter
HEIGHT_SAMPLES=30

# Define angle sampling density  per meter
ANGLE_SAMPLES=30

#mkdir $DATASET_DIR
$CYLINDER_GENERATION_EXEC $DATASET_DIR $ITERATIONS $HEIGHTS $RADII $NOISE_STD_DEVS  $OUTLIERS $OCCLUSION_LEVELS $HEIGHT_SAMPLES $ANGLE_SAMPLES

