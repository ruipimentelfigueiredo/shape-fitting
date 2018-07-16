#!/bin/bash
SPHERE_GENERATION_EXEC="/home/rui/ws/src/shape_detection_fitting/lib/shape-fitting/build/sphere_generation"

# Directory where dataset is stored
DATASET_DIR="/home/rui/ws/src/shape_detection_fitting/lib/shape-fitting/dataset/sphere/"

# Number of iterations per noise and parameter
ITERATIONS=500

# Define considered sphere radii 
RADII="0.5"

# Define noise (std_dev) levels
NOISE_STD_DEVS="0.0,0.05,0.10,0.15,0.20,0.25,0.30,0.35,0.40,0.45,0.50"
#NOISE_STD_DEVS="0.0"
#NOISE_STD_DEVS="0.001"
#NOISE_STD_DEVS="0.001"


# Define outlier levels
OUTLIERS="0"

#OUTLIERS="0"

# Define occlusion levels
#OCCLUSION_LEVELS="0.0,0.2,0.4,0.6,0.8"
OCCLUSION_LEVELS="0.0"


# Define angle sampling density
ANGLE_SAMPLES=30


#mkdir $DATASET_DIR
$SPHERE_GENERATION_EXEC $DATASET_DIR $ITERATIONS $RADII $NOISE_STD_DEVS $OUTLIERS $OCCLUSION_LEVELS $ANGLE_SAMPLES

