#!/bin/bash
CYLINDER_GENERATION_EXEC=$PWD"/build/cylinder_generation"

# Number of iterations per noise and parameter
ITERATIONS=1000

# Define considered cylinder heights
HEIGHTS="1.0,2.0,3.0,4.0,5.0"

# Define considered cylinder radii 
RADII="0.3"

# Define height sampling density
HEIGHT_SAMPLES=30

# Define angle sampling density
ANGLE_SAMPLES=30

## declare an array variable
declare -a arr=("occlusion" "outliers" "noise")

## now loop through the above array
for TEST_TYPE in "${arr[@]}"
do
	DATASET_DIR=$PWD"/dataset/cylinder_"$TEST_TYPE
	mkdir $DATASET_DIR
	# ROBUSTNESS TO OCCLUSION
	if   [ $TEST_TYPE == "occlusion" ] 
	then
	  NOISE_STD_DEVS="0.01"
	  OUTLIERS="0.00"
	  OCCLUSION_LEVELS="0.00,0.10,0.20,0.30,0.40,0.50,0.60,0.70,0.80,0.90"
	elif [ $TEST_TYPE == "outliers" ]
	# ROBUSTNESS TO OUTLIERS
	then
	  NOISE_STD_DEVS="0.01"
	  OUTLIERS="0.00,0.25,0.5,0.75,1.0,1.25,1.5,1.75,2.0,2.25,2.5,2.75,3.0,3.25,3.5"
	  OCCLUSION_LEVELS="0.00"
	# ROBUSTNESS TO NOISE
	elif  [ $TEST_TYPE == "noise" ]
	then
	  NOISE_STD_DEVS="0.00,0.10,0.20,0.30,0.40,0.50,0.60,0.70,0.80,0.90,1.00"
	  OUTLIERS="0.00"
	  OCCLUSION_LEVELS="0.00"
	fi
	# or do whatever with individual element of the array
        $CYLINDER_GENERATION_EXEC $DATASET_DIR $ITERATIONS $HEIGHTS $RADII $NOISE_STD_DEVS $OUTLIERS $OCCLUSION_LEVELS $HEIGHT_SAMPLES $ANGLE_SAMPLES
done


