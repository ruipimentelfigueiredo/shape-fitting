/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include "sphere_fitting_hough.h"

SphereFittingHough::SphereFittingHough(const OrientationAccumulatorSpace & gaussian_sphere_, unsigned int position_bins_, unsigned int radius_bins_,double min_radius_,double max_radius_, double accumulator_peak_threshold_, bool do_refine_, bool soft_voting_) : 
	SphereFitting(min_radius_,max_radius_),
	gaussian_sphere(new OrientationAccumulatorSpace(gaussian_sphere_)),
	radius_bins(radius_bins_),
	position_bins(position_bins_),
	r_step((max_radius_-min_radius_)/radius_bins),
	accumulator_peak_threshold(accumulator_peak_threshold_),
	soft_voting(soft_voting_),
	cloud_normals(new pcl::PointCloud<pcl::Normal>),
	tree(new pcl::search::KdTree<PointT> ())
{
	// Alocate memory for sphere accumulator
	sphere_accum.resize(radius_bins);
	radii.resize(radius_bins);
	for(unsigned int r_index=0;r_index<radius_bins;++r_index)
	{
		radii[r_index]=r_index*r_step+min_radius;
		sphere_accum[r_index].resize(position_bins);
		for(unsigned int x_index=0;x_index<position_bins;++x_index)
		{
			sphere_accum[r_index][x_index].resize(position_bins);
			for(unsigned int y_index=0;y_index<position_bins;++y_index)
			{
				sphere_accum[r_index][x_index][y_index].resize(position_bins);
			}
		}
	}

	ne.setKSearch (50);
	ne.setSearchMethod (tree);
};

FittingData SphereFittingHough::fit(const PointCloudT::ConstPtr & point_cloud_in_)
{
	// Reset Accumulator;
	for(unsigned int r_index=0;r_index<radius_bins;++r_index)
	{
		for(unsigned int x_index=0;x_index<position_bins;++x_index)
		{
			for(unsigned int y_index=0;y_index<position_bins;++y_index)
			{
				std::fill(sphere_accum[r_index][x_index][y_index].begin(),sphere_accum[r_index][x_index][y_index].end(), 0);
			}
		}
	}

	const std::vector<Eigen::Vector3d> & gaussian_sphere_voting=gaussian_sphere->getOrientationAccumulatorSpace();

	// Get position voting boundaries
	Eigen::Vector4f min_pt,max_pt;
	pcl::getMinMax3D(*point_cloud_in_,min_pt,max_pt);

	double x_position_step=(double)((2.0*max_radius) + (max_pt-min_pt)[0])/position_bins;
	double y_position_step=(double)((2.0*max_radius) + (max_pt-min_pt)[1])/position_bins;
	double z_position_step=(double)((2.0*max_radius) + (max_pt-min_pt)[2])/position_bins;

	double x_position_step_inv=1.0/x_position_step;
	double y_position_step_inv=1.0/y_position_step;
	double z_position_step_inv=1.0/z_position_step;
	
	double x_offset=min_pt[0] - max_radius;
	double y_offset=min_pt[1] - max_radius;
	double z_offset=min_pt[2] - max_radius;
	// Vote
	if(soft_voting)
	{
		ne.setInputCloud (point_cloud_in_);
		ne.compute (*cloud_normals);
		for(unsigned int s = 0; s < point_cloud_in_->size(); ++s) {
			if(cloud_normals->points[s].getNormalVector3fMap ().dot(point_cloud_in_->points[s].getVector3fMap ())<0)
			{
				cloud_normals->points[s].getNormalVector3fMap ()=-cloud_normals->points[s].getNormalVector3fMap ();
			}
		}
		for(unsigned int i=0; i<gaussian_sphere->gaussian_sphere_points_num; ++i)
		{
			double theta=atan2(gaussian_sphere_voting[i][1],gaussian_sphere_voting[i][0]);
			double phi=acos(gaussian_sphere_voting[i][2]);
		
			double cos_theta=cos(theta);
			double sin_theta=sin(theta);
			double cos_phi=cos(phi);
			double sin_phi=sin(phi);
			double cos_theta_sin_phi=cos_theta*sin_phi;
			double sin_theta_sin_phi=sin_theta*sin_phi;
			for(unsigned int s = 0; s < point_cloud_in_->size(); ++s)
			{
				/*if(Eigen::Vector3d(point_cloud_in_->points[s].x+cos_theta_sin_phi,point_cloud_in_->points[s].y+sin_theta_sin_phi,point_cloud_in_->points[s].z+cos_phi).normalized().dot(cloud_normals->points[s].getNormalVector3fMap ().cast<double>()) <0) 
					continue;*/

				for(unsigned int r_index=0; r_index<radius_bins; ++r_index)
				{
					double cx=(point_cloud_in_->points[s].x-radii[r_index]*cos_theta_sin_phi);
					double cy=(point_cloud_in_->points[s].y-radii[r_index]*sin_theta_sin_phi);
					double cz=(point_cloud_in_->points[s].z-radii[r_index]*cos_phi);

					unsigned int cx_index=floor( (cx - x_offset ) * x_position_step_inv);
					unsigned int cy_index=floor( (cy - y_offset ) * y_position_step_inv);
					unsigned int cz_index=floor( (cz - z_offset ) * z_position_step_inv);
					sphere_accum[r_index][cx_index][cy_index][cz_index]+=Eigen::Vector3d(point_cloud_in_->points[s].x-cx,point_cloud_in_->points[s].y-cy,point_cloud_in_->points[s].z-cz).normalized().dot(cloud_normals->points[s].getNormalVector3fMap ().cast<double>());
				}
			}
		}
	}
	else
	{
		for(unsigned int i=0; i<gaussian_sphere->gaussian_sphere_points_num; ++i)
		{
			double theta=atan2(gaussian_sphere_voting[i][1],gaussian_sphere_voting[i][0]);
			double phi=acos(gaussian_sphere_voting[i][2]);
		
			double cos_theta=cos(theta);
			double sin_theta=sin(theta);
			double cos_phi=cos(phi);
			double sin_phi=sin(phi);
			double cos_theta_sin_phi=cos_theta*sin_phi;
			double sin_theta_sin_phi=sin_theta*sin_phi;
			for(unsigned int r_index=0; r_index<radius_bins; ++r_index)
			{
				double aux_x=-radii[r_index]*cos_theta_sin_phi - x_offset;
				double aux_y=-radii[r_index]*sin_theta_sin_phi - y_offset;
				double aux_z=-radii[r_index]*cos_phi           - z_offset;
				for(unsigned int s = 0; s < point_cloud_in_->size(); ++s)
				{
					unsigned int cx=floor( (point_cloud_in_->points[s].x+aux_x ) * x_position_step_inv);
					unsigned int cy=floor( (point_cloud_in_->points[s].y+aux_y ) * y_position_step_inv);
					unsigned int cz=floor( (point_cloud_in_->points[s].z+aux_z ) * z_position_step_inv);

					++sphere_accum[r_index][cx][cy][cz];
				}
			}
		}
	}

	// Get best
	unsigned int best_r_bin=0, best_x_bin=0, best_y_bin=0, best_z_bin=0;
	double most_votes=0.0;
	for(unsigned int r_index=0;r_index<radius_bins;++r_index)
	{
		for(unsigned int x_index=0;x_index<position_bins;++x_index)
		{
			for(unsigned int y_index=0;y_index<position_bins;++y_index)
			{
				for(unsigned int z_index=0;z_index<position_bins;++z_index)
				{
					if(sphere_accum[r_index][x_index][y_index][z_index]>most_votes) 
					{
						best_r_bin=r_index;
						best_x_bin=x_index;
						best_y_bin=y_index;
						best_z_bin=z_index;
						most_votes=sphere_accum[r_index][x_index][y_index][z_index];
					}
				}
			}
		}
	}

	// Recover
	double best_r=best_r_bin*r_step+min_radius;
	double best_x=best_x_bin*x_position_step+x_offset;
	double best_y=best_y_bin*y_position_step+y_offset;
	double best_z=best_z_bin*z_position_step+z_offset;

	Eigen::VectorXf coeffs(4,1);
	coeffs <<  best_x, best_y, best_z, best_r;

	// Create the filtering object
	PointCloudT::Ptr cloud_projected(new PointCloudT);
	pcl::SampleConsensusModelSphere<PointT>::Ptr dit (new pcl::SampleConsensusModelSphere<PointT> (point_cloud_in_)); 

	std::vector<int> inliers; 
	dit -> selectWithinDistance (coeffs, 0.01, inliers); 
	pcl::copyPointCloud<PointT>(*point_cloud_in_, inliers, *cloud_projected); 
	double inlier_ratio_=((double)cloud_projected->size()/(double)point_cloud_in_->size());
	return FittingData(coeffs,inlier_ratio_,FittingData::SPHERE,cloud_projected);
};

