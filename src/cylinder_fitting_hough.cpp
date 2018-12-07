/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include "cylinder_fitting_hough.h"

CylinderFittingHough::CylinderFittingHough(const OrientationAccumulatorSpace & gaussian_sphere_, unsigned int angle_bins_, unsigned int radius_bins_, unsigned int position_bins_, double min_radius_, double max_radius_, double accumulator_peak_threshold_, unsigned int mode_, bool do_refine_, bool soft_voting_) : 
	CylinderFitting(min_radius_,max_radius_,do_refine_),
	gaussian_sphere(gaussian_sphere_),
	angle_bins(angle_bins_),
	angle_step(2*M_PI/angle_bins),
	radius_bins(radius_bins_),
	position_bins(position_bins_),
	r_step((max_radius-min_radius)/radius_bins),
	accumulator_peak_threshold(accumulator_peak_threshold_),
	mode(mode_),
	soft_voting(soft_voting_)
{
	// Alocate memory for direction accumulator
	cyl_direction_accum.resize(gaussian_sphere.gaussian_sphere_points_num);

	// Alocate memory for circle accumulator
	cyl_circ_accum.resize(position_bins);
	for(unsigned int i=0;i<position_bins;++i)
	{
		cyl_circ_accum[i].resize(position_bins);
		for(unsigned int j=0;j<position_bins;++j)
		{
			cyl_circ_accum[i][j].resize(radius_bins);
		}
	}

	cos_angle.resize(angle_bins);
	sin_angle.resize(angle_bins);
	for(unsigned int w=0; w<angle_bins;++w)
	{
		double current_angle=w*angle_step;
		cos_angle[w]=cos(current_angle);// TODO PRECOMPUTE TRIGONOMETRIC FUNCTION
		sin_angle[w]=sin(current_angle);// TODO PRECOMPUTE TRIGONOMETRIC FUNCTION
	}

  	//ne.setRadiusSearch (0.03);
	ne.setKSearch (6);
	ne.setSearchMethod (tree);

	// Use the same KdTree from the normal estimation
	principal_curvatures_estimation.setSearchMethod (tree);

	//principal_curvatures_estimation.setRadiusSearch (0.03);
	principal_curvatures_estimation.setKSearch (6);
};




Eigen::Vector3d CylinderFittingHough::findCylinderDirection(const NormalCloudT::ConstPtr & cloud_normals, const PointCloudT::ConstPtr & point_cloud_in_)
{
	double curvature_epsilon=0.000001;

	// Setup the principal curvatures computation

	// Provide the original point cloud (without normals)
	principal_curvatures_estimation.setInputCloud ( point_cloud_in_);

	// Provide the point cloud with normals
	principal_curvatures_estimation.setInputNormals (cloud_normals);

	// Compute the principal curvatures
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
	principal_curvatures_estimation.compute (*principal_curvatures);

	//3. for each point normal
	std::fill(cyl_direction_accum.begin(),cyl_direction_accum.end(), 0);

	const std::vector<Eigen::Vector3d> & gaussian_sphere_voting=gaussian_sphere.getOrientationAccumulatorSpace();

	/*double step=10.0*M_PI/180.0;
	unsigned int steps=floor(M_PI/step);
	unsigned int
	for(unsigned int s = 0; s < cloud_normals->size(); ++s)
	{

		Eigen::Matrix3d identity=Eigen::Matrix3d::Identity();
		Eigen::Vector3d b=(Eigen::Vector3d::UnitZ()-cloud_normals->points[s].getNormalVector3fMap().cast<double>());
		Eigen::Matrix3d rotation_matrix=identity-2*b*b.transpose();
		Eigen::Vector3d normal=cloud_normals->points[s].getNormalVector3fMap().cast<double>();
		for(unsigned int t_step = 0; t_step < steps; ++t_step)
		{
			double t=step*t_step;
			double x=cos(t);
			double y=sin(t);
			double z=0;
			Eigen::Vector3d rotated_vector=(rotation_matrix*cloud_normals->points[s].getNormalVector3fMap().cast<double>().normalized()).normalized();
			//std::cout << rotated_vector.norm() << " " << cloud_normals->points[s].getNormalVector3fMap().cast<double>().norm() << std::endl;
			double phi=atan2(y,x);
			double theta=acos(z);

			unsigned int phi_bin=(phi/M_PI))*sqrt(gaussian_sphere_voting.size());
			unsigned int theta_bin=(theta/(M_PI))*gaussian_sphere_voting.size()*0.5;
			std::cout << "phi bin:" << phi_bin << " " << "theta bin:" << theta_bin << std::endl;
		}
	}*/

        if(soft_voting)
	{
		for(unsigned int i=0; i<gaussian_sphere_voting.size(); ++i)
		{
			Eigen::Vector3d voting_direction=gaussian_sphere_voting[i];
			for(unsigned int s=0; s<cloud_normals->size(); ++s)
			{
				double normal_weight=(1.0-fabs(cloud_normals->points[s].getNormalVector3fMap().cast<double>().dot(voting_direction)));
				if(mode==NORMAL)
				{
					//1 - fabs(dot.product)
					cyl_direction_accum[i]+=normal_weight;
				}
				else if(mode==HYBRID)
				{
					double curvature_weight=(1.0-fabs(Eigen::Vector3d(principal_curvatures->points[s].principal_curvature[0],principal_curvatures->points[s].principal_curvature[1],principal_curvatures->points[s].principal_curvature[2]).dot(voting_direction)));
					double sum_curvatures=principal_curvatures->points[s].pc1+principal_curvatures->points[s].pc2;
					
					if(sum_curvatures>curvature_epsilon)
						cyl_direction_accum[i]+=normal_weight*curvature_weight*( (principal_curvatures->points[s].pc1-principal_curvatures->points[s].pc2) / sum_curvatures );
					else
						cyl_direction_accum[i]+=normal_weight;

					//cyl_direction_accum[i]+=normal_weight*curvature_weight*principal_curvatures->points[s].pc1;
				}
			}
		}
	}
	else
	{
		// Simplified implementation of Rabanni et al.
		double voting_thresh=M_PI/sqrt(gaussian_sphere_voting.size());
		for(unsigned int s = 0; s < cloud_normals->size(); ++s)
		{
			for(unsigned int i=0; i<gaussian_sphere_voting.size(); ++i)
			{
				Eigen::Vector3d voting_direction=gaussian_sphere_voting[i];
				double normal_weight=(fabs(cloud_normals->points[s].getNormalVector3fMap().cast<double>().dot(voting_direction)));
				if(acos(1.0-normal_weight)<voting_thresh)
				{
					cyl_direction_accum[i]+=1;	
				}
			}
		}
	}

	// Get best orientation
	double most_votes=0.0;
	unsigned int best_direction_index=0;
	for (unsigned int i=0; i<gaussian_sphere_voting.size(); ++i)
	{
		if(cyl_direction_accum[i]>most_votes)
		{
			best_direction_index=i;
			most_votes=cyl_direction_accum[i];
		}
	}

	/*
	// TODO: HERE WE CAN CLUSTER
	std::vector<Eigen::Vector3d> best_orientations;
	best_orientations.push_back(gaussian_sphere_voting[best_direction_index]);

	// Choose orientation whose votes are a percentage above a given threshold of the best orientation
	cyl_direction_accum[best_direction_index]=0; 	

	// This is more efficient than having an if condition to verify if we are considering the best pose again
	for (unsigned int i=0; i<gaussian_sphere_voting.size(); ++i)
	{
		if(cyl_direction_accum[i]>=accumulator_peak_threshold*most_votes)
		{
                        best_orientations.push_back(gaussian_sphere_voting[i]);
		}
	}

	*/


	return gaussian_sphere_voting[best_direction_index];
}

Eigen::Matrix<double,5,1> CylinderFittingHough::findCylinderPositionRadius(const PointCloudT::ConstPtr & point_cloud_in_)
{
	// Get position voting boundaries
	Eigen::Vector4f min_pt,max_pt;
	pcl::getMinMax3D(*point_cloud_in_,min_pt,max_pt);

	double u_position_step=(double)(max_pt-min_pt)[0]/position_bins;
	double v_position_step=(double)(max_pt-min_pt)[1]/position_bins;
	double u_position_step_inv=(double)1.0/u_position_step;
	double v_position_step_inv=(double)1.0/v_position_step;

	// Reset accumulator
	for (unsigned int u_index=0; u_index < position_bins; ++u_index) 
	{
		for (unsigned int v_index=0; v_index < position_bins; ++v_index)
		{
			std::fill(cyl_circ_accum[u_index][v_index].begin(),cyl_circ_accum[u_index][v_index].end(), 0);
		}
	}

 	// Vote
	for(unsigned int r=0; r<radius_bins;++r)
	{	
		double current_radius=r_step*r+min_radius;
		for(unsigned int w=0; w<angle_bins;++w)
		{
			for(PointCloudT::const_iterator it = point_cloud_in_->begin(); it != point_cloud_in_->end(); ++it)
			{
				double u=it->x - (double)min_pt[0];
				double v=it->y - (double)min_pt[1] ;
		
				// Get discretized coordinates
				unsigned int u_hough=floor( (current_radius*cos_angle[w]+u)*u_position_step_inv); 
				unsigned int v_hough=floor( (current_radius*sin_angle[w]+v)*v_position_step_inv); 


				if(u_hough>=position_bins)
					continue;//u_hough=position_bins-1;
				if(v_hough>=position_bins)
					continue;//v_hough=position_bins-1;

				++cyl_circ_accum[u_hough][v_hough][r];
			}
		}
	}

	// Get best
	unsigned int best_u_bin=0, best_v_bin=0, best_r_bin=0;
	double most_votes=0.0;
	for (unsigned int u_index=0; u_index < position_bins; ++u_index) 
	{
		for (unsigned int v_index=0; v_index < position_bins; ++v_index) 
		{
			for (unsigned int r_index=0; r_index < radius_bins; ++r_index)
			{
				if(cyl_circ_accum[u_index][v_index][r_index]>most_votes) 
				{
					best_u_bin=u_index;
					best_v_bin=v_index;
					best_r_bin=r_index;
					most_votes=cyl_circ_accum[u_index][v_index][r_index];
				}
			}
		}
	}

	// Recover
	double best_u=best_u_bin*u_position_step+(double)min_pt[0];
	double best_v=best_v_bin*v_position_step+(double)min_pt[1];
	double best_r=best_r_bin*r_step+min_radius;

	// Get u v in original frame
	Eigen::Matrix<double,5,1> result;
	result << best_u, best_v, min_pt[2], best_r, ((double)max_pt[2]-(double)min_pt[2]);
	return result;
}

FittingData CylinderFittingHough::fit(const PointCloudT::ConstPtr & point_cloud_in_)
{
	//1.  Estimate point normals
	ne.setInputCloud (point_cloud_in_);
	ne.compute (*cloud_normals);
    	Eigen::Vector3d cylinder_direction=findCylinderDirection(cloud_normals,point_cloud_in_);
	
	//Get rotation matrix
	Eigen::Matrix4d R2;
	R2=Eigen::Matrix4d::Identity();

   	Eigen::Vector3d up = Eigen::Vector3d::UnitZ();

	if(up.dot(cylinder_direction)<0)
	{
		cylinder_direction=-cylinder_direction;
	}

	Eigen::Vector3d rot_axis = cylinder_direction.cross(up);

	rot_axis.normalize();
	if(std::isnan(rot_axis[0])||std::isnan(rot_axis[1])||std::isnan(rot_axis[2]))
	{
		R2=Eigen::Matrix4d::Identity();
	}
	else
	{
		Eigen::Matrix3d aux;
		aux=Eigen::AngleAxisd(acos(cylinder_direction.dot(up)),rot_axis);
		R2.block(0,0,3,3)=aux;
	}

	// Extract the cylinder inliers from the input cloud
	double thresh_=fabs(cos(angle_step));
	pcl::PointIndices::Ptr  inliers_cylinder (new pcl::PointIndices);
	for (unsigned i=0; i < cloud_normals->points.size(); ++i) 
	{
		double dot_product=cloud_normals->points[i].getNormalVector3fMap ().cast<double>().dot(cylinder_direction);

		if(fabs(dot_product)<thresh_)
		{
			inliers_cylinder->indices.push_back(i);
		}
	}

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (point_cloud_in_);
	extract.setIndices (inliers_cylinder);
	extract.setNegative (false);
	extract.filter (*transformed_cloud);

	// Executing the transformation that aligns the cylinder rotation axis with z_up)
	pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, R2);

	Eigen::Matrix<double,5,1> position_and_radius=findCylinderPositionRadius(transformed_cloud);

	double radius=position_and_radius[3];
	double height=position_and_radius[4];
	// Convert back to original coordinates
	Eigen::Vector3d cylinder_position=R2.block(0,0,3,3).transpose()*position_and_radius.block(0,0,3,1);


	Eigen::VectorXf coeffs(7,1);
	coeffs << 
		cylinder_position[0]+0.5*height*cylinder_direction[0],
		cylinder_position[1]+0.5*height*cylinder_direction[1],
		cylinder_position[2]+0.5*height*cylinder_direction[2],
		cylinder_direction[0],
		cylinder_direction[1],
		cylinder_direction[2],
		radius;
		//height;

	// Create the filtering object
	PointCloudT::Ptr cloud_projected(new PointCloudT);
	pcl::SampleConsensusModelCylinder<PointT, NormalT>::Ptr dit (new pcl::SampleConsensusModelCylinder<PointT,NormalT> (point_cloud_in_)); 
    	dit->setInputNormals(cloud_normals); 

	std::vector<int> inliers; 
	dit -> selectWithinDistance (coeffs, 0.01, inliers); 
	pcl::copyPointCloud<PointT>(*point_cloud_in_, inliers, *cloud_projected); 

	double inlier_ratio_=((double)cloud_projected->size()/(double)point_cloud_in_->size());

	// Refine height
	//Eigen::Vector4d min_pt,max_pt;
	//pcl::getMinMax3D(*cloud_projected,min_pt,max_pt);
	//height=max_pt[2]-min_pt[2];

	// Redefine cylinder position (base);
	//Eigen::Vector4d refined_cylinder_position=R2.transpose()*Eigen::Vector4d(best_u,best_v,min_pt[2],0.0);
    	//coefficients_cylinder->values[0]=refined_cylinder_position[0];
    	//coefficients_cylinder->values[1]=refined_cylinder_position[1];
    	//coefficients_cylinder->values[2]=refined_cylinder_position[2];

	//std::cout << "height:" << height << std::endl;


	Eigen::VectorXf final_coeffs(8,1);
	final_coeffs << coeffs,
			height;

	FittingData cylinder_fitting(final_coeffs,inlier_ratio_,FittingData::CYLINDER,cloud_projected);

	return cylinder_fitting;
}

