/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	//Time segmentation process
	auto startTime = std::chrono::steady_clock::now();
	
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO:DONE Fill in this function
	// For max iterations 
	// Randomly sample subset and fit line	
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers
	
	while(maxIterations--){
		
		// randomly pick two points
		std::unordered_set<int> inliers;
		while (inliers.size() < 2 )
			inliers.insert(rand()%(cloud->points.size()));
		
		float x1, y1, x2, y2;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		// line calculation coefficients
		float A = y1-y2;
		float B = x2-x1;
		float C = x1*y2-x2*y1;

		for(int index = 0;index < cloud->points.size();index++){

			//skip if the point already in the set
			if(inliers.count(index)>0)
				continue;

			pcl::PointXYZ point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;

			// calc distance from the line
			float d = fabs(A*x3+B*y3+C)/sqrt(A*A+B*B);
			
			// add to set if distance is smaller than threshold
			if (d <= distanceTol)
				inliers.insert(index);
		}

		//check which set contains more points
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;

	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);
	std::cout<< "Randsac took " << elapsedTime.count() <<" milliseconds" << std::endl;

	return inliersResult;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	//Time segmentation process
	auto startTime = std::chrono::steady_clock::now();
	
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO:DONE Fill in this function
	// For max iterations 
	// Randomly sample subset and fit line	
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers
	
	while(maxIterations--){
		
		// randomly pick three points
		std::unordered_set<int> inliers;
		while (inliers.size() < 3 )
			inliers.insert(rand()%(cloud->points.size()));
		
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		float v1x, v1y, v1z, v2x, v2y, v2z;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		v1x = x2-x1;
		v1y = y2-y1;
		v1z = z2-z1;
		v2x = x3-x1;
		v2y = y3-y1;
		v2z = z3-z1;

		float i = (v1y*v2z)-(v1z*v2y);
		float j = (v1z*v2x)-(v1x*v2z);
		float k = (v1x*v2y)-(v1y*v2x);

		// plane calculation coefficients
		float A = i;
		float B = j;
		float C = k;
		float D = -(i*x1+j*y1+k*z1);

		for(int index = 0;index < cloud->points.size();index++){

			//skip if the point already in the set
			if(inliers.count(index)>0)
				continue;

			pcl::PointXYZ point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			// calc distance from the line
			float d = fabs(A*x4+B*y4+C*z4+D)/sqrt(A*A+B*B+C*C);
			
			// add to set if distance is smaller than threshold
			if (d <= distanceTol)
				inliers.insert(index);
		}

		//check which set contains more points
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;

	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);
	std::cout<< "Randsac took " << elapsedTime.count() <<" milliseconds" << std::endl;

	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO:DONE Change the max iteration and distance tolerance arguments for Ransac function
	// 2D RANSAC
	//std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);
	// 3D RANSAC
	std::unordered_set<int> inliers = Ransac3D(cloud, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
