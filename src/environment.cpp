/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false; //true - draws the cars and road
    bool rendrer_obst = false;
    bool rendrer_plane = true;
    bool rendrer_cluster = true;
    bool rendrer_box = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:DONE Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //debug: draw laser rays
    //renderRays(viewer, lidar->position, inputCloud);
    // debug: draw only pcd
    //renderPointCloud(viewer, inputCloud, "inputCloud");

    //segmentation
    // TODO:DONE Create point processor
    // create point processor on the stack
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    // create point processor on the heap
    // ProcessPointClouds<pcl::PointXYZ> * pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, 0.2);
    // draw the obstacle(first) and the plane(second) points 
    if (rendrer_obst)
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    if (rendrer_plane)
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(1, 1, 1));

    // clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);
    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), Color(1,1,0), Color(0,0,1) };

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if(rendrer_cluster)
        { 
            std::cout << "cluster size ";
            // write the cluster size (number of the points)
            pointProcessor.numPoints(cluster);
            //pointProcessor->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);
        }
        if(rendrer_box)
        { 
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // create point processor on the heap
    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    
    // create point processor on the stack
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI.loadPcd("../../../src/sensors/data/pcd/data_1/0000000000.pcd"); //the executable file is in out/build/x64-Debug folder
    
    //draw original pointcloud
    renderPointCloud(viewer, inputCloud, "inputCloud");

    float x_min = -10;
    float y_min = -5;
    float z_min = -2;
    float x_max = 30;
    float y_max = 8;
    float z_max = 2;

    /*
    // helper box to figure out the cropBox size
    Box box;
    box.x_min = x_min;
    box.y_min = y_min;
    box.z_min = z_min;
    box.x_max = x_max;
    box.y_max = y_max;
    box.z_max = z_max;
    renderBox(viewer, box, 0, Color(1,0,1), 0.5f);
    */

    /*
    // helper box to figure out the cropBox size for roof points
    Box roofBox;
    roofBox.x_min = -1.5;
    roofBox.y_min = -1.7;
    roofBox.z_min = -1.1;
    roofBox.x_max = 2.6;
    roofBox.y_max = 1.7;
    roofBox.z_max = -0.4;
    renderBox(viewer, roofBox, 0, Color(0.5, 0.5, 0.5), 0.25f);
    */

    //filter point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI.FilterCloud(inputCloud, 0.3 , Eigen::Vector4f(x_min, y_min, z_min, 1), Eigen::Vector4f(x_max, y_max, z_max, 1));
    //draw filtered pointcloud
    renderPointCloud(viewer, filterCloud, "filterCloud");
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}