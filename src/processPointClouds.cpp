// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:DONE Fill in the function to do voxel grid point reduction and region based filtering

    // Create the VoxelGrid filtering object
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    //std::cout << typeid(vg).name() << endl;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes); // downsample the dataset using a leaf size of .2m
    vg.filter(*cloudFiltered);

    //region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    //remove roof points from the cloud
    // cloect indices inside of box
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    region.setMin(Eigen::Vector4f(-1.5, -1.7, -1.1, 1));
    region.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    region.setInputCloud(cloudRegion);
    region.filter(indices);
    //create inliers from indices
    pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };
    for (int point : indices)
        inliers->indices.push_back(point);
    //remove indices from cloudRegion
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO:DONE Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    // debug
    std::cout << std::endl << "SeparateClouds inliers size: " << inliers->indices.size() << std::endl;
    std::cout << "SeparateClouds cloud size: " << cloud->size() << std::endl;
    std::cout << "SeparateClouds obstCloud init size: " << obstCloud->size() << std::endl;
    std::cout << "SeparateClouds planeCloud init size: " << planeCloud->size() << std::endl;

    for (int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    // debug
    std::cout << "SeparateClouds planeCloud filled size: " << planeCloud->size() << std::endl;

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    // The obstCloud array indexes all points of cloud that are not indexed by inliers
    extract.setNegative(true);
    extract.filter(*obstCloud);

    // debug
    std::cout << "SeparateClouds after extract obstCloud size: " << obstCloud->size() << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);

    return segResult;
}

/*
//using PCL RANSAC
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:DONE Fill in this function to find inliers for the cloud.  - copied from the Extracting indices from a PointCloud tutorial
        // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}
*/

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    //Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    while (maxIterations--) {

        // randomly pick three points
        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
            inliers.insert(rand() % (cloud->points.size()));

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

        v1x = x2 - x1;
        v1y = y2 - y1;
        v1z = z2 - z1;
        v2x = x3 - x1;
        v2y = y3 - y1;
        v2z = z3 - z1;

        float i = (v1y * v2z) - (v1z * v2y);
        float j = (v1z * v2x) - (v1x * v2z);
        float k = (v1x * v2y) - (v1y * v2x);

        // plane calculation coefficients
        float A = i;
        float B = j;
        float C = k;
        float D = -(i * x1 + j * y1 + k * z1);

        for (int index = 0; index < cloud->points.size(); index++) {

            //skip if the point already in the set
            if (inliers.count(index) > 0)
                continue;

            PointT point = cloud->points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;

            // calc distance from the line
            float d = fabs(A * x4 + B * y4 + C * z4 + D) / sqrt(A * A + B * B + C * C);

            // add to set if distance is smaller than threshold
            if (d <= distanceTol)
                inliers.insert(index);
        }

        //check which set contains more points
        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;

    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Randsac took " << elapsedTime.count() << " milliseconds" << std::endl;

    return inliersResult;
}

//using own RANSAC
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    //pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    std::unordered_set<int> inliers = Ransac3D(cloud, maxIterations, distanceThreshold);
    pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for (int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


/*
//using PCL KDTree and EuclideanCluster
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:DONE Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 0.02 = 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    int j = 0;
    for (pcl::PointIndices getIndices : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

        for (int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
*/

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int index, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
    processed[index] = true;
    cluster.push_back(index);

    std::vector<int> nearest = tree->search(points[index], distanceTol);

    for (int id : nearest)
    {
        if (!processed[id])
        {
            clusterHelper(id, points, cluster, processed, tree, distanceTol);
        }
    }

}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
    std::vector<std::vector<int>> clusters;

    std::vector<bool> processed(points.size(), false);

    int i = 0;

    while (i < points.size())
    {
        if (processed[i])
        {
            i++;
            continue;
        }

        std::vector<int> cluster;

        clusterHelper(i, points, cluster, processed, tree, distanceTol);
        clusters.push_back(cluster);
        i++;
    }

    return clusters;

}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::CreateVectorToPointCloudData(std::vector<std::vector<float>> points)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());

    for (int i = 0; i < points.size(); i++)
    {
        PointT point;
        point.x = points[i][0];
        point.y = points[i][1];
        point.z = points[i][2];

        cloud->points.push_back(point);

    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;
}

template<typename PointT>
std::vector<std::vector<float>> ProcessPointClouds<PointT>::CreatePointCloudToVectorData(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::vector<std::vector<float>> points;

    for (int i = 0; i < cloud->points.size(); i++)
    {
        std::vector<float> point;
        point.push_back(cloud->points[i].x);
        point.push_back(cloud->points[i].y);
        point.push_back(cloud->points[i].z);

        points.push_back(point);
    }

    return points;
}

template<typename PointT>
std::vector<std::vector<float>> ProcessPointClouds<PointT>::findMedian(std::vector<std::vector<std::vector<float>>>& tempPoints, int depIndx, int pointDim)
{
    std::vector<std::vector<float>> medianPoints;
    std::vector<std::vector<std::vector<float>>> newVectors;
    depIndx = depIndx % pointDim;

    int tempPointSize = tempPoints.size();
    for (auto points : tempPoints)
    {
        std::sort(points.begin(), points.end(), [&depIndx](const std::vector<float>& a, const std::vector<float>& b) {return a[depIndx] < b[depIndx]; });
        int pointSize = points.size();
        int median;
        median = floor(pointSize / 2.0) + 1;

        medianPoints.push_back(points[median - 1]);

        std::vector<std::vector<float>> firstHalf(points.begin(), points.begin() + median - 1);
        std::vector<std::vector<float>> secondHalf(points.begin() + median, points.end());
        if (firstHalf.size() >= 1)
            newVectors.push_back(firstHalf);
        if (secondHalf.size() >= 1)
            newVectors.push_back(secondHalf);
    }
    tempPoints.insert(tempPoints.end(), newVectors.begin(), newVectors.end());
    tempPoints.erase(tempPoints.begin(), tempPoints.begin() + tempPointSize);

    return medianPoints;
}

//using own KDTree and EuclideanCluster
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<std::vector<float>> points = CreatePointCloudToVectorData(cloud);
    /*
    // balanced insertion
    std::vector <std::vector<std::vector<float>>> tempPoints;
    tempPoints.push_back(points);

    int depth = 0;
    int pointDim = 3;
    points.erase(points.begin(), points.end());
    std::vector<std::vector<float>> medianPoints;
    while (tempPoints.size() >= 1) {
        medianPoints = findMedian(tempPoints, depth, pointDim);
        points.insert(points.end(), medianPoints.begin(), medianPoints.end());
        depth++;
    }
    */
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating the KdTree object for the search method of the extraction
    KdTree* tree = new KdTree;

    // not balanced insertion
    for (int i = 0; i < points.size(); i++)
        tree->insert(points[i], i);

    std::vector<std::vector<int>> clusters_cust = euclideanCluster(points, tree, clusterTolerance);

    // collect clusters clusters
    for (std::vector<int> cluster : clusters_cust)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        for (int indice : cluster)
            clusterCloud->points.push_back(PointT(points[indice][0], points[indice][1], points[indice][2]));

        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;

        if ((clusterCloud->points.size() >= minSize) && (clusterCloud->points.size() <= maxSize))
            clusters.push_back(clusterCloud);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{ dataPath }, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}