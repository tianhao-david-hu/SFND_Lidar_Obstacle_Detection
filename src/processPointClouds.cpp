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

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_cropped(new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> crop_box(false);
    crop_box.setInputCloud(cloud_filtered);
    crop_box.setMax(maxPoint);
    crop_box.setMin(minPoint);
    crop_box.filter(*cloud_cropped);

    //std::vector<int> indices;
    typename pcl::PointCloud<PointT>::Ptr roof_cropped(new pcl::PointCloud<PointT>);

    //pcl::CropBox<PointT> roof_cropped(true);
    crop_box.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    crop_box.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    crop_box.setInputCloud(cloud_cropped);
    crop_box.setNegative(true);
    crop_box.filter(*roof_cropped);

    /*pcl::PointIndices:;Ptr inliers{new pcl::PointIndices};
    for(int point : indices)
        inliers0>indices.push_back(point);*/

    


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return roof_cropped;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>), obstCloud (new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*planeCloud);
    std::cerr << "PointCloud representing the planar component: " << planeCloud->width * planeCloud->height << " data points." << std::endl;

    // Create the filtering object
    extract.setNegative (false);
    extract.filter (*obstCloud);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(planeCloud, obstCloud);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with MOST inliers
	while(maxIterations--)
	{
		// Randomly pick two points
		std::unordered_set<int> inliers;
		while(inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));

		// Line fitting using picked points:
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

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

		double v1x, v1y, v1z, v2x, v2y, v2z;
		v1x = x2 - x1;
		v1y = y2 - y1;
		v1z = z2 - z1;

		v2x = x3 - x1;
		v2y = y3 - y1;
		v2z = z3 - z1;

		double i,j,k;
		i = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		j = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		k = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);

		float A = i;
		float B = j;
		float C = k;
		float D = -(i*x1+j*y1+k*z1);


		for(int index = 0; index < cloud->points.size(); index++)
		{
			//Don't use existing points
			if(inliers.count(index)>0)
				continue;
			
			PointT point = cloud->points[index];
			float x = point.x;
			float y = point.y;
			float z = point.z;

			float d = fabs(A*x + B*y + C*z + D)/sqrt(A*A + B*B + C*C);

			if(d <= distanceTol)
				inliers.insert(index);
		}

		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}

		
	}
	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
#ifdef USE_PCL
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planr model for the given dataset." << std::endl;
    } 
    segResult = SeparateClouds(inliers,cloud);
#else
    std::unordered_set<int> inliers = Ransac(cloud, maxIterations, distanceThreshold);

	typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    segResult = std::make_pair(cloudOutliers,cloudInliers);
#endif
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
   

    return segResult;
}

template<typename PointT>
void Proximity(typename pcl::PointCloud<PointT>::Ptr cloud, const int index, typename pcl::PointCloud<PointT>::Ptr cluster, std::vector<bool>& processed, KdTree<PointT>* tree, const float distanceTol)
{
	processed[index] = true;
	cluster->push_back(cloud->points[index]);
    
	auto nearby_pt_idx = tree->search(cloud->points[index], distanceTol);
	for(auto &idx: nearby_pt_idx)
	{
		if(processed[idx] == false)
		{
			Proximity(cloud,idx,cluster,processed,tree,distanceTol);
		}
	}
}


template<typename PointT>
void euclideanCluster(std::vector<typename pcl::PointCloud<PointT>::Ptr>& clusters, 
                      typename pcl::PointCloud<PointT>::Ptr cloud,
                      KdTree<PointT>* tree, const float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<bool> processed(cloud->points.size(),false);

	for(int i = 0; i<cloud->points.size(); i++)
	{
		if(processed[i]==false)
		{
			typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
			Proximity(cloud, i, cluster, processed, tree, distanceTol);
			clusters.push_back(cluster);
		}
	}
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
#ifdef USE_PCL
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for(const auto& cluster : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for(const auto& idx: cluster.indices)
            cloud_cluster->points.push_back((*cloud)[idx]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        
        clusters.push_back(cloud_cluster);
    }
#else
    KdTree<PointT>* tree = new KdTree<PointT>;
    for(int index = 0; index < cloud->points.size(); index++)
         tree->insert(cloud->points[index],index);
    euclideanCluster(clusters, cloud, tree, clusterTolerance);
#endif
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
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}