//
// Created by lifan on 3/10/17.
//

#include "LidarPointCloudHandler.h"

const bool IS_DEBUG = false;

LidarPointCloudHandler::LidarPointCloudHandler(CloudPointer srcCloud) {
    _srcCloud = srcCloud->makeShared();
    _isCalibrated = false;
}

LidarPointCloudHandler::~LidarPointCloudHandler() {

}

CloudPointerRGB LidarPointCloudHandler::pointCloudDecomposition() {
    if(!_srcCloud || _srcCloud->points.size() == 0 || !_refCloud || _refCloud->points.size() == 0){
        cout << "Source cloud or refer cloud is not defined." << endl;
        return NULL;
    }
    
    
    // Initialize some point cloud temp variables
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIntegration(new pcl::PointCloud<pcl::PointXYZRGB>);   // The labeled integration of all clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    // Create the segmentation object for the planar model
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;

    // Set all the parameters for plane fitting (M-SAC)
    Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_MSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.4); //0.01
    double secondDistanceThreshold(0.1);
    seg.setAxis(axis);

    // Initialize variables here
    bool firstEnterLoop = true;
    int nr_points = (int) _srcCloud->points.size ();
    int failureCounter = 0;


    /*-------------------- 1.1 Remove ground plane from source point cloud ---------------------*/
    while (_srcCloud->points.size () > 0)
    {
        if(!firstEnterLoop)
            break;
        // Segment the largest planar component
        seg.setInputCloud(_srcCloud);
        seg.segment(*inliers, *coefficients);
        if (!utils::checkIfGroundPlaneCoeffi(coefficients) || inliers->indices.size() < 100)
        {
            std::cout << "Could not estimate more ground plane mdoels for the given dataset." << std::endl;
            failureCounter++;
            if(failureCounter > 10) break;
            continue;
        }
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        float curvature;
        Eigen::Vector4f planeParamters;
        pcl::computePointNormal(*_srcCloud, inliers->indices, planeParamters, curvature);
        coefficients->values[0] = planeParamters[0];    //  initial guess of the plane
        coefficients->values[1] = planeParamters[1];
        coefficients->values[2] = planeParamters[2];
        coefficients->values[3] = planeParamters[3];
        double a = coefficients->values[0];
        double b = coefficients->values[1];
        double c = coefficients->values[2];
        double d = coefficients->values[3];
        double yFraction = fabs(b)/(a*a + b*b + c*c);
        //cout << "Super yFraction is " << yFraction << " d is " << d << endl;
        utils::getInliersByWeightedDistance(_srcCloud, coefficients, secondDistanceThreshold, inliers);

        if(firstEnterLoop){
            firstEnterLoop = false;
        }


        // Extract the planar inliers from the input cloud
        extract.setInputCloud(_srcCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        if(IS_DEBUG)
            utils::visualizeCloud(cloud_plane);
        for(int i = 0; i < cloud_plane->points.size(); i++){
            pcl::PointXYZ curPoint = cloud_plane->points[i];
            if(true){     // utils::checkWithPriorInfo(curPoint)
                pcl::PointXYZRGB point;
                point.x = cloud_plane->points[i].x;
                point.y = cloud_plane->points[i].y;
                point.z = cloud_plane->points[i].z;
                point.b = 0;
                point.r = 255;
                point.g = 0;
                cloudIntegration->points.push_back(point);
            }
        }
//        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points."
//                  << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *_srcCloud = *cloud_f;
    }
    if(IS_DEBUG)
        utils::visualizeCloud(_srcCloud);




//    cout << "After removing the ground plane " << endl;
//    cout << "Reference: " << _refCloud->points.size() << endl;
//    cout << "Source: " << _srcCloud->points.size() << endl;
    if(IS_DEBUG)
        utils::visualizeCloud(_srcCloud);

    firstEnterLoop = true;
    /*------------------------- 1.1 Remove ground plane from reference point cloud ------------------------*/
    failureCounter = 0;
    while (_refCloud->points.size () > 0)
    {
        if(!firstEnterLoop)
            break;
        seg.setInputCloud(_refCloud);
        seg.segment(*inliers, *coefficients);
        if (!utils::checkIfGroundPlaneCoeffi(coefficients) || inliers->indices.size() < 100)
        {
            std::cout << "Could not estimate more ground plane mdoels for the given dataset." << std::endl;
            failureCounter++;
            if(failureCounter > 10) break;
            continue;
        }
        float curvature;
        Eigen::Vector4f planeParamters;
        pcl::computePointNormal(*_refCloud, inliers->indices, planeParamters, curvature);
        coefficients->values[0] = planeParamters[0];    //  initial guess of the plane
        coefficients->values[1] = planeParamters[1];
        coefficients->values[2] = planeParamters[2];
        coefficients->values[3] = planeParamters[3];
        double a = coefficients->values[0];
        double b = coefficients->values[1];
        double c = coefficients->values[2];
        double d = coefficients->values[3];
        double yFraction = fabs(b)/(a*a + b*b + c*c);
        cout << "Super yFraction is " << yFraction << " d is " << d << endl;
        utils::getInliersByWeightedDistance(_refCloud, coefficients, secondDistanceThreshold, inliers);

        if(firstEnterLoop){
            firstEnterLoop = false;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(_refCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        if(IS_DEBUG)
            utils::visualizeCloud(cloud_plane);
//        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points."
//                  << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *_refCloud = *cloud_f;
    }

//    cout << "After removing the ground plane " << endl;
//    cout << "Reference: " << _refCloud->points.size() << endl;
//    cout << "Source: " << _srcCloud->points.size() << endl;
    if(IS_DEBUG)
        utils::visualizeCloud(_refCloud);


    /*----------------------------------------- 2.Get outliers from full pointcloud ----------------------------------*/
    // KDTree
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeRefPointCloud(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    kdtreeRefPointCloud->setInputCloud(_refCloud);

    // Threshold to reject static points
    double outlierThreshold = 0.2; // 1.0m

    // For search result
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    std::vector<bool> outlierMarker(_srcCloud->points.size(), false);
    for (int i = 0; i < _srcCloud->points.size(); ++i)
    {
        pcl::PointXYZ curPoint = _srcCloud->points[i];
        kdtreeRefPointCloud->nearestKSearch(curPoint, 1, pointSearchInd, pointSearchSqDis);

        if (pointSearchSqDis[0] > outlierThreshold)
        {
            if(true)         // If prior of cars is given here
                outlierMarker[i] = true;
        }
    }
    // Get outlier
    for (int i = 0; i < _srcCloud->width; i++)
    {
        if(outlierMarker[i])
        {
            cloud->push_back(_srcCloud->points[i]);
        }
    }

    if(IS_DEBUG)
        utils::visualizeCloud(cloud);
    cout << cloud->points.size() << endl;



    //Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (2.0);
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    inliers->indices.clear();
    std::vector<utils::BoundingBox> vehicleBoundingBoxVec;

    for (auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (auto pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        //utils::visualizeCloud(cloud_cluster);
        utils::BoundingBox tmp(cloud_cluster);
        if(cloud_cluster->width > 30 && tmp.isvalid)
        {
            vehicleBoundingBoxVec.push_back(tmp);
            for(int i = 0; i < it->indices.size(); i++) {
                pcl::PointXYZ srcPoint = cloud->points[it->indices[i]];
                if (true) {  //utils::checkWithPriorInfo(srcPoint)
                    pcl::PointXYZRGB point;
                    point.x = srcPoint.x;
                    point.y = srcPoint.y;
                    point.z = srcPoint.z;
                    point.b = 255;
                    point.g = 0;
                    point.r = 0;
                    cloudIntegration->points.push_back(point);
                    for (int j = 0; j < _srcCloud->points.size(); j++) {
                        pcl::PointXYZ curPoint = _srcCloud->points[j];
                        if (curPoint.x == point.x && curPoint.y == point.y && curPoint.z == point.z) {
                            inliers->indices.push_back(j);
                            break;
                        }
                    }
                }
            }

        }
    }

    // Extract all the remaining static points
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(_srcCloud);
    extract.setNegative(true);
    extract.setIndices(inliers);
    extract.filter(*cloud_f);
    *_srcCloud = *cloud_f;
    for(int i = 0; i < _srcCloud->points.size(); i++){
        pcl::PointXYZ curPoint = _srcCloud->points[i];
        if(true) {   // utils::checkWithPriorInfo(curPoint)
            pcl::PointXYZRGB point;
            point.x = curPoint.x;
            point.y = curPoint.y;
            point.z = curPoint.z;
            point.b = 0;
            point.g = 255;
            point.r = 0;
            cloudIntegration->points.push_back(point);
        }
    }

    //utils::visualizeCloud(cloudIntegration);
    return cloudIntegration;
}

void LidarPointCloudHandler::setReferenceCloud(CloudPointer refCloud) {
    _refCloud = refCloud->makeShared();
}

CloudPointer LidarPointCloudHandler::getPointsInsideImageBoundingBox(utils::ImageBoundingBox box) {
    return this->getPointsInsideImageBoundingBox(box, _srcCloud);
}


void LidarPointCloudHandler::setImageCalibration(string transformPath, string KPath) {
    _K = cv::Mat(3, 3, CV_64F);
    _P = cv::Mat(4, 4, CV_64F);
    utils::loadKFromTXT(KPath, _K);
    utils::loadTransformFromTXT(transformPath, _P);
    _P.at<double>(3, 0) = 0;
    _P.at<double>(3, 1) = 0;
    _P.at<double>(3, 2) = 0;
    _P.at<double>(3, 3) = 1;
    cout << _P <<endl;
    cout << _K <<endl;
    _P = _P.inv();
    _isCalibrated = true;
}

void LidarPointCloudHandler::convertImageBoxToLidarBox(utils::ImageBoundingBox box) {
    CloudPointer extracted = this->getPointsInsideImageBoundingBox(box);
    // the inliers and coefficients of the plane
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    // Set all the parameters for plane fitting (M-SAC)
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_MSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.05); //0.01
    seg.setInputCloud(extracted);

    seg.setInputCloud(_srcCloud);
    seg.segment(*inliers, *coefficients);

    const double a = coefficients->values[0];
    const double b = coefficients->values[1];
    const double c = coefficients->values[2];
    const double d = coefficients->values[3];
    if(b/sqrt(a*a + b*b + c*c + d*d) > 0.05){
        cout << "Can not estimate car model" << endl;
        return;
    }
    if(a*a < c*c){
        //assume a parellel line
    }
    else{
        //assume a perpendicular line
    }
}

void LidarPointCloudHandler::removeGroundPlane() {
    if(!_srcCloud || _srcCloud->points.size() == 0){
        cout << "Source cloud is not defined." << endl;
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    // Create the segmentation object for the planar model
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

    // Set all the parameters for plane fitting (M-SAC)
    Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_MSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.4); //0.01
    double secondDistanceThreshold(0.1);
    seg.setAxis(axis);

    // Initialize variables here
    bool firstEnterLoop = true;
    int nr_points = (int) _srcCloud->points.size ();
    int failureCounter = 0;


    /*-------------------- 1.1 Remove ground plane from source point cloud ---------------------*/
    while (_srcCloud->points.size () > 0)
    {
        if(!firstEnterLoop)
            break;
        // Segment the largest planar component
        seg.setInputCloud(_srcCloud);
        seg.segment(*inliers, *coefficients);
        if (!utils::checkIfGroundPlaneCoeffi2(coefficients) || inliers->indices.size() < 100)
        {
            std::cout << "Could not estimate more ground plane mdoels for the given dataset." << std::endl;
            failureCounter++;
            if(failureCounter > 10) break;
            continue;
        }
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        float curvature;
        Eigen::Vector4f planeParamters;
        pcl::computePointNormal(*_srcCloud, inliers->indices, planeParamters, curvature);
        coefficients->values[0] = planeParamters[0];    //  initial guess of the plane
        coefficients->values[1] = planeParamters[1];
        coefficients->values[2] = planeParamters[2];
        coefficients->values[3] = planeParamters[3];
        double a = coefficients->values[0];
        double b = coefficients->values[1];
        double c = coefficients->values[2];
        double d = coefficients->values[3];
        double yFraction = fabs(b)/(a*a + b*b + c*c);
        utils::getInliersByWeightedDistance(_srcCloud, coefficients, secondDistanceThreshold, inliers);
        if(firstEnterLoop){
            firstEnterLoop = false;
        }

        // Extract the planar inliers from the input cloud
        extract.setInputCloud(_srcCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);

        for(int i = 0; i < cloud_plane->points.size(); i++){
            pcl::PointXYZ curPoint = cloud_plane->points[i];
            if(true){     // utils::checkWithPriorInfo(curPoint)
                pcl::PointXYZRGB point;
                point.x = cloud_plane->points[i].x;
                point.y = cloud_plane->points[i].y;
                point.z = cloud_plane->points[i].z;
                point.b = 0;
                point.r = 255;
                point.g = 0;

            }
        }
        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *_srcCloud = *cloud_f;
    }
}

CloudPointer LidarPointCloudHandler::getSourcePointCloud() {
    return _srcCloud;
}

vector<CloudPointer> LidarPointCloudHandler::pointCloudSegmentation() {
    vector<CloudPointer> rVec;
    utils::getVehicleClusterFromCloud(_srcCloud, rVec);
    return rVec;
}

void LidarPointCloudHandler::setSourceCLoud(CloudPointer srcCloud) {
    _srcCloud =  srcCloud;
}

CloudPointer LidarPointCloudHandler::getPointsInsideImageBoundingBox(utils::ImageBoundingBox box, CloudPointer cloud) {
    if(cloud->points.size() == 0){
        return cloud;
    }
    CloudPointer resCloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(!_isCalibrated){
        return resCloud;
    }
    for(auto point:cloud->points){
        pcl::PointXYZ pointCamera(point);

        // Reallocate the coordiantes (TOCHECK)
//        float temp = pointCamera.x;
//        pointCamera.x = pointCamera.z;
//        pointCamera.z = pointCamera.y;
//        pointCamera.y = temp;

        cv::Point2f pointDehomo;
        if(!utils::transformLidarPointToCamera(pointCamera, pointDehomo, _K, _P))
            continue;
//        cout << pointDehomo.x << " " << pointDehomo.y << endl;
        if(box.isPointInBox(pointDehomo.x, pointDehomo.y)) {

            resCloud->points.push_back(pointCamera);
        }
    }
    return resCloud;
}





