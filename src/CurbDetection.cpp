//
// Created by lifan on 2/27/17.
//

#include "utils.h"
#include "ModelRepresentation.h"
#include "LidarPointCloudHandler.h"
#include <math.h>




void findCurbObject(CloudPointer laserCloudSrc, CloudPointer laserCloudRef, std::vector<float> transformMatrix,
                    std::string folderName, vector<double> &final, vector<double> initial,
                    vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &savedPoints,
                    const bool isInitialProvided = false){
    const bool IS_DEBUG = false;
    // Transform to raw if the transform is given
    if(transformMatrix.size()>0) {
        utils::pointCloudMapToRaw(laserCloudSrc, transformMatrix);
        utils::pointCloudMapToRaw(laserCloudRef, transformMatrix);
    }

    std::unique_ptr<LidarPointCloudHandler> cloudHandler(new LidarPointCloudHandler(laserCloudSrc));
    cloudHandler->setReferenceCloud(laserCloudRef);
    CloudPointerRGB cloudIntegration = cloudHandler->pointCloudDecomposition();

    // Save the result by color label in RGB point cloud
    CloudPointer staticObject(new pcl::PointCloud<pcl::PointXYZ>);
    CloudPointer movingObject(new pcl::PointCloud<pcl::PointXYZ>);
    CloudPointer groundObject(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i = 0; i < cloudIntegration->points.size(); i++){
        pcl::PointXYZ newPoint;
        pcl::PointXYZRGB curPoint = cloudIntegration->points[i];
        newPoint.x = curPoint.x;
        newPoint.y = curPoint.y;
        newPoint.z = curPoint.z;
        if(transformMatrix.size() >0){
            if(!utils::checkWithEffectiveRegion(newPoint)){
                continue;
            }
        }
        else{
            if(!utils::checkWithPriorInfo(newPoint)){
                continue;
            }
        }
        if(curPoint.r == 255){
            if(true) {  // Check with some prior info here
                //staticObject->points.push_back(newPoint);   // Notice
                groundObject->points.push_back(newPoint);
            }
        }
        if(curPoint.g == 255){
            if(true) {  // Check with some prior info here
                staticObject->points.push_back(newPoint);
            }
        }
        else if(curPoint.b == 255){
            if(true) {  // Check with some prior info here
                movingObject->points.push_back(newPoint);
            }
        }
    }
    if(IS_DEBUG) {
        utils::visualizeCloud(staticObject);
        utils::visualizeCloud(movingObject);
        utils::visualizeCloud(groundObject);
    }

    CloudPointer srcStaticObject(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto point:staticObject->points){
        pcl::PointXYZ newPoint = utils::pointAssociateTobeMapped(point, transformMatrix);
        srcStaticObject->points.push_back(newPoint);
    }


    CloudPointer planeSrcStaticObject(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto point:srcStaticObject->points){
        pcl::PointXYZ newPoint = point;
        newPoint.y = 0;
        planeSrcStaticObject->points.push_back(newPoint);
    }
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(planeSrcStaticObject);
    sor.setLeafSize (0.03f, 0.03f, 0.03f);
    sor.filter (*planeSrcStaticObject);

    //utils::visualizeCloud(srcStaticObject);
    // Part for define the runnable area
    TwoLineModel *runnableArea(new TwoLineModel);
    runnableArea->setInputPointCloud(planeSrcStaticObject);
    if(isInitialProvided) {
        runnableArea->setInitialGuess(initial[0], initial[1], initial[2], initial[3]);
    }
    // Brute force searching for best parameters
    runnableArea->RANSACSearching();
//    runnableArea->BFSearching();    // discarded

    const double PI = M_PI;

    vector<double> representation = runnableArea->getRepresentation();
    // Create two plane coefficients representing to curb planes
    pcl::ModelCoefficients coeffs;
    coeffs.values.resize(4);
    coeffs.values[0] = float(representation[0]);
    coeffs.values[1] = 0;
    coeffs.values[2] = float(representation[1]);
    coeffs.values[3] = float(representation[2]);

    pcl::ModelCoefficients coeffs2;
    coeffs2.values.resize(4);
    coeffs2.values[0] = float(representation[0]);
    coeffs2.values[1] = 0;
    coeffs2.values[2] = float(representation[1]);
    coeffs2.values[3] = float(representation[3]);

    // Define points near the runnable plane.
    CloudPointer curbObject(new pcl::PointCloud<pcl::PointXYZ>);

    double distThreshold = 0.3;
    for(int i = 0; i < srcStaticObject->points.size(); i++){
        pcl::PointXYZ newPoint;
        pcl::PointXYZ curPoint = srcStaticObject->points[i];
        if(utils::calcPointPlaneDist(coeffs, curPoint)< distThreshold  || utils::calcPointPlaneDist(coeffs2, curPoint) < distThreshold) {
            newPoint.x = curPoint.x;
            newPoint.y = curPoint.y;
            newPoint.z = curPoint.z;
            curbObject->points.push_back(newPoint);
        }
    }

    utils::visualizeCloud(cloudIntegration);
    utils::visualizeCloud(curbObject);
    final = representation;

    pcl::PointXYZ intersectionLeft, intersectionRight;
    intersectionLeft.x = max(-coeffs2.values[3]/coeffs2.values[0], -coeffs.values[3]/coeffs.values[0]);
    intersectionLeft.y = 0;
    intersectionLeft.z = 0;

    intersectionRight.x = min(-coeffs2.values[3]/coeffs2.values[0], -coeffs.values[3]/coeffs.values[0]);
    intersectionRight.y = 0;
    intersectionRight.z = 0;

    intersectionLeft = utils::pointAssociateToMap(intersectionLeft, transformMatrix);
    intersectionRight = utils::pointAssociateToMap(intersectionRight, transformMatrix);

    std::pair<pcl::PointXYZ, pcl::PointXYZ> savedPairs;
    savedPairs.first = intersectionLeft;
    savedPairs.second = intersectionRight;
    savedPoints.push_back(savedPairs);

    // save all the objects into a folder
    std::string SaveFolderPrefix = "/home/lifan/LIDAR/result_curb/";
    SaveFolderPrefix = SaveFolderPrefix + '/';
    std::string curbFileName = folderName + "_curb";
    utils::pclPointCloudWriter(SaveFolderPrefix, curbFileName, curbObject);

    // recycle pointers;
    delete runnableArea;
}


void findRoadObject(CloudPointerRGB laserCloudSrcRGB, const std::vector<float> transformMatrix,
                    const std::string folderName, vector<double> &final, const vector<double> initial,
                    vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &savedPoints, vector<pcl::PointXYZ> &savedCenters,
                    vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &savedPoints_map, const bool isInitialProvided = false){
    const bool isVisualize = false;
    const double PI = M_PI;

    if(isVisualize)
        utils::myPCLViewer::getInstance()->changePointCloud(laserCloudSrcRGB);
    CloudPointerRGB planeSrcObject(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(auto point:laserCloudSrcRGB->points){
        pcl::PointXYZRGB newPoint(point);
        newPoint.y = 0;
        planeSrcObject->points.push_back(newPoint);
    }

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(planeSrcObject);
    sor.setLeafSize (0.03f, 0.03f, 0.03f);
    sor.filter (*planeSrcObject);
    RectangleModel *model(new RectangleModel);
    model->setInputPointCloud(planeSrcObject);
    if(!isInitialProvided) {
        //model->setInitialGuess(1, 0, -5, 5);    // If the initial prior is given
    }
    else{
        model->setInitialGuess(initial[0], initial[1], initial[2], initial[3]);
    }
    model->BFSearching();
    vector<double> representation = model->getRepresentation();

    // Define points near the runnable plane.
    CloudPointer roadObject(new pcl::PointCloud<pcl::PointXYZ>);
    CloudPointerRGB visualizeObject(new pcl::PointCloud<pcl::PointXYZRGB>);

    for(auto point:laserCloudSrcRGB->points){
        pcl::PointXYZ curPoint;
        pcl::PointXYZRGB colorPoint(point);
        if(model->getPointCost(point, true, representation[0], representation[1], representation[2], representation[3]) <= 1e-3){
            curPoint.x = point.x;
            curPoint.y = point.y;
            curPoint.z = point.z;
            roadObject->points.push_back(curPoint);
            colorPoint.r = 255;
            colorPoint.g = 0;
        }
        else{
            colorPoint.r = 0;
            colorPoint.g = 255;
        }
        visualizeObject->points.push_back(colorPoint);
    }
    if(isVisualize) {
        utils::myPCLViewer::getInstance()->changePointCloud(roadObject);
        utils::myPCLViewer::getInstance()->changePointCloud(visualizeObject);
//        utils::visualizeCloud(roadObject);
//        utils::visualizeCloud(visualizeObject);
    }
    final = representation;

    pcl::PointXYZ intersectionLeft, intersectionRight;
    pcl::ModelCoefficients coeffs;
    coeffs.values.resize(4);
    coeffs.values[0] = float(representation[0]);
    coeffs.values[1] = 0;
    coeffs.values[2] = float(representation[1]);
    coeffs.values[3] = float(representation[2]);

    pcl::ModelCoefficients coeffs2;
    coeffs2.values.resize(4);
    coeffs2.values[0] = float(representation[0]);
    coeffs2.values[1] = 0;
    coeffs2.values[2] = float(representation[1]);
    coeffs2.values[3] = float(representation[3]);

    intersectionLeft.x = max(-coeffs2.values[3]/coeffs2.values[0], -coeffs.values[3]/coeffs.values[0]);
    cout << "leftmost: " <<  intersectionLeft.x <<endl;
    intersectionLeft.y = 0;
    intersectionLeft.z = 0;
    intersectionRight.x = min(-coeffs2.values[3]/coeffs2.values[0], -coeffs.values[3]/coeffs.values[0]);
    cout << "leftmost: " <<  intersectionRight.x <<endl;
    intersectionRight.y = 0;
    intersectionRight.z = 0;


    pcl::PointXYZ intersectionLeft_map = utils::pointAssociateToMap(intersectionLeft, transformMatrix);
    pcl::PointXYZ intersectionRight_map = utils::pointAssociateToMap(intersectionRight, transformMatrix);

    std::pair<pcl::PointXYZ, pcl::PointXYZ> savedPairs;
    savedPairs.first = intersectionLeft;
    savedPairs.second = intersectionRight;
    savedPoints.push_back(savedPairs);


    std::pair<pcl::PointXYZ, pcl::PointXYZ> savedPairs_map;
    savedPairs_map.first = intersectionLeft_map;
    savedPairs_map.second = intersectionRight_map;
    savedPoints_map.push_back(savedPairs_map);


    pcl::PointXYZ center(0, 0, 0);
    center = utils::pointAssociateToMap(center, transformMatrix);
    savedCenters.push_back(center);


    // create the directory if not exist
//    boost::filesystem::path dir(SaveFolderPrefix);
//    boost::filesystem::create_directory(dir);
//    std::string SaveFolderPrefix = "/home/lifan/LIDAR/result_road/";
//    SaveFolderPrefix = SaveFolderPrefix + '/';
//
//    std::string curbFileName = folderName + "_road";
//    utils::pclPointCloudWriter(SaveFolderPrefix, curbFileName, roadObject);
    // recycle pointers;
    delete model;
}

void initKalmanFilter(cv::KalmanFilter& KF, const int states, const int measurements, const int inputs, const double dt) {
    KF.init(states, measurements, inputs, CV_64F);                 // init Kalman Filter
    cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));       // set process noise
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-4));   // set measurement noise
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));             // error covariance

    // position
    KF.transitionMatrix.at<double>(0,0) = 1;   //b
    KF.transitionMatrix.at<double>(1,1) = 1;   //c
    KF.transitionMatrix.at<double>(2,2) = 1;   //cprime

    /* MEASUREMENT MODEL */
    KF.measurementMatrix.at<double>(0,0) = 1;  // x
    KF.measurementMatrix.at<double>(1,1) = 1;  // y
    KF.measurementMatrix.at<double>(2,2) = 1;  // z
}


int main(){
    const int mode = 2;  // 1 for curb, 2 for road;
    long start = 148830759210;
    const int buffer = 5;
    const int refDeviation = 25;
    const int numberOfFrames = 400;
    utils::myPCLViewer::createInstance();
    switch(mode) {
        case 1: {
            vector<vector<double>> curbEstimations;
            vector<double> currentEstimate;
            vector<double> priorEstimate;
            vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> savedPoints;

            for (int i = 0; i < numberOfFrames; i++) {
                long startString = start + 5 * i;

                std::string cloudDir = "/home/lifan/LIDAR/workspace/20170228_3/mappoints/";
                std::string fileSuffix = "_FullRes.pcd";
                std::string srcCloudNumber = utils::convertTimeStampToFileString(startString);
                std::string refCloudNumber = utils::convertTimeStampToFileString(startString + buffer * refDeviation);

                std::string srcPointCloudPathString = cloudDir + srcCloudNumber + fileSuffix;
                std::string refPointCloudPathString = cloudDir + refCloudNumber + fileSuffix;
                std::string srcTransform = cloudDir + srcCloudNumber + ".txt";
                CloudPointer laserCloudSrc = utils::loadCloudFromPath(srcPointCloudPathString);
                CloudPointer laserCloudRef = utils::loadCloudFromPath(refPointCloudPathString);
                if (laserCloudSrc->points.size() == 0) {
                    start = start + 1;
                    startString = start + 5 * i;
                    srcCloudNumber = utils::convertTimeStampToFileString(startString);
                    srcPointCloudPathString = cloudDir + srcCloudNumber + fileSuffix;
                    srcTransform = cloudDir + srcCloudNumber + ".txt";
                    laserCloudSrc = utils::loadCloudFromPath(srcPointCloudPathString);

                    if (laserCloudSrc->points.size() == 0) {
                        start = start - 1;
                        continue;
                    }

                    refCloudNumber = utils::convertTimeStampToFileString(startString + buffer * refDeviation);
                    refPointCloudPathString = cloudDir + refCloudNumber + fileSuffix;
                    laserCloudRef = utils::loadCloudFromPath(refPointCloudPathString);
                }
                if (laserCloudRef->points.size() == 0) {
                    refCloudNumber = utils::convertTimeStampToFileString(startString + 1 + buffer * refDeviation);
                    refPointCloudPathString = cloudDir + refCloudNumber + fileSuffix;
                    laserCloudRef = utils::loadCloudFromPath(refPointCloudPathString);
                }
                vector<float> transform = utils::loadTransformFromTXT(srcTransform);
                if (currentEstimate.size() == 0) {
                    findCurbObject(laserCloudSrc, laserCloudRef, transform, srcCloudNumber, currentEstimate,
                                   priorEstimate, savedPoints, false);
                } else {
                    findCurbObject(laserCloudSrc, laserCloudRef, transform, srcCloudNumber, currentEstimate,
                                   curbEstimations[curbEstimations.size() - 1], savedPoints, true);
                }
                curbEstimations.push_back(currentEstimate);
                cout << "Finish Loading " << endl;
                cout << srcPointCloudPathString << endl;
                cout << refPointCloudPathString << endl;
            }
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
                    new pcl::visualization::PCLVisualizer("3D Viewer"));
            viewer->setBackgroundColor(0, 0, 0);
            int count = 0;
            for (auto i = 0; i < savedPoints.size() - 1; i++) {
                viewer->addLine<pcl::PointXYZ>(savedPoints[i].first, savedPoints[i + 1].first,
                                               "line" + to_string(count++));
                viewer->addLine<pcl::PointXYZ>(savedPoints[i].second, savedPoints[i + 1].second,
                                               "line" + to_string(count++));
            }
            viewer->addCoordinateSystem(1.0);
            viewer->initCameraParameters();
            while (!viewer->wasStopped()) {
                viewer->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            }
            viewer->close();
        }
            break;
        case 2: {
            vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> savedPoints;
            vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> savedPoints_map;
            vector<pcl::PointXYZ> savedCenters;
            cv::KalmanFilter KF;         // instantiate Kalman Filter
            int nStates = 3;            // the number of states
            int nMeasurements = 3;       // the number of measured states
            int nInputs = 0;             // the number of action control
            double dt = 0.05;           // time between measurements (1/FPS)

            vector<double> initial;
            vector<double> final;
            vector<vector<double>> roadEstimations;
            initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);    // init function

            for (int i = 0; i < numberOfFrames; i++) {
                long startString = start + 5 * i;

                std::string cloudDir = "/home/lifan/LIDAR/final2/";
                std::string fileSuffix = "_final.pcd";
                std::string srcCloudNumber = utils::convertTimeStampToFileString(startString);
                std::string refCloudNumber = utils::convertTimeStampToFileString(startString + buffer * refDeviation);

                std::string srcPointCloudPathString = cloudDir + srcCloudNumber + fileSuffix;
                std::string refPointCloudPathString = cloudDir + refCloudNumber + fileSuffix;
                std::string srcTransform = cloudDir + srcCloudNumber + ".txt";
                CloudPointerRGB laserCloudSrc = utils::loadRGBCloudFromPath(srcPointCloudPathString);
                CloudPointerRGB laserCloudRef = utils::loadRGBCloudFromPath(refPointCloudPathString);
                if (laserCloudSrc->points.size() == 0) {
                    start = start + 1;
                    startString = start + 5 * i;
                    srcCloudNumber = utils::convertTimeStampToFileString(startString);
                    srcPointCloudPathString = cloudDir + srcCloudNumber + fileSuffix;
                    srcTransform = cloudDir + srcCloudNumber + ".txt";
                    laserCloudSrc = utils::loadRGBCloudFromPath(srcPointCloudPathString);

                    if (laserCloudSrc->points.size() == 0) {
                        start = start - 1;
                        continue;
                    }

                    refCloudNumber = utils::convertTimeStampToFileString(startString + buffer * refDeviation);
                    refPointCloudPathString = cloudDir + refCloudNumber + fileSuffix;
                    laserCloudRef = utils::loadRGBCloudFromPath(refPointCloudPathString);
                }
                if (laserCloudRef->points.size() == 0) {
                    refCloudNumber = utils::convertTimeStampToFileString(startString + 1 + buffer * refDeviation);
                    refPointCloudPathString = cloudDir + refCloudNumber + fileSuffix;
                    laserCloudRef = utils::loadRGBCloudFromPath(refPointCloudPathString);
                }
                vector<float> transform = utils::loadTransformFromTXT(srcTransform);
                if (i == 0) { // The initial state
                    findRoadObject(laserCloudSrc, transform, srcCloudNumber, final, initial, savedPoints, savedCenters, savedPoints_map);
                }
                else{
                    findRoadObject(laserCloudSrc, transform, srcCloudNumber, final, roadEstimations[roadEstimations.size()-1], savedPoints, savedCenters, savedPoints_map, true);
                }
                roadEstimations.push_back(final);
                cout << "Finish Loading " << endl;
                cout << srcPointCloudPathString << endl;
                cout << refPointCloudPathString << endl;
            }
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
                    new pcl::visualization::PCLVisualizer("3D Viewer"));
            viewer->setBackgroundColor(0, 0, 0);
            int count = 0;
            const int slidingWindowSize = 13;
            std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> smoothPoints;
            for (auto i = 0; i < savedPoints.size() - 1 - slidingWindowSize; i++) {
                pcl::PointXYZ curLeftPoint(savedPoints[i].first);
                pcl::PointXYZ curRightPoint(savedPoints[i].second);
                curLeftPoint.x = 0;
                curRightPoint.x = 0;
                for(auto j = i; j < i + slidingWindowSize; j ++) {
                    curLeftPoint.x += savedPoints[j].first.x;
                    curRightPoint.x += savedPoints[j].second.x;
                }
                curLeftPoint.x/= slidingWindowSize;
                curRightPoint.x/= slidingWindowSize;
                std::pair<pcl::PointXYZ, pcl::PointXYZ> savedPairs;
                savedPairs.first = curLeftPoint;
                savedPairs.second = curRightPoint;
                smoothPoints.push_back(savedPairs);
            }
            ofstream outputFile("./road_result.txt");
            for (auto i = 0; i < smoothPoints.size() - 1; i++) {
                outputFile << smoothPoints[i].first.x << " " <<smoothPoints[i].second.x << endl;
                viewer->addLine<pcl::PointXYZ>(smoothPoints[i].first, smoothPoints[i + 1].first,
                                               "line" + to_string(count++));
                viewer->addLine<pcl::PointXYZ>(savedCenters[i], savedCenters[i+1],
                                               "line" + to_string(count++));
                viewer->addLine<pcl::PointXYZ>(smoothPoints[i].second, smoothPoints[i + 1].second,
                                               "line" + to_string(count++));
            }
            viewer->addCoordinateSystem(1.0);
            viewer->initCameraParameters();
            while (!viewer->wasStopped()) {
                viewer->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            }
            viewer->close();
            break;
        }
    }
}