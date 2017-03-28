#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <time.h>

//#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>


#include <unordered_map>


#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transformation_validation_euclidean.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transformation_validation_euclidean.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <pcl/search/kdtree.h>

#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>
#include <pcl-1.8/pcl/common/eigen.h>
#include <pcl-1.8/pcl/registration/transformation_validation.h>


#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>





#include "CeresICP.h"
#include "utils.h"
#include "ModelRepresentation.h"
#include "LidarPointCloudHandler.h"

const bool IS_DEBUG = false;
typedef  pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPointer;
typedef pcl::PointXYZI PointType;




void findAllObject(CloudPointer laserCloudSrc, CloudPointer laserCloudRef, std::vector<float> transformMatrix,
                   std::string folderName){
    // Transform to raw if the transform is given
    if(transformMatrix.size()>0) {
        utils::pointCloudMapToRaw(laserCloudSrc, transformMatrix);
        utils::pointCloudMapToRaw(laserCloudRef, transformMatrix);
    }
    std::unique_ptr<LidarPointCloudHandler> cloudHandler(new LidarPointCloudHandler(laserCloudSrc));
    cloudHandler->setReferenceCloud(laserCloudRef);
    CloudPointerRGB cloudIntegration = cloudHandler->pointCloudDecomposition();

    utils::myPCLViewer::getInstance()->changePointCloud(cloudIntegration);
    // Save the result
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
//            if(!utils::checkWithEffectiveRegion(newPoint, transformMatrix)){
//                continue;
//            }
        }
        else{
            if(!utils::checkWithPriorInfo(newPoint)){
                continue;
            }
        }
        if(curPoint.r == 255){
            if(true) {  // Check with some prior info here
                staticObject->points.push_back(newPoint);   // Notice
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
                //if(utils::checkWithEffectiveRegion(newPoint))//, transformMatrix))
                    movingObject->points.push_back(newPoint);
            }
        }
    }
    utils::pointCloudRawToMap(movingObject, transformMatrix);
    utils::pointCloudRawToMap(staticObject, transformMatrix);
    // save all the objects into a folder
    //std::string SaveFolderPrefix = "/home/lifan/result_all/" + folderName;
    //boost::filesystem::path dir(SaveFolderPrefix);
    //boost::filesystem::create_directory(dir);
    std::string SaveFolderPrefix = "/home/lifan/LIDAR/result_all/";
    //SaveFolderPrefix = SaveFolderPrefix + '/';
    //std::string groundFileName = folderName + "_ground";
    std::string movingFileName = folderName + "_moving";
    std::string staticFileName = folderName + "_static";
    //std::string curbFileName = folderName + "_curb";
    //utils::pclPointCloudWriter(SaveFolderPrefix, groundFileName, groundObject);
    utils::pclPointCloudWriter(SaveFolderPrefix, movingFileName, movingObject);
    utils::pclPointCloudWriter(SaveFolderPrefix, staticFileName, staticObject);
    //utils::pclPointCloudWriter(SaveFolderPrefix, curbFileName, curbObject);

}

std::vector<std::pair<int, int>> clusterMatchingICP(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters1,
                                                    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters2)
{
    // parameters
    int pointNumThreshold = 100;  // a cloud contains less points will be discarded.
    double scoreThreshold = 0.0;  // matches under this will be discarded.
    int maxIterNum = 100000;
    double distanceThreshold = 5;

    int clusterNum1 = clusters1.size();
    int clusterNum2 = clusters2.size();

    std::vector<std::pair<int, int>> matches;

    // create ICP and config
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // Set the max correspondence distance to 5m
    icp.setMaxCorrespondenceDistance(3); // 5

    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(1000000);   // 100000  //1000000

    // Set the transformation epsilon (criterion 2)
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon(0.01);   // 3

    std::set<int> unmatched;
    std::vector<double> distVec;
    for (int i = 0; i < clusterNum2; i++){
        unmatched.insert(i);
    }

    for (int i = 0; i < clusterNum1; i++){
        std::cout<< i <<endl;
        icp.setInputSource(clusters1[i]);
        double maxScore = 0;
        int bestMatch = -1;
        for (int j = 0; j< clusterNum2; j++){
            if (unmatched.find(j) == unmatched.end()){        //  first come, first use rule. may use a global optima later
                continue;
            }
            icp.setInputTarget(clusters2[j]);
            pcl::PointCloud<pcl::PointXYZ> Final;
            icp.align(Final);
            bool isConverge = icp.hasConverged();
            if (!isConverge){
                continue;
            }
            double score = icp.getFitnessScore();
            std::cout << "has converged:" << isConverge << " score: " <<
                      score << std::endl;
            std::cout << icp.getFinalTransformation() << std::endl;
            if (score > maxScore and score >= scoreThreshold){     // if current score is larger and is over the threshold, save it
//                pcl::PointCloud<pcl::PointXYZ>::Ptr transformed = utils::transformCloudFromMatrix(icp.getFinalTransformation(), clusters1[i]);
//                utils::pairwiseVisualizeCloud(transformed, clusters2[j]);
                maxScore = score;
                bestMatch = j;
            }
        }
        if (bestMatch == -1){   // no valid match found
            continue;
        }
        unmatched.erase(bestMatch);   // remove from unmatched clusters
        matches.push_back(std::make_pair(i, bestMatch));
        double distance = utils::calcDistanceBetweenClouds(*clusters1[i], *clusters2[bestMatch]);
        distVec.push_back(distance);
        std::cout << i<< "  " << bestMatch <<endl;
    }

    // show all matches
//    for(int i =0; i<matches.size(); i++){
//        const std::string current = "Match " +  std::to_string(matches[i].first) + " on " + std::to_string(matches[i].second) +
//                                    " with distance of " + std::to_string(distVec[i]);
//        cout << current << endl;
//        utils::pairwiseVisualizeCloud(clusters1[matches[i].first], clusters2[matches[i].second], current);
//    }
    return matches;

}


std::vector<std::pair<int, int>> clusterMatchingICP(std::string pointCloudPath1,
                                                    int clusterNum1, std::string pointCloudPath2, int clusterNum2)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters1;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters2;
    std::vector<std::pair<int, int>> matches;

    clusters1 = utils::loadCloudsFromDirectory(pointCloudPath1, clusterNum1);
    clusters2 = utils::loadCloudsFromDirectory(pointCloudPath2, clusterNum2);
    return clusterMatchingICP(clusters1, clusters2);
}



// TOCHECK the threshold here
int clusterRematchingICP(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> templates,
                         std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> targets){
    // create ICP and config
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(3);
    icp.setMaximumIterations(1000000);
    icp.setEuclideanFitnessEpsilon(0.01);
    double scoreThreshold = 0.0;



    icp.setInputSource(templates[templates.size()-1]);
    double maxScore = 0;
    int bestMatch = -1;
    for (int j = 0; j< targets.size(); j++){
        icp.setInputTarget(targets[j]);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);
        bool isConverge = icp.hasConverged();
        if (!isConverge){
            continue;
        }
        double score = icp.getFitnessScore();
        std::cout << "has converged:" << isConverge << " score: " <<
                  score << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
        if (score > maxScore and score >= scoreThreshold){     // if current score is larger and is over the threshold, save it
//                pcl::PointCloud<pcl::PointXYZ>::Ptr transformed = utils::transformCloudFromMatrix(icp.getFinalTransformation(), clusters1[i]);
//                utils::pairwiseVisualizeCloud(transformed, clusters2[j]);
            maxScore = score;
            bestMatch = j;
        }
    }
    return bestMatch;

}



// do multi-frame matching among a list of clouds, based on the first frame
// assume all cluster in first frame is valid (may not necessary)
// visualize in the same window
void multiFrameMatching(std::string pointCloudPath, int cloudNum){
    std::string folderSuffix = "cloud";
    std::string clusterPrefix = "cloud_cluster_";
    std::string clusterSuffix = ".pcd";
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>> results;
    std::vector<int> groups;
    std::string templateFolder = pointCloudPath + folderSuffix + std::to_string(1) + '/';
    for(int _begin = 1;_begin < cloudNum; _begin ++){
        std::string curFolder = pointCloudPath + folderSuffix + std::to_string(_begin) + '/';
        std::string nextFolder = pointCloudPath + folderSuffix + std::to_string(_begin+1) + '/';
        int curClusterNum = utils::getFileCountInFolder(curFolder)-1;   // -1 for the viewer.sh in folder
        int nextClusterNum = utils::getFileCountInFolder(nextFolder)-1;
        if(_begin == 1){   // create the vector to hold each cloud in the first frame
            groups.resize(curClusterNum);
            for(int i=0; i<curClusterNum; i++){
                std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> curClouds;
                std::string curPath = curFolder + clusterPrefix + std::to_string(i) + clusterSuffix;
                pcl::PointCloud<pcl::PointXYZ>::Ptr curCloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PCLPointCloud2 cloud_blob;
                pcl::io::loadPCDFile(curPath.c_str(), cloud_blob);
                pcl::fromPCLPointCloud2(cloud_blob, *curCloud); // convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>
                curClouds.push_back(curCloud->makeShared());
                results.push_back(curClouds);
                groups[i] = i;
            }
        }
        std::vector<std::pair<int, int>> matches = clusterMatchingICP(curFolder, curClusterNum, nextFolder, nextClusterNum);
        for(int i=0; i<groups.size(); i++){
            bool isFound = false;
            if(groups[i] == -1){   // to help unmatched clusters

            }
            for (int j=0; j<matches.size(); j++){
                if(matches[j].first == groups[i]){
                    groups[i] = matches[j].second;
                    isFound = true;
                    break;
                }
            }
            if(!isFound){
                groups[i] = -1;
            }
        }
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> nextClusters = utils::loadCloudsFromDirectory(nextFolder, nextClusterNum);
        for(int i=0; i<groups.size(); i++){
            if(groups[i] == -1)
                continue;
            results[i].push_back(nextClusters[groups[i]]);
        }
        cout << "Cur Group is: " << groups[0] << endl;
    }
    for(int i=0; i<results.size(); i++){
        cout << "Current showing cluster " << i << " The size is " << results[i].size() << endl;
        utils::multiVisualizeCloud(results[i]);
    }


}



void curbFilter(std::string pointCloudPath){
    CloudPointer curCloud = utils::loadCloudFromPath(pointCloudPath);
    CloudPointer outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<std::vector<int>> pointIndices;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    std::set<int> inliersSet;
    pointIndices = utils::voxelizePointCloud(curCloud);
    vector<double> groundCoeffi = {0, 1, 0};
    vector<double> frontCoeffi = {0, 0, 1};
    for(int i = 0; i < pointIndices.size(); i++){
        CloudPointer vCloud(new pcl::PointCloud<pcl::PointXYZ>);
        for(int j = 0; j < pointIndices[i].size(); j++){
            vCloud->points.push_back(curCloud->points[pointIndices[i][j]]);
        }
        // utils::visualizeCloud(vCloud);
        CurbModel *cModel = new CurbModel();
        cModel->setInputPointCloud(vCloud);
        cModel->setGroundCoeffi(groundCoeffi);
        cModel->setFrontCoeffi(frontCoeffi);
        if(cModel->validateModel()){
            for(int j = 0; j < pointIndices[i].size(); j++){
                inliersSet.insert(pointIndices[i][j]);
            }
        }
    }
    std::set<int>::iterator it;
    for(it = inliersSet.begin(); it != inliersSet.end(); ++it){
        inliers->indices.push_back(*it);
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(curCloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*outputCloud);
    utils::visualizeCloud(outputCloud);
}


void staticCloudPointMatch(CloudPointer laserCloudSrc, CloudPointer laserCloudRef, cv::Mat image, vector<float> transform, std::string pairName){
    pcl::search::KdTree<pcl::PointXYZ> KdTree;
    KdTree.setInputCloud(laserCloudRef->makeShared());
    ofstream outputFile("/home/lifan/LIDAR/workspace/corr/" + pairName + "-points.txt");


    //part for transform
    cv::Mat rotation(3, 3, CV_64F);
    cv::Mat translation(3, 1, CV_64F);
    cv::Mat K(3, 3, CV_64F);
    cv::Mat H(3, 4, CV_64F);

    string transformPath = "/home/lifan/camera-lidar-test/calib_cam_to_range_00.txt";
    string KPath = "/home/lifan/camera-lidar-test/calib_cam_to_cam.txt";

    utils::loadKFromTXT(KPath, K);

    cv::Mat P(4, 4, CV_64F);
    utils::loadTransformFromTXT(transformPath, P);
    P.at<double>(3, 0) = 0;
    P.at<double>(3, 1) = 0;
    P.at<double>(3, 2) = 0;
    P.at<double>(3, 3) = 1;
    cout << P <<endl;
    cout << K <<endl;
    P = P.inv();

    vector<cv::Point2f> outputPoints;

    for(auto point:laserCloudSrc->points){
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);

        KdTree.nearestKSearch(point, 1, indices, sqr_distances);
        pcl::PointXYZ matchPoint = laserCloudRef->points[indices[0]];

        pcl::PointXYZ pointCamera = utils::pointAssociateTobeMapped(point, transform);
        pcl::PointXYZ matchCamera = utils::pointAssociateTobeMapped(matchPoint, transform);
        float temp = pointCamera.x;
        pointCamera.x = pointCamera.z;
        pointCamera.z = pointCamera.y;
        pointCamera.y = temp;

        temp = matchCamera.x;
        matchCamera.x = matchCamera.z;
        matchCamera.z = matchCamera.y;
        matchCamera.y = temp;


        cv::Point2f pointDehomo;
        if(!utils::transformLidarPointToCamera(pointCamera, pointDehomo, K, P))
            continue;

        outputPoints.push_back(pointDehomo);

        cv::Point2f matchPointDehomo;
        utils::transformLidarPointToCamera(matchCamera, matchPointDehomo, K, P);


        // For output
        outputFile << point.x << " " <<point.y << " " << point.z << " "
                   << matchPoint.x << " " << matchPoint.y << " " << matchPoint.z << endl;
        outputFile << pointDehomo.x << " "<< pointDehomo.y << " "  <<  matchPointDehomo.x << " "  << matchPointDehomo.y << endl;
    }
    outputFile.close();


//    for (int i = 0; i < outputPoints.size(); i++) {
//
//             if (outputPoints[i].x > 0 && outputPoints[i].x < image.cols && outputPoints[i].y > 0 && outputPoints[i].y < image.rows) {
//                cv::circle(image, outputPoints[i], 1, cv::Scalar(0, 255, 0), 1);
//            }
//        }
//        cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);// Create a window for display.
//        cv::imshow("Display window", image);                   // Show our image inside it.
//        cv::waitKey(0);
}


void movingCloudClusterMatch(CloudPointer laserCloudSrc, CloudPointer laserCloudRef, cv::Mat image, vector<float> transform, std::string pairName){
    ofstream outputFile("/home/lifan/LIDAR/workspace/corr/" + pairName + "-clusters.txt");

    //part for transform
    cv::Mat rotation(3, 3, CV_64F);
    cv::Mat translation(3, 1, CV_64F);
    cv::Mat K(3, 3, CV_64F);
    cv::Mat H(3, 4, CV_64F);

    string transformPath = "/home/lifan/camera-lidar-test/calib_cam_to_range_00.txt";
    string KPath = "/home/lifan/camera-lidar-test/calib_cam_to_cam.txt";

    utils::loadKFromTXT(KPath, K);

    cv::Mat P(4, 4, CV_64F);
    utils::loadTransformFromTXT(transformPath, P);
    P.at<double>(3, 0) = 0;
    P.at<double>(3, 1) = 0;
    P.at<double>(3, 2) = 0;
    P.at<double>(3, 3) = 1;
    cout << P <<endl;
    cout << K <<endl;
    P = P.inv();

    vector<cv::Point2f> outputPoints;

    std::vector<CloudPointer> srcClouds;
    std::vector<CloudPointer> refClouds;

    utils::getVehicleClusterFromCloud(laserCloudSrc, srcClouds);
    utils::getVehicleClusterFromCloud(laserCloudRef, refClouds);

    std::vector<std::pair<int, int>> matches = clusterMatchingICP(srcClouds, refClouds);
    for(auto match:matches){
        CloudPointer srcCloud = srcClouds[match.first];
        CloudPointer refCloud = refClouds[match.second];
        outputFile << "{" << endl;
        outputFile << "[" << endl;
        for(auto point:srcCloud->points){

            pcl::PointXYZ pointCamera = utils::pointAssociateTobeMapped(point, transform);
            float temp = pointCamera.x;
            pointCamera.x = pointCamera.z;
            pointCamera.z = pointCamera.y;
            pointCamera.y = temp;

            cv::Point2f pointDehomo;
            if(!utils::transformLidarPointToCamera(pointCamera, pointDehomo, K, P))
                continue;

            outputPoints.push_back(pointDehomo);

            // For output
            outputFile << point.x << " " <<point.y << " " << point.z << endl;
            outputFile << pointDehomo.x << " " << pointDehomo.y << endl;
        }
        outputFile << "]" << endl;
        outputFile << "[" << endl;
        for(auto point:refCloud->points){

            pcl::PointXYZ pointCamera = utils::pointAssociateTobeMapped(point, transform);
            float temp = pointCamera.x;
            pointCamera.x = pointCamera.z;
            pointCamera.z = pointCamera.y;
            pointCamera.y = temp;

            cv::Point2f pointDehomo;
            if(!utils::transformLidarPointToCamera(pointCamera, pointDehomo, K, P))
                continue;

            outputPoints.push_back(pointDehomo);

            // For output
            outputFile << point.x << " " <<point.y << " " << point.z << endl;
            outputFile << pointDehomo.x << " " << pointDehomo.y << endl;
        }
        outputFile << "]" << endl;
        outputFile << "}" << endl;



    }
//    for (int i = 0; i < outputPoints.size(); i++) {
//
//        if (outputPoints[i].x > 0 && outputPoints[i].x < image.cols && outputPoints[i].y > 0 && outputPoints[i].y < image.rows) {
//            cv::circle(image, outputPoints[i], 1, cv::Scalar(0, 255, 0), 1);
//        }
//    }
//    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);// Create a window for display.
//    cv::imshow("Display window", image);                   // Show our image inside it.
//    cv::waitKey(0);
}




void imageLidarVisualizeAll(CloudPointer laserCloudSrc, cv::Mat image, vector<float> transform){
    //part for transform
    cv::Mat rotation(3, 3, CV_64F);
    cv::Mat translation(3, 1, CV_64F);
    cv::Mat K(3, 3, CV_64F);
    cv::Mat H(3, 4, CV_64F);
    cv::Mat D(1, 5, CV_64F);

    string transformPath = "/home/lifan/camera-lidar-test/calib_cam_to_range_00.txt";
    string KPath = "/home/lifan/camera-lidar-test/calib_cam_to_cam.txt";
//    string transformPath = "/home/lifan/camera-lidar-test/old_params/2/calib_cam_to_range_00.txt";
//    string KPath = "/home/lifan/camera-lidar-test/old_params/2/calib_cam_to_cam.txt";

    utils::loadKFromTXT(KPath, K);
    utils::loadDFromTXT(KPath, D);
    cv::Mat P(4, 4, CV_64F);
    utils::loadTransformFromTXT(transformPath, P);
    P.at<double>(3, 0) = 0;
    P.at<double>(3, 1) = 0;
    P.at<double>(3, 2) = 0;
    P.at<double>(3, 3) = 1;
    cout << P <<endl;
    cout << K <<endl;
    cout << D << endl;
    P = P.inv();
    vector<cv::Point2f> outputPoints;
    vector<double> distance;
    cv::Mat rvec;
    cv::Rodrigues(P(cv::Range(0, 3), cv::Range(0, 3)), rvec);
    for(auto point:laserCloudSrc->points){

        pcl::PointXYZ pointCamera = utils::pointAssociateTobeMapped(point, transform);

        float temp = pointCamera.x;
        pointCamera.x = pointCamera.z;
        pointCamera.z = pointCamera.y;
        pointCamera.y = temp;

        cv::Point2f pointDehomo;
        cv::Point3f pointCamera_new(pointCamera.x, pointCamera.y, pointCamera.z);
        vector<cv::Point3f> input;
        vector<cv::Point2f> output;
        input.push_back(pointCamera_new);
        if(!utils::transformLidarPointToCamera(pointCamera, pointDehomo, K, P))
            continue;

//        cv::projectPoints(input,  rvec, P(cv::Range(0, 3), cv::Range(3, 4)), K,  D, output);
//        utils::correctDistortion(pointDehomo, D);
//        outputPoints.push_back(output[0]);
        distance.push_back((pointCamera_new.x * pointCamera_new.x) + (pointCamera_new.y * pointCamera_new.y) + (pointCamera_new.z
                                                                                                               * pointCamera_new.z));
        outputPoints.push_back(pointDehomo);

    }
    double maxDistance = *max_element(distance.begin(), distance.end());
    for (int i = 0; i < outputPoints.size(); i++) {

        if (outputPoints[i].x > 0 && outputPoints[i].x < image.cols && outputPoints[i].y > 0 && outputPoints[i].y < image.rows) {
            cv::circle(image, outputPoints[i], 1, cv::Scalar(0, 255 - 255 * min((distance[i] / 350), 1.0), 0), 1);
        }
    }
    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);// Create a window for display.
    cv::imshow("Display window", image);                   // Show our image inside it.
    cv::waitKey(0);

}


int main(int argc, char** argv)
{
    const int mode = 3;
//    int mode = std::atoi(argv[1]);
//    long startString = std::atol(argv[2]);

    long start = 148882466705;
    const int buffer = 4;
    const int deviation = 40;
    utils::myPCLViewer::createInstance();
    for(int i = 0; i < 2300; i++) {
        long startString = start + 5 * i;
        std::string cloudDir = "/home/lifan/LIDAR/workspace/6/mappoints/";
        std::string fileSuffix = ".pcd";
//        std::string cloudDir = "/home/lifan/LIDAR/result_all/";
//        std::string fileSuffix = "_moving.pcd";

        std::string imageDir = "/home/lifan/LIDAR/workspace/6/imgs/";
//        std::string cloudDir = "/home/lifan/LIDAR/workspace/20170228/mappoints/";
//        std::string imageDir = "/home/lifan/LIDAR/workspace/20170228/image-lidar/";


        std::string srcCloudNumber = utils::convertTimeStampToFileString(startString);
        std::string imageNumber = utils::convertTimeStampToFileString(startString);
        std::string refCloudNumber = utils::convertTimeStampToFileString(startString + buffer * deviation);

        std::string srcPointCloudPathString = cloudDir + srcCloudNumber + fileSuffix;
        std::string refPointCloudPathString = cloudDir + refCloudNumber + fileSuffix;
        std::string srcTransform = cloudDir + srcCloudNumber + ".txt";
        CloudPointer laserCloudSrc = utils::loadCloudFromPath(srcPointCloudPathString);
        CloudPointer laserCloudRef = utils::loadCloudFromPath(refPointCloudPathString);
        if(laserCloudSrc->points.size() == 0){
            start = start + 1;
            startString = start + 5 * i;
            srcCloudNumber = utils::convertTimeStampToFileString(startString);
            srcPointCloudPathString = cloudDir + srcCloudNumber + fileSuffix;
            srcTransform = cloudDir + srcCloudNumber + ".txt";
            laserCloudSrc = utils::loadCloudFromPath(srcPointCloudPathString);

            if(laserCloudSrc->points.size() == 0){
                start = start - 1;
                continue;
            }

            refCloudNumber = utils::convertTimeStampToFileString(startString + buffer * deviation);
            refPointCloudPathString = cloudDir + refCloudNumber + fileSuffix;
            laserCloudRef = utils::loadCloudFromPath(refPointCloudPathString);
        }
        if(laserCloudRef->points.size() == 0){
            refCloudNumber = utils::convertTimeStampToFileString(startString + 1 + buffer * deviation);
            refPointCloudPathString = cloudDir + refCloudNumber + fileSuffix;
            laserCloudRef = utils::loadCloudFromPath(refPointCloudPathString);
        }
        if(laserCloudRef->points.size() == 0){
            continue;
        }
        vector<float> transform = utils::loadTransformFromTXT(srcTransform);
        std::string imagePathString = imageDir + srcCloudNumber + ".jpg";
        cout << "Finish Loading " << endl;
        cout << srcPointCloudPathString << endl;
        cout << refPointCloudPathString << endl;
        switch(mode){
            case 1: {
                cv::Mat image = cv::imread(imagePathString, CV_LOAD_IMAGE_COLOR);
                movingCloudClusterMatch(laserCloudSrc, laserCloudRef, image, transform, srcCloudNumber);
            }
                break;
            case 2: {
                cv::Mat image = cv::imread(imagePathString, CV_LOAD_IMAGE_COLOR);
                if(image.cols == 0)
                    continue;
                imageLidarVisualizeAll(laserCloudSrc, image, transform);
            }
                break;
            case 3: {
                findAllObject(laserCloudSrc, laserCloudRef, transform, srcCloudNumber);
            }
                break;
            case 4: {
                cv::Mat image = cv::imread(imagePathString, CV_LOAD_IMAGE_COLOR);
                staticCloudPointMatch(laserCloudSrc, laserCloudRef, image, transform, srcCloudNumber);
            }
                break;
            // TEST HERE
            case 5: {
//                utils::pointCloudMapToRaw(laserCloudSrc, transform);
//                std::unique_ptr<LidarPointCloudHandler> cloudHandler(new LidarPointCloudHandler(laserCloudSrc));
//
//                cloudHandler->removeGroundPlane();
//                //utils::myPCLViewer::getInstance()->changePointCloud(cloudHandler->getSourcePointCloud());
//                std::vector<CloudPointer> res = cloudHandler->pointCloudSegmentation();
//                utils::myPCLViewer::getInstance()->changeMultiPointCloud(res);
            }
                break;
            default:break;
        }
    }
    return 0;
}
