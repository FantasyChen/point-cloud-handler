//
// Created by lifan on 3/15/17.
//


#include "utils.h"
#include "LidarPointCloudHandler.h"
#include <json/json.h>

using namespace std;

int main(){
    const bool isVisualize = true;

    Json::Value config = utils::parseJsonFile(globalConfigPath);


    string workingDir = config["working_directory"].asString();
    vector<string> fileNames = utils::findFilesInFolderWithSuffix(workingDir, ".pcd");
    sort(fileNames.begin(), fileNames.end());
    string transformPath = config["camera_lidar_P_path"].asString();
    string KPath = config["camera_K_path"].asString();
//    string transformPath = "/home/lifan/camera-lidar-test/calib_cam_to_range_00.txt";
//    string KPath = "/home/lifan/camera-lidar-test/calib_cam_to_cam.txt";

    // multi-frame
    int count = -1;
    for(auto fileName:fileNames) {
        count = (count + 1) % 20;
        if(count > 5) continue;
        string imagePath = workingDir + fileName + ".jpg";   // may change to "jpg"
        string pointCloudPath = workingDir + fileName + ".pcd";
        string bboxPath = workingDir + fileName + ".txt";
        ofstream outputFile(workingDir + "json/" + fileName + ".json");

        // temp variable initialized here
        cv::Mat image = cv::imread(imagePath, CV_LOAD_IMAGE_COLOR);
        vector<utils::ImageBoundingBox> iVec;
        utils::loadImageBoxFromTXT(iVec, bboxPath);
        CloudPointer cloud = utils::loadCloudFromPath(pointCloudPath);
        unique_ptr<LidarPointCloudHandler> cHandler(new LidarPointCloudHandler(cloud));
        cHandler->removeGroundPlane();
        cHandler->setImageCalibration(transformPath, KPath);
        vector<CloudPointer> segs = cHandler->pointCloudSegmentation();
        vector<CloudPointer> result;
        vector<utils::ImageBoundingBox> savedBox;



        for (int i = 0; i < iVec.size(); i++) {
            float minDist = INT_MAX;
            CloudPointer savedSeg;
            for (auto seg:segs) {
                int numPoints = int(seg->points.size());
                CloudPointer inlier = cHandler->getPointsInsideImageBoundingBox(iVec[i], seg);
                int numInliers = int(inlier->points.size());
                utils::BoundingBox vehicleBox(seg);
                if ((numInliers > 20 || (numInliers * 1.0 / numPoints > 0.4 && numInliers > 5))) {
                    pcl::search::KdTree<pcl::PointXYZ> kdTree;
                    kdTree.setInputCloud(seg->makeShared());
                    std::vector<int> indices(1);
                    std::vector<float> sqr_distances(1);
                    kdTree.nearestKSearch(pcl::PointXYZ(0, 0, 0), 1, indices, sqr_distances);
                    if(sqr_distances[0] < minDist && sqr_distances[0] > 2) {    // Remove the noise close around
                        savedSeg = seg->makeShared();
                        minDist = sqr_distances[0];
                    }
                }
            }
            if(savedSeg){
                result.push_back(savedSeg);
                savedBox.push_back(iVec[i]);
            }
        }
        if(isVisualize){
            utils::myPCLViewer::getInstance()->changePointCloud(cloud);
            utils::myPCLViewer::getInstance()->changeMultiPointCloud(segs);
            utils::myPCLViewer::getInstance()->changeMultiPointCloud(result);
            for (int i = 0; i < iVec.size(); i++) {
                cv::rectangle(image, cv::Point2d(iVec[i].getMinX(), iVec[i].getMinY()),
                              cv::Point2d(iVec[i].getMaxX(), iVec[i].getMaxY()), (0, 0, 255));
            }
            cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);// Create a window for display.
            cv::imshow("Display window", image);                   // Show our image inside it.
            cv::waitKey(0);
            cv::destroyWindow("Display window");
        }



        Json::Value arrayRoot;
        for (int i = 0; i < result.size(); i++) {
            Json::Value root;
            root["left"] = savedBox[i].getMinX();
            root["right"] = savedBox[i].getMaxX();
            root["top"] = savedBox[i].getMaxY();
            root["bottom"] = savedBox[i].getMinY();
            Json::Value cluster;
            for (auto point:result[i]->points) {
                Json::Value curPoint;
                curPoint["x"] = point.x;
                curPoint["y"] = point.y;
                curPoint["z"] = point.z;
                cluster.append(curPoint);
            }
            root["cluster"] = cluster;
            arrayRoot.append(root);
        }

        outputFile << arrayRoot << endl;
        cout << "finish writing to json" << endl;
        outputFile.close();
    }

}