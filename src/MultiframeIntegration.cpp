//
// Created by lifan on 3/3/17.
//

#include "utils.h"


using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPointer;




int main() {
    long start = 148830671549;
    const int buffer = 5;
    const int integrationSize = 5;
    for (int i = 0; i < 200; i++) {
        long startString = start + 5 * i;
        std::string cloudDir = "/home/lifan/LIDAR/workspace/20170228/mappoints/";
        std::string fileSuffix = "_FullRes.pcd";
        vector<CloudPointer> cloudVec;
        for(int j = 0; j < integrationSize; j++){
            long currentNumber = startString + j * buffer;
            std::string srcCloudNumber = utils::convertTimeStampToFileString(currentNumber);
            std::string srcPointCloudPathString = cloudDir + srcCloudNumber + fileSuffix;
            CloudPointer laserCloudSrc = utils::loadCloudFromPath(srcPointCloudPathString);
            if(laserCloudSrc->points.size() == 0) {
                if(j == 0){
                    start ++;
                    startString = start + 5 * i;
                }
                else{
                    startString = startString + 1;
                }
                currentNumber = startString + j * buffer;
                srcCloudNumber = utils::convertTimeStampToFileString(currentNumber);
                srcPointCloudPathString = cloudDir + srcCloudNumber + fileSuffix;
                laserCloudSrc = utils::loadCloudFromPath(srcPointCloudPathString);
                if(laserCloudSrc->points.size() == 0){
                    if(j == 0 ) {
                        start = start - 1;
                        break;
                    }
                    else {
                        startString = startString - 1;
                        break;
                    }
                }

            }
            cloudVec.push_back(laserCloudSrc->makeShared());
        }
            if(cloudVec.size() > 0){
            std::string prefix = utils::convertTimeStampToFileString(startString);
            utils::multiframeIntegration(cloudVec, prefix);
        }
    }
}
