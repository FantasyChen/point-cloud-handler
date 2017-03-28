//
// Created by lifan on 3/10/17.
//

#ifndef POINTS_GROUPING_LIDARPOINTCLOUDHANDLER_H
#define POINTS_GROUPING_LIDARPOINTCLOUDHANDLER_H

#include "utils.h"
#include "ModelRepresentation.h"

class LidarPointCloudHandler {
public:
    LidarPointCloudHandler(CloudPointer srcCloud);
    virtual ~LidarPointCloudHandler();


    /**
     * Decomposite the source point cloud into ground, static, and moving object.
     * Both source cloud and reference cloud should be given.
     * @return The RGB point color using color to indicate the label
     */
    CloudPointerRGB pointCloudDecomposition();
    /**
     * Return a point cloud contains all the points inside given image bounding box
     * @param box: image bounding box
     * @return poing cloud type
     */
    CloudPointer getPointsInsideImageBoundingBox(utils::ImageBoundingBox box);
    CloudPointer getPointsInsideImageBoundingBox(utils::ImageBoundingBox box, CloudPointer cloud);
    void convertImageBoxToLidarBox(utils::ImageBoundingBox box);
    void removeGroundPlane();
    vector<CloudPointer> pointCloudSegmentation();

    /*----- Set parameters -----*/
    void setReferenceCloud(CloudPointer refCloud);
    void setImageCalibration(string transformPath, string KPath);
    void setSourceCLoud(CloudPointer srcCloud);

    /*----- Get parameters -----*/
    CloudPointer getSourcePointCloud();

private:
    CloudPointer _srcCloud;  // necessary
    CloudPointer _refCloud;  // necessary if moving object need to take into account
    cv::Mat _K;  //camera calibration
    cv::Mat _P;  // Lidar to image projective matrix
    bool _isCalibrated;
};


#endif //POINTS_GROUPING_LIDARPOINTCLOUDHANDLER_H
