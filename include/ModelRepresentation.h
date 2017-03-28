//
// Created by lifan on 2/9/17.
//

#ifndef POINTS_GROUPING_MODELREPRESENTATION_H
#define POINTS_GROUPING_MODELREPRESENTATION_H




#include <opencv2/opencv.hpp>





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

#include <dirent.h>




using namespace std;
typedef  pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPointer;
typedef  pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudPointerRGB;

// Base class for all representations
class ModelRepresentation {

public:
    ModelRepresentation();
    virtual ~ModelRepresentation();
    virtual vector<double> getRepresentation();
    virtual void setInputPointCloud(CloudPointer cloud);

protected:
    vector<double> _coeffi;
    CloudPointer _cloud;

};





class CurbModel: public ModelRepresentation{
public:
    CurbModel();
    void calcParameters();
    bool validateModel();
    void setGroundCoeffi(vector<double> coeffi);
    void setFrontCoeffi(vector<double> coeffi);


private:
    bool _isParameterEstimated;
    double _angleThreshold;
    vector<double> _groundCoeffi;
    vector<double> _frontCoeffi;


};


/**
 * Two Line Model for global curb fitting
 * RANSAC Searching Method
 */
class TwoLineModel: public ModelRepresentation{
public:
    TwoLineModel();
    void setInitialGuess(double a, double b, double c, double cprime);
    void BFSearching();
    void RANSACSearching();
    vector<double> getRepresentation();


private:
    vector<vector<double>> getLineModelByThreePoints(vector<pcl::PointXYZ> samplePoints);

    // representation :{a, b, c, c'}, where ax + bz + c = 0, ax + bz + c' = 0.
    vector<double> _representation;
    bool _isInitialSet;
};


/**
 * Rectangle Model for global road fitting
 * Brute-Force Searching Method
 */
class RectangleModel: public ModelRepresentation{
public:
    RectangleModel();
    vector<double> getRepresentation();
    void setInitialGuess(double a, double b, double c, double cprime);
    void BFSearching();
    void setInputPointCloud(CloudPointerRGB cloud);
    /**
    *
    * @param point
    * @param label: True for road points and false for other points
    * @param a Line parameters
    * @param b
    * @param c
    * @param cprime
    * @return
    */
    double getPointCost(pcl::PointXYZRGB point, bool label, double a, double b, double c, double cprime);

private:
    vector<double> _representation;  // representation :{a, b, c, c'}, where ax + bz + c = 0, ax + bz + c' = 0.
    bool _isInitialSet;
    CloudPointerRGB _cloud;    // RGB Point cloud is used as labeled data
    double getConfidenceCoefficient(pcl::PointXYZRGB point);




};




#endif //POINTS_GROUPING_MODELREPRESENTATION_H
