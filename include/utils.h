//
// Created by Lifan on 1/25/17.
//

#ifndef POINTS_GROUPING_UTILS_H
#define POINTS_GROUPING_UTILS_H


//#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <json/json.h>


// Eigens
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>



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

/*
 * Utilities of PCL point cloud library and basic load/manipulation/calculation/visualization/writing of point cloud files
 */
const static std::string globalConfigPath = "../config.json";
namespace utils{

    typedef  pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPointer;
    typedef  pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudPointerRGB;
    using namespace std;


    /*
     * - Some class definition
     */
    /**
     * The Singleton class designed for visualizer
     * Only keep one copy in the global (one single viusalize window)
     * Input/change the point cloud by calling changePointCloud()
     */
    class myPCLViewer: public pcl::visualization::PCLVisualizer{
    public:
        static boost::shared_ptr<myPCLViewer> getInstance(){
            if(!instance){
                createInstance();
            }
            return instance;
        }

        /*
         * Manually creation of the instance
         */
        static void createInstance(){
            if(instance) {
                std::cout << "Singleton has already been created" << std::endl;
            }
            else {
                boost::shared_ptr<myPCLViewer> temp(new myPCLViewer("3D Viewer"));
                instance = temp;
//                instance->setBackgroundColor(0, 0, 0);
//                instance->addCoordinateSystem(1.0);
//                instance->initCameraParameters();
            }
        }
        void changePointCloud(CloudPointer cloud, int viewport = 0){
            instance->resetStoppedFlag();
            instance->removeAllPointClouds();
            instance->setBackgroundColor(0, 0, 0);
            instance->addCoordinateSystem(1.0);
            instance->initCameraParameters();
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h (cloud, 0, 255, 0);
            instance->addPointCloud<pcl::PointXYZ>(cloud, tgt_h, "sample cloud", viewport);
            instance->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud", viewport);
            while (!instance->wasStopped())
            {
                instance->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            }
        }

        void changePointCloud(CloudPointerRGB cloud, int viewport = 0){
            instance->resetStoppedFlag();
            instance->removeAllPointClouds();
            instance->setBackgroundColor(0, 0, 0);
            instance->addCoordinateSystem(1.0);
            instance->initCameraParameters();
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> tgt_h (cloud);
            instance->addPointCloud<pcl::PointXYZRGB>(cloud, tgt_h, "sample cloud", viewport);
            instance->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud", viewport);
            while (!instance->wasStopped())
            {
                instance->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            }
        }

        void changeMultiPointCloud(vector<CloudPointer> clouds, int viewport = 0){
            int _size = clouds.size();
            instance->resetStoppedFlag();
            instance->removeAllPointClouds();
            instance->setBackgroundColor(0, 0, 0);
            instance->addCoordinateSystem(1.0);
            instance->initCameraParameters();
            for(int i=0; i<_size; i++) {
                pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> src_h(clouds[i]);
                instance->addPointCloud<pcl::PointXYZ>(clouds[i], src_h, "sample cloud" + std::to_string(i), viewport);
                instance->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,  "sample cloud" + std::to_string(i), viewport);

            }
            while (!instance->wasStopped())
            {
                instance->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            }
        }
        ~myPCLViewer(){};
    private:
        myPCLViewer(std::string name):pcl::visualization::PCLVisualizer(name){
        }
        static boost::shared_ptr<myPCLViewer> instance;

    };


    /**
     * 2D bounding box defined on image coordinate, with
     */
    class ImageBoundingBox{
    public:
        ImageBoundingBox(const double maxX, const double minX, const double maxY, const double minY, const double maxZ = 0, const double minZ = 0){
            _maxX = maxX;
            _maxY = maxY;
            _maxZ = maxZ;
            _minX = minX;
            _minY = minY;
            _minZ = minZ;
            _confi = 0;

        }
        bool isPointInBox(double qX, double qY){
            return qX <= _maxX && qX >= _minX && qY <= _maxY && qY >= _minY;
        }
        void setConfidence(const double confidence){
            _confi = confidence;
        }

        double getConfidence(){
            return _confi;
        }

        double getMaxX(){
            return _maxX;
        }

        double getMinX(){
            return _minX;
        }

        double getMaxY(){
            return _maxY;
        }

        double getMinY(){
            return _minY;
        }

    private:
        double _maxX, _minX, _maxY, _minY, _maxZ, _minZ, _confi;
    };


    class BoundingBox
    {
    public:
        float maxLengthOfBoundingBox = 10.0;
        float minLengthOfBoundingBox = 0.0; // 1.5
        BoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudInput)
        {
            maxX = -FLT_MAX;
            maxY = -FLT_MAX;
            maxZ = -FLT_MAX;
            minX = FLT_MAX;
            minY = FLT_MAX;
            minZ = FLT_MAX;
            calcuBoundingBox(laserCloudInput);

            // calculate maxLength
            isvalid = true;
            maxLength = maxX - minX;
            if(maxLength < (maxY - minY))
                maxLength = maxY - minY;
            if(maxLength < (maxZ - minZ))
                maxLength = maxZ - minZ;
            if(maxLength > maxLengthOfBoundingBox)
            {
                isvalid = false;
            }

            // calculate minLength
            minLength = maxX - minX;
            if(minLength < (maxY - minY))
                maxLength = maxY - minY;
            if(minLength < (maxZ - minZ))
                maxLength = maxZ - minZ;
            if(minLength < minLengthOfBoundingBox)
            {
                isvalid = false;
            }
        }
        void calcuBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudInput)
        {
            for (int i = 0; i < laserCloudInput->points.size(); ++i)
            {
                float x = laserCloudInput->points[i].x;
                float y = laserCloudInput->points[i].y;
                float z = laserCloudInput->points[i].z;
                if(x > maxX) maxX = x;
                if(y > maxY) maxY = y;
                if(z > maxZ) maxZ = z;
                if(x < minX) minX = x;
                if(y < minY) minY = y;
                if(z < minZ) minZ = z;
            }
        }
        bool checkIfContainedBy(BoundingBox& largerOne)
        {
            if(largerOne.minX > this->minX) return false;
            if(largerOne.minY > this->minY) return false;
            if(largerOne.minZ > this->minZ) return false;
            if(largerOne.maxX < this->maxX) return false;
            if(largerOne.maxY < this->maxY) return false;
            if(largerOne.maxZ < this->maxZ) return false;
            return true;
        }

        bool isvalid;
        float maxLength;
        float minLength;

    private:
        float maxX;
        float maxY;
        float maxZ;
        float minX;
        float minY;
        float minZ;
    };


    class GroundPlane
    {
    public:
        GroundPlane()
        {
            CloudPointer pointCloud(new pcl::PointCloud<pcl::PointXYZ> ());
            _pointCloud = pointCloud;
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            _coefficients = coefficients;
            maxX = -INT_MAX;
            maxZ = -INT_MAX;
            minX = INT_MAX;
            minZ = INT_MAX;
        }
        void addPoint(pcl::PointXYZ point){
            _pointCloud->push_back(point);
            if(point.x > maxX){
                maxX = point.x;
            }
            if(point.x < minX){
                minX = point.x;
            }
            if(point.z > maxZ){
                maxZ = point.z;
            }
            if(point.z < minZ){
                minZ = point.z;
            }
        }

        void setCoefficients(pcl::ModelCoefficients::Ptr coeffi){
            *_coefficients = *coeffi;
        }

        bool isPointInsideGroundPlane(pcl::PointXYZ point){
            double buffer = -10;
            if(point.x <= buffer + maxX && point.x >= minX - buffer &&  point.z <= buffer + maxZ && point.z >= minZ - buffer)
                return true;
            else
                return false;
        }

        void printSqure(){
            cout<< "The ground region is " << maxX << " " << minX << " " << maxZ << " " << minZ << " "<< endl;
        }


    private:
        CloudPointer _pointCloud;
        pcl::ModelCoefficients::Ptr _coefficients;
        float maxX;
        float maxZ;
        float minX;
        float minZ;
    };



    /**
     * Check if the width follows the rule of write before write to file.
     * @param dirPath
     * @param fileName
     * @param cloud
     */
    void pclPointCloudWriter(string &dirPath, string &fileName, CloudPointer cloud);


    /**
     * Calc point to plane distance. Given the coefficients of the plane
     * @param plane
     * @param point
     * @return
     */
    double calcPointPlaneDist(pcl::ModelCoefficients::Ptr plane, pcl::PointXYZ point);
    double calcPointPlaneDist(pcl::ModelCoefficients plane, pcl::PointXYZ point);


    /**
     * visualize cloud
     * @param first
     * @param msg
     */
    void visualizeCloud(CloudPointer first, const std::string msg = "");
    void visualizeCloud(CloudPointerRGB first, const std::string msg = "");



    /**
     * [important prior parameters]
     * @param curPoint
     * @return
     */
    bool checkWithPriorInfo(pcl::PointXYZ& curPoint);


    /**
     *
     * @param coefficients
     * @return
     */
    bool checkIfGroundPlaneCoeffi(pcl::ModelCoefficients::Ptr coefficients);


    bool checkIfGroundPlaneCoeffi2(pcl::ModelCoefficients::Ptr coefficients);


    /**
     * visualize point clouds pairwisely. Can be replaced with multiVisualizeCloud
     * @param first
     * @param second
     * @param msg
     */
    void pairwiseVisualizeCloud( CloudPointer first,  CloudPointer second, const std::string msg = "");



    /**
     *
     * @param vPath
     * @return
     */
    CloudPointer loadCloudFromPath(std::string vPath);

    CloudPointerRGB loadRGBCloudFromPath(std::string vPath);

    /**
     *
     * @param vPath
     * @param cloudNum
     * @param prefix
     * @param suffix
     * @return
     */
    std::vector<CloudPointer> loadCloudsFromDirectory(std::string vPath, int cloudNum, const std::string
                        prefix = "cloud_cluster_", const std::string suffix = ".pcd");

    /*!
     * visualize all the point clouds within a vector
     * @param clouds
     * @param msg
     */
    void multiVisualizeCloud( std::vector<CloudPointer> clouds, const std::string msg = "");

    /*!
     * given the transformation matrix, calculate the distance (translation distance).
     * @param mat
     * @return
     */

    double calcDistanceByTranslation(Eigen::Matrix<float, 4, 4> mat);
    /*!
     * get the fitness score of two matches
     * @param cloud_a
     * @param cloud_b
     * @return
     */
    double calcFitnessScore(pcl::PointCloud<pcl::PointXYZ> cloud_a, pcl::PointCloud<pcl::PointXYZ> cloud_b);

    /*!
     * get the minimum distance among two clouds.
     * @param cloud_a
     * @param cloud_b
     * @return
     */
    double calcDistanceBetweenClouds(pcl::PointCloud<pcl::PointXYZ> cloud_a, pcl::PointCloud<pcl::PointXYZ> cloud_b);

    /*!
     * get the total count of files in a folder (excludes . and ..)
     * @param vPath
     * @return
     */
    int getFileCountInFolder(string vPath);


    /*!
     * perform a certain transformation matrix on a point cloud
     * @param transform
     * @param cloud
     * @return
     */
    CloudPointer transformCloudFromMatrix(Eigen::Matrix4f transform, CloudPointer cloud);

    /*!
     * get the distance factor of a point and a plane
     * @param point
     * @param coefficients
     * @param dist2center
     * @return
     */
    double getPointDistanceFactor(pcl::PointXYZ point, pcl::ModelCoefficients::Ptr coefficients,
                                  double dist2center = -1);


    /**
     *
     * @param cloud
     * @param coefficients
     * @param threshold
     * @param inliers
     */
    void getInliersByWeightedDistance(CloudPointer cloud, pcl::ModelCoefficients::Ptr coefficients, double threshold,
                                      pcl::PointIndices::Ptr inliers);

    /**
     *
     * @param cloud
     * @param coefficients
     * @param inliers
     * @return
     */
    double getTotalCost(CloudPointer cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers );


    /**
     *
     * @param input
     * @param target
     * @param minx
     * @param maxx
     * @param minz
     * @param maxz
     * @param miny
     * @param maxy
     */
    void extractSubarea(CloudPointer input, CloudPointer target, double minx = -INT_MAX, double maxx = INT_MAX,
                        double minz = -INT_MAX, double maxz = INT_MAX, double miny = -INT_MAX, double maxy = INT_MAX);



    /**
     * MSAC fitting of ground plane
     * @param cloud
     * @param resultCoeffi
     * @param resultInliers
     */
    void fitGroundPlane(CloudPointer cloud, pcl::ModelCoefficients::Ptr resultCoeffi, pcl::PointIndices::Ptr resultInliers);



    /**
     *
     * @param input
     * @return
     */
    std::vector<std::vector<int>> voxelizePointCloud(CloudPointer input);


    /**
     * inner product of two vectors
     * @param v
     * @param u
     * @return
     */
    double inner(vector<double> v, vector<double> u);


    /**
     * norm of a vector
     * @param v
     * @return
     */
    double norm(vector<double> v);


    /**
     *
     * @param v
     * @param u
     * @return
     */
    double calcAngleBetweenTwoVector(vector<double> v, vector<double> u);


    /**
     * Transform point from raw to map (SLAM)
     * @param pi: point
     * @param transformTobeMapped: transform matrix
     * @return
     */
    pcl::PointXYZ pointAssociateToMap(pcl::PointXYZ pi, const std::vector<float> transformTobeMapped);

    /**
     *
     * @param pi
     * @param transformTobeMapped
     * @return
     */
    pcl::PointXYZ pointAssociateTobeMapped(pcl::PointXYZ pi, const std::vector<float> transformTobeMapped);



    /**
     * Check with the lidar effective region. If the transform is given then tranform to the origin.
     * @param pori
     * @param transform
     * @return
     */
    bool checkWithEffectiveRegion(pcl::PointXYZ pori, const std::vector<float> transform = vector<float>());

    /**
     * Test Cloud Alignment
     * @param transformTobeMapped
     * @param src
     */
    void cloudAlignment(const std::vector<float> transformTobeMapped, CloudPointer src);

    /**
     * Load transform from txt file
     * @param vPath
     * @return
     */
    std::vector<float> loadTransformFromTXT(const std::string vPath);


    /**
     * load P from txt file
     * @param vPath
     * @param P
     * @param translation
     */
    void loadTransformFromTXT(const std::string vPath, cv::Mat &P);

    /**
     *
     * @param vPath
     * @param K
     */
    void loadKFromTXT(const std::string &vPath, cv::Mat &K);

    /**
     * Transform 3D Lidar point cloud to 2D camera point on image
     * @param lidarPoint
     * @param cameraPoint
     * @param K
     * @param P
     * @return
     */
    bool transformLidarPointToCamera(pcl::PointXYZ &lidarPoint, cv::Point2f &cameraPoint, cv::Mat K, cv::Mat P);

    /**
     * Notice the Lidar point here is a line
     * @param lidarPoint
     * @param cameraPoint
     * @param K
     * @param P
     * @return
     */
    bool transformCameraPointToLidar(pcl::PointXYZ &lidarPoint, cv::Point2f &cameraPoint, cv::Mat K, cv::Mat P);

    /**
     * Get cluster in the point cloud
     * @param cloud
     * @param cloudVector
     */
    void getVehicleClusterFromCloud(CloudPointer &cloud, std::vector<CloudPointer> &cloudVector);

    /**
     * file timestamp pre-processing
     * @param start
     * @return
     */
    string convertTimeStampToFileString(long start);

    /**
     * load D(distortion) from text file
     * @param vPath
     * @param D
     */
    void loadDFromTXT(const std::string vPath, cv::Mat &D);

    /**
     * Correct distortion by D matrix
     * @param pointDehomo
     * @param D
     */
    void correctDistortion(cv::Point2f &pointDehomo, cv::Mat &D);

    /**
     * Convert point cloud from world coordinate to raw(LIDAR) coordinate
     * @param cloud
     * @param transform
     */
    void pointCloudMapToRaw(CloudPointer cloud, const std::vector<float> transform);

    void pointCloudRawToMap(CloudPointer cloud, const std::vector<float> transform);


    /**
     * Save all the clouds in the vector to a specific file as a integration
     * @param clouds
     * @param prefix
     */
    void multiframeIntegration(vector<CloudPointer> clouds, string prefix);


    void loadImageBoxFromTXT(vector<ImageBoundingBox> &rVec, string vPath, const int imageWidth = 1280, const int imageHeight = 720);

    Json::Value parseJsonFile(string path);


    /**
     * Return all the file inside a folder with given suffix. The return results do not contain the suffix.
     * @param path
     * @param suffix
     * @return
     */
    vector<string> findFilesInFolderWithSuffix(const string &path, const string &suffix);

}




#endif //POINTS_GROUPING_UTILS_H
