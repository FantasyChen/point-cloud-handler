//
// Created by lifan on 2/9/17.
//

#include "ModelRepresentation.h"
#include "utils.h"


ModelRepresentation::ModelRepresentation(){
    // pass
}


void ModelRepresentation::setInputPointCloud(CloudPointer cloud) {
    _cloud = cloud->makeShared();
}

ModelRepresentation::~ModelRepresentation() {
}

vector<double> ModelRepresentation::getRepresentation() {
    return vector<double>();
}


CurbModel::CurbModel() {
    _angleThreshold = 15;
    _coeffi.resize(3);
    _groundCoeffi.resize(3);
    _frontCoeffi.resize(3);
    _isParameterEstimated = false;
}


void CurbModel::calcParameters() {
    if(_cloud->points.size() < 8){
        return;
    }
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_MSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.05); //0.01
    pcl::ModelCoefficients::Ptr coefficients1 (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers1 (new pcl::PointIndices);
    pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices);
    CloudPointer temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ> ());
    *temp = *_cloud;

    // extract for the first plane
    seg.setInputCloud(temp);
    seg.segment(*inliers1, *coefficients1);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(temp);
    extract.setIndices(inliers1);
    extract.setNegative(false);
    extract.filter(*cloud_plane);
    // utils::visualizeCloud(cloud_plane);
    extract.setNegative(true);
    extract.filter(*cloud_f);
    *temp = *cloud_f;


    // Set up the inner coefficient (normalize)
    double a = coefficients1->values[0]/coefficients1->values[3];
    double b = coefficients1->values[1]/coefficients1->values[3];
    double c = coefficients1->values[2]/coefficients1->values[3];

    _coeffi[0] = a;
    _coeffi[1] = b;
    _coeffi[2] = c;
    _isParameterEstimated = true;


}

bool CurbModel::validateModel() {
    calcParameters();
    if(_isParameterEstimated == false)
        return false;
    double angleBetweenFront = utils::calcAngleBetweenTwoVector(_coeffi, _frontCoeffi);
    double angleBetweenGround = utils::calcAngleBetweenTwoVector(_coeffi, _groundCoeffi);
    return ((90 - angleBetweenFront) < _angleThreshold) && ((90 - angleBetweenGround) < _angleThreshold);
}

void CurbModel::setGroundCoeffi(vector<double> coeffi){
    for(int i = 0; i < coeffi.size(); i++){
        _groundCoeffi[i] = coeffi[i];
    }
}

void CurbModel::setFrontCoeffi(vector<double> coeffi) {
    for(int i = 0; i < coeffi.size(); i++){
        _frontCoeffi[i] = coeffi[i];
    }
}

TwoLineModel::TwoLineModel() {
    _representation.resize(4);
    _isInitialSet = false;
}

void TwoLineModel::setInitialGuess(double a, double b, double c, double cprime) {
    _representation[0] = a;
    _representation[1] = b;
    _representation[2] = c;
    _representation[3] = cprime;
    _isInitialSet = true;
}

void TwoLineModel::BFSearching() {   // discarded
}

vector<double> TwoLineModel::getRepresentation(){
    return  _representation;
}

vector<vector<double>> TwoLineModel::getLineModelByThreePoints(vector<pcl::PointXYZ> samplePoints) {
    vector<vector<double>> ret;
    for(auto i = 0; i < 3; i++){
        pcl::PointXYZ point1 = samplePoints[i%3];
        pcl::PointXYZ point2 = samplePoints[(i+1)%3];
        pcl::PointXYZ point3 = samplePoints[(i+2)%3];
        // representation :{a, b, c, c'}, where ax + bz + c = 0, ax + bz + c' = 0.
        vector<double> representation(4);
        if(point1.x - point2.x < 1e-5){
            representation[0] = 1;
            representation[1] = 0;
            representation[2] = -point1.x;
            representation[3] = point3.x;
        }
        else{
            representation[0] = (point1.z - point2.z)/(point1.x - point2.x);
            representation[1] = -1;
            representation[2] = -(point1.z - point2.z)/(point1.x - point2.x)*point1.x + point1.z;
            representation[3] = -(point1.z - point2.z)/(point1.x - point2.x)*point3.x + point3.z;
        }
        ret.push_back(representation);
    }
    return ret;
}


void TwoLineModel::RANSACSearching() {
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd());
    int pointNum = static_cast<int>(_cloud->points.size());
    std::uniform_int_distribution<> dis(0, pointNum - 1);
    int numOfIterations = 1000;
    double bestCost = INT_MAX;
    int i = 0;
    int bestNumberOfInliers = 0;
    vector<double> bestModel;
    vector<int> bestInliers;

    double prob = 0.99;

//    double best
    double inlierThreshold = 0.2;

    // RANSAC iteration
    while(i < numOfIterations) {
        vector<int> randomSamples;    // randomly pick three points to form a two line representation
        while(randomSamples.size()< 3) {
            int temp = dis(gen);
            if (std::find(randomSamples.begin(), randomSamples.end(), temp) == randomSamples.end()) {
                randomSamples.push_back(temp);
            }
        }
        vector<pcl::PointXYZ> samplePoints;
        for(auto j = 0; j < 3; j++){
            pcl::PointXYZ newPoint;
            newPoint.x = _cloud->points[randomSamples[j]].x;
            newPoint.y = _cloud->points[randomSamples[j]].y;
            newPoint.z = _cloud->points[randomSamples[j]].z;
            samplePoints.push_back(newPoint);
        }
        vector<vector<double>> reps = getLineModelByThreePoints(samplePoints);
        for(auto j = 0; j < 3; j ++){
            double cur_cost = 0;
            int numInliers = 0;
            vector<int> inliers;
            vector<double> representation = reps[j];
            double paramA = representation[0];
            double paramB = representation[1];
            double paramC = representation[2];
            double paramC_prime = representation[3];
            if(abs(paramC/paramA - paramC_prime/paramA) < 5) continue;
            for (int p = 0; p < _cloud->points.size(); p++) {
                pcl::PointXYZ curPoint = _cloud->points[p];
                double distance = min(abs(paramA * curPoint.x + paramB * curPoint.z + paramC)/sqrt(paramA * paramA + paramB * paramB),
                                      abs(paramA * curPoint.x + paramB * curPoint.z + paramC_prime)/sqrt(paramA * paramA + paramB * paramB));
                if(distance < inlierThreshold){
                    cur_cost += distance;
                    numInliers ++;
                    inliers.push_back(p);
                }
                else{
                    cur_cost += inlierThreshold;
                }
            }
            if(cur_cost < bestCost){
                bestCost = cur_cost;
                bestNumberOfInliers = numInliers;
                bestModel = representation;
                double w = bestNumberOfInliers * 1.0 / pointNum;
                numOfIterations = int(log10(1-prob)/log10(1 - pow(w, 3.0)));
                bestInliers = inliers;
                cout << cur_cost << endl;
            }
        }
        i++;
    }
    cout << "Final size of inlier " <<  bestNumberOfInliers << endl;
    _representation = bestModel;



}


RectangleModel::RectangleModel() {
    _representation.resize(4);
    _isInitialSet = false;
}

vector<double> RectangleModel::getRepresentation() {
    return  _representation;
}

void RectangleModel::setInitialGuess(double a, double b, double c, double cprime) {
    _representation[0] = a;
    _representation[1] = b;
    _representation[2] = c;
    _representation[3] = cprime;
    _isInitialSet = true;
}

void RectangleModel::BFSearching(){
    double distanceChangeRange = 4.5;
    double angleChangeRange = 0.2;
    if(!_isInitialSet){
        _representation[0] = 1;
        _representation[1] = 0;
        _representation[2] = 3;
        _representation[3] = -3;
    }
    else{
        distanceChangeRange = 1.5;
    }
    vector<double> bestModel;
    bestModel.resize(4);
    double bestCost = INT_MAX;
    for(double b = _representation[1] - angleChangeRange; b <= _representation[1] + angleChangeRange; b = b + 0.05){
        for(double c = _representation[2] - distanceChangeRange; c <= _representation[2] + distanceChangeRange; c = c + 0.1){
            for(double cprime = _representation[3] - distanceChangeRange; cprime <= _representation[3] + distanceChangeRange; cprime = cprime + 0.1){
                double cost = 0;
                for(auto point:_cloud->points){
                    bool label = (point.r == 255);
                    cost += getPointCost(point, label, _representation[0], b, c, cprime);
                }
                if(cost < bestCost){
                    bestCost = cost;
                    bestModel[0] = _representation[0];
                    bestModel[1] = b;
                    bestModel[2] = c;
                    bestModel[3] = cprime;
                    cout << bestCost << endl;
                }
            }
        }
    }
    for(auto i = 0; i < 4; i ++){
        _representation[i] = bestModel[i];
    }
}

void RectangleModel::setInputPointCloud(CloudPointerRGB cloud) {
    _cloud = cloud->makeShared();
}

double RectangleModel::getPointCost(pcl::PointXYZRGB point, bool label, double a, double b, double c, double cprime) {
    if(label) {
        if (abs(b) <= 1e-2) {  // line perpendicular to x axis
            if (point.x <= max(c, cprime) / a && point.x >= min(c, cprime) / a) {
                return 0;
            } else {
                return min(abs(point.x - c / a), abs(point.x - cprime / a)) * getConfidenceCoefficient(point);
            }
        }
        double yIntersection1 = -(a * point.x + c) / b;
        double yIntersection2 = -(a * point.x + cprime) / b;

        if (point.z <= max(yIntersection1, yIntersection2) && point.z >= min(yIntersection1, yIntersection2)) {
            return 0;
        } else {
            double distance = min(abs(a * point.x + b * point.z + c) / sqrt(a * a + b * b),
                                  abs(a * point.x + b * point.z + cprime) / sqrt(a * a + b * b));
            return distance * getConfidenceCoefficient(point);
        }
    }
    else{
        if (abs(b) <= 1e-2) {  // line perpendicular to x axis
            if (point.x > max(c, cprime) / a  || point.x < min(c, cprime) / a) {
                return 0;
            } else {
                return min(abs(point.x - c / a), abs(point.x - cprime / a)) * getConfidenceCoefficient(point);
            }
        }
        double yIntersection1 = -(a * point.x + c) / b;
        double yIntersection2 = -(a * point.x + cprime) / b;

        if (point.z > max(yIntersection1, yIntersection2) || point.z < min(yIntersection1, yIntersection2)) {
            return 0;
        } else {
            double distance = min(abs(a * point.x + b * point.z + c) / sqrt(a * a + b * b),
                                  abs(a * point.x + b * point.z + cprime) / sqrt(a * a + b * b));
            return distance * getConfidenceCoefficient(point);
        }
    }
}

double RectangleModel::getConfidenceCoefficient(pcl::PointXYZRGB point) {
    double distance = sqrt((point.x * point.x) + (point.z * point.z));
    if(distance < 3){
        return 1;
    }
    else{
        return max(0.1, 1 - (distance-3)/10);
    }
}
