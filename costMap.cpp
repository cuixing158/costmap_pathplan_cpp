#include<fstream>
#include "costMap.h"
#include "costMapType.h"

CostMap::CostMap() {
    m_resolution = 0.1;
    m_worldXRange[0] = 0;
    m_worldXRange[1] = 50;

    m_worldYRange[0] = 0;
    m_worldYRange[1] = 50;

    m_widthPixels = (m_worldXRange[1] - m_worldXRange[0]) / m_resolution;
    m_heightPixels = (m_worldYRange[1] - m_worldYRange[0]) / m_resolution;
}

CostMap::~CostMap() {
}

CostMap::CostMap(std::vector<ParkSpace>& parkingSpaces,float resolution)
{
    m_resolution = resolution;
    if (parkingSpaces.empty())
    {
        m_worldXRange[0] = 0;
        m_worldXRange[1] = 50;

        m_worldYRange[0] = 0;
        m_worldYRange[1] = 50;

        m_widthPixels = (m_worldXRange[1] - m_worldXRange[0]) / m_resolution;
        m_heightPixels = (m_worldYRange[1] - m_worldYRange[0]) / m_resolution;
        return;
    }
   
    std::vector<float> vecX, vecY;
    for (size_t i = 0; i < parkingSpaces.size(); i++)
    {
        vecX.push_back(parkingSpaces[i].x1);
        vecX.push_back(parkingSpaces[i].x2);
        vecX.push_back(parkingSpaces[i].x3);
        vecX.push_back(parkingSpaces[i].x4);

        vecY.push_back(parkingSpaces[i].y1);
        vecY.push_back(parkingSpaces[i].y2);
        vecY.push_back(parkingSpaces[i].y3);
        vecY.push_back(parkingSpaces[i].y4);
    }
   
    auto it = std::minmax_element(vecX.begin(), vecX.end());
    float minX = *it.first;
    float maxX = *it.second;
    it = std::minmax_element(vecY.begin(), vecY.end());
    float minY = *it.first;
    float maxY = *it.second;

    float rangeX = maxX - minX;
    float rangeY = maxY - minY;

    float expandRatio = 0.25;
    float worldX = 2 * expandRatio * rangeX + rangeX;
    float worldY = 2 * expandRatio * rangeY + rangeY;

    m_worldXRange[0] = minX - expandRatio*rangeX;
    m_worldXRange[1] = maxX + expandRatio * rangeX;

    m_worldYRange[0] = minY - expandRatio * rangeY;
    m_worldYRange[1] = maxY + expandRatio * rangeY;

    m_widthPixels = std::round(worldX/m_resolution);
    m_heightPixels = std::round(worldY/m_resolution);

    m_parkSpace = parkingSpaces;
}

void  CostMap::getOccupancyMatrix(cv::Mat& occupancyMatrix) const{
    occupancyMatrix = 100*cv::Mat::ones(m_heightPixels, m_widthPixels, CV_8UC1);
    for (size_t i = 0; i < m_parkSpace.size(); i++)
    {
        std::vector<cv::Point> pts;
        pts.push_back(cv::Point((m_parkSpace[i].x1 - m_worldXRange[0]) / m_resolution, (m_parkSpace[i].y1 - m_worldYRange[0]) / m_resolution));
        pts.push_back(cv::Point((m_parkSpace[i].x2 - m_worldXRange[0]) / m_resolution, (m_parkSpace[i].y2 - m_worldYRange[0]) / m_resolution));
        pts.push_back(cv::Point((m_parkSpace[i].x3 - m_worldXRange[0]) / m_resolution, (m_parkSpace[i].y3 - m_worldYRange[0]) / m_resolution));
        pts.push_back(cv::Point((m_parkSpace[i].x4 - m_worldXRange[0]) / m_resolution, (m_parkSpace[i].y4 - m_worldYRange[0]) / m_resolution));
        //cv::fillPoly(occupancyMatrix, pts, cv::Scalar::all(255));
        cv::line(occupancyMatrix, pts[0], pts[1], cv::Scalar::all(255), 1);
        cv::line(occupancyMatrix, pts[1], pts[2], cv::Scalar::all(255), 1);
        cv::line(occupancyMatrix, pts[2], pts[3], cv::Scalar::all(255), 1);
        cv::line(occupancyMatrix, pts[3], pts[0], cv::Scalar::all(255), 1);
        //cv::putText(occupancyMatrix, std::to_string(i), cv::Point((pts[0].x + pts[2].x) / 2, (pts[0].y + pts[2].y) / 2), 0, 1, cv::Scalar::all(255));
    }
    cv::flip(occupancyMatrix, occupancyMatrix, 0);
    occupancyMatrix.convertTo(occupancyMatrix, CV_64FC1, 1.0 / 255);
}

void  CostMap::getOccupancyMatrixFromTrajectory(std::vector<cv::Point2f> vecWorldTrajPts, cv::Mat& occupancyMatrix) const{
    if (vecWorldTrajPts.empty())
    {
        throw std::invalid_argument("vecWorldTrajPts must not empty.");
    }
    occupancyMatrix = 100*cv::Mat::ones(m_heightPixels, m_widthPixels, CV_8UC1);
    int pixelWidth = ROAD_WIDTH / m_resolution;
    for (size_t i = 0; i < vecWorldTrajPts.size() - 1; i++)
    {
        cv::Point pt1((vecWorldTrajPts[i].x - m_worldXRange[0]) / m_resolution, (vecWorldTrajPts[i].y - m_worldYRange[0]) / m_resolution);
        cv::Point pt2((vecWorldTrajPts[i + 1].x - m_worldXRange[0]) / m_resolution, (vecWorldTrajPts[i + 1].y - m_worldYRange[0]) / m_resolution);
        cv::line(occupancyMatrix, pt1, pt2, cv::Scalar::all(0), pixelWidth);
    }
    for (size_t i = 0; i < m_parkSpace.size(); i++)
    {
        std::vector<cv::Point> pts;
        pts.push_back(cv::Point((m_parkSpace[i].x1 - m_worldXRange[0]) / m_resolution, (m_parkSpace[i].y1 - m_worldYRange[0]) / m_resolution));
        pts.push_back(cv::Point((m_parkSpace[i].x2 - m_worldXRange[0]) / m_resolution, (m_parkSpace[i].y2 - m_worldYRange[0]) / m_resolution));
        pts.push_back(cv::Point((m_parkSpace[i].x3 - m_worldXRange[0]) / m_resolution, (m_parkSpace[i].y3 - m_worldYRange[0]) / m_resolution));
        pts.push_back(cv::Point((m_parkSpace[i].x4 - m_worldXRange[0]) / m_resolution, (m_parkSpace[i].y4 - m_worldYRange[0]) / m_resolution));
        //cv::fillPoly(occupancyMatrix, pts, cv::Scalar::all(255));
        cv::line(occupancyMatrix, pts[0], pts[1], cv::Scalar::all(255), 1);
        cv::line(occupancyMatrix, pts[1], pts[2], cv::Scalar::all(255), 1);
        cv::line(occupancyMatrix, pts[2], pts[3], cv::Scalar::all(255), 1);
        cv::line(occupancyMatrix, pts[3], pts[0], cv::Scalar::all(255), 1);
    }
    cv::flip(occupancyMatrix, occupancyMatrix, 0);
    occupancyMatrix.convertTo(occupancyMatrix, CV_64FC1, 1.0 / 255);
}

void CostMap::getAllWorldParkSpace(std::vector<ParkSpace>& sps) const {
    sps = m_parkSpace;
}

void CostMap::getLocalOriginInWorld(double localOriginInWorld[2]) const{
    localOriginInWorld[0] = m_worldXRange[0];
    localOriginInWorld[1] = m_worldYRange[0];
}

void CostMap::grid2world(int ij[2], double xy[2]) const{
    xy[0] = (double)ij[1] * m_resolution+m_worldXRange[0];
    xy[1] = ((double)m_heightPixels- (double)ij[0]) * m_resolution+m_worldYRange[0];
}

void CostMap::world2grid( double xy[2], int ij[2]) const{
    ij[0] = m_heightPixels - (xy[1]- m_worldYRange[0]) / m_resolution;
    ij[1] = (xy[0]-m_worldXRange[0]) / m_resolution;
}

void  CostMap::getLocalOccupancyMatrix(std::vector<ParkSpace> newDetections,cv::Mat& occupancyMatrix) const{
    CostMap localMap = CostMap(newDetections, m_resolution);
    localMap.getOccupancyMatrix(occupancyMatrix);
}

void  CostMap::getLocalOccupancyMatrixFromTrajectory(std::vector<ParkSpace> newDetections, std::vector<cv::Point2f> vecWorldTrajPts, cv::Mat& occupancyMatrix) const{
    CostMap localMap = CostMap(newDetections, m_resolution);
    localMap.getOccupancyMatrixFromTrajectory(vecWorldTrajPts,occupancyMatrix);
}


std::vector<ParkSpace> multiObjectTracker::update(std::vector<ParkSpace> newDetections) {
    std::vector<ParkSpace> outParkSp;
    if (m_trackers.size() == 0)
    {
        for (size_t i = 0; i < newDetections.size(); i++)
        {
            objectTrack oT;
            oT.sp = newDetections[i];
            oT.age = 1;
            oT.trackID = m_trackers.size();
            oT.cx = (oT.sp.x1 + oT.sp.x2 + oT.sp.x3 + oT.sp.x4) / 4;
            oT.cy = (oT.sp.y1 + oT.sp.y2 + oT.sp.y3 + oT.sp.y4) / 4;
            m_trackers.push_back(oT);
            outParkSp.push_back(oT.sp);
        }
        return outParkSp;
    }
    m_costMatrix = cv::Mat::zeros(m_trackers.size(), newDetections.size(), CV_32FC1);
    for (size_t i = 0; i < m_trackers.size(); i++)
    {
        float* ptr = m_costMatrix.ptr<float>(i);
        for (size_t j = 0; j < newDetections.size(); j++)
        {
            float dx = (newDetections[j].x1 + newDetections[j].x2 + newDetections[j].x3 + newDetections[j].x4) /(float)4;
            float dy = (newDetections[j].y1 + newDetections[j].y2 + newDetections[j].y3 + newDetections[j].y4) / (float)4;
            ptr[j] = std::sqrt((m_trackers[i].cx - dx) * (m_trackers[i].cx - dx) + (m_trackers[i].cy - dy) * (m_trackers[i].cy - dy));
        }
        outParkSp.push_back(m_trackers[i].sp);
    }

    for (size_t iDet = 0; iDet < newDetections.size(); iDet++)
    {
        cv::Mat col = m_costMatrix.col(iDet).clone();
        double minVal;
        int minIdx[2] = {255,255};
        cv::minMaxIdx(col, &minVal, NULL, minIdx, NULL);
        if (minVal<m_costThreshold)
        {
            m_trackers[minIdx[0]].age += 1;

            float dx = (newDetections[iDet].x1 + newDetections[iDet].x2 + newDetections[iDet].x3 + newDetections[iDet].x4) / 4;
            float dy = (newDetections[iDet].y1 + newDetections[iDet].y2 + newDetections[iDet].y3 + newDetections[iDet].y4) / 4;
            m_trackers[minIdx[0]].cx = (m_trackers[minIdx[0]].cx + dx) / 2;
            m_trackers[minIdx[0]].cy = (m_trackers[minIdx[0]].cy + dy) / 2;
        }
        else
        {
            objectTrack oT;
            oT.sp = newDetections[iDet];
            oT.age = 1;
            oT.trackID = m_trackers.size();
            oT.cx = (oT.sp.x1 + oT.sp.x2 + oT.sp.x3 + oT.sp.x4) / 4;
            oT.cy = (oT.sp.y1 + oT.sp.y2 + oT.sp.y3 + oT.sp.y4) / 4;
            m_trackers.push_back(oT);
            outParkSp.push_back(oT.sp);
        }
    }
    return outParkSp;
}

void  CostMap::saveCostMap( const char* filepath, std::vector<cv::Point2f>& vecOdo ) {
    std::ofstream fid(filepath);
    cereal::JSONOutputArchive archiveJson(fid);
    odometery odometery{ vecOdo };
    archiveJson(CEREAL_NVP(m_parkSpace), CEREAL_NVP(m_resolution), CEREAL_NVP(m_worldXRange), CEREAL_NVP(m_worldYRange), CEREAL_NVP(odometery));
}


void CostMap::loadCostMap(const char* filepath, std::vector<cv::Point2f>& vecOdo) {
    std::ifstream fid(filepath);
    cereal::JSONInputArchive archive1(fid);
    odometery odometery;
    archive1(CEREAL_NVP(m_parkSpace), CEREAL_NVP(m_resolution), CEREAL_NVP(m_worldXRange), CEREAL_NVP(m_worldYRange), CEREAL_NVP(odometery));
    *this = CostMap(m_parkSpace, m_resolution);
    vecOdo = odometery.vecOdometery;
}
