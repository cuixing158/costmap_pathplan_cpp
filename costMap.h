#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include"costMapType.h"


class multiObjectTracker
{
public:

    multiObjectTracker(float costConfirm):m_costThreshold(costConfirm){  };

    ~multiObjectTracker() {};

    std::vector<ParkSpace> update(std::vector<ParkSpace> newDetections);

public:

    std::vector<objectTrack> m_trackers;
    float m_costThreshold ;
    cv::Mat m_costMatrix;
};

class CostMap 
{
public:
    /*构造函数*/
    CostMap();


    /*析构函数*/
     ~CostMap();

     /**
    * @brief       CostMap构建函数
    * @details     This is the detail description.
    * @param[in]   parkingSpaces，世界车位坐标.
    * @param [in]   resolution分辨率，单位:m / pixel.
    * @param[out]  outArgName output argument description.
    * @return      返回值
    * @retval      返回值类型
    * @par 标识符
    *     保留
    * @par 其它
    *
    * @par 修改日志
    *      cuixingxing于2024/02/02创建
    */
     CostMap(std::vector<ParkSpace>& parkingSpaces,float resolution=1);


     /**
  * @brief       返回全局occupancy矩阵
  * @details     This is the detail description.
  * @param[in]   None.
  * @param[out]  occupancyMatrix返回occupancy像素矩阵.
  * @return      返回值
  * @retval      返回值类型
  * @par 标识符
  *     保留
  * @par 其它
  *
  * @par 修改日志
  *      cuixingxing于2024/02/02创建
  */
     void getOccupancyMatrix(cv::Mat &occupancyMatrix) const;

     /**
    * @brief       返回全局occupancy矩阵
    * @details     This is the detail description.
    * @param[in]   vecPts全局egoVehicle航点坐标
    * @param[out]  occupancyMatrix返回全局occupancy像素矩阵.
    * @return      返回值
    * @retval      返回值类型
    * @par 标识符
    *     保留
    * @par 其它
    *
    * @par 修改日志
    *      cuixingxing于2024/02/20创建
    */
     void  CostMap::getOccupancyMatrixFromTrajectory(std::vector<cv::Point2f> vecPts, cv::Mat& occupancyMatrix) const;

     /**
    * @brief       返回局部occupancy矩阵
    * @details     This is the detail description.
    * @param[in]   newDetections输入当前感知的局部车位.
    * @param[out]  occupancyMatrix返回局部occupancy像素矩阵.
    * @return      返回值
    * @retval      返回值类型
    * @par 标识符
    *     保留
    * @par 其它
    *
    * @par 修改日志
    *      cuixingxing于2024/02/20创建
    */
     void CostMap::getLocalOccupancyMatrix(std::vector<ParkSpace> newDetections, cv::Mat& occupancyMatrix) const;

     /**
    * @brief       返回局部occupancy矩阵
    * @details     This is the detail description.
    * @param[in]   newDetections输入当前感知的局部车位.
    * @param[in]   vecWorldTrajPts输入当前egoVehicle的航点坐标.
    * @param[out]  occupancyMatrix返回局部occupancy像素矩阵.
    * @return      返回值
    * @retval      返回值类型
    * @par 标识符
    *     保留
    * @par 其它
    *
    * @par 修改日志
    *      cuixingxing于2024/02/20创建
    */
     void CostMap::getLocalOccupancyMatrixFromTrajectory(std::vector<ParkSpace> newDetections, std::vector<cv::Point2f> vecWorldTrajPts, cv::Mat& occupancyMatrix) const;

     void CostMap::getAllWorldParkSpace(std::vector<ParkSpace>& sps) const;

     void CostMap::getLocalOriginInWorld(double localOriginInWorld[2]) const;

   void CostMap::grid2world(int ij[2], double xy[2]) const;

   void CostMap::world2grid(double xy[2], int ij[2]) const;

     /**
    * @brief       保存全局costmap地图
    * @details     This is the detail description.
    * @param[in]   filepath:保存文件名，目前必须为json文件
    * @param[in]   vecOdo:ego vehicle建图走的路径,[x,y]，单位：米
    * @return      返回值
    * @retval      返回值类型
    * @par 标识符
    *     保留
    * @par 其它
    *
    * @par 修改日志
    *      cuixingxing于2024/02/20创建
    */
     void saveCostMap(const char* filepath="./costMapFile.json", std::vector<cv::Point2f>& vecOdo= std::vector<cv::Point2f>());

     /**
    * @brief       加载全局costmap地图
    * @details     This is the detail description.
    * @param[in]   filepath:指定地图文件加载costmap地图
    * @param[out]  vecOdo:ego vehicle建图走的路径,[x,y]，单位：米
    * @return      返回值
    * @retval      返回值类型
    * @par 标识符
    *     保留
    * @par 其它
    *
    * @par 修改日志
    *      cuixingxing于2024/02/20创建
    */
     void loadCostMap(const char* filepath="./costMapFile.json",  std::vector<cv::Point2f>& vecOdo = std::vector<cv::Point2f>());

private:
    float m_resolution;  //分辨率，单位:m / pixel.
    float m_worldXRange[2];
    float m_worldYRange[2];

    int m_widthPixels;
    int m_heightPixels;
    std::vector<ParkSpace> m_parkSpace;
};


