#pragma once
#include<iostream>
#include "cereal/archives/portable_binary.hpp"
#include "cereal/archives/json.hpp"
#include "cereal/types/memory.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/string.hpp"


#define MAX_AI_SLOT_NUM        (10)
#define IMAGE_INPUT_WIDTH      (1280)  //单位：pixel
#define IMAGE_INPUT_HEIGHT     (720) //单位：pixel
#define RESOLUTION             (0.0181818181)  //单位：米/pixel
#define PARKING_SPACE_RATIO    (2.1154)  // 车位长度比宽度一般的比例
#define PI                     (3.141592653589793)
#define ROAD_WIDTH             (6) //单位：meter
#define SHOW_PARKING_DEBUG     (1)

class CostMap;

// 在世界坐标系下车位的坐标表示方式
typedef struct ParkSpace {
    std::string   idStr;     /**< 车位地面标识符号，可能含有字母等字符 */
    float   clsId;  /**< 车位类别:0-可用; 1-被占用; 2-窄车位 */
    float   score;  /**< 车位分数：范围[0.0,1.0] */

    float x1; // 世界物理坐标，顺序取的4个顶点(x1,y1),(x2,y2),(x3,y3),(x4,y4)
    float y1;
    float x2;
    float y2;
    float x3;
    float y3;
    float x4;
    float y4;

    template <class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(idStr), CEREAL_NVP(clsId), CEREAL_NVP(score), 
            CEREAL_NVP(x1), CEREAL_NVP(y1),
            CEREAL_NVP(x2), CEREAL_NVP(y2),
            CEREAL_NVP(x3), CEREAL_NVP(y3),
            CEREAL_NVP(x4), CEREAL_NVP(y4));
    }
}ParkSpace;


// 在世界坐标系下的表示里程计航位点
typedef struct pt2f {
    float x;
    float y;

    template<class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(x),CEREAL_NVP(y));
    }
}pt2f;

typedef struct odometery {
    std::vector<cv::Point2f> vecOdometery;

    template<class Archive>
    void save(Archive& ar) const{
        std::vector<pt2f> a;
        for (size_t i = 0; i < vecOdometery.size(); i++)
        {
            a.push_back(pt2f{ vecOdometery[i].x,vecOdometery[i].y });
        }
        ar(cereal::make_nvp("vecOdometery", a));
    }

    template<class Archive>
    void load(Archive& ar) {
        std::vector<pt2f> a;
        ar(cereal::make_nvp("vecOdometery", a));
        for (size_t i = 0; i < a.size(); i++)
        {
            vecOdometery.push_back(cv::Point2f(a[i].x, a[i].y));
        }
    }
}odometery;



// 在世界坐标系下车位的坐标表示方式
typedef struct objectTrack {
    int trackID;
    int age;
    ParkSpace sp;
    float cx; // 中心坐标
    float cy;
};

typedef struct EgoVehiclePose {
    float x;     // 处在世界中的物理坐标，单位：米
    float y;     // 处在世界中的物理坐标，单位：米
    float theta; // 处在世界中的物理坐标，单位：弧度
}EgoVehiclePose;

typedef struct BoxInfo {
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
    int label;
} BoxInfo;

// 在680*680图像上车位的yolo内部输出坐标表示
typedef struct SlotInfo {
    float x1; // 当前帧像素横坐标
    float y1;
    float x2; 
    float y2; 
    float cos2;
    float sin2;
    float score;
    int label;
    float cx;
    float cy;
    float lenth;
} SlotInfo;

// 在1280*720图像上车位的4个像素点坐标，直观表示,连续四个车位顺序点(x1,y1),(x2,y2),(x3,y3),(x4,y4)表示
typedef struct parkingSpaceInPixel {
    float x1; 
    float y1;
    float x2;
    float y2;
    float x3; 
    float y3;
    float x4;
    float y4;

    float score;
    int label;
}parkingSpaceInPixel;

struct Net_config {
    float confThreshold;  // Confidence threshold
    float nmsThreshold;   // Non-maximum suppression threshold
    float xthr;
    float ythr;
    float objThreshold;
    std::string modelpath;
};

typedef struct costMapSerializeParam{
    std::vector<ParkSpace> parkingSpaces;
    float resolution;
  
    template <class Archive>
    void save(Archive& ar) const {
        std::vector<ParkSpace> sps;
        for (size_t i = 0; i < parkingSpaces.size(); i++) {
            sps.push_back(ParkSpace{ parkingSpaces[i].idStr, parkingSpaces[i].clsId, parkingSpaces[i].score,
                parkingSpaces[i].x1 ,parkingSpaces[i].y1,
                parkingSpaces[i].x2,parkingSpaces[i].y2,
                parkingSpaces[i].x3,parkingSpaces[i].y3, 
                parkingSpaces[i].x4,parkingSpaces[i].y4});
        }
        ar(sps, resolution);
    }
    template <class Archive>
    void load(Archive& ar) {
        std::vector<ParkSpace> sps;
        ar(sps, resolution);
        for (size_t i = 0; i < sps.size(); i++) {
            parkingSpaces.push_back(ParkSpace{ sps[i].idStr, sps[i].clsId, sps[i].score,
                sps[i].x1 ,sps[i].y1,
                sps[i].x2,sps[i].y2,
                sps[i].x3,sps[i].y3,
                sps[i].x4,sps[i].y4});
        }
    }
}costMapSerializeParam;

inline cv::Mat rotx(float radian)
{
    cv::Mat A = (cv::Mat_<float>(3, 3) << 1.0, 0.0, 0.0, 0.0, std::cos(radian), -std::sin(radian), 0.0, std::sin(radian), std::cos(radian));
    return A;
}


inline cv::Mat roty(float radian)
{
    cv::Mat A = (cv::Mat_<float>(3, 3) << std::cos(radian), 0, std::sin(radian), 0.0, 1, 0.0, -std::sin(radian), 0, std::cos(radian));
    return A;
}


inline cv::Mat rotz(float radian)
{
    cv::Mat A = (cv::Mat_<float>(3, 3) << std::cos(radian), -std::sin(radian), 0, std::sin(radian), std::cos(radian), 0.0, 0.0, 0.0, 1.0);
    return A;
}

inline ParkSpace convertParking2World(parkingSpaceInPixel s, EgoVehiclePose currS) {
    ParkSpace sp;
    sp.clsId = s.label;
    sp.score = s.score;
    sp.idStr = ""; // 检测器暂无法检测输出此内容

    cv::Mat Rframe = rotx(PI) * rotz(PI/2);
    cv::Mat A1;
    cv::Mat c1 = cv::Mat::zeros(3, 1,CV_32FC1);
    cv::Mat_<float> r1 =(cv::Mat_<float>(1, 4) << 0.0, 0.0, 0.0, 1.0);
    cv::hconcat(Rframe, c1, A1);
    cv::vconcat(A1, r1, A1);

    cv::Mat_<float> A2 = (cv::Mat_<float>(4, 4) << RESOLUTION, 0, 0, IMAGE_INPUT_HEIGHT / 2.0 * RESOLUTION,
        0, RESOLUTION, 0, IMAGE_INPUT_WIDTH / 2.0 * RESOLUTION,
        0, 0, 1, 0,
        0, 0, 0, 1);
    cv::Mat A3;
    cv::Mat_<float> c2 = (cv::Mat_<float>(3, 1) << currS.x, currS.y, 0);
    cv::hconcat(rotz(currS.theta), c2, A3);
    cv::vconcat(A3, r1, A3);
   
    cv::Mat_<float> src = (cv::Mat_<float>(4, 4) << s.x1, s.x2,s.x3, s.x4,
        s.y1, s.y2, s.y3, s.y4,
        0, 0, 0, 0,
        1, 1, 1, 1);
    cv::Mat dst = A3 * A2 * A1 * src;
    sp.x1 = dst.at<float>(0, 0);
    sp.y1 = dst.at<float>(1, 0);

    sp.x2 = dst.at<float>(0, 1);
    sp.y2 = dst.at<float>(1, 1);

    sp.x3 = dst.at<float>(0, 2);
    sp.y3 = dst.at<float>(1, 2);

    sp.x4 = dst.at<float>(0, 3);
    sp.y4 = dst.at<float>(1, 3);
    return sp;
}

// "marshalling"
template <typename T>
void convertCVToMatrix(const cv::Mat& originImg, int rows, int cols, int channels, T* dst)
{
    cv::Mat srcImg = originImg.clone();
    size_t elems = (unsigned int)rows * (unsigned int)cols;
    if (channels == 3) {
        cv::Mat channels[3];
        cv::split(srcImg.t(), channels);

        memcpy(dst, channels[2].data, elems * sizeof(T));              //copy channel[2] to the red channel
        memcpy(dst + elems, channels[1].data, elems * sizeof(T));      // green
        memcpy(dst + 2 * elems, channels[0].data, elems * sizeof(T));  // blue
    }
    else {
        srcImg = srcImg.t();
        memcpy(dst, srcImg.data, elems * sizeof(T));
    }
}

//"marshalling"
inline void convertToMat(const unsigned char inImg[], int rows, int cols, int channels, cv::Mat& matBigImg) {
    int elems = rows * cols;
    if (channels == 3) {
        matBigImg = cv::Mat(rows, cols, CV_8UC3, cv::Scalar::all(0));

        for (int i = 0; i < rows; i++) {
            cv::Vec3b* data = matBigImg.ptr<cv::Vec3b>(i);
            for (int j = 0; j < cols; j++) {
                data[j][2] = (uchar)inImg[i + rows * j];
                data[j][1] = (uchar)inImg[i + rows * j + elems];
                data[j][0] = (uchar)inImg[i + rows * j + 2 * elems];
            }
        }
    }
    else {
        matBigImg = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(0));

        for (int i = 0; i < rows; i++) {
            uchar* data = matBigImg.ptr<uchar>(i);
            for (int j = 0; j < cols; j++) {
                data[j] = (uchar)inImg[i + rows * j];
            }
        }
    }
}
