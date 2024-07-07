#pragma once
#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "det_pp.h"
#include "path.h"

using namespace cv;
using namespace dnn;
using namespace std;

class YOLO {
public:
    YOLO(Net_config config);
    void detect(Mat& frame);
    void slot_detect_1280_720(Mat& frame, vector<parkingSpaceInPixel>& spaces);

    //void slot_track(vector<SlotInfo> slots);
    float anchor_yolov5n[3][3][2];
    int OD_stride[3];
    int slot_stride;

private:
    float* anchors;
    int num_stride;
    int inpWidth;
    int inpHeight;
    vector<string> class_names;
    int num_class;

    float confThreshold;
    float nmsThreshold;
    float xthr;
    float ythr;
    float objThreshold;
    const bool keep_ratio = true;
    Net net;
    void drawPred(float conf, int left, int top, int right, int bottom, Mat& frame, int classid);
    Mat resize_image(Mat srcimg, int* newh, int* neww, int* top, int* left);
};
