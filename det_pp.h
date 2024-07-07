#ifndef __SOURCE_DET_PP__
#define __SOURCE_DET_PP__

#include <vector>
#include <opencv2/opencv.hpp>
#include"costMapType.h"

//std::vector<BoxInfo> boxes;
//std::vector<SlotInfo> slots;
void box_nms(std::vector<BoxInfo>& input_boxes, float NMS_THRESH);
void slot_nms(std::vector<SlotInfo>& input_slots, float xThr, float yThr);
void get_yolo_slot_output(std::vector<cv::Mat> feat, int w, int h, int stride, int anchor_num, int class_num, float conf_thr, int yoloOutNum, int Input_w, int Input_h, float anchors[3][3][2], std::vector<SlotInfo>& slots);
// void get_yolo_output3(std::vector<float> feat, int w, int h, int anchor_num, int class_num,  float obj_thr, float cls_thr,int Input_w,int Input_h, int img_w, int img_h, float anchors[], std::vector<BoxInfo>& boxes);
// void get_yolo_output3(std::vector<float> feat, int w, int h, int anchor_num, int class_num,  float conf_thr,int Input_w,int Input_h, int img_w, int img_h, float anchors[], std::vector<BoxInfo>& boxes);
void get_yolo_output3(std::vector<float> feat, int w, int h, int yoloOut, int anchor_num, int class_num, int stride, float conf_thr, int Input_w, int Input_h, float anchor_yolov5n[3][3][2], std::vector<BoxInfo>& boxes);
void get_yolo_one_boxes(std::vector<float> feat, int class_num, float obj_thr, int Input_w, int Input_h, int img_w, int img_h, std::vector<BoxInfo>& boxes);
// void get_yolo_boxes(std::vector<float> feat, int w, int h, int anchor_num, int class_num, float obj_thr, float cls_thr,int Input_w,int Input_h,int img_w, int img_h, float anchors[], std::vector<BoxInfo>& boxes, std::string type = "HWC");
void get_yolo_boxes(std::vector<float> feat, int w, int h, int anchor_num, int class_num, float obj_thr, float cls_thr, int Input_w, int Input_h, int img_w, int img_h, float anchors[], std::vector<BoxInfo>& boxes);

void get_yolo_box2(std::vector<float> ort_outputs, int w, int h, int stride, int img_w, int img_h, float anchors[], float obj_thr, float score_thr);

void draw_box(cv::Mat& cv_mat, std::vector<BoxInfo>& boxes, const std::vector<std::string>& labels, int offset);
#endif
