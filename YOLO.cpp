#include "YOLO.h"

YOLO::YOLO(Net_config config) {
    this->confThreshold = config.confThreshold;
    this->nmsThreshold = config.nmsThreshold;
    this->xthr = config.xthr;
    this->ythr = config.xthr;
    this->objThreshold = config.objThreshold;

    this->net = readNet(config.modelpath);
    ifstream ifs("./costMap/class.txt");
    std::string line;
    while (getline(ifs, line)) {
        printf("%s", line.c_str());
        this->class_names.push_back(line);
    }
    this->num_class = class_names.size();

    /*if (endsWith(config.modelpath, "6.onnx"))
    {
        anchors = (float*)anchors_1280;
        this->num_stride = 4;
        this->inpHeight = 1280;
        this->inpWidth = 1280;
    }*/
    if (config.modelpath.find("ps")) {
        anchor_yolov5n[0][0][0] = 96;
        anchor_yolov5n[0][1][0] = 224;
        slot_stride = 32;
        this->inpHeight = 640;
        this->inpWidth = 640;
    }
    else {
        float anchor[3][3][2] = { 10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326 };
        memcpy(anchor_yolov5n, anchor, sizeof(anchor));
        this->num_stride = 3;
        this->inpHeight = 640;
        this->inpWidth = 640;
    }
}

Mat YOLO::resize_image(Mat srcimg, int* newh, int* neww, int* top, int* left) {
    int srch = srcimg.rows, srcw = srcimg.cols;
    *newh = this->inpHeight;
    *neww = this->inpWidth;
    Mat dstimg;
    if (this->keep_ratio && srch != srcw) {
        float hw_scale = (float)srch / srcw;
        if (hw_scale > 1) {
            *newh = this->inpHeight;
            *neww = int(this->inpWidth / hw_scale);
            resize(srcimg, dstimg, Size(*neww, *newh), INTER_AREA);
            *left = int((this->inpWidth - *neww) * 0.5);
            copyMakeBorder(dstimg, dstimg, 0, 0, *left, this->inpWidth - *neww - *left, BORDER_CONSTANT, 114);
        }
        else {
            *newh = (int)this->inpHeight * hw_scale;
            *neww = this->inpWidth;
            resize(srcimg, dstimg, Size(*neww, *newh), INTER_AREA);
            *top = (int)(this->inpHeight - *newh) * 0.5;
            copyMakeBorder(dstimg, dstimg, *top, this->inpHeight - *newh - *top, 0, 0, BORDER_CONSTANT, 114);
        }
    }
    else {
        resize(srcimg, dstimg, Size(*neww, *newh), INTER_AREA);
    }
    return dstimg;
}

void YOLO::slot_detect_1280_720(Mat& src, vector<parkingSpaceInPixel>& spaces) {
    spaces.clear();
    cv::Mat frame = src.clone();
    if ((frame.rows != 720) || (frame.cols != 1280)) {
        std::cerr << "input bird eye image is not 1280*720." << std::endl;
        return;
    }
    int startX = frame.cols / 2 - 340;
    int startY = frame.rows / 2 - 340;
    frame = frame(Rect(startX, startY, 680, 680));
   // cv::imwrite("aa.jpg", frame);

    int newh = 0, neww = 0, padh = 0, padw = 0;
    Mat dstimg = this->resize_image(frame, &newh, &neww, &padh, &padw);
    Mat blob = blobFromImage(dstimg, 1 / 255.0, Size(this->inpWidth, this->inpHeight), Scalar(0, 0, 0), true, false);
    this->net.setInput(blob);
    vector<Mat> outs;
    this->net.forward(outs, this->net.getUnconnectedOutLayersNames());
    int Input_w = frame.cols;
    int Input_h = frame.rows;
    int feat_width = dstimg.cols / slot_stride;
    int feat_height = dstimg.rows / slot_stride;
    int anchor_num = 2;
    int yoloOutNum = 1;
    int cls_num = this->num_class;
    float conf_thr = this->confThreshold;

    vector<SlotInfo> slots;
    get_yolo_slot_output(outs, feat_width, feat_height, slot_stride, anchor_num, cls_num,
        conf_thr, yoloOutNum, Input_w, Input_h, anchor_yolov5n, slots);
    slot_nms(slots, this->xthr, this->ythr);

    for (int iter = 0; iter < slots.size(); iter++) {
        int x1 = slots[iter].x1;
        int y1 = slots[iter].y1;
        int x2 = slots[iter].x2;
        int y2 = slots[iter].y2;
        float parkingSpaceWidth = std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
        float parkingSpaceLength = PARKING_SPACE_RATIO * parkingSpaceWidth;
        int x3 = slots[iter].cos2 * parkingSpaceLength + x2;
        int y3 = slots[iter].sin2 * parkingSpaceLength + y2;
        int x4 = slots[iter].cos2 * parkingSpaceLength + x1;
        int y4 = slots[iter].sin2 * parkingSpaceLength + y1;

#if SHOW_PARKING_DEBUG
        float score = slots[iter].score;
        if (slots[iter].label == 0) {
            // cv::fillPoly(srcimg,ppt,npt,1,cv::Scalar(0,255,0),0);
            cv::line(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
            cv::line(frame, cv::Point(x1, y1), cv::Point(x4, y4), cv::Scalar(0, 255, 0), 2);
            cv::line(frame, cv::Point(x2, y2), cv::Point(x3, y3), cv::Scalar(0, 255, 0), 2);
            cv::circle(frame, cv::Point((x1 + x2) / 2, (y1 + y2) / 2), 3, cv::Scalar(255, 255, 0), 2);
            cv::putText(frame, std::to_string(score), cv::Point((x1 + x2) / 2, (y1 + y2) / 2 + 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
            cv::putText(frame, std::to_string(1), cv::Point(x1, y1), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
            cv::putText(frame, std::to_string(2), cv::Point(x2, y2), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
            cv::putText(frame, std::to_string(3), cv::Point(x3, y3), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
        }
        else if (slots[iter].label == 1) {
            cv::line(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 2);
            cv::line(frame, cv::Point(x1, y1), cv::Point(x4, y4), cv::Scalar(0, 0, 255), 2);
            cv::line(frame, cv::Point(x2, y2), cv::Point(x3, y3), cv::Scalar(0, 0, 255), 2);
            cv::circle(frame, cv::Point((x1 + x2) / 2, (y1 + y2) / 2), 3, cv::Scalar(255, 255, 0), 2);
            cv::putText(frame, std::to_string(score), cv::Point((x1 + x2) / 2, (y1 + y2) / 2 + 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255));
            // cv::fillPoly(srcimg,ppt,npt,1,cv::Scalar(0,0,255),0);
        }
        cv::imshow("yolo detect", frame);
#endif // SHOW_PARKING_DEBUG


        parkingSpaceInPixel sp;
        sp.x1 = x1+ startX;
        sp.y1 = y1+ startY;
        sp.x2 = x2+ startX;
        sp.y2 = y2+ startY;
        sp.x3 = x3+ startX;
        sp.y3 = y3+ startY;
        sp.x4 = x4+ startX;
        sp.y4 = y4+ startY;

        sp.label = slots[iter].label;
        sp.score = slots[iter].score;
        spaces.push_back(sp);
    }
    /*SlotTrack tracker;
    tracker.process(frame, slots);*/
}
