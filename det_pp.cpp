#include "det_pp.h"

void box_nms(std::vector<BoxInfo>& input_boxes, float NMS_THRESH) {
    std::sort(input_boxes.begin(), input_boxes.end(), [](BoxInfo a, BoxInfo b) { return a.score > b.score; });
    std::vector<float> vArea(input_boxes.size());
    for (int i = 0; i < int(input_boxes.size()); ++i) {
        vArea[i] = (input_boxes.at(i).x2 - input_boxes.at(i).x1) * (input_boxes.at(i).y2 - input_boxes.at(i).y1);
    }
    for (int i = 0; i < int(input_boxes.size()); ++i) {
        for (int j = i + 1; j < int(input_boxes.size());) {
            float xx1 = std::max(input_boxes[i].x1, input_boxes[j].x1);
            float yy1 = std::max(input_boxes[i].y1, input_boxes[j].y1);
            float xx2 = std::min(input_boxes[i].x2, input_boxes[j].x2);
            float yy2 = std::min(input_boxes[i].y2, input_boxes[j].y2);
            float w = std::max(float(0), xx2 - xx1);
            float h = std::max(float(0), yy2 - yy1);
            float inter = w * h;
            float ovr = inter / (vArea[i] + vArea[j] - inter);
            if (ovr >= NMS_THRESH) {
                input_boxes.erase(input_boxes.begin() + j);
                vArea.erase(vArea.begin() + j);
            } else {
                j++;
            }
        }
    }
}

void slot_nms(std::vector<SlotInfo>& input_slots, float xThr, float yThr) {
    std::sort(input_slots.begin(), input_slots.end(), [](SlotInfo a, SlotInfo b) { return a.score > b.score; });
    for (int i = 0; i < int(input_slots.size()); ++i) {
        float cx1 = (input_slots[i].x1 + input_slots[i].x2) / 2;
        float cy1 = (input_slots[i].y1 + input_slots[i].y2) / 2;
        for (int j = i + 1; j < int(input_slots.size());) {
            float cx2 = (input_slots[j].x1 + input_slots[j].x2) / 2;
            float cy2 = (input_slots[j].y1 + input_slots[j].y2) / 2;
            if ((abs(cx2 - cx1) < xThr) && (abs(cy2 - cy1) < yThr)) {
                input_slots.erase(input_slots.begin() + j);
            } else {
                j++;
            }
        }
    }
}

inline float sigmoid(float x) {
    // return x;
    return 1.0f / (1.0f + std::exp(-1.0f * x));
}

/*
for slot det(CHW)
*/
void get_yolo_slot_output(std::vector<cv::Mat> feat, int w, int h, int stride, int anchor_num, int class_num, float conf_thr, int yoloOutNum, int Input_w, int Input_h, float anchors[3][3][2], std::vector<SlotInfo>& slots) {
    float* pO = nullptr;

    int outSize = w * h;
    int num_grid_x = w;
    int num_grid_y = h;
    float ratio = float(Input_w) / (w * stride);
    //float ratioh=float(Input_h)/(h*stride);
    float cx, cy, length, cos1, sin1, cos2, sin2, bp, cp, maxCP;
    for (int i = 0; i < yoloOutNum; i++) {
        //pO = &feat[0];
        pO = (float*)feat[0].data;
        for (int k = 0; k < anchor_num; k++) {
            int xBeginIndex = k * (8 + class_num) * outSize;
            int yBeginIndex = xBeginIndex + outSize;
            int c1BeginIndex = yBeginIndex + outSize;
            int s1BeginIndex = c1BeginIndex + outSize;
            int c2BeginIndex = s1BeginIndex + outSize;
            int s2BeginIndex = c2BeginIndex + outSize;
            int lBeginIndex = s2BeginIndex + outSize;
            int bpBeginIndex = lBeginIndex + outSize;
            int cpBeginIndex = bpBeginIndex + outSize;

            for (int y = 0; y < num_grid_y; y++) {
                for (int x = 0; x < num_grid_x; x++) {
                    int index = y * num_grid_x + x;
                    int bpIndex = bpBeginIndex + index;
                    // float bp = sigmoid(pO[bpIndex]);
                    float bp = pO[bpIndex];

                    if (bp >= conf_thr) {
                        int classId = 0;
                        float maxCP = 0.0;
                        for (int o = 0; o < class_num; o++) {
                            // float cp = sigmoid(pO[cpBeginIndex + o * outSize + index]);
                            float cp = pO[cpBeginIndex + o * outSize + index];
                            if (cp > maxCP) {
                                maxCP = cp;
                                classId = o;
                            }
                        }

                        float score = maxCP * bp;
                        // fprintf(stderr,"slot score: %f\n",score);
                        if (score > conf_thr) {
                            cx = (2.0f * (pO[xBeginIndex + index]) - 0.5 + x) * stride;
                            cy = (2.0f * (pO[yBeginIndex + index]) - 0.5 + y) * stride;
                            cos1 = 2.0 * (pO[c1BeginIndex + index] - 0) - 1.0;
                            sin1 = 2.0 * (pO[s1BeginIndex + index] - 0) - 1.0;
                            cos2 = 2.0 * (pO[c2BeginIndex + index] - 0) - 1.0;
                            sin2 = 2.0 * (pO[s2BeginIndex + index] - 0) - 1.0;
                            length = (pO[lBeginIndex + index] - 0) * anchors[i][k][0];
                            float x1 = cx - length * cos1;
                            float y1 = cy - length * sin1;
                            float x2 = cx + length * cos1;
                            float y2 = cy + length * sin1;
                            x1 *= ratio;
                            y1 *= ratio;
                            x2 *= ratio;
                            y2 *= ratio;
                            cx *= ratio;
                            cy *= ratio;
                            length *= ratio;
                            slots.push_back({x1, y1, x2, y2, cos2, sin2, score, classId, cx, cy, length});
                            // fprintf(stderr,"slot coord %f, %f, %f, %f,%f, %f,%f,%d\n",cx, cy, cos1, sin1, cos2, sin2, score, classId);
                            if (slots.size() > 1000) {
                                return;
                            }
                        }
                    }
                }
            }
        }
    }
}

/**

for CHW OD out
*/
void get_yolo_output3(std::vector<float> feat, int w, int h, int yoloOut, int anchor_num, int class_num, int stride, float conf_thr, int Input_w, int Input_h, float anchor_yolov5n[3][3][2], std::vector<BoxInfo>& boxes) {
    float* pO = nullptr;
    pO = &feat[0];
    int yoloOutSize = w * h;
    int num_grid_x = w;
    int num_grid_y = h;
    float ratiow = float(Input_w) / (w * stride);
    float ratioh = float(Input_h) / (h * stride);
    for (int k = 0; k < anchor_num; k++) {
        int xBeginIndex = k * (5 + class_num) * yoloOutSize;
        int yBeginIndex = xBeginIndex + yoloOutSize;
        int wBeginIndex = yBeginIndex + yoloOutSize;
        int hBeginIndex = wBeginIndex + yoloOutSize;
        int bpBeginIndex = hBeginIndex + yoloOutSize;
        int cpBeginIndex = bpBeginIndex + yoloOutSize;

        for (int y = 0; y < num_grid_y; y++) {
            for (int x = 0; x < num_grid_x; x++) {
                int index = y * num_grid_x + x;
                int bpIndex = bpBeginIndex + index;
                float bp = sigmoid(pO[bpIndex]);
                if (bp >= conf_thr) {
                    int classId = 0;
                    float maxCP = 0.0;
                    for (int o = 0; o < class_num; o++) {
                        float cp = sigmoid(pO[cpBeginIndex + o * num_grid_x * num_grid_y + index]);
                        if (cp > maxCP) {
                            maxCP = cp;
                            classId = o;
                        }
                    }
                    // box.classIndex = classId;
                    float score = maxCP * bp;
                    if (score > conf_thr) {
                        // BBox box;
                        float bx = sigmoid(pO[xBeginIndex + index]);
                        float by = sigmoid(pO[yBeginIndex + index]);
                        float bw = sigmoid(pO[wBeginIndex + index]);
                        float bh = sigmoid(pO[hBeginIndex + index]);
                        bx = (2.0f * bx - 0.5 + x) * stride;
                        by = (2.0f * by - 0.5 + y) * stride;
                        bw = 2.0f * bw;
                        bw = bw * bw * anchor_yolov5n[yoloOut][k][0];
                        bh = 2.0f * bh;
                        bh = bh * bh * anchor_yolov5n[yoloOut][k][1];

                        // bx *= ratiow;
                        // by *= ratioh;
                        // bw *= ratiow;
                        // bh *= ratioh;

                        float left = bx - bw / 2;
                        float top = by - bh / 2;
                        float right = (left + bw);
                        float bottom = (top + bh);
                        boxes.push_back({left, top, right, bottom, score, classId});
                        // fprintf(stderr,"bxby %f, %f, %f, %f,%f, %d\n",left, top, right, bottom, score, classId);
                        if (boxes.size() > 1000) {
                            return;
                        }
                    }
                }
            }
        }
    }
}
void get_yolo_one_boxes(std::vector<float> feat, int class_num, float obj_thr, int Input_w, int Input_h, int img_w, int img_h, std::vector<BoxInfo>& boxes) {
    float size = 3 * (80 * 80 + 40 * 40 + 20 * 20);
    float ratiow = float(Input_w) / img_w;
    float ratioh = float(Input_h) / img_h;
    int padw = 0;
    int padh = 0;
    int offset = class_num + 5;
    int npos = 0;
    for (int r = 0; r < size; r++) {
        float box_score = feat[npos + 4];
        if (box_score >= obj_thr) {
            int cls_id = 0;
            float max_cls_prob = 0.0;
            for (int cls = 5; cls < class_num + 5; cls++) {
                float p = feat[npos + cls];
                if (p > max_cls_prob) {
                    max_cls_prob = p;
                    cls_id = cls - 5;
                }
            }
            float cx = (feat[npos + 0]) * ratiow;
            float cy = (feat[npos + 1]) * ratioh;
            float cw = feat[npos + 2] * ratiow;
            float ch = feat[npos + 3] * ratioh;
            // cv::Mat confs = out.row(r).colRange(5, 85);
            // confs *= sc;
            // double minV = 0, maxV = 0;
            // double *minI = &minV;
            // double *maxI = &maxV;
            // cv::minMaxIdx(confs, minI, maxI);
            float left = (cx - cw / 2.0f);
            float top = (cy - ch / 2.0f);
            float right = (cx + cw / 2.0f);
            float bottom = (cy + ch / 2.0f);
            float score = box_score * max_cls_prob;
            if (left <= 0 || top <= 0 || right > Input_w || bottom > Input_h || left > right || top > bottom || score < 0.5) {
                npos += offset;
                continue;
            }

            boxes.push_back({left, top, right, bottom, score, cls_id});
        }
        npos += offset;
    }
}

//for HWC out
void get_yolo_boxes(std::vector<float> feat, int w, int h, int anchor_num, int class_num, float obj_thr, float cls_thr, int Input_w, int Input_h, int img_w, int img_h, float anchors[], std::vector<BoxInfo>& boxes) {
    float ratiow = float(Input_w) / img_w;
    float ratioh = float(Input_h) / img_h;
    int padw = 0;
    int padh = 0;
    int offset = class_num + 5;
    int num_grid_x = w;
    int num_grid_y = h;
    //const float *pdata = &(feat[0]);
    int npos = 0;
    {
        for (int k = 0; k < anchor_num; k++) {
            for (int i = 0; i < num_grid_y; i++) {
                for (int j = 0; j < num_grid_x; j++) {
                    float box_score = sigmoid(feat[npos + 4]);

                    if (box_score > obj_thr) {
                        int class_idx = 0;
                        float max_class_socre = 0;
                        for (int m = 0; m < class_num; m++) {
                            float cp = sigmoid(feat[npos + 5 + m]);
                            if (cp > max_class_socre) {
                                max_class_socre = cp;
                                class_idx = m;
                            }
                        }
                        max_class_socre *= box_score;
                        if (max_class_socre > cls_thr) {
                            float cx = (2.0 * sigmoid(feat[npos]) + j - 0.5) * (img_w / w);      /// cx
                            float cy = (2.0 * sigmoid(feat[npos + 1]) + i - 0.5) * (img_h / h);  /// cy
                            float cw = anchors[2 * k] * std::pow(sigmoid(feat[npos + 2]) * 2, 2);
                            float ch = anchors[2 * k + 1] * std::pow(sigmoid(feat[npos + 3]) * 2, 2);
                            float bx = (cx - padw) * ratiow;
                            float by = (cy - padh) * ratioh;
                            float bw = cw * ratiow;
                            float bh = ch * ratioh;
                            float left = (bx - bw / 2.0f);
                            float top = (by - bh / 2.0f);
                            float right = (bx + bw / 2.0f);
                            float bottom = (by + bh / 2.0f);
                            // fprintf(stderr,"bxby %f, %f, %f, %f,%f, %d\n",left, top, right, bottom, max_class_socre, class_idx);
                            if (left <= 0 || top <= 0 || right > Input_w || bottom > Input_h) {
                                npos += offset;
                                continue;
                            }
                            if (boxes.size() < 1000) {
                                boxes.push_back({left, top, right, bottom, max_class_socre, class_idx});
                            }
                        }
                    }
                    //row_ind++;
                    npos += offset;
                }
            }
        }
    }
}

//void draw_box(cv::Mat & cv_mat, std::vector<BoxInfo> &boxes, const std::vector<std::string> &labels, int offset)
//{
//    //int CNUM = labels.size();
//	int CNUM = 2;
//    // fprintf(stderr,"CNUM: %d\n",CNUM);
//    cv::RNG rng(0xFFFFFFFF);
//	cv::Scalar_<int> randColor[CNUM];
//	for (int i = 0; i < CNUM; i++)
//		rng.fill(randColor[i], cv::RNG::UNIFORM, 0, 256);
//    int img_w = cv_mat.cols;
//    int img_h = cv_mat.rows;
//    for(auto box : boxes)
//    {
//        // int width = (box.x2-box.x1)*img_w;
//        // int height = (box.y2-box.y1)*img_h;
//        // int x = box.x1*img_w;
//        // int y = box.y1*img_h;
//        int width = (box.x2-box.x1);
//        int height = (box.y2-box.y1);
//        int x = box.x1;
//        int y = box.y1;
//        // fprintf(stderr,"box type: %s, confidence: %0.3f, axis: top-left-x: %d, top-left-y: %d, width: %d, height: %d\n",
//        //     labels[box.label+offset].c_str(), box.score, x, y, width, height);
//        cv::Point p = cv::Point(x, y);
//        cv::Rect rect = cv::Rect(x, y, width, height);
//        cv::rectangle(cv_mat, rect, cv::Scalar(255,0,0));/*randColor[box.label+offset]);;*/
//        std::string text = labels[box.label+offset] + ":" + std::to_string(box.score) ;
//        cv::putText(cv_mat, text, p, cv::FONT_HERSHEY_PLAIN, 1, randColor[box.label+offset]);
//    }
//}
// int astride = class_num + 5;
// int c_stride = 1;
// int j_stride = astride*anchor_num;
// int i_stride = j_stride*w;
// if (type == "CHW")
// {
//     j_stride = 1;
//     i_stride = w;
//     c_stride = w*h;
// }
// for (int i = 0; i < h; i++)
// {
//     for(int j = 0; j < w; j++)
//     {
//         for(int k = 0; k < anchor_num; k ++){
//             float tx = feat[i*i_stride + j*j_stride + (0 + astride * k)*c_stride];
//             float ty = feat[i*i_stride + j*j_stride + (1 + astride * k)*c_stride];
//             float tw = feat[i*i_stride + j*j_stride + (2 + astride * k)*c_stride];
//             float th = feat[i*i_stride + j*j_stride + (3 + astride * k)*c_stride];
//             float cf = feat[i*i_stride + j*j_stride + (4 + astride * k)*c_stride];

//             // float bx = (2.0*tx -0.5 + j) * (img_w / w);
//             // float by = (2.0*ty -0.5 +i) * (img_h / h);
//             float bx = (2.0*sigmoid(tx) -0.5 + j) * (img_w / w);
//             float by = (2.0*sigmoid(ty) -0.5 +i) * (img_h / h);
//             float bw = anchors[2*k] * std::pow(sigmoid(tw)*2,2) ;
//             float bh = anchors[2*k+1] * std::pow(sigmoid(th)*2,2) ;
//             // float bx = (sigmoid(tx) + j) / (float)w;
//             // float by = (sigmoid(ty) + i) / (float)h;
//             // float bw = anchors[2*k] * std::exp(tw) / (float)img_w;
//             // float bh = anchors[2*k+1] * std::exp(th) / (float)img_h;
//             float b_confidence = sigmoid(cf);
//             // float b_confidence = cf;
//             float b_class_score = 0.0f;
//             int b_class_index = -1;
//             for(int c = 0; c < class_num; c++)
//             {
//                 float b_scores = b_confidence*sigmoid(feat[i*i_stride + j*j_stride + (5 + astride * k+c)*c_stride]);
//                 // float b_scores = b_confidence*(feat[i*i_stride + j*j_stride + (5 + astride * k+c)*c_stride]);
//                 if (b_scores > conf_thr && b_scores > b_class_score)
//                 {
//                     b_class_score = b_scores;
//                     b_class_index = c;
//                 }
//             }

//             if (b_class_index >= 0)
//             {
//                 fprintf(stderr,"bxby %f, %f, %f, %f, %d\n",bx,by,bw,bh, b_class_index);
//                 boxes.push_back({(bx - bw /2.0f), (by - bh /2.0f), (bx + bw /2.0f), (by + bh /2.0f), b_class_score, b_class_index});
//             }
//         }
//     }
// }

// void get_yolo_box2(std::vector<float> ort_outputs, int w, int h, int stride, int img_w, int img_h, float anchors[], float obj_thr, float score_thr)
// {
//     int q,i,j,k;

//     int newh = 640, neww = 640, padh = 0, padw = 0;
//     float ratioh = (float)img_h / newh, ratiow = (float)img_w / neww;

// 	int classNum = 2;
// 	{
// 		const float* pdata = &(ort_outputs[0]);

// 		int offset = 2 + 5;

// 		int num_grid_x = w;
// 		int num_grid_y = h;
// 		for (q = 0; q < 3; q++)    ///anchor
// 		{
// 			const float anchor_w = anchors[q * 2];
// 			const float anchor_h = anchors[q * 2 + 1];
// 			for (i = 0; i < num_grid_y; i++)
// 			{
// 				for (j = 0; j < num_grid_x; j++)
// 				{
// 					float box_score = sigmoid(pdata[4]);

// 					if (box_score > obj_thr)
// 					{
// 						int class_idx = 0;
// 						float max_class_socre = 0;
// 						for (k = 0; k < classNum; k++)
// 						{
// 							float cp = sigmoid(pdata[5 + k]);
// 							if (cp > max_class_socre)
// 							{
// 								max_class_socre = cp;
// 								class_idx = k;
// 							}
// 						}
// 						max_class_socre *= box_score;
// 						if (max_class_socre > score_thr)
// 						{
// 							float cx = (sigmoid(pdata[0]) + j) * stride;  ///cx
// 							float cy = (sigmoid(pdata[1]) + i) * stride;   ///cy
// 							float w = expf(pdata[2]) * anchor_w;   ///w
// 							float h = expf(pdata[3]) * anchor_h;  ///h
// 							cx = (cx - padw) * ratiow;
// 							cy = (cy - padh) * ratioh;
// 							w *= ratiow;
// 							h *= ratioh;

// 							// BBox boxInfo = BBox(class_idx, max_class_socre, cx - w / 2, cy - h / 2, w, h, 0);

// 							// generate_boxes.push_back(boxInfo);
// 						}
// 					}
// 					//row_ind++;
// 					pdata += (2 + 5);
// 				}
// 			}
// 		}
// 	}
// }
