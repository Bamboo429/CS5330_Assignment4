//
// Created by Chu-Hsuan Lin on 2022/3/16.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstdio>
#include "txt_util.h"

using namespace std;
using namespace cv;


int main() {

    cv::VideoCapture *capdev;
    cv::Mat frame, gray;

    // open the video device
    capdev = new cv::VideoCapture(0);
    if (!capdev->isOpened()) {
        printf("Unable to open video device\n");
        return (-1);
    }
    if (!capdev->isOpened()) {
        printf("Unable to open video device\n");
        return (-1);
    }

    // set image size for quick computation
    //capdev->set(cv::CAP_PROP_FRAME_WIDTH,720);
    //capdev->set(cv::CAP_PROP_FRAME_HEIGHT,1280);

    // get some properties of the image
    cv::Size refS((int) capdev->get(cv::CAP_PROP_FRAME_WIDTH),
                  (int) capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Expected size: %d %d\n", refS.width, refS.height);

    cv::namedWindow("Video", 1); // identifies a window


    int s=0; // save image index
    for (;;) {
        *capdev >> frame; // get a new frame from the camera, treat as a stream

        if (frame.empty()) {
            printf("frame is empty\n");
            break;
        }

        // BGR to gray image
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // harris corners feature
        //set parameter
        int blockSize = 7;
        int ksize = 5;
        double k = 0.04;
        Mat dst = Mat::zeros(frame.size(), CV_32FC1);

        // cal Harris corner
        cornerHarris(gray, dst, blockSize, ksize, k);

        Mat dst_norm, dst_norm_scaled;
        normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
        convertScaleAbs(dst_norm, dst_norm_scaled);

        double min, max;
        cv::minMaxLoc(dst, &min, &max);


        // show the corner
        int count =0;
        for (int i = 0; i < dst_norm.rows; i++) {
            for (int j = 0; j < dst_norm.cols; j++) {
                if( dst.at<float>(i, j) > max*0.01) {
                    //count++;
                    //std::cout << "v " <<  dst_norm.at<float>(i, j) << endl;
                    //std::cout << "pos " << j << "  " << i << endl;
                    circle(frame, Point(j, i), 2, Scalar(0, 0, 255), 2);
                }
            }
        }
        //std::cout << "count " << count << endl;


        char key = cv::waitKey(10);
        if( key == 'q') {
            break;
        }
        else if(key == 's'){
            s += 1;
            string filename = "Harris";
            filename = filename + to_string(s) + ".jpg";

            cv::imwrite(filename, frame);
            std::cout << filename << "   saved" << endl;
        }

        //cv::imshow("Video", gray);
        cv::imshow("Harris", frame);

    }
    delete capdev;
    return 0;

}
