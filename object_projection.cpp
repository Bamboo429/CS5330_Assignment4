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

    // read the parameters saved in the file
    vector<float> camera_coeff, dist_coeff;
    readInstrinsic("intrinsic_parameters.txt", camera_coeff, dist_coeff);

    // initialize matrix to save parameters
    cv::Mat camera_matrix, dist_matrix;// = cv::Mat(3, 3, CV_64FC1, &arr);
    camera_matrix.create(3,3, CV_64FC1);
    dist_matrix.create(1,5, CV_64FC1);

    // camera matrix
    for (int i=0; i< camera_coeff.size(); i++){
        camera_matrix.at<double> (i) = camera_coeff[i];
    }
    std::cout << "camera matrix: " << endl << camera_matrix << endl;

    // dist coeff
    for (int i=0; i<dist_coeff.size(); i++){
        dist_matrix.at<double> (i) = dist_coeff[i];
    }
    std::cout << "distortion coefficients: " << endl << dist_matrix << endl;

    std::vector<cv::Point3f> point_set;
    int fieldSize = 18;
    cv::Size patternsize(9,6); //interior number of corners
    vector<Point2f> corners; //this will be filled by the detected corners
    cv::Mat rvecs, tvecs;

    // set real world coordinate
    for(int i=0; i<patternsize.height; i++){
        for(int j=0; j<patternsize.width; j++){
            point_set.push_back( cv::Point3f(j*fieldSize, -i*fieldSize, 0));
        }
    }

    int s=0; // save image index
    // flag for different mode
    bool flag_p = false;
    bool flag_x = false;
    bool flag_v = false;
    bool flag_h = false;

    for (;;) {
        *capdev >> frame; // get a new frame from the camera, treat as a stream

        if (frame.empty()) {
            printf("frame is empty\n");
            break;
        }

        // BGR to gray image
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // find chessboard corners
        bool patternfound = findChessboardCorners(gray, patternsize, corners,
                                                  CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                                                  + CALIB_CB_FAST_CHECK);

        // if find the chess board
        if(patternfound) {
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));

            //drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);

            // get the board's pose (rotation and translation)
            solvePnP(point_set, corners, camera_matrix, dist_matrix, rvecs, tvecs);

            // shot the rotation and translation matrix
            if (flag_p) {
                std::cout << "rotation matrix:" << endl;
                std::cout << rvecs << endl;
                std::cout << "translation matrix:" << endl;
                std::cout << tvecs << endl;
            }

            // Project Outside Corners and 3D Axes
            else if(flag_x) {
                //project 3D points to image plane
                vector<Point3f> axis, corner;
                vector<Point2f> img_points;

                // set the corners' location
                corner.push_back(cv::Point3f(-1 * fieldSize, 1 * fieldSize, 0)); // set length as 3 times square size
                corner.push_back(cv::Point3f(9 * fieldSize, 1 * fieldSize, 0));
                corner.push_back(cv::Point3f(-1 * fieldSize, -6 * fieldSize, 0));
                corner.push_back(cv::Point3f(9 * fieldSize, -6 * fieldSize, 0));
                // project 4 corners
                projectPoints(corner, rvecs, tvecs, camera_matrix, dist_matrix, img_points);

                // draw
                circle(frame, img_points[0], 5, Vec3b{0, 215, 255}, 10);
                circle(frame, img_points[1], 5, Vec3b{0, 215, 255}, 10);
                circle(frame, img_points[2], 5, Vec3b{0, 215, 255}, 10);
                circle(frame, img_points[3], 5, Vec3b{0, 215, 255}, 10);

                // set the axis location
                axis.push_back(cv::Point3f(3 * fieldSize, 0, 0)); // set length as 3 times square size
                axis.push_back(cv::Point3f(0, -3 * fieldSize, 0));
                axis.push_back(cv::Point3f(0, 0, 3 * fieldSize));

                // project the 3D axis
                projectPoints(axis, rvecs, tvecs, camera_matrix, dist_matrix, img_points);

                // draw x, y, z axis
                line(frame, corners[0], img_points[0], Vec3b{255, 0, 0}, 5);
                line(frame, corners[0], img_points[1], Vec3b{0, 255, 0}, 5);
                line(frame, corners[0], img_points[2], Vec3b{0, 0, 255}, 5);

            }

            else if(flag_v){
                //draw virtual object, cubic
                vector<Point3f> axis;
                vector<Point2f> img_points;
                axis.push_back(cv::Point3f(0 * fieldSize, 0 * fieldSize, 1 * fieldSize)); //0
                axis.push_back(cv::Point3f(0 * fieldSize, -3 * fieldSize, 0 * fieldSize)); //1
                axis.push_back(cv::Point3f(0 * fieldSize, -3 * fieldSize, 1 * fieldSize)); //2
                axis.push_back(cv::Point3f(5 * fieldSize, 0 * fieldSize, 1 * fieldSize)); //3
                axis.push_back(cv::Point3f(5 * fieldSize, 0 * fieldSize, 0 * fieldSize)); //4
                axis.push_back(cv::Point3f(5 * fieldSize, -3 * fieldSize, 1 * fieldSize)); //5
                axis.push_back(cv::Point3f(5 * fieldSize, -3 * fieldSize, 0 * fieldSize)); //6

                projectPoints(axis, rvecs, tvecs, camera_matrix, dist_matrix, img_points);
                line(frame, img_points[0], img_points[3], Vec3b{0, 215, 255}, 5); //(0,0,1) - (5,0,1)
                line(frame, img_points[0], img_points[2], Vec3b{0, 215, 255}, 5); //(0,0,1) - (0,-3,1)
                line(frame, img_points[2], img_points[5], Vec3b{0, 215, 255}, 5); //(0,-3,1) - (5,-3,1)
                line(frame, img_points[3], img_points[5], Vec3b{0, 215, 255}, 5); //(5,0,1) - (5,-3,1)
                line(frame, corners[0], img_points[1], Vec3b{0, 215, 255}, 5); //(0,0,0) - (0,-3,0)
                line(frame, corners[0], img_points[4], Vec3b{0, 215, 255}, 5); //(0,0,0) - (5,0,0)
                line(frame, img_points[1], img_points[6], Vec3b{0, 215, 255}, 5); //(0,-3,0) - (5,-3,0)
                line(frame, img_points[4], img_points[6], Vec3b{0, 215, 255}, 5); //(5,0,0) - (5,-3,0)
                line(frame, corners[0], img_points[0], Vec3b{0, 215, 255}, 5); //(0,0,0) - (0,0,1)
                line(frame, img_points[1], img_points[2], Vec3b{0, 215, 255}, 5); //(0,-3,0) - (0,-3,1)
                line(frame, img_points[6], img_points[5], Vec3b{0, 215, 255}, 5); //(5,-3,0) - (5,-3,1)
                line(frame, img_points[4], img_points[3], Vec3b{0, 215, 255}, 5); //(5,0,0) - (5,0,1)
            }

        }

        // harris corners feature
        if (flag_h) {
            //set parameter
            int blockSize = 7;
            int apertureSize = 5;
            double k = 0.04;
            Mat dst = Mat::zeros(frame.size(), CV_32FC1);

            // cal Harris corner
            cornerHarris(gray, dst, blockSize, apertureSize, k);

            Mat dst_norm, dst_norm_scaled;
            normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
            convertScaleAbs(dst_norm, dst_norm_scaled);

            double min, max;
            cv::minMaxLoc(dst, &min, &max);


            // show the corner
            for (int i = 0; i < dst.rows; i++) {
                for (int j = 0; j < dst.cols; j++) {
                    if( dst.at<float>(i, j) > max*0.01) {
                        circle(frame, Point(j, i), 2, Scalar(0, 0, 255), 2);
                    }
                }
            }

        }
        char key = cv::waitKey(10);
        if( key == 'q') {
            break;
        }
        else if(key == 's'){
            s += 1;
            string filename = "cameraPos_";
            filename = filename + to_string(s) + ".jpg";

            cv::imwrite(filename, frame);
            std::cout << filename << "   saved" << endl;
        }

        else if(key == 'p'){
            flag_p = true;
            flag_x = false;
            flag_v = false;
            flag_h = false;
        }

        else if(key == 'x'){
            flag_p = false;
            flag_x = true;
            flag_v = false;
            flag_h = false;
        }

        else if(key == 'v'){
            flag_p = false;
            flag_x = false;
            flag_v = true;
            flag_h = false;
        }
        else if(key == 'h'){
            flag_p = false;
            flag_x = false;
            flag_v = false;
            flag_h = true;
        }
        cv::imshow("Video", frame);
        //cv::imshow("Harris", dst_norm_scaled);

    }
    delete capdev;
    return 0;

}
