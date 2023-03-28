//
// Created by Chu-Hsuan Lin on 2022/3/21.
//

// Aruco library application

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "txt_util.h"

using namespace std;
using namespace cv;

int getIndex(vector<int> v, int K)
{
    int index;
    auto it = find(v.begin(), v.end(), K);

    // If element was found
    if (it != v.end()){
        // calculating the index of K
        index = it - v.begin();
    }
    else {
        index = -1;
    }

    return index;
}

int main() {

    //prepare - read and save Aruco marker
    // If no markers open commend here
    /*
    cv::Mat markerImage1, markerImage2, markerImage3, markerImage4, markerImage5;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::drawMarker(dictionary, 23, 200, markerImage1, 1);
    cv::imwrite("marker23.png", markerImage1);
    cv::aruco::drawMarker(dictionary, 50, 200, markerImage2, 1);
    cv::imwrite("marker50.png", markerImage2);
    cv::aruco::drawMarker(dictionary, 104, 200, markerImage3, 1);
    cv::imwrite("marker104.png", markerImage3);
    cv::aruco::drawMarker(dictionary, 218, 200, markerImage4, 1);
    cv::imwrite("marker218.png", markerImage4);
    cv::aruco::drawMarker(dictionary, 159, 200, markerImage5, 1);
    cv::imwrite("159.png", markerImage5);
    */  // If no markers open until here

    // read project images
    string img_foloder = "/Users/chuhsuanlin/Desktop/Aruco image/";
    cv::Mat img_23 = cv::imread(img_foloder + "m_23.jpeg");
    cv::Mat img_50 = cv::imread(img_foloder + "m_50.jpeg");
    cv::Mat img_104 = cv::imread(img_foloder + "m_104.jpeg");
    cv::Mat img_159 = cv::imread(img_foloder + "m_159.jpeg");
    cv::Mat img_218 = cv::imread(img_foloder + "m_218.jpeg");

    // save images as vector <cv::Mat>
    vector<cv::Mat> img;
    img.push_back(img_23);
    img.push_back(img_50);
    img.push_back(img_104);
    img.push_back(img_159);
    img.push_back(img_218);

    //set id number
    vector<int> imgIds = {23,50,104,159,218};

    vector<vector<Point2f> > imgCorners;
    vector<Point2f > corner;

    // set real world corner location
    for(int i=0; i< img.size();i++){

        corner.push_back(Point2f(0,0));
        corner.push_back(Point2f(img[i].cols,0));
        corner.push_back(Point2f(img[i].cols,img[i].rows));
        corner.push_back(Point2f(0,img[i].rows));
        imgCorners.push_back(corner);
        corner.clear();
    }
    //std::cout << imgCorners[1] << endl;

    // read camera parameters
    vector<float> camera_coeff, dist_coeff;
    readInstrinsic("intrinsic_parameters.txt", camera_coeff, dist_coeff);

    // camera matrix
    cv::Mat camera_matrix, dist_matrix;// = cv::Mat(3, 3, CV_64FC1, &arr);
    camera_matrix.create(3,3, CV_64FC1);
    dist_matrix.create(1,5, CV_64FC1);
    for (int i=0; i< camera_coeff.size(); i++){
        camera_matrix.at<double> (i) = camera_coeff[i];
    }

    //dist coef
    for (int i=0; i<dist_coeff.size(); i++){
        //std::cout << i << dist_coeff[i] << endl;
        dist_matrix.at<double> (i) = dist_coeff[i];
    }

    cv::VideoCapture *capdev;
    cv::Mat frame, gray, imOut;
    cv::Mat frame_ar;

    bool flag_n = false;
    bool flag_x = false;
    bool flag_a = false;

    int s = 0; //save index

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

    // get some properties of the image
    cv::Size refS((int) capdev->get(cv::CAP_PROP_FRAME_WIDTH),
                  (int) capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Expected size: %d %d\n", refS.width, refS.height);

    cv::namedWindow("Video", 1); // identifies a window

    for (;;) {
        *capdev >> frame; // get a new frame from the camera, treat as a stream

        if (frame.empty()) {
            printf("frame is empty\n");
            break;
        }

        // markers id detection
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);


        if (markerIds.size() > 0) {

            if(flag_n) {
                // show and draw id number
                cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
            }

            // marker pose estimation
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, camera_matrix, dist_matrix, rvecs, tvecs);

            if(flag_x) {
                // draw axes for each marker
                for (int i = 0; i < markerIds.size(); i++) {
                    //std::cout << "m" << markerIds[i] << endl;
                    cv::aruco::drawAxis(frame, camera_matrix, dist_matrix, rvecs[i], tvecs[i], 0.1);
                }
            }

            //AR , project images on the markers
            if(flag_a) {
                for (int i = 0; i < markerIds.size(); i++) {

                    int index = getIndex(imgIds, markerIds[i]); //index of marker id
                    if (index != -1) {
                        // Compute homography from source and destination points
                        cv::Mat h = cv::findHomography(imgCorners[index], markerCorners[i]);
                        // Warped image
                        cv::Mat warpedImage;
                        // warp source image to destination
                        warpPerspective(img[index], warpedImage, h, frame.size());

                        // transfer data type
                        vector<Point2i> intVec;
                        cv::Mat(markerCorners[i]).convertTo(intVec, cv::Mat(intVec).type());

                        // mask for copy img to frame
                        Mat mask = Mat::zeros(frame.rows, frame.cols, CV_8UC1);
                        fillConvexPoly(mask, intVec, Scalar(255, 255, 255));

                        // Erode the mask to not copy the boundary effects from the warping
                        Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
                        erode(mask, mask, element);

                        // Copy the  warped image into the frame, in the mask region
                        imOut = frame.clone();
                        warpedImage.copyTo(frame, mask);

                    }
                }
            }
        }

        // key choose
        char key = cv::waitKey(10);
        if( key == 'q') {
            break;
        }

        else if (key =='s'){
            s += 1;
            string filename = "arcuo_";
            filename = filename + to_string(s) + ".jpg";

            cv::imwrite(filename, frame);
            std::cout << filename << "   saved" << endl;
        }
        // show detected ID
        else if(key == 'n'){
            flag_n = true;
            flag_x = false;
            flag_a = false;
        }

        // draw axex
        else if(key == 'x'){
            flag_n = false;
            flag_x = true;
            flag_a = false;
        }

        // AR, project image
        else if(key == 'a'){
            flag_n = false;
            flag_x = false;
            flag_a = true;
        }

        cv::imshow("Video", frame);
    }

    delete capdev;
    return(0);

}


