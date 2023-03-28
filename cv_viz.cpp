//
// Created by Chu-Hsuan Lin on 2022/3/24.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/viz.hpp>

using namespace std;
using namespace cv;

int main_v() {
    viz::Viz3d window("Coordinate Frame");
    window.setWindowSize(Size(500, 500));
    window.setWindowPosition(Point(150, 150));
    window.setBackgroundColor(); // black by default
    cv::viz::Mesh x;
    x = cv::viz::Mesh::load("/Users/chuhsuanlin/Desktop/test.obj", cv::viz::Mesh::LOAD_OBJ);
    cv::viz::WMesh a(x);
/// Create and show widgets


    window.showWidget("point_cloud", a);


/// Wait for key 'q' to close the window
    cout << endl << "Press 'q' to close each windows ... " << endl;

    window.spin();
}