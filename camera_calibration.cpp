#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstdio>

using namespace std;
using namespace cv;

int main() {

    cv::VideoCapture *capdev;
    cv::Mat frame,gray;

    // open the video device
    capdev = new cv::VideoCapture(0 );
    if( !capdev->isOpened() ) {
        printf("Unable to open video device\n");
        return(-1);
    }
    if (!capdev->isOpened()) {
        printf("Unable to open video device\n");
        return (-1);
    }

    // set image size for quick computation
    // capdev->set(cv::CAP_PROP_FRAME_WIDTH,720);
    // capdev->set(cv::CAP_PROP_FRAME_HEIGHT,1280);

    // get some properties of the image
    cv::Size refS((int) capdev->get(cv::CAP_PROP_FRAME_WIDTH),
                  (int) capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Expected size: %d %d\n", refS.width, refS.height);

    cv::namedWindow("Video", 1); // identifies a window

    std::vector<cv::Point3f> point_set;
    std::vector<std::vector<cv::Point3f> > point_list;
    std::vector<std::vector<cv::Point2f> > corner_list;

    int fieldSize = 18; // chessboard size
    cv::Size patternsize(9,6); //interior number of corners
    vector<Point2f> corners;
    vector<Point2f> last_corner;

    // real world ordinate
    for(int i=0; i<patternsize.height; i++){
        for(int j=0; j<patternsize.width; j++){
            point_set.push_back( cv::Point3f(j*fieldSize, i*fieldSize, 0));
        }
    }

    int s = 0; // save index
    for(;;) {
        *capdev >> frame; // get a new frame from the camera, treat as a stream

        if (frame.empty()) {
            printf("frame is empty\n");
            break;
        }

        // Detect and Extract Chessboard Corners
        // BGR to gray image
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // find chessboard corners
        bool patternfound = findChessboardCorners(gray, patternsize, corners,
                                                  CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                                                  + CALIB_CB_FAST_CHECK);

        // if corner exist
        if(patternfound) {
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));

            // draw
            drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);
        }

        char key = cv::waitKey(10);
        if( key == 'q') {
            break;
        }

        // save calibrate images
        else if (key == 's' and patternfound){
            s += 1;
            corner_list.push_back(corners);
            point_list.push_back(point_set);

            //char c = to_string(s).c_str();
            string filename = "calibrate_";
            filename = filename + to_string(s) + ".jpg";

            cv::imwrite(filename, frame);
            std::cout << filename << "   saved" << endl;

        }

        // calibrate camera
        else if (key == 'c'){

            std::vector<float> dist_coeff;
            std::vector<cv::Mat> rvecs, tvecs;
            int flag = cv::CALIB_FIX_ASPECT_RATIO;
            int minNumber = 5; //set min number of calibrate images

            // initialize camera matrix
            cv::Mat camera_matrix;// = cv::Mat(3, 3, CV_64FC1, arr);
            camera_matrix.create(3,3, CV_64FC1);
            camera_matrix.at<double>(0,0) = 1;
            camera_matrix.at<double>(0,1) = 0;
            camera_matrix.at<double>(0,2) = (frame.cols)/2.0;
            camera_matrix.at<double>(1,0) = 0;
            camera_matrix.at<double>(1,1) = 1;
            camera_matrix.at<double>(1,2) = (frame.rows)/2.0;
            camera_matrix.at<double>(2,0) = 0;
            camera_matrix.at<double>(2,1) = 0;
            camera_matrix.at<double>(2,2) = 1;
            //std::cout << camera_matrix.at<float>(1,1) << endl;

            // save camera matrix, dist coef file
            FILE *file;

            // calibrate images more than min number
            if (s>minNumber){

                file=fopen("intrinsic_parameters.txt","w"); // save file name

                std::cout << "calibrating ....." << endl;

                // calibrateCamera
                double err = cv::calibrateCamera(point_list, corner_list, refS, camera_matrix, dist_coeff, rvecs, tvecs, flag);

                // save and print parameter
                // camera matrix
                std::cout << "camera matrix: " << endl << camera_matrix << endl;
                fprintf(file, "camera matrix:");
                fprintf (file,"\n");
                for (int i=0;i<camera_matrix.rows;i++){
                    for (int j=0;j<camera_matrix.cols;j++){
                        fprintf (file, "%f",camera_matrix.at<double>(i,j));
                        fprintf (file, ",");
                    }
                }
                fprintf (file,"\n");

                //dist coef
                std::cout << "distortion coefficients: " << endl;
                fprintf(file, "distortion coefficients:");
                fprintf (file,"\n");
                for (float coef: dist_coeff){
                    std::cout << coef ;
                    std::cout << ", " ;
                    fprintf (file, "%f",coef);
                    fprintf (file, ",");
                }
                fclose (file);
                std::cout << endl;
                std::cout << "re-projection error: " << err << endl;
            }
            else{
                std::cout << " calibration images are not enough, please collect more than 5! " << endl;
            }
        }

        cv::imshow("Video", frame);
    }

    delete capdev;
    return(0);
}




