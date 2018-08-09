
#include <opencv2/opencv.hpp>
#include <iostream>
#include<opencv2/video/video.hpp>
#include<opencv2/video/background_segm.hpp>
#include <opencv2/aruco.hpp>



using namespace std;
using namespace cv;

int main()
{
    Mat image;
    image = imread("b.jpg", CV_LOAD_IMAGE_COLOR); 

    Mat gray;
    cvtColor(image, gray, CV_BGR2GRAY);

    if(! gray.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    int numSquares=6;

    Size patternsize(6,6); //interior number of corners

    vector<Point2f> corners;

    bool patternfound = findChessboardCorners(gray, patternsize, corners);

    if(!patternfound)
    { cout <<  "No corner detected" << std::endl ;
    }
    
    if(patternfound)

    {
        cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
        TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        drawChessboardCorners(image, patternsize, Mat(corners), patternfound);
    


        vector<vector<Point3f> > object_points;
        vector<vector<Point2f> > image_points;
        vector<vector<Point2f> > check_image_points;
        Point check_image_point;

        vector<Point3f> obj;
        for(int j=0;j<36;j++)
            obj.push_back(Point3f(j/numSquares, j%numSquares, 0.0f));

        image_points.push_back(corners);
        object_points.push_back(obj);

        
        Mat cameraMatrix = Mat(3, 3, CV_64FC1);
        Mat distCoeffs;

        vector<Mat> rvecs;
        vector<Mat> tvecs;
        
        calibrateCamera(object_points, image_points, image.size(), cameraMatrix, distCoeffs, rvecs, tvecs);

        Mat rvec;
        Mat tvec;

        /*

        cv::Mat cameraMatrix(3, 3, CV_64FC1);
		cv::Mat distCoeffs(5, 1, CV_64FC1);

		
		
		cv::setIdentity(cameraMatrix);

		cameraMatrix.at<double>(0, 0) = 0.5;
		cameraMatrix.at<double>(1, 1) = 0.5;
		cameraMatrix.at<double>(0, 2) = 0.5;
		cameraMatrix.at<double>(1, 2) = 0.5;


		distCoeffs.at<double>(0, 0) = 0.5;
		distCoeffs.at<double>(1, 0) = 0.5;
		distCoeffs.at<double>(2, 0) = 0.5;
		distCoeffs.at<double>(3, 0) = 0.5;
		distCoeffs.at<double>(4, 0) = 0.5;

        */
        
        vector<Point3f> object_point = object_points[0];
        vector<Point2f> image_point = image_points[0];
        solvePnP(object_point, image_point, cameraMatrix, distCoeffs, rvec, tvec);

        vector<Point2f> image_points2;

        //draw axis 
		float length = 2.0f;
		vector<Point3f> axisPoints;
		axisPoints.push_back(Point3f(0, 0, 0));
		axisPoints.push_back(Point3f(length, 0, 0));
		axisPoints.push_back(Point3f(0, length, 0));
		axisPoints.push_back(Point3f(0, 0, length));
		vector< Point2f > imagePoints;
		projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

		// draw axis lines
		line(image, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);
		line(image, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 3);
		line(image, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);

        float tvecX=tvec.at<double>(0,0)*0.032;
        float tvecY=tvec.at<double>(1,0)*0.032;
        float tvecZ=tvec.at<double>(2,0)*0.032;

        double fx = cameraMatrix.at<double>(0,0);
        double fy = cameraMatrix.at<double>(1,1);
        double cx = cameraMatrix.at<double>(0,2);
        double cy = cameraMatrix.at<double>(1,2);

        
        cv::Point2f final_point(float(fx*tvecX/tvecZ+cx),float(fy*tvecY/tvecZ+cy));
        circle(image,final_point ,100,Scalar(1,1,1),5);


    
        //Point2f p=image_points2[0];

        //circle(image,p,20,Scalar(1,1,1),5);

        //aruco::drawAxis(image, intrinsic, distCoeffs, rvec, tvec, 3.0f);

        //Convert rotation vector into rotation matrix 
        Mat rot;
        //Mat rotTransMat;
        Rodrigues(rvec, rot);

      
        

        //cout << cameraMatrix.at<double>(0, 2);


        //Mat Chess_position_cameraspace = tvec;


        
        //Mat imageUndistorted;
        //undistort(image, imageUndistorted, intrinsic, distCoeffs);

        //Get rotation matrices
        //for( int i = 0; i < 36; i++ ){
            //Mat rot;
            //Rodrigues(rvecs[i], rot);
            //rotation[i]=rot;
            //projectPoints(object_points[i],rvecs[i],tvecs[i],intrinsic,distCoeffs,check_image_point);
            //check_image_points[i]=check_image_point;
            //circle(image,check_image_point,2,Scalar(100,10,10));
            //aruco::drawAxis(image, intrinsic, distCoeffs, rvecs[0], tvecs[0], 0.5f);
        //}

        }


    namedWindow( "Display window", WINDOW_AUTOSIZE ); 
    imshow( "Display window", image ); 
    
                              

    waitKey(0);                                      
}
