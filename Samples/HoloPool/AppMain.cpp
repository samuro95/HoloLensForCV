
#include "pch.h"

#include "AppMain.h"

#define ENABLE_HOLOLENS_RESEARCH_MODE_SENSORS 1

using namespace cv;
using namespace std;



using namespace Concurrency;
using namespace Platform;
using namespace Windows::Foundation;
using namespace Windows::Foundation::Numerics;
using namespace Windows::Graphics::Holographic;
using namespace Windows::Graphics::Imaging;
using namespace Windows::Media::Capture;
using namespace Windows::Media::Capture::Frames;
using namespace Windows::Media::Devices::Core;
using namespace Windows::Perception::Spatial;
using namespace Windows::UI::Input::Spatial;

using namespace std::placeholders;

using namespace Microsoft::WRL::Wrappers;
using namespace Microsoft::WRL;

using namespace DirectX;

namespace HoloPool
{
	AppMain::AppMain(
		const std::shared_ptr<Graphics::DeviceResources>& deviceResources)
		: Holographic::AppMainBase(deviceResources)
		, _selectedHoloLensMediaFrameSourceGroupType(
			HoloLensForCV::MediaFrameSourceGroupType::PhotoVideoCamera)
		, _holoLensMediaFrameSourceGroupStarted(false)
		, _undistortMapsInitialized(false)
		, _isActiveRenderer(false)
		, _isPoolDetected(false)
		, m_WorldCoordinateSystem_Set(false)
	{
	}

	//Constructs a plane From a point and the normal 
	float4 AppMain::ConstructPlaneFromPointNormal(Windows::Foundation::Numerics::float3 Point, Windows::Foundation::Numerics::float3 normal)
	{
		float4 plane;
		// plane definded by ax+by+cz+d = 0 avec a=plane.x / b=plane.y / c= plane.z / d=plane.w
		float3 Normalized_normal = normalize(normal);
		plane.x = Normalized_normal.x;
		plane.y = Normalized_normal.y;
		plane.z = Normalized_normal.z;
		plane.w = -dot(Point, Normalized_normal);
		return plane;
	}

	float4 AppMain::ConstructPlaneFromPoints(Windows::Foundation::Numerics::float3 Point0, Windows::Foundation::Numerics::float3 Point1, Windows::Foundation::Numerics::float3 Point2)
	{
		float3 normal = normalize(cross(Point1 - Point0, Point2 - Point0));
		return ConstructPlaneFromPointNormal(Point0, normal);
	}

	//Computes the intersection between a plane and a line defines by 2 points
	float3 AppMain::IntersectionLinePlane(float3 p1, float3 p2, float4 plane)
	{
		float3 inters;
		float3 diff = (p1 - p2);
		float Denominator = plane.x * diff.x + plane.y * diff.y + plane.z * diff.z;
		if (Denominator == 0.0f)
		{
			inters = (p1 + p2) * 0.5f;
		}
		else
		{
			float u = (plane.x * p1.x + plane.y * p1.y + plane.z * p1.z + plane.w) / Denominator;
			inters = (p1 + u * (p2 - p1));
		}
		return inters;
	}


	//Computes the intersection between a plane and a line defines by 1 point and 1 vector
	float3 AppMain::IntersectionLinePointVectorPlane(float3 p1, float3 vector, float4 plane)
	{
		float3 inters;
		float3 vect = -vector;
		float Denominator = plane.x * vect.x + plane.y * vect.y + plane.z * vect.z;
		if (Denominator == 0.0f)
		{
			inters = p1;
		}
		else
		{
			float u = (plane.x * p1.x + plane.y * p1.y + plane.z * p1.z + plane.w) / Denominator;
			inters = (p1 + u * vect);
		}
		return inters;
	}

	//Transform a Point from one coordinate system to another, with the transformation defined by Transform. 
	float3 AppMain::m_transform(float3 In , Windows::Foundation::Numerics::float4x4 Transform)
	{
		Mat tvec(3, 1, cv::DataType<double>::type); // translation vector
		Mat R(3, 3, cv::DataType<double>::type); //rotation vector 

		tvec.at<double>(0, 0) = Transform.m41;
		tvec.at<double>(1, 0) = Transform.m42;
		tvec.at<double>(2, 0) = Transform.m43;

		R.at<double>(0, 0) = Transform.m11;
		R.at<double>(1, 0) = Transform.m12;
		R.at<double>(2, 0) = Transform.m13;
		R.at<double>(0, 1) = Transform.m21;
		R.at<double>(1, 1) = Transform.m22;
		R.at<double>(2, 1) = Transform.m23;
		R.at<double>(0, 2) = Transform.m31;
		R.at<double>(1, 2) = Transform.m32;
		R.at<double>(2, 2) = Transform.m33;

		Mat mat_in(3, 1, cv::DataType<double>::type);
		mat_in.at<double>(0, 0) = In.x;
		mat_in.at<double>(1, 0) = In.y;
		mat_in.at<double>(2, 0) = In.z;

		Mat mat_out(3, 1, cv::DataType<double>::type);
		mat_out = R * mat_in + tvec;

		return float3{ float(mat_out.at<double>(0, 0)) , float(mat_out.at<double>(1, 0)), float(mat_out.at<double>(2, 0)) };
		
	}

	Windows::Foundation::Numerics::float4x4 AppMain::Invert_transform(Windows::Foundation::Numerics::float4x4 Transform)
	{
		Mat tvec(3, 1, cv::DataType<double>::type); // translation vector
		Mat R(3, 3, cv::DataType<double>::type); //rotation vector 

		Mat tR;

		tvec.at<float>(0, 0) = Transform.m41;
		tvec.at<float>(1, 0) = Transform.m42;
		tvec.at<float>(2, 0) = Transform.m43;

		R.at<float>(0, 0) = Transform.m11;
		R.at<float>(1, 0) = Transform.m12;
		R.at<float>(2, 0) = Transform.m13;
		R.at<float>(0, 1) = Transform.m21;
		R.at<float>(1, 1) = Transform.m22;
		R.at<float>(2, 1) = Transform.m23;
		R.at<float>(0, 2) = Transform.m31;
		R.at<float>(1, 2) = Transform.m32;
		R.at<float>(2, 2) = Transform.m33;

		transpose(R, tR);
		Mat tvec_inv = - tR * tvec;

		
		float4x4 Transform_inv = float4x4(tR.at<float>(0, 0), R.at<float>(1, 0), R.at<float>(2, 0),0.f,
			R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),0.f,
			R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2),0.f,
			tvec_inv.at<float>(0, 0), tvec_inv.at<float>(1, 0), tvec_inv.at<float>(2, 0),1.f);
	
		return Transform_inv;

	}

	//Find point (x,y) where two parameterized lines intersect :p Returns 0 if lines are parallel 
	int AppMain::parametricIntersect(float r1, float t1, float r2, float t2, int *x, int *y) 
	{
		float ct1 = cosf(t1);     //matrix element a
		float st1 = sinf(t1);     //b
		float ct2 = cosf(t2);     //c
		float st2 = sinf(t2);     //d
		float d = ct1 * st2 - st1 * ct2;        //determinative (rearranged matrix for inverse)
		if (d != 0.0f) {
			*x = (int)((st2*r1 - st1 * r2) / d);
			*y = (int)((-ct2 * r1 + ct1 * r2) / d);
			return(1);
		}
		else { //lines are parallel and will NEVER intersect!
			return(0);
		}
	}

	Point2f AppMain::OnRenderedFrame(bool Ball, Mat frame, Windows::Foundation::Numerics::float3 PointWorldSpace, Windows::Media::Devices::Core::CameraIntrinsics^ cameraIntrinsics)
	{
		float3 PointPositionRenderedFrame3D = IntersectionLinePlane(CameraPositionWorldSpace, PointWorldSpace, plane_frame_world_space);

		float xt = dot(PointPositionRenderedFrame3D - center_plane_world_space, X_frame_world_space);
		float yt = dot(PointPositionRenderedFrame3D - center_plane_world_space, Y_frame_world_space);
		float xt_im = frame.cols / 2.0f + xt * cameraIntrinsics->FocalLength.x;
		float yt_im = frame.rows / 2.0f + yt * cameraIntrinsics->FocalLength.y;

		Point2f PointPositionRenderedFrame2D = Point2f{ xt_im,yt_im };
		

		if (Ball)
		{
			float ltb = length(PointPositionRenderedFrame3D - CameraPositionWorldSpace);
			float Ltb = length(PointWorldSpace - CameraPositionWorldSpace);
			float rtb = (ltb*ball_real_diameter) / (2.f*Ltb);

			cv::circle(frame, PointPositionRenderedFrame2D, 2*int(rtb*cameraIntrinsics->FocalLength.x), Scalar(0, 255, 0), 2);
			//cv::circle(frame, PointPositionRenderedFrame2D, 30, Scalar(0, 255, 0), 2);
		}
		return PointPositionRenderedFrame2D;

	}


	void AppMain::OnHolographicSpaceChanged(
		Windows::Graphics::Holographic::HolographicSpace^ holographicSpace)
	{
		//
		// Initialize the HoloLens media frame readers
		//

		StartHoloLensMediaFrameSourceGroup();

		m_holographicSpace = holographicSpace;

		// Use the default SpatialLocator to track the motion of the device.

		m_locator = SpatialLocator::GetDefault();
		

		// Be able to respond to changes in the positional tracking state.
		m_locatabilityChangedToken =
			m_locator->LocatabilityChanged +=
			ref new Windows::Foundation::TypedEventHandler<SpatialLocator^, Object^>(
				std::bind(&AppMain::OnLocatabilityChanged, this, _1, _2)
				);

		m_referenceFrame = m_locator->CreateStationaryFrameOfReferenceAtCurrentLocation();
		m_reference_attached_Frame = m_locator->CreateAttachedFrameOfReferenceAtCurrentHeading();

		//Create a "world" coordinate system with the origin placed at the device's position as the app is launched.
		m_WorldCoordinateSystem = m_referenceFrame->CoordinateSystem;

		// Initialize the rendering background 
		_isActiveRenderer = true;

		_currentSlateRenderer = std::make_shared<Rendering::SlateRenderer>(_deviceResources);
		m_markerTargetBall = std::make_shared<Rendering::PolylineRenderer>(_deviceResources);
		m_markerWhiteBall = std::make_shared<Rendering::PolylineRenderer>(_deviceResources);


		cue_target = std::make_shared<Rendering::BlueLineRenderer>(_deviceResources);
		target_pocket1 = std::make_shared<Rendering::LineRenderer>(_deviceResources);
		target_pocket2 = std::make_shared<Rendering::LineRenderer>(_deviceResources);
		target_pocket3 = std::make_shared<Rendering::LineRenderer>(_deviceResources);
		target_pocket4 = std::make_shared<Rendering::LineRenderer>(_deviceResources);
		target_pocket5 = std::make_shared<Rendering::LineRenderer>(_deviceResources);
		target_pocket6 = std::make_shared<Rendering::LineRenderer>(_deviceResources);


		m_markerpocket1 = std::make_shared<Rendering::MarkerRenderer>(_deviceResources);
		m_markerpocket2 = std::make_shared<Rendering::MarkerRenderer>(_deviceResources);
		m_markerpocket3 = std::make_shared<Rendering::MarkerRenderer>(_deviceResources);
		m_markerpocket4 = std::make_shared<Rendering::MarkerRenderer>(_deviceResources);
		m_markerpocket5 = std::make_shared<Rendering::MarkerRenderer>(_deviceResources);
		m_markerpocket6 = std::make_shared<Rendering::MarkerRenderer>(_deviceResources);
		

		_slateRendererList.push_back(_currentSlateRenderer);
		
		_markerRendererList.push_back(m_markerpocket1);
		_markerRendererList.push_back(m_markerpocket2);
		_markerRendererList.push_back(m_markerpocket3);
		_markerRendererList.push_back(m_markerpocket4);
		_markerRendererList.push_back(m_markerpocket5);
		_markerRendererList.push_back(m_markerpocket6);
		_markerRendererList.push_back(m_markerpocket7);
		
		
		_polylineRendererList.push_back(m_markerTargetBall);
		_polylineRendererList.push_back(m_markerWhiteBall);

		_BluelineRendererList.push_back(cue_target);

		_lineRendererList.push_back(target_pocket1);
		_lineRendererList.push_back(target_pocket2);
		_lineRendererList.push_back(target_pocket3);
		_lineRendererList.push_back(target_pocket4);
		_lineRendererList.push_back(target_pocket5);
		_lineRendererList.push_back(target_pocket6);
		
	}

	

	void AppMain::OnSpatialInput(_In_ Windows::UI::Input::Spatial::SpatialInteractionSourceState^ pointerState) //On a click 
	{

		if (order_airtap==0)
			ready_to_detect_chessboard = !ready_to_detect_chessboard;
		if (order_airtap == 1)
			ready_to_detect_target_ball = !ready_to_detect_target_ball;
		if (order_airtap == 2)
			ready_to_detect_white_ball = !ready_to_detect_white_ball;
		if (order_airtap == 3)
		{ 
			ready_to_detect_white_ball = !ready_to_detect_white_ball;
			ready_to_detect_target_ball = !ready_to_detect_target_ball;
		}
	}



	void AppMain::DetectPoolTable(Mat frame)
	{
		Mat frame2 = frame;
		frame2.convertTo(frame2, CV_64FC4);
		frame2 = frame2 / 255.;
		vector<Mat> channels(3);
		cv::split(frame2, channels);
		Mat B = channels[0];
		Mat G = channels[1];
		Mat R = channels[2];
		Mat I = R + B + G;

		Mat gc = Mat(G.size(), CV_64FC1);
		for (int i = 0; i < G.rows; ++i)
		{
			for (int j = 0; j < G.cols; ++j)
			{
				double g = G.at<double>(i, j);
				double sRGB = I.at<double>(i, j);
				if (sRGB>0.0001)
					gc.at<double>(i, j) = g / sRGB;
				else
					gc.at<double>(i, j) = 0.;
			}
		}

		double maxval;
		minMaxLoc(gc, 0, &maxval, 0, 0);
		dbg::trace(L"%f", maxval);

		gc = gc * 255.;
		gc.convertTo(gc, CV_8UC1);


		Mat thres;
		threshold(gc, thres, 90, 255, CV_THRESH_BINARY);

		vector<vector<cv::Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(thres, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		vector<vector<cv::Point> > contours_poly(contours.size());
		Scalar color_table = Scalar(255, 255, 255);
		Mat temp = Mat::zeros(gc.size(), CV_8UC3);
		Mat drawing = Mat::zeros(gc.size(), CV_8UC1);
		vector<vector<cv::Point> >hull(contours.size());
		double largest_area = 0;
		int largest_contour_index = 0;

		for (size_t i = 0; i < contours.size(); i++)
		{
			convexHull(Mat(contours[i]), hull[i]);
			cv::Rect _boundingRect = boundingRect(contours[i]);
			Scalar mean_color_contour = mean(frame(_boundingRect));
			double area = contourArea(hull[i], false);

			if (area>largest_area)
			{
				largest_area = area;
				largest_contour_index = i;
			}
		}

		drawContours(drawing, hull, largest_contour_index, Scalar(255), 2, 8, vector<Vec4i>(), 0, cv::Point());


		vector<Vec2f> lines;
		HoughLines(drawing, lines, 1, 1 * CV_PI / 180, 380, 0, 0);

		/// Show the result
		for (size_t i = 0; i < lines.size(); i++)
		{
			float r = lines[i][0], t = lines[i][1];
			double cos_t = cos(t), sin_t = sin(t);
			double x0 = r * cos_t, y0 = r * sin_t;
			double alpha = 1000;

			cv::Point pt1(cvRound(x0 + alpha * (-sin_t)), cvRound(y0 + alpha * cos_t));
			cv::Point pt2(cvRound(x0 - alpha * (-sin_t)), cvRound(y0 - alpha * cos_t));
			cv::line(frame, pt1, pt2, Scalar(255, 0, 0), 3, LINE_AA);

			for (size_t j = 0; j < lines.size(); ++j)
			{
				int x, y;
				int rr = parametricIntersect(lines[i][0], lines[i][1], lines[j][0], lines[j][1], &x, &y);
				if (rr != 0)
				{
					if (x > 0 && x < frame.cols &&
						y > 0 && y < frame.rows)
					{
						cv::circle(frame, cv::Point(x, y), 30, Scalar(255, 0, 0), 2);
					}
				}
			}
		}

		Mat GCcolor = Mat(frame.size(), frame.type());
		cvtColor(drawing, GCcolor, COLOR_GRAY2BGRA);



	}
		

	

	
	//Detect the PoolTable pose through the ChessBoard, and sets the pocket positions in world space. 
	void AppMain::DetectPoolTableChessBoard(Mat frame, SpatialCoordinateSystem^ CameraCoordinateSystem, Windows::Media::Devices::Core::CameraIntrinsics^ cameraIntrinsics, Windows::Foundation::Numerics::float4x4 CameraToWorld)
	{

		Mat cameraMatrix(3, 3, CV_64FC1);
		Mat distCoeffs(5, 1, CV_64FC1);

		if (nullptr != cameraIntrinsics)
		{
			// Set the CameraMatrix and the distortion matrix in the OpenCV format. See https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html. 
			cv::setIdentity(cameraMatrix);

			cameraMatrix.at<double>(0, 0) = cameraIntrinsics->FocalLength.x;
			cameraMatrix.at<double>(1, 1) = cameraIntrinsics->FocalLength.y;
			cameraMatrix.at<double>(0, 2) = cameraIntrinsics->PrincipalPoint.x;
			cameraMatrix.at<double>(1, 2) = cameraIntrinsics->PrincipalPoint.y;

			
			distCoeffs.at<double>(0, 0) = cameraIntrinsics->RadialDistortion.x;
			distCoeffs.at<double>(1, 0) = cameraIntrinsics->RadialDistortion.y;
			distCoeffs.at<double>(2, 0) = cameraIntrinsics->TangentialDistortion.x;
			distCoeffs.at<double>(3, 0) = cameraIntrinsics->TangentialDistortion.y;
			distCoeffs.at<double>(4, 0) = cameraIntrinsics->RadialDistortion.z;

		}


		// ChessBoard Detection and Drawing  

		Mat gray;
		cv::cvtColor(frame, gray, CV_BGR2GRAY);
		int numSquares = 6; //interior number of corners
		cv::Size patternsize(numSquares, numSquares); 
		vector<Point2f> corners;
		bool patternfound = findChessboardCorners(gray, patternsize, corners); //Returns True if the chess board is detected  
		
		
		if (patternfound && chess_iteration < num_chess_iteration)
		{

			cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1)); //Refines corner location
			drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);

			vector<Point3f> object_point; //Corner positions in the "ChessBoard coordinate system". 
			for (int j = 0; j < 36; j++)
				object_point.push_back(Point3f(float(j / numSquares), float(j%numSquares), 0.0f));

			vector<Point2f> image_point = corners; //Corner positions in the frame.

			Mat rvec; //Rotation from Camera Space to Pool Table Space.

			Mat tvec; //Translation from Camera Space to Pool Table Space.

			solvePnP(object_point, image_point, cameraMatrix, distCoeffs, rvec, tvec); //Find the ChessBoard pose.  


			dbg::trace(L"chessboard found");


			float length = 2.0f;
			vector<Point3f> axisPoints;
			axisPoints.push_back(Point3f(0, 0, 0));
			axisPoints.push_back(Point3f(length, 0, 0));
			axisPoints.push_back(Point3f(0, length, 0));
			axisPoints.push_back(Point3f(0, 0, length));
			vector< Point2f > imagePoints;
			projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);


			cv::line(frame, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);
			cv::line(frame, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 3);
			cv::line(frame, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);

			if (chess_iteration == 0)
			{
				sum_rvec = cv::Mat(rvec.size(), rvec.type());
				sum_rvec = 0;
				sum_tvec = cv::Mat(tvec.size(), tvec.type());
				sum_tvec = 0;
			}

			sum_rvec = sum_rvec + rvec;
			sum_tvec = sum_tvec + tvec;

			chess_iteration = chess_iteration + 1;

			dbg::trace(L"%f %f %f", sum_tvec.at<double>(0, 0), sum_tvec.at<double>(1, 0), sum_tvec.at<double>(2, 0));

		}
		

		if(chess_iteration == num_chess_iteration)
		{
			_isPoolDetected = true;
			
			Mat rvec = sum_rvec / float(num_chess_iteration);
			Mat tvec = sum_tvec / float(num_chess_iteration);

			dbg::trace(L"%f %f %f", tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));

			//draw axis on the frame. 
			float length = 2.0f;
			vector<Point3f> axisPoints;
			axisPoints.push_back(Point3f(0, 0, 0));
			axisPoints.push_back(Point3f(length, 0, 0));
			axisPoints.push_back(Point3f(0, length, 0));
			axisPoints.push_back(Point3f(0, 0, length));
			vector< Point2f > imagePoints;
			projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

			cv::line(frame, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);
			cv::line(frame, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 3);
			cv::line(frame, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);

			double square_size = 0.025; 
	

			Mat R, tR;
			Rodrigues(rvec, R);
			transpose(R, tR);


			//Set Pool pockets posiotins in world space
			float midlength_table = 1.15f;
			float width_table = 1.11f;

			//offset in meters between the chessBoard origin and the corner pocket. 
			double offset_lenght = 0.06;
			Mat offset(3, 1, cv::DataType<double>::type);
			offset.at<double>(0, 0) = -offset_lenght;
			offset.at<double>(1, 0) = -offset_lenght;
			offset.at<double>(2, 0) = ball_real_diameter/2.;
			

			Mat tvec_transform = R * offset + tvec * square_size;

			float4x4 ChessToCamera = float4x4(float(tR.at<double>(0, 0)), -float(tR.at<double>(0, 1)), -float(tR.at<double>(0, 2)), 0.f,
											float(tR.at<double>(1, 0)), -float(tR.at<double>(1, 1)), -float(tR.at<double>(1, 2)), 0.f,
											float(tR.at<double>(2, 0)), -float(tR.at<double>(2, 1)), -float(tR.at<double>(2, 2)), 0.f,
											float(tvec_transform.at<double>(0, 0)), -float(tvec_transform.at<double>(1, 0)), -float(tvec_transform.at<double>(2, 0)), 1.f);

			ChessToWorld = operator* (ChessToCamera, CameraToWorld);

			WorldToChess = Invert_transform(ChessToWorld);

			//pocket1
			float3 pocket1_table_space = float3{ 0.f , 0.f , 0.f};
			float3 pocket1_world_space = m_transform(pocket1_table_space, ChessToWorld);
			pocket_world_space.push_back(pocket1_world_space);
			

			//pocket2
			float3 pocket2_table_space = float3{0.f , midlength_table, 0.f };
			float3 pocket2_world_space = m_transform(pocket2_table_space, ChessToWorld);
			pocket2_world_space.y = pocket1_world_space.y;
			pocket_world_space.push_back(pocket2_world_space);
			

			//pocket3
			float3 pocket3_table_space = float3{ 0.f , 2 * midlength_table, 0.f };
			float3 pocket3_world_space = m_transform(pocket3_table_space, ChessToWorld);
			pocket3_world_space.y = pocket1_world_space.y;
			pocket_world_space.push_back(pocket3_world_space);
			

			//pocket4
			float3 pocket4_table_space = float3{width_table , 2 * midlength_table, 0.f };
			float3 pocket4_world_space = m_transform(pocket4_table_space, ChessToWorld);
			pocket4_world_space.y = pocket1_world_space.y;
			pocket_world_space.push_back(pocket4_world_space);
			

			//pocket5
			float3 pocket5_table_space = float3{width_table, midlength_table, 0.f };
			float3 pocket5_world_space = m_transform(pocket5_table_space, ChessToWorld);
			pocket5_world_space.y = pocket1_world_space.y;
			pocket_world_space.push_back(pocket5_world_space);
			
			float3 pocket6_table_space = float3{ width_table, 0.f ,  0.f };
			float3 pocket6_world_space = m_transform(pocket6_table_space, ChessToWorld);
			pocket6_world_space.y = pocket1_world_space.y;
			pocket_world_space.push_back(pocket6_world_space);
			

			pool_table_plane = ConstructPlaneFromPoints(pocket1_world_space, pocket2_world_space, pocket6_world_space);
			
		}
		

	}



	
	void AppMain::DetectTargetBall(Mat frame, cv::Rect2i Window)
	{
		

		target_ball_found = false;

		Mat HSVframe;
		vector<Mat> channels(3);
		Mat frame2 = frame;
		//cv::blur(frame, frame2, cv::Size(10, 10));
		cv::cvtColor(frame2, HSVframe, COLOR_BGR2HSV);
		cv::split(HSVframe, channels);
		Mat hframe, sframe, vframe;
		hframe = channels[0];
		sframe = channels[1];
		vframe = channels[2];

		int histSize = 180;

		// Set the range
		float range[] = { 0, 180 };
		const float* histRange = { range };
		bool uniform = true; bool accumulate = false;

		Mat hist;

		// Compute the histogram
		calcHist(&hframe, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

		double maxVal = 0;
		cv::Point hmaxPoint;
		minMaxLoc(hist, 0, &maxVal, 0, &hmaxPoint);
		int hmax = hmaxPoint.y;

		int histSize2 = 255;

		// Set the range
		float range2[] = { 0, 255 };
		const float* histRange2 = { range2 };

		Mat vhist;

		// Compute the histogram
		calcHist(&vframe, 1, 0, Mat(), vhist, 1, &histSize2, &histRange2, uniform, accumulate);

		cv::Point vmaxPoint;
		minMaxLoc(vhist, 0, &maxVal, 0, &vmaxPoint);
		int vmax = vmaxPoint.y;

		double thresvmax;
		double thresvmin;

		if (vmax < 205)
			thresvmax = vmax + 10;
		else
			thresvmax = 255;

		if (vmax > 50)
			thresvmin = vmax - 50;
		else
			thresvmin = 0;

		Scalar mintable = { double(hmax - 10), 2 , thresvmin };
		Scalar maxtable = { double(hmax + 10) , 240 , thresvmax };
		Mat threshold;
		cv::inRange(HSVframe, mintable, maxtable, threshold);

		// Create a structuring element


		int erosion_size2 = 3;
		Mat element2 = getStructuringElement(cv::MORPH_CROSS,
			cv::Size(2 * erosion_size2 + 1, 2 * erosion_size2 + 1),
			cv::Point(erosion_size2, erosion_size2));

		// Apply erosion or dilation on the image
		cv::erode(threshold, threshold, element2);
		cv::dilate(threshold, threshold, element2);

		cv::erode(threshold, threshold, element2);
		cv::dilate(threshold, threshold, element2);

		cv::erode(threshold, threshold, element2);
		cv::dilate(threshold, threshold, element2);

		cv::erode(threshold, threshold, element2);
		cv::dilate(threshold, threshold, element2);

		Mat threshold_window = threshold(Window);

		int MaxRadius = int(ball_window_width / 2.);
		int MinRadius = 10;
		Scalar color_ball = Scalar(0, 0, 0);

		//Detect contours avec FindContours
		Mat contours;
		Canny(threshold_window, contours, 50, 100);

		cv::Point pts1 = Point2i{ int(frame.cols / 2. - ball_window_width / 2.), int(frame.rows / 2. - ball_window_width / 2.) };
		
		std::vector<cv::Vec3f> circles;
		HoughCircles(contours, circles, CV_HOUGH_GRADIENT,
			2, //accumulator resolution 
			5, //min distance between two circles
			100, //Canny high threshold
			80, // min number of votes
			MinRadius, MaxRadius); // min and max radius


		cv::Point center_frame = { int(frame.cols / 2.f), int(frame.rows / 2.f) };

		if (circles.size() > 0)
		{
			int bestballindex = 0;
			double bestlenght = 0.;

			for (size_t i = 0; i < circles.size(); i++)
			{
				cv::Vec3f ball = circles[i];

				
				cv::Point center = { int(ball[0] + pts1.x), int(ball[1] + pts1.y) };

				double lenght = cv::norm(center - center_frame);

				if (lenght < bestlenght)
				{
					bestlenght = lenght;
					bestballindex = i;
				}
			}

			targetball = circles[bestballindex];

			
			target_ball_found = true;

		}

	}


	
	
	void AppMain::DetectWhiteBall(Mat frame, cv::Rect2i Window)
	{
		Mat frame_window = frame(Window);
		frame_window.convertTo(frame_window, CV_64FC4);
		frame_window = frame_window / 255.;
		vector<Mat> channels(3);
		cv::split(frame_window, channels);
		Mat B = channels[0];
		Mat G = channels[1];
		Mat R = channels[2];
		Mat I = R + B + G;

		white_ball_found = false;
		
		int MaxRadius = int(ball_window_width / 2.);
		int MinRadius = 10;
		Scalar color_ball = Scalar(0, 0, 0);

		double ImaxVal = 0;
		cv::Point ImaxPoint;
		minMaxLoc(I, 0, &ImaxVal, 0, &ImaxPoint);
		I = (I / ImaxVal) * 255;

		I.convertTo(I, CV_8UC1);

		cv::Mat Ithreshold;
		cv::threshold(I, Ithreshold, 180, 255, THRESH_BINARY);

		Mat Icontours;
		Canny(Ithreshold, Icontours, 50, 100);

		std::vector<cv::Vec3f> Icircles;
		HoughCircles(Icontours, Icircles, CV_HOUGH_GRADIENT,
			2, //accumulator resolution 
			100, //min distance between two circles
			100, //Canny high threshold
			80, // min number of votes
			MinRadius, MaxRadius); // min and max radius

		

		cv::Point center_frame = { int(frame.cols / 2.f), int(frame.rows / 2.f) };

		cv::Point pts1 = Point2i{ int(frame.cols / 2. - ball_window_width / 2.), int(frame.rows / 2. - ball_window_width / 2.) };
		
		if (Icircles.size()>0)
		{
			int bestballindex = 0;
			double bestlenght = 0.;
		 
			for (size_t i = 0; i < Icircles.size(); i++)
			{
				cv::Vec3f ball = Icircles[i];
				cv::Point center = { int(ball[0] + pts1.x), int(ball[1] + pts1.y) };

				double lenght = cv::norm(center - center_frame);

				if (lenght < bestlenght)
				{
					bestlenght = lenght;
					bestballindex = i;
				}
			}

			whiteball = Icircles[bestballindex];

			white_ball_found = true;

		}
	}
	

	
	
	void AppMain::OnUpdate(
		_In_ Windows::Graphics::Holographic::HolographicFrame^ holographicFrame,
		_In_ const Graphics::StepTimer& stepTimer)
	
	{
		UNREFERENCED_PARAMETER(holographicFrame);

		dbg::TimerGuard timerGuard(
			L"AppMain::OnUpdate",
			50.0 /* minimum_time_elapsed_in_milliseconds */);

	
		//for (auto& r : _slateRendererList)
		//{
		//	r->Update(
		//		stepTimer);
		//}

		time(&now);

		if (!_holoLensMediaFrameSourceGroupStarted)
		{
			return;
		}

		//Get latest frame
		HoloLensForCV::SensorFrame^ latestFrame;


		latestFrame =
			_holoLensMediaFrameSourceGroup->GetLatestSensorFrame(
				HoloLensForCV::SensorType::PhotoVideo);


		if (nullptr == latestFrame)
		{
			return;
		}

		if (_latestSelectedCameraTimestamp.UniversalTime == latestFrame->Timestamp.UniversalTime)
		{
			return;
		}


		_latestSelectedCameraTimestamp = latestFrame->Timestamp;

		//Get Camera parameters
		Windows::Media::Devices::Core::CameraIntrinsics^ cameraIntrinsics = latestFrame->CoreCameraIntrinsics;
		Windows::Perception::Spatial::SpatialCoordinateSystem^ CameraCoordinateSystem = latestFrame->CameraCoordinateSystem;
		Windows::Foundation::Numerics::float4x4 CameraViewTransform = latestFrame->CameraViewTransform;
		Windows::Foundation::Numerics::float4x4 CameraProjectionTransform = latestFrame->CameraProjectionTransform;

		

		float distanceFromUser = 1.15f; // meters
		

		//Get the prediction of the position/orientation of the users's head, in a specified coodinate system, when the camera will be ready to render. 
		HolographicFramePrediction^ prediction = holographicFrame->CurrentPrediction;

		Windows::Perception::Spatial::SpatialCoordinateSystem^ AttachedCoordinateSystem = m_reference_attached_Frame->GetStationaryCoordinateSystemAtTimestamp(prediction->Timestamp);
		//Windows::Perception::Spatial::SpatialCoordinateSystem^ AttachedCoordinateSystem = _spatialPerception->GetOriginFrameOfReference()->CoordinateSystem;

		SpatialPointerPose^ pose = SpatialPointerPose::TryGetAtTimestamp(m_WorldCoordinateSystem, prediction->Timestamp);
		

		

	
		if (_isActiveRenderer)
		{
			if (pose != nullptr)
			{
				//_currentSlateRenderer->PositionHologram(pose);

				_currentSlateRenderer->Update(pose, stepTimer);
		


				float3 headPosition = pose->Head->Position;
				float3 headDirection = pose->Head->ForwardDirection;
				float3 headUp = pose->Head->UpDirection;
				float3 headRight = cross(headDirection, headUp);
				float3 headBack = -headDirection;

				float3 TargetPosition = headPosition + headDirection * distanceFromUser;
			
				center_plane_world_space = TargetPosition;
				
				float _rotationInRadiansY = std::atan2(headDirection.z,headDirection.x) + DirectX::XM_PIDIV2;
				const float radiansY = static_cast<float>(fmod(_rotationInRadiansY, DirectX::XM_2PI));

				float4x4 RotationRenderedPlane = make_float4x4_rotation_y(-radiansY);

				
				//dbg::trace(L"RotationRenderedPlane Calcul");
				//dbg::trace(L"%f %f %f %f ", RotationRenderedPlane.m11, RotationRenderedPlane.m12, RotationRenderedPlane.m13, RotationRenderedPlane.m14);
				//dbg::trace(L"%f %f %f %f ", RotationRenderedPlane.m21, RotationRenderedPlane.m22, RotationRenderedPlane.m23, RotationRenderedPlane.m24);
				//dbg::trace(L"%f %f %f %f ", RotationRenderedPlane.m31, RotationRenderedPlane.m32, RotationRenderedPlane.m33, RotationRenderedPlane.m34);
				//dbg::trace(L"%f %f %f %f ", RotationRenderedPlane.m41, RotationRenderedPlane.m42, RotationRenderedPlane.m43, RotationRenderedPlane.m44);

				//dbg::trace(L"trans Calcul");
				//dbg::trace(L"%f %f %f %f ", trans.m11, trans.m12, trans.m13, trans.m14);
				//dbg::trace(L"%f %f %f %f ", trans.m21, trans.m22, trans.m23, trans.m24);
				//dbg::trace(L"%f %f %f %f ", trans.m31, trans.m32, trans.m33, trans.m34);
				//dbg::trace(L"%f %f %f %f ", trans.m41, trans.m42, trans.m43, trans.m44);
				
				

				//dbg::trace(L"mul Calcul");
				//dbg::trace(L"%f %f %f %f ", mul.m11, mul.m12, mul.m13, mul.m14);
				//dbg::trace(L"%f %f %f %f ", mul.m21, mul.m22, mul.m23, mul.m24);
				//dbg::trace(L"%f %f %f %f ", mul.m31, mul.m32, mul.m33, mul.m34);
				//dbg::trace(L"%f %f %f %f ", mul.m41, mul.m42, mul.m43, mul.m44);

				normal_plane_world_space = transform(float3{ 0,0,1 }, RotationRenderedPlane);
				
				//normal_plane_world_space = headDirection;

				X_frame_world_space = normalize(headRight);
				Y_frame_world_space = cross(-normal_plane_world_space, X_frame_world_space);

				plane_frame_world_space = ConstructPlaneFromPointNormal(center_plane_world_space, normal_plane_world_space);
				
				
				
				
				
				

				
			}
			else
			{
				return;
			}
		}

		//dbg::trace(L"TargetPosition");
		//dbg::trace(L"%f %f %f ", TargetPosition.x, TargetPosition.y, TargetPosition.z);


		
		/*
		
		for (auto cameraPose : prediction->CameraPoses)
		{
			// The HolographicCameraRenderingParameters class provides access to set
			// the image stabilization parameters.
			HolographicCameraRenderingParameters^ renderingParameters = holographicFrame->GetRenderingParameters(cameraPose);

			// put the focus point on the cube
			renderingParameters->SetFocusPoint(m_WorldCoordinateSystem, center_plane_world_space, normal_plane_world_space);
		}
		
		*/

		//Convert the latest frame in a cv::Mat object. 
		cv::Mat frame;
		rmcv::WrapHoloLensSensorFrameWithCvMat(latestFrame, frame);
		
		render_frame = false;

		cv::Point textposition = Point2i{ 200 , 255};
		
		cv::Point pts1 = Point2i{ int(frame.cols / 2. - ball_window_width / 2.)-100, int(frame.rows / 2. - ball_window_width / 2.) + 100};
		cv::Point pts2 = Point2i{ int(frame.cols / 2. + ball_window_width / 2.)-100, int(frame.rows / 2. + ball_window_width / 2.) + 100};
		cv::Rect2i Window = cv::Rect2i(pts1, pts2);
	
		
		
		const auto tryTransformCtW = CameraCoordinateSystem->TryGetTransformTo(m_WorldCoordinateSystem);
		
		if (tryTransformCtW == nullptr)
			dbg::trace(L"CameraToWorld null");
		else
		{ 
			Windows::Foundation::Numerics::float4x4 CameraToWorld = tryTransformCtW->Value;
			CameraPositionWorldSpace = m_transform(float3{ 0.f,0.f,0.f }, CameraToWorld);

			if (!ready_to_detect_chessboard)
			{
				render_frame = true;
				frame = 0 * frame;
				order_airtap = 0;
				cv::putText(frame, "Get placed at red cross and look toward the chessboard. When Ready, airtap", textposition, FONT_HERSHEY_SIMPLEX, 0.7 , Scalar{ 255,255,255 }, 2);				
			}

			else
			{
				if (!_isPoolDetected)
				{ 
					render_frame = true;
					DetectPoolTableChessBoard(frame, CameraCoordinateSystem, cameraIntrinsics, CameraToWorld);
					frame = 0 * frame;
					cv::putText(frame, "Wait until the chessboard is detected", textposition, FONT_HERSHEY_SIMPLEX, 0.7, Scalar{ 255,255,255 }, 2);
				}
				else
				{
					order_airtap = 1;

					float3 pocket_world_space1 = pocket_world_space[0];
					float3 pocket_world_space2 = pocket_world_space[1];
					float3 pocket_world_space3 = pocket_world_space[2];
					float3 pocket_world_space4 = pocket_world_space[3];
					float3 pocket_world_space5 = pocket_world_space[4];
					float3 pocket_world_space6 = pocket_world_space[5];

					m_markerpocket1->Update(pocket_world_space1, stepTimer);
					m_markerpocket2->Update(pocket_world_space2, stepTimer);
					m_markerpocket3->Update(pocket_world_space3, stepTimer);
					m_markerpocket4->Update(pocket_world_space4, stepTimer);
					m_markerpocket5->Update(pocket_world_space5, stepTimer);
					m_markerpocket6->Update(pocket_world_space6, stepTimer);

					m_markerWhiteBall->Update(pocket_world_space1, stepTimer);

					
					
					if (!ready_to_detect_target_ball)
					{
						render_frame = true;
						target_identified = false;
						cv::putText(frame, "Put the target ball in the black square. When Ready, airtap", textposition, FONT_HERSHEY_SIMPLEX, 0.7, Scalar{255,255,255}, 2);
						cv::rectangle(frame, Window, Scalar{ 0,0,0 }, 2);
						target_count = 0;
						sum_x_target = 0;
						sum_y_target = 0;
						sum_radius_target = 0;
					}
					else
					{
						if (!target_identified)
						{
							render_frame = true;
							cv::putText(frame, "Wait until the ball is detected", textposition, FONT_HERSHEY_SIMPLEX, 0.7, Scalar{ 255,255,255 }, 2);
							cv::rectangle(frame, Window, Scalar{ 0,0,0 }, 2);

							if (target_count < number_target_avarage)
							{
								DetectTargetBall(frame, Window);

								if (target_ball_found)
								{
									sum_x_target = sum_x_target + targetball[0];
									sum_y_target = sum_y_target + targetball[1];
									sum_radius_target = sum_radius_target + targetball[2];

									cv::Point center = { int(targetball[0] + pts1.x), int(targetball[1] + pts1.y) };

									//cv::circle(frame, center, int(targetball[2]), Scalar{ 255,255,0 }, 2, 8, 0);

									target_count = target_count + 1;

								}

							}
							else
							{


								int x_target = int(sum_x_target / float(target_count));
								int y_target = int(sum_y_target / float(target_count));
								int radius_targetball = int(sum_radius_target / float(target_count));

								cv::Point center_targetball = { x_target + pts1.x, y_target + pts1.y };

								//cv::circle(frame, center_targetball, radius_targetball, Scalar{ 255,255,0 }, 2, 8, 0);

								float BallWidthInMeters = 0.0525f;
								float const estimatedTargetBallDepth = cameraIntrinsics->FocalLength.x * BallWidthInMeters / (2.0f * radius_targetball);
								float unproject_xtarget = float((center_targetball.x - cameraIntrinsics->PrincipalPoint.x) / cameraIntrinsics->FocalLength.x);
								float unproject_ytarget = float((center_targetball.y - cameraIntrinsics->PrincipalPoint.y) / cameraIntrinsics->FocalLength.y);
								float3 vectorTowardstargetcenter_unormalized = float3{ unproject_xtarget ,  -unproject_ytarget , -1.0f };
								float3 vectorTowardstargetcenter = normalize(vectorTowardstargetcenter_unormalized);

								TargetBallPositionInCameraSpace = vectorTowardstargetcenter * estimatedTargetBallDepth;
								float3 TargetBallPositionInWorldSpace1 = m_transform(TargetBallPositionInCameraSpace, CameraToWorld);

								TargetBallPositionInWorldSpace = IntersectionLinePlane(CameraPositionWorldSpace, TargetBallPositionInWorldSpace1, pool_table_plane);

							
		
								
								m_markerTargetBall->Update(TargetBallPositionInWorldSpace, stepTimer);

								target_identified = true;

								target_count = 0;
							}

						}


						else
						{
							order_airtap = 2;

							if (!ready_to_detect_white_ball)
							{
								white_identified = false;
								render_frame = true;
								cv::putText(frame, "Put the white ball in the square. When Ready, airtap", textposition, FONT_HERSHEY_SIMPLEX, 0.7, Scalar{ 255,0,0 }, 2);
								cv::rectangle(frame, Window, Scalar{ 255, 0 , 0 }, 2);
								white_count = 0;
								sum_x_white = 0;
								sum_y_white = 0;
								sum_radius_white = 0;
							}
							else
							{
								if (!white_identified)
								{
									render_frame = true;
									cv::putText(frame, "Wait until the ball is detected", textposition, FONT_HERSHEY_SIMPLEX, 0.7, Scalar{ 255,0,0 }, 2);
									cv::rectangle(frame, Window, Scalar{ 255, 0 , 0 }, 2);

									if (white_count < number_target_avarage)
									{
										DetectWhiteBall(frame, Window);

										if (white_ball_found)
										{
											sum_x_white = sum_x_white + whiteball[0];
											sum_y_white = sum_y_white + whiteball[1];
											sum_radius_white = sum_radius_white + whiteball[2];

											//cv::Point centerwhite = { int(whiteball[0] + pts1.x), int(whiteball[1] + pts1.y) };

											white_count = white_count + 1;

										}

									}
									else
									{


										int x_white = int(sum_x_white / float(white_count));
										int y_white = int(sum_y_white / float(white_count));
										int radius_whiteball = int(sum_radius_white / float(white_count));

										cv::Point center_whiteball = { x_white + pts1.x, y_white + pts1.y };

										float BallWidthInMeters = 0.0525f;
										float const estimatedWhiteBallDepth = cameraIntrinsics->FocalLength.x * BallWidthInMeters / (2.0f * radius_whiteball);
										float unproject_xwhite = float((center_whiteball.x - cameraIntrinsics->PrincipalPoint.x) / cameraIntrinsics->FocalLength.x);
										float unproject_ywhite = float((center_whiteball.y - cameraIntrinsics->PrincipalPoint.y) / cameraIntrinsics->FocalLength.y);
										float3 vectorTowardswhitecenter_unormalized = float3{ unproject_xwhite ,  -unproject_ywhite , -1.0f };
										float3 vectorTowardswhitecenter = normalize(vectorTowardswhitecenter_unormalized);

										WhiteBallPositionInCameraSpace = vectorTowardswhitecenter * estimatedWhiteBallDepth;
										float3 WhiteBallPositionInWorldSpace1 = m_transform(WhiteBallPositionInCameraSpace, CameraToWorld);

										WhiteBallPositionInWorldSpace = IntersectionLinePlane(CameraPositionWorldSpace, WhiteBallPositionInWorldSpace1, pool_table_plane);

										

										m_markerWhiteBall->Update(WhiteBallPositionInWorldSpace, stepTimer);

										white_identified = true;

										white_count = 0;
									}

								}

								else
								{
									order_airtap = 3;

									frame = 0 * frame;

									//Point2f WhiteBallPositionRenderedFrame2D = OnRenderedFrame(false, frame, WhiteBallPositionInWorldSpace, cameraIntrinsics);
									//Point2f TargetBallPositionRenderedFrame2D = OnRenderedFrame(false, frame, TargetBallPositionInWorldSpace, cameraIntrinsics);

									

									cue_target->Update(TargetBallPositionInWorldSpace, WhiteBallPositionInWorldSpace , stepTimer);
									
									target_pocket1->Update(TargetBallPositionInWorldSpace, pocket_world_space1, stepTimer);
									target_pocket2->Update(TargetBallPositionInWorldSpace, pocket_world_space2, stepTimer);
									target_pocket3->Update(TargetBallPositionInWorldSpace, pocket_world_space3, stepTimer);
									target_pocket4->Update(TargetBallPositionInWorldSpace, pocket_world_space4, stepTimer);
									target_pocket5->Update(TargetBallPositionInWorldSpace, pocket_world_space5, stepTimer);
									target_pocket6->Update(TargetBallPositionInWorldSpace, pocket_world_space6, stepTimer);
									
									//cv::line(frame, WhiteBallPositionRenderedFrame2D, TargetBallPositionRenderedFrame2D, Scalar(0, 0, 255), 2);

									//cv::line(frame, pocket1_rendered_frame, TargetBallPositionRenderedFrame2D, Scalar(100, 0, 0), 2);
									//cv::line(frame, pocket2_rendered_frame, TargetBallPositionRenderedFrame2D, Scalar(100, 0, 0), 2);
									//cv::line(frame, pocket3_rendered_frame, TargetBallPositionRenderedFrame2D, Scalar(100, 0, 0), 2);
									//cv::line(frame, pocket4_rendered_frame, TargetBallPositionRenderedFrame2D, Scalar(100, 0, 0), 2);
									//cv::line(frame, pocket5_rendered_frame, TargetBallPositionRenderedFrame2D, Scalar(100, 0, 0), 2);
									//cv::line(frame, pocket6_rendered_frame, TargetBallPositionRenderedFrame2D, Scalar(100, 0, 0), 2);

									


								}

							}

						}

						


					}

					
					
				}

					
			}
			
		}



/*
	cv::Point pts1 = Point2i{ int(frame.cols / 2. - ball_window_width / 2.) - 100, int(frame.rows / 2. - ball_window_width / 2.) + 100 };
	cv::Point pts2 = Point2i{ int(frame.cols / 2. + ball_window_width / 2.) - 100, int(frame.rows / 2. + ball_window_width / 2.) + 100 };
	cv::Rect2i Window = cv::Rect2i(pts1, pts2);

		render_frame = true;
		Mat HSVframe;
		vector<Mat> channels(3);
		Mat frame2 = frame;
		
		cv::cvtColor(frame2, HSVframe, COLOR_BGR2HSV);
		cv::split(HSVframe, channels);
		Mat hframe, sframe, vframe;
		hframe = channels[0];
		sframe = channels[1];
		vframe = channels[2];

		int histSize = 180;

		// Set the range
		float range[] = { 0, 180 };
		const float* histRange = { range };
		bool uniform = true; bool accumulate = false;

		Mat hist;

		// Compute the histogram
		calcHist(&hframe, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

		double maxVal = 0;
		cv::Point hmaxPoint;
		minMaxLoc(hist, 0, &maxVal, 0, &hmaxPoint);
		int hmax = hmaxPoint.y;

		int histSize2 = 255;

		// Set the range
		float range2[] = { 0, 255 };
		const float* histRange2 = { range2 };

		Mat vhist;

		// Compute the histogram
		calcHist(&vframe, 1, 0, Mat(), vhist, 1, &histSize2, &histRange2, uniform, accumulate);

		cv::Point vmaxPoint;
		minMaxLoc(vhist, 0, &maxVal, 0, &vmaxPoint);
		int vmax = vmaxPoint.y;

		double thresvmax;
		double thresvmin;

		if (vmax < 205)
			thresvmax = vmax + 10;
		else
			thresvmax = 255;

		if (vmax > 50)
			thresvmin = vmax - 50;
		else
			thresvmin = 0;

		Scalar mintable = { double(hmax - 10), 2 , thresvmin };
		Scalar maxtable = { double(hmax + 10) , 240 , thresvmax };
		Mat threshold;
		cv::inRange(HSVframe, mintable, maxtable, threshold);

		// Create a structuring element
		int erosion_size2 = 3;
		Mat element2 = getStructuringElement(cv::MORPH_CROSS,
			cv::Size(2 * erosion_size2 + 1, 2 * erosion_size2 + 1),
			cv::Point(erosion_size2, erosion_size2));

		// Apply erosion or dilation on the image
		cv::erode(threshold, threshold, element2);
		cv::dilate(threshold, threshold, element2);

		cv::erode(threshold, threshold, element2);
		cv::dilate(threshold, threshold, element2);

		cv::erode(threshold, threshold, element2);
		cv::dilate(threshold, threshold, element2);

		cv::erode(threshold, threshold, element2);
		cv::dilate(threshold, threshold, element2);

		Mat threshold_window = threshold(Window);

		int MaxRadius = int(ball_window_width / 2.);
		int MinRadius = 10;
		Scalar color_ball = Scalar(0, 0, 0);

		//Detect contours avec FindContours
		//Mat contours;
		//Canny(threshold_window, contours, 50, 100);

		//cv::Point pts1 = Point2i{ int(frame.cols / 2. - ball_window_width / 2.), int(frame.rows / 2. - ball_window_width / 2.) };

		std::vector<cv::Vec3f> circles;
		HoughCircles(threshold, circles, CV_HOUGH_GRADIENT,
			2, //accumulator resolution 
			5, //min distance between two circles
			100, //Canny high threshold
			80, // min number of votes
			MinRadius, MaxRadius); // min and max radius


		std::vector<Vec3f>::const_iterator Icir = circles.begin();

		while (Icir != circles.end())
		{
			//circle(Icontours, cv::Point((* Icir)[0],(* Icir)[1]),(* Icir)[2],Scalar{150},2);

			circle(frame, cv::Point(int((*Icir)[0]), int((*Icir)[1])), int( (*Icir)[2]), Scalar{ 0,0,0 }, 2);
			++Icir;
		}
	*/	
	
		
		//Update 2D texture to suit with blurredPVCameraImage

		OpenCVHelpers::CreateOrUpdateTexture2D(
			_deviceResources,
			frame,
			_currentVisualizationTexture);

		

	}

	void AppMain::OnPreRender()
	{
	}

	// Renders the current frame to each holographic camera, according to the
	// current application and spatial positioning state.

	void AppMain::OnRender()
	{

		// Draw the sample hologram.
		//for (size_t i = 0; i < _visualizationTextureList.size(); ++i)
		//{
		//	_slateRendererList[i]->Render(_visualizationTextureList[i]);
		//}

		if (_isActiveRenderer)
		{
			if (render_frame)
				_currentSlateRenderer->Render(_currentVisualizationTexture);

			m_markerTargetBall->Render();
			m_markerWhiteBall->Render();

			m_markerpocket1->Render();
			m_markerpocket2->Render();
			m_markerpocket3->Render();
			m_markerpocket4->Render();
			m_markerpocket5->Render();
			m_markerpocket6->Render();
			
			cue_target->Render();

			target_pocket1->Render();
			target_pocket2->Render();
			target_pocket3->Render();
			target_pocket4->Render();
			target_pocket5->Render();
			target_pocket6->Render();
		}
	}



	void AppMain::OnLocatabilityChanged(SpatialLocator^ sender, Object^ args)
	{
		switch (sender->Locatability)
		{
		case SpatialLocatability::Unavailable:
			// Holograms cannot be rendered.
		{
			Platform::String^ message = L"Warning! Positional tracking is " +
				sender->Locatability.ToString() + L".\n";
			OutputDebugStringW(message->Data());
		}
		break;

		// In the following three cases, it is still possible to place holograms using a
		// SpatialLocatorAttachedFrameOfReference.
		case SpatialLocatability::PositionalTrackingActivating:
			// The system is preparing to use positional tracking.

		case SpatialLocatability::OrientationOnly:
			// Positional tracking has not been activated.
			break;

		case SpatialLocatability::PositionalTrackingInhibited:
			// Positional tracking is temporarily inhibited. User action may be required
			// in order to restore positional tracking.
			break;

		case SpatialLocatability::PositionalTrackingActive:
			// Positional tracking is active. World-locked content can be rendered.
			break;
		}
	}

	void AppMain::OnCameraAdded(
		HolographicSpace^ sender,
		HolographicSpaceCameraAddedEventArgs^ args
	)
	{
		Deferral^ deferral = args->GetDeferral();
		HolographicCamera^ holographicCamera = args->Camera;
		create_task([this, deferral, holographicCamera]()
		{
			// Create device-based resources for the holographic camera and add it to the list of
			// cameras used for updates and rendering. Notes:
			//   * Since this function may be called at any time, the AddHolographicCamera function
			//     waits until it can get a lock on the set of holographic camera resources before
			//     adding the new camera. At 60 frames per second this wait should not take long.
			//   * A subsequent Update will take the back buffer from the RenderingParameters of this
			//     camera's CameraPose and use it to create the ID3D11RenderTargetView for this camera.
			//     Content can then be rendered for the HolographicCamera.
			m_deviceResources->AddHolographicCamera(holographicCamera);

			// Holographic frame predictions will not include any information about this camera until
			// the deferral is completed.
			deferral->Complete();
		});
	}

	void AppMain::OnCameraRemoved(
		HolographicSpace^ sender,
		HolographicSpaceCameraRemovedEventArgs^ args
	)
	{
		// Before letting this callback return, ensure that all references to the back buffer
		// are released.
		// Since this function may be called at any time, the RemoveHolographicCamera function
		// waits until it can get a lock on the set of holographic camera resources before
		// deallocating resources for this camera. At 60 frames per second this wait should
		// not take long.
		m_deviceResources->RemoveHolographicCamera(args->Camera);
	}


	// Notifies classes that use Direct3D device resources that the device resources
	// need to be released before this method returns.
	void AppMain::OnDeviceLost()
	{

		for (auto& r : _slateRendererList)
		{
			r->ReleaseDeviceDependentResources();
		}

		_holoLensMediaFrameSourceGroup = nullptr;
		_holoLensMediaFrameSourceGroupStarted = false;


		for (auto& v : _visualizationTextureList)
		{
			v.reset();
		}
		_currentVisualizationTexture.reset();
	}

	// Notifies classes that use Direct3D device resources that the device resources
	// may now be recreated.


	void AppMain::OnDeviceRestored()
	{
		for (auto& r : _slateRendererList)
		{
			r->CreateDeviceDependentResources();
		}

		StartHoloLensMediaFrameSourceGroup();

	}

	void AppMain::StartHoloLensMediaFrameSourceGroup()
	{
		_sensorFrameStreamer =
			ref new HoloLensForCV::SensorFrameStreamer();

		_sensorFrameStreamer->EnableAll();

		_holoLensMediaFrameSourceGroup =
			ref new HoloLensForCV::MediaFrameSourceGroup(
				_selectedHoloLensMediaFrameSourceGroupType,
				_spatialPerception,
				_sensorFrameStreamer);

		_holoLensMediaFrameSourceGroup->Enable(
			HoloLensForCV::SensorType::PhotoVideo);

		concurrency::create_task(_holoLensMediaFrameSourceGroup->StartAsync()).then(
			[&]()
		{
			_holoLensMediaFrameSourceGroupStarted = true;
		});
	}

}
