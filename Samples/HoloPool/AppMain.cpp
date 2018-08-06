
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
		float3 vect = normalize(vector);
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

		// Respond to camera added events by creating any resources that are specific
		// to that camera, such as the back buffer render target view.
		// When we add an event handler for CameraAdded, the API layer will avoid putting
		// the new camera in new HolographicFrames until we complete the deferral we created
		// for that handler, or return from the handler without creating a deferral. This
		// allows the app to take more than one frame to finish creating resources and
		// loading assets for the new holographic camera.
		// This function should be registered before the app creates any HolographicFrames.
		m_cameraAddedToken =
			m_holographicSpace->CameraAdded +=
			ref new Windows::Foundation::TypedEventHandler<HolographicSpace^, HolographicSpaceCameraAddedEventArgs^>(
				std::bind(&AppMain::OnCameraAdded, this, _1, _2)
				);

		// Respond to camera removed events by releasing resources that were created for that
		// camera.
		// When the app receives a CameraRemoved event, it releases all references to the back
		// buffer right away. This includes render target views, Direct2D target bitmaps, and so on.
		// The app must also ensure that the back buffer is not attached as a render target, as
		// shown in DeviceResources::ReleaseResourcesForBackBuffer.
		m_cameraRemovedToken =
			m_holographicSpace->CameraRemoved +=
			ref new Windows::Foundation::TypedEventHandler<HolographicSpace^, HolographicSpaceCameraRemovedEventArgs^>(
				std::bind(&AppMain::OnCameraRemoved, this, _1, _2)
				);

		m_referenceFrame = m_locator->CreateStationaryFrameOfReferenceAtCurrentLocation();
		m_reference_attached_Frame = m_locator->CreateAttachedFrameOfReferenceAtCurrentHeading();

		//Create a "world" coordinate system with the origin placed at the device's position as the app is launched.
		m_WorldCoordinateSystem = m_referenceFrame->CoordinateSystem;

		// Initialize the rendering background 
		_isActiveRenderer = true;

		_currentSlateRenderer = std::make_shared<Rendering::SlateRenderer>(_deviceResources);
		m_markerRenderer = std::make_shared<Rendering::MarkerRenderer>(_deviceResources);

		_slateRendererList.push_back(_currentSlateRenderer);

	}

	

	void AppMain::OnSpatialInput(_In_ Windows::UI::Input::Spatial::SpatialInteractionSourceState^ pointerState) //On a click 
	{
		target_ball_found = !target_ball_found;
	}



	
	//Detect the PoolTable pose through the ChessBoard, and sets the pocket positions in world space. 
	void AppMain::DetectPoolTable(Mat frame, SpatialCoordinateSystem^ CameraCoordinateSystem, Windows::Media::Devices::Core::CameraIntrinsics^ cameraIntrinsics, Windows::Foundation::Numerics::float4x4 CameraToWorld)
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


		Mat tvec_world(3, 1, cv::DataType<double>::type);
		Mat R_world(3, 3, cv::DataType<double>::type);

		tvec_world.at<double>(0, 0) = CameraToWorld.m41;
		tvec_world.at<double>(1, 0) = CameraToWorld.m42;
		tvec_world.at<double>(2, 0) = CameraToWorld.m43;

		R_world.at<double>(0, 0) = CameraToWorld.m11;
		R_world.at<double>(1, 0) = CameraToWorld.m12;
		R_world.at<double>(2, 0) = CameraToWorld.m13;
		R_world.at<double>(0, 1) = CameraToWorld.m21;
		R_world.at<double>(1, 1) = CameraToWorld.m22;
		R_world.at<double>(2, 1) = CameraToWorld.m23;
		R_world.at<double>(0, 2) = CameraToWorld.m31;
		R_world.at<double>(1, 2) = CameraToWorld.m32;
		R_world.at<double>(2, 2) = CameraToWorld.m33;

		// ChessBoard Detection and Drawing  

		Mat gray;
		cv::cvtColor(frame, gray, CV_BGR2GRAY);
		int numSquares = 6; //interior number of corners
		cv::Size patternsize(numSquares, numSquares); 
		vector<Point2f> corners;
		bool patternfound = findChessboardCorners(gray, patternsize, corners); //Returns True if the chess board is detected  

		
		if (patternfound)
		{
			_isPoolDetected = true;
			dbg::trace(L"pool detected");

			cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1)); //Refines corner location
			drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);
		
			vector<Point3f> object_point; //Corner positions in the "ChessBoard coordinate system". 
			for (int j = 0; j < 36; j++)
				object_point.push_back(Point3f(float(j / numSquares), float(j%numSquares), 0.0f));

			vector<Point2f> image_point = corners; //Corner positions in the frame.

			Mat rvec; //Rotation from Camera Space to Pool Table Space.
			Mat tvec; //Translation from Camera Space to Pool Table Space.
			solvePnP(object_point, image_point, cameraMatrix, distCoeffs, rvec, tvec); //Find the ChessBoard pose.  

			//draw axis on the frame. 
			float length = 2.0f;
			vector<Point3f> axisPoints;
			axisPoints.push_back(Point3f(0, 0, 0));
			axisPoints.push_back(Point3f(length, 0, 0));
			axisPoints.push_back(Point3f(0, length, 0));
			axisPoints.push_back(Point3f(0, 0, length));
			vector< Point2f > imagePoints;
			projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

			line(frame, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);
			line(frame, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 3);
			line(frame, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);

			double square_size = 0.029; //the pose is given with the unit = a ChessBoard corner size.

			float3 Chess_origin_camera_space = { float(tvec.at<double>(0, 0)*square_size), float(tvec.at<double>(1, 0)*square_size), float(tvec.at<double>(2, 0)*square_size)};

			Chess_position_world_space = m_transform(Chess_origin_camera_space, CameraToWorld);

			Mat R;
			Rodrigues(rvec, R);

			//Set Pool pockets posiotins in world space
			float midlength_table = 1.17f;
			float width_table = 1.17f;

			//offset in meters between the chessBoard origin and the corner pocket. 
			double offset_origin = 0.07 ; 


			
			Mat pocket1_table_space(3, 1, cv::DataType<double>::type);
			pocket1_table_space.at<double>(0, 0) = -offset_origin / square_size;
			pocket1_table_space.at<double>(1, 0) = -offset_origin / square_size;
			pocket1_table_space.at<double>(2, 0) = 0.0;

			Mat pocket1_camera_space = R * pocket1_table_space + tvec;
			
			pocket1_camera_space.at<double>(0, 0) = square_size * pocket1_camera_space.at<double>(0, 0);
			pocket1_camera_space.at<double>(1, 0) = -square_size * pocket1_camera_space.at<double>(1, 0);
			pocket1_camera_space.at<double>(2, 0) = -square_size * pocket1_camera_space.at<double>(2, 0);

			Mat pocket1_world_space = R_world * pocket1_camera_space + tvec_world;

			float3 pocket1_world_spacef = { float(pocket1_world_space.at<double>(0, 0)), float(pocket1_world_space.at<double>(1, 0)) , float(pocket1_world_space.at<double>(2, 0)) };

			pocket_world_space.push_back(pocket1_world_spacef);

			//pocket2

			Mat pocket2_table_space(3, 1, cv::DataType<double>::type);
			pocket2_table_space.at<double>(0, 0) = -0.07 / square_size;
			pocket2_table_space.at<double>(1, 0) = (-0.07 + midlength_table) / square_size;
			pocket2_table_space.at<double>(2, 0) = 0.0;

			Mat pocket2_camera_space = R * pocket2_table_space + tvec;

			pocket2_camera_space.at<double>(0, 0) = square_size * pocket2_camera_space.at<double>(0, 0);
			pocket2_camera_space.at<double>(1, 0) = -square_size * pocket2_camera_space.at<double>(1, 0);
			pocket2_camera_space.at<double>(2, 0) = -square_size * pocket2_camera_space.at<double>(2, 0);

			Mat pocket2_world_space = R_world * pocket2_camera_space + tvec_world;

			float3 pocket2_world_spacef = { float(pocket2_world_space.at<double>(0, 0)), float(pocket2_world_space.at<double>(1, 0)) , float(pocket2_world_space.at<double>(2, 0)) };

			pocket_world_space.push_back(pocket2_world_spacef);

			//pocket3

			Mat pocket3_table_space(3, 1, cv::DataType<double>::type);
			pocket3_table_space.at<double>(0, 0) = -0.07 / square_size;
			pocket3_table_space.at<double>(1, 0) = (-0.07 + 2*midlength_table) / square_size;
			pocket3_table_space.at<double>(2, 0) = 0.0;

			Mat pocket3_camera_space = R * pocket3_table_space + tvec;

			pocket3_camera_space.at<double>(0, 0) = square_size * pocket3_camera_space.at<double>(0, 0);
			pocket3_camera_space.at<double>(1, 0) = -square_size * pocket3_camera_space.at<double>(1, 0);
			pocket3_camera_space.at<double>(2, 0) = -square_size * pocket3_camera_space.at<double>(2, 0);

			Mat pocket3_world_space = R_world * pocket3_camera_space + tvec_world;

			float3 pocket3_world_spacef = { float(pocket3_world_space.at<double>(0, 0)), float(pocket3_world_space.at<double>(1, 0)) , float(pocket3_world_space.at<double>(2, 0)) };

			pocket_world_space.push_back(pocket3_world_spacef);

			//pocket4

			Mat pocket4_table_space(3, 1, cv::DataType<double>::type);
			pocket4_table_space.at<double>(0, 0) = (-0.07 + width_table)/ square_size;
			pocket4_table_space.at<double>(1, 0) = (-0.07 + 2 * midlength_table) / square_size;
			pocket4_table_space.at<double>(2, 0) = 0.0;

			Mat pocket4_camera_space = R * pocket4_table_space + tvec;

			pocket4_camera_space.at<double>(0, 0) = square_size * pocket4_camera_space.at<double>(0, 0);
			pocket4_camera_space.at<double>(1, 0) = -square_size * pocket4_camera_space.at<double>(1, 0);
			pocket4_camera_space.at<double>(2, 0) = -square_size * pocket4_camera_space.at<double>(2, 0);

			Mat pocket4_world_space = R_world * pocket4_camera_space + tvec_world;

			float3 pocket4_world_spacef = { float(pocket4_world_space.at<double>(0, 0)), float(pocket4_world_space.at<double>(1, 0)) , float(pocket4_world_space.at<double>(2, 0)) };

			pocket_world_space.push_back(pocket4_world_spacef);

			//pocket5

			Mat pocket5_table_space(3, 1, cv::DataType<double>::type);
			pocket5_table_space.at<double>(0, 0) = 
			pocket5_table_space.at<double>(1, 0) = (-0.07 + midlength_table) / square_size;
			pocket5_table_space.at<double>(2, 0) = 0.0;

			Mat pocket5_camera_space = R * pocket5_table_space + tvec;

			pocket5_camera_space.at<double>(0, 0) = square_size * pocket5_camera_space.at<double>(0, 0);
			pocket5_camera_space.at<double>(1, 0) = -square_size * pocket5_camera_space.at<double>(1, 0);
			pocket5_camera_space.at<double>(2, 0) = -square_size * pocket5_camera_space.at<double>(2, 0);

			Mat pocket5_world_space = R_world * pocket5_camera_space + tvec_world;

			float3 pocket5_world_spacef = { float(pocket5_world_space.at<double>(0, 0)), float(pocket5_world_space.at<double>(1, 0)) , float(pocket5_world_space.at<double>(2, 0)) };

			pocket_world_space.push_back(pocket5_world_spacef);
			
			//pocket6

			Mat pocket6_table_space(3, 1, cv::DataType<double>::type);
			pocket6_table_space.at<double>(0, 0) = (-0.07 + width_table) / square_size;
			pocket6_table_space.at<double>(1, 0) = -0.07  / square_size;
			pocket6_table_space.at<double>(2, 0) = 0.0;

			Mat pocket6_camera_space = R * pocket6_table_space + tvec;

			pocket6_camera_space.at<double>(0, 0) = square_size * pocket6_camera_space.at<double>(0, 0);
			pocket6_camera_space.at<double>(1, 0) = -square_size * pocket6_camera_space.at<double>(1, 0);
			pocket6_camera_space.at<double>(2, 0) = -square_size * pocket6_camera_space.at<double>(2, 0);

			Mat pocket6_world_space = R_world * pocket6_camera_space + tvec_world;

			float3 pocket6_world_spacef = { float(pocket6_world_space.at<double>(0, 0)), float(pocket6_world_space.at<double>(1, 0)) , float(pocket6_world_space.at<double>(2, 0)) };

			pocket_world_space.push_back(pocket6_world_spacef);
		}
	}



	
	void AppMain::DetectTargetBall(Mat frame, Windows::Media::Devices::Core::CameraIntrinsics^ cameraIntrinsics, Windows::Perception::Spatial::SpatialCoordinateSystem^ CameraCoordinateSystem, Windows::Foundation::Numerics::float4x4 CameraToWorld)
	{
		Mat cameraMatrix(3, 3, CV_64FC1);
		Mat distCoeffs(5, 1, CV_64FC1);

		if (nullptr != cameraIntrinsics)
		{
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


		Mat HSVframe;
		vector<Mat> channels(3);
		Mat frame2;
		cv::blur(frame, frame2, cv::Size(10, 10));
		cv::cvtColor(frame2, HSVframe, CV_BGR2HSV);
		cv::split(HSVframe, channels);
		Mat hframe;
		hframe = channels[0];

		//calculate mean Hue channel
		Scalar tempval = mean(hframe);
		double Hmean = tempval.val[0];

		//threshold on the Hue channel
		double minthres = Hmean - 10;
		double maxthres = Hmean + 10;
		Scalar mintable = { minthres,0,0 };
		Scalar maxtable = { maxthres,255,255 };
		Mat threshold;
		cv::inRange(HSVframe, mintable, maxtable, threshold);

		// Create a structuring element
		int erosion_size = 4;
		Mat element = getStructuringElement(cv::MORPH_CROSS,
			cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
			cv::Point(erosion_size, erosion_size));

		// Apply erosion or dilation on the image
		cv::erode(threshold, threshold, element);
		cv::dilate(threshold, threshold, element);

		//Detect contours avec FindContours
		vector<vector<cv::Point> > contours;
		vector<Vec4i> hierarchy;

		cv::findContours(threshold, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		vector<vector<cv::Point> > contours_poly(contours.size());
		vector<Point2f>center(contours.size());
		vector<float>radius(contours.size());

		double fx = cameraIntrinsics->FocalLength.x;
		//double fy = cameraIntrinsics->FocalLength.y;
		double maxRadius = 0.1*fx;
		double MinRadius = 0.01*fx;
		Scalar color_ball = Scalar(0, 0, 0);
		vector<vector<cv::Point>>hull(contours.size());

		int bestball = 0;
		float bestDotProduct = -1.0f;
		
		
		for (size_t i = 0; i < contours.size(); i++)
		{
			convexHull(Mat(contours[i]), hull[i]);
			approxPolyDP(hull[i], contours_poly[i], 3, true);
			minEnclosingCircle(contours_poly[i], center[i], radius[i]);

			// Calculate the vector towards the center of the ball.
			float unproject_xi = float((center[i].x - cameraIntrinsics->PrincipalPoint.x) / cameraIntrinsics->FocalLength.x);
			float unproject_yi = float((center[i].y - cameraIntrinsics->PrincipalPoint.y) / cameraIntrinsics->FocalLength.y);
			float3 vectorTowardscenter_unormalizedi = float3{ unproject_xi ,  unproject_yi , -1.0f };

			float3 vectorTowardscenteri = normalize(vectorTowardscenter_unormalizedi);

			// Get the dot product between the vector towards the face and the gaze vector.
			// The closer the dot product is to 1.0, the closer the face is to the middle of the video image.
			float const dotFaceWithGaze = dot(vectorTowardscenteri, -float3::unit_z());

			// Pick the ball that best matches the users gaze.
			if (dotFaceWithGaze > bestDotProduct && radius[i]<maxRadius && radius[i]>MinRadius)
			{
				bestDotProduct = dotFaceWithGaze;
				bestball = i;
			}
		}

		float BallWidthInMeters = 0.051f;
		float const estimatedBallDepth = cameraIntrinsics->FocalLength.x * BallWidthInMeters / (2.0f * radius[bestball]);

		float unproject_x = float((center[bestball].x - cameraIntrinsics->PrincipalPoint.x) / cameraIntrinsics->FocalLength.x);
		float unproject_y = float((center[bestball].y - cameraIntrinsics->PrincipalPoint.y) / cameraIntrinsics->FocalLength.y);
		float3 vectorTowardscenter_unormalized = float3{ unproject_x ,  -unproject_y , -1.0f };
		float3 vectorTowardscenter = normalize(vectorTowardscenter_unormalized);
		TargetBallPositionInCameraSpace = vectorTowardscenter * estimatedBallDepth;
		TargetBallPositionInWorldSpace = m_transform(TargetBallPositionInCameraSpace, CameraToWorld);
		cv::circle(frame, center[bestball], (int)radius[bestball], color_ball, 2, 8, 0);
	}
	


	void AppMain::DetectWhiteBall(Mat frame, Windows::Media::Devices::Core::CameraIntrinsics^ cameraIntrinsics, Windows::Perception::Spatial::SpatialCoordinateSystem^ CameraCoordinateSystem, Windows::Foundation::Numerics::float4x4 CameraToWorld)
	{
		Mat cameraMatrix(3, 3, CV_64FC1);
		Mat distCoeffs(5, 1, CV_64FC1);

		if (nullptr != cameraIntrinsics)
		{
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

		//BallDetection

		Mat HSVframe;
		vector<Mat> channels(3);
		Mat frame2;
		cv::blur(frame, frame2, cv::Size(10	, 10));
		cv::cvtColor(frame2, HSVframe, CV_BGR2HSV);
		cv::split(HSVframe, channels);
		Mat hframe;
		hframe = channels[0];

		//calculate mean Hue channel
		Scalar tempval = mean(hframe);
		double Hmean = tempval.val[0];

		//threshold on the Hue channel
		double minthres = Hmean - 10;
		double maxthres = Hmean + 10;
		Scalar mintable = { minthres,0,0 };
		Scalar maxtable = { maxthres,255,255 };
		Mat threshold;
		cv::inRange(HSVframe, mintable, maxtable, threshold);

		// Create a structuring element
		int erosion_size = 4;
		Mat element = getStructuringElement(cv::MORPH_CROSS,
			cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
			cv::Point(erosion_size, erosion_size));

		// Apply erosion or dilation on the image
		cv::erode(threshold, threshold, element);
		cv::dilate(threshold, threshold, element);

		//Detect contours avec FindContours
		vector<vector<cv::Point> > contours;
		vector<Vec4i> hierarchy;

		cv::findContours(threshold, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		vector<vector<cv::Point> > contours_poly(contours.size());
		vector<Point2f>center(contours.size());
		vector<float>radius(contours.size());

		double fx = cameraIntrinsics->FocalLength.x;
		//double fy = cameraIntrinsics->FocalLength.y;
		double maxRadius = 0.1*fx;
		double MinRadius = 0.01*fx;
		Scalar color_ball = Scalar(0, 0, 0);
		vector<vector<cv::Point>>hull(contours.size());

		
		float3 vectorTowardscenter;

		size_t i = 0;
		int white_ball = 0;
		while (i < contours.size() && white_ball_found == false)
		{
			convexHull(Mat(contours[i]), hull[i]);
			approxPolyDP(hull[i], contours_poly[i], 3, true);
			minEnclosingCircle(contours_poly[i], center[i], radius[i]);
			cv::Rect _boundingRect = boundingRect(contours[i]);
			Scalar mean_color_contour = mean(frame(_boundingRect));


			if (radius[i]<maxRadius && radius[i]>MinRadius && mean_color_contour.val[0] + mean_color_contour.val[0] + mean_color_contour.val[0] > 500)
			{
				white_ball_found = true;
				white_ball = i;

				float unproject_x = float((center[white_ball].x - cameraIntrinsics->PrincipalPoint.x) / cameraIntrinsics->FocalLength.x);
				float unproject_y = float((center[white_ball].y - cameraIntrinsics->PrincipalPoint.y) / cameraIntrinsics->FocalLength.y);
				float3 vectorTowardscenter_unormalized = float3{ unproject_x ,  -unproject_y , -1.0f };
				vectorTowardscenter = normalize(vectorTowardscenter_unormalized);
				float BallWidthInMeters = 0.051f;
				float const estimatedBallDepth = cameraIntrinsics->FocalLength.x * BallWidthInMeters / (2.0f * radius[white_ball]);
				WhiteBallPositionInCameraSpace = vectorTowardscenter * estimatedBallDepth;
			}
		i=i+1;
		}
			
		WhiteBallPositionInWorldSpace = m_transform(WhiteBallPositionInCameraSpace,CameraToWorld);
		cv::circle(frame, center[white_ball], (int)radius[white_ball], Scalar{ 0,255,0 }, 2, 8, 0);

		dbg::trace(L"ca");

		float3 CameraPositionWorldSpace = m_transform(float3{ 0,0,0 },CameraToWorld);

		vectorTowardscenterWorldSpace =  normalize(m_transform(vectorTowardscenter,CameraToWorld));

		anchor = SpatialAnchor::TryCreateRelativeTo(m_WorldCoordinateSystem, WhiteBallPositionInWorldSpace);

	}

	
	int count = 0;
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

		

		float distanceFromUser = 1.17f; // meters
		//float distanceFromUser = 2.f; // meters

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

				//dbg::trace(L"TargetPosition");
				//dbg::trace(L"%f %f %f ", TargetPosition.x, TargetPosition.y, TargetPosition.z);

				float _rotationInRadiansY = std::atan2(headDirection.z,headDirection.x) + DirectX::XM_PIDIV2;
				const float radiansY = static_cast<float>(fmod(_rotationInRadiansY, DirectX::XM_2PI));

				float4x4 RotationRenderedPlane = make_float4x4_rotation_y(-radiansY);

				float4x4 trans = make_float4x4_translation(TargetPosition);


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
				
				float4x4 mul = trans * RotationRenderedPlane;

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

		
		for (auto cameraPose : prediction->CameraPoses)
		{
			// The HolographicCameraRenderingParameters class provides access to set
			// the image stabilization parameters.
			HolographicCameraRenderingParameters^ renderingParameters = holographicFrame->GetRenderingParameters(cameraPose);

			// put the focus point on the cube
			renderingParameters->SetFocusPoint(m_WorldCoordinateSystem, center_plane_world_space, normal_plane_world_space);
		}
		

		//Convert the latest frame in a cv::Mat object. 
		cv::Mat frame;
		rmcv::WrapHoloLensSensorFrameWithCvMat(latestFrame, frame);

		
		//Set the transform from the Camera Space to the World Space
		
		
		
		const auto tryTransformCtW = CameraCoordinateSystem->TryGetTransformTo(m_WorldCoordinateSystem);

		
		if (tryTransformCtW != nullptr)
		{
			Windows::Foundation::Numerics::float4x4 CameraToWorld = tryTransformCtW->Value;
			

			//if (!_isPoolDetected)
			//{ 
				//if the pool table has not been detected yet
			//	DetectPoolTable(frame, CameraCoordinateSystem, cameraIntrinsics, CameraToWorld);
			//}
			//else
			//{ 
				if (!target_ball_found)
					DetectTargetBall(frame, cameraIntrinsics, CameraCoordinateSystem, CameraToWorld);

				if (target_ball_found)
				{

					
					float3 CameraPositionWorldSpace = m_transform(float3{ 0.f,0.f,0.f }, CameraToWorld);
					float3 TargetBallPositionRenderedFrame3D = IntersectionLinePlane(CameraPositionWorldSpace, TargetBallPositionInWorldSpace, plane_frame_world_space);

					float xt = dot(TargetBallPositionRenderedFrame3D - center_plane_world_space, X_frame_world_space);
					float yt = dot(TargetBallPositionRenderedFrame3D - center_plane_world_space, Y_frame_world_space);
					float xt_im = frame.cols / 2.0f + xt * cameraIntrinsics->FocalLength.x;
					float yt_im = frame.rows / 2.0f + yt * cameraIntrinsics->FocalLength.y;

					float ltb = length(TargetBallPositionRenderedFrame3D - CameraPositionWorldSpace);
					float Ltb = length(TargetBallPositionInWorldSpace - CameraPositionWorldSpace);
					float rtb = (ltb*ball_real_diameter) / (2.f*Ltb);
					Point2f TargetBallPositionRenderedFrame2D = Point2f{ xt_im,yt_im };
					cv::circle(frame, TargetBallPositionRenderedFrame2D, int(rtb*cameraIntrinsics->FocalLength.x), Scalar(0, 255, 0), 2);

					if (!white_ball_found && count < 30) 
						DetectWhiteBall(frame, cameraIntrinsics, CameraCoordinateSystem, CameraToWorld);
					else
					{
						frame = 0.f * frame;
						if (anchor != nullptr)
						{
							SpatialCoordinateSystem^ anchorspace = anchor->CoordinateSystem;

							const auto tryTransformAWBtW = anchorspace->TryGetTransformTo(m_WorldCoordinateSystem);
							if (tryTransformAWBtW != nullptr)
							{
								Windows::Foundation::Numerics::float4x4 AnchorWBToWorld = tryTransformAWBtW->Value;
								WhiteBallPositionInWorldSpace = m_transform(float3{ 0,0,0 }, AnchorWBToWorld);
								dbg::trace(L"anchor");
								dbg::trace(L"%f %f %f", WhiteBallPositionInWorldSpace.x, WhiteBallPositionInWorldSpace.y, WhiteBallPositionInWorldSpace.z);
							}
						}

						float3 WhiteBallPositionRenderedFrame3D = IntersectionLinePlane(CameraPositionWorldSpace, WhiteBallPositionInWorldSpace, plane_frame_world_space);
					
						float xw = dot(WhiteBallPositionRenderedFrame3D - center_plane_world_space, X_frame_world_space);
						float yw = dot(WhiteBallPositionRenderedFrame3D - center_plane_world_space, Y_frame_world_space);
						float xw_im = frame.cols / 2.0f + xw*cameraIntrinsics->FocalLength.x;
						float yw_im = frame.rows / 2.0f + yw*cameraIntrinsics->FocalLength.y;

								//dbg::trace(L"point in plane");
								//dbg::trace(L"%f", plane_frame_attached_space.x*WhiteBallPositionRenderedFrame3D.x + plane_frame_attached_space.y*WhiteBallPositionRenderedFrame3D.y + plane_frame_attached_space.z*WhiteBallPositionRenderedFrame3D.z + plane_frame_attached_space.w);

								//dbg::trace(L"point in line");
								//dbg::trace(L"%f", dot(normalize(CameraPositionWorldSpace - WhiteBallPositionRenderedFrame3D), normalize(CameraPositionWorldSpace - WhiteBallPositionInWorldSpace)));

						float lw = length(WhiteBallPositionRenderedFrame3D - CameraPositionWorldSpace);
						float Lw = length(WhiteBallPositionInWorldSpace - CameraPositionWorldSpace);
						float rw = (lw*ball_real_diameter) / (2.f*Lw);
						Point2f WhiteBallPositionRenderedFrame2D = Point2f{ xw_im,yw_im };
						cv::circle(frame, WhiteBallPositionRenderedFrame2D, int(rw*cameraIntrinsics->FocalLength.x), Scalar(255, 0, 0), 2);

						cv::line(frame, WhiteBallPositionRenderedFrame2D, TargetBallPositionRenderedFrame2D, Scalar(0, 0, 255), 1);

						count = count + 1;

					
					}
				}
		}
		
		
	
		cv::line(frame, Point2f{ 0.f,0.f }, Point2f{ float(frame.cols), 0.f }, Scalar(255, 255, 255), 2);
		cv::line(frame, Point2f{ float(frame.cols), 0.f }, Point2f{ float(frame.cols), float(frame.rows) }, Scalar(255, 255, 255), 2);
		cv::line(frame, Point2f{ float(frame.cols), float(frame.rows) }, Point2f{ 0.f,float(frame.rows) }, Scalar(255, 255, 255), 2);
		cv::line(frame, Point2f{ 0.f,float(frame.rows) }, Point2f{ 0.f,0.f } , Scalar(255, 255, 255), 2);
		

		


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
			_currentSlateRenderer->Render(_currentVisualizationTexture);
			
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
