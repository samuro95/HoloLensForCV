
///*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#include "pch.h"

#include "AppMain.h"


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

namespace ComputeOnDevice
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
	{
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
		
		//m_locator = SpatialLocator::GetDefault();
		m_locator = _spatialPerception->GetSpatialLocator();

		// The simplest way to render world-locked holograms is to create a stationary reference frame
		// when the app is launched. This is roughly analogous to creating a "world" coordinate system
		// with the origin placed at the device's position as the app is launched.
		
		//m_referenceFrame = m_locator->CreateAttachedFrameOfReferenceAtCurrentHeading();
		m_referenceFrame = _spatialPerception->GetOriginFrameOfReference();

		//Get the world coordinate system in use 
		m_WorldCoordinateSystem = m_referenceFrame->CoordinateSystem;
	}




	void AppMain::OnSpatialInput(
		_In_ Windows::UI::Input::Spatial::SpatialInteractionSourceState^ pointerState) //On a click 

	{
		
		//Get the coordinate system in use 
		Windows::Perception::Spatial::SpatialCoordinateSystem^ currentCoordinateSystem =
			_spatialPerception->GetOriginFrameOfReference()->CoordinateSystem;


		if (!_isActiveRenderer) //isActiveRendere is True when the Hologram visualisation window is activated but not frozen 
		{
			_currentSlateRenderer =
				std::make_shared<Rendering::SlateRenderer>(
					_deviceResources);
			_slateRendererList.push_back(_currentSlateRenderer);

			// When a Pressed gesture is detected, the sample hologram will be repositioned
			// two meters in front of the user.

			_currentSlateRenderer->PositionHologram(
				pointerState->TryGetPointerPose(currentCoordinateSystem));

			_isActiveRenderer = true;
		}
		else
		{
			// Freeze frame
			_visualizationTextureList.push_back(_currentVisualizationTexture);
			_currentVisualizationTexture = nullptr; //freeze 
			_isActiveRenderer = false;
		}
	}

	void AppMain::sign(_In_ float x, _Out_ int res)
	{
		res = (x > 0) - (x < 0);
	}


	void AppMain::DetectPoolTable(Mat frame, SpatialCoordinateSystem^ CameraCoordinateSystem, Windows::Media::Devices::Core::CameraIntrinsics^ cameraIntrinsics, Windows::Foundation::Numerics::float4x4 CameraViewTransform, Windows::Foundation::Numerics::float4x4 FrameToOrigin, Windows::Foundation::Numerics::float4x4 OriginToFrame)
	{

		//Use ChessBoardDetection to detect a corner and set a coordinate system linked with the plan of the pool table

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


		Mat tvec_cam(3, 1, cv::DataType<double>::type); // translation vector extrisics camera 
		Mat R_cam(3, 3, cv::DataType<double>::type); //rotation vector extrisics camera 

		tvec_cam.at<double>(0, 0) = CameraViewTransform.m14;
		tvec_cam.at<double>(1, 0) = CameraViewTransform.m24;
		tvec_cam.at<double>(2, 0) = CameraViewTransform.m34;

		R_cam.at<double>(0, 0) = CameraViewTransform.m11;
		R_cam.at<double>(1, 0) = CameraViewTransform.m21;
		R_cam.at<double>(2, 0) = CameraViewTransform.m31;
		R_cam.at<double>(0, 1) = CameraViewTransform.m12;
		R_cam.at<double>(1, 1) = CameraViewTransform.m22;
		R_cam.at<double>(2, 1) = CameraViewTransform.m32;
		R_cam.at<double>(0, 2) = CameraViewTransform.m13;
		R_cam.at<double>(1, 2) = CameraViewTransform.m23;
		R_cam.at<double>(2, 2) = CameraViewTransform.m33;

		Mat rvec_cam(3, 1, cv::DataType<double>::type);//rodrigues rotation matrix
		Rodrigues(R_cam, rvec_cam);


		Mat gray;
		cv::cvtColor(frame, gray, CV_BGR2GRAY);

		int numSquares = 6;

		cv::Size patternsize(numSquares, numSquares); //interior number of corners

		vector<Point2f> corners;

		bool patternfound = findChessboardCorners(gray, patternsize, corners);

		if (patternfound)
		{


			cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
				TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);


			vector<vector<Point3f> > object_points;
			vector<vector<Point2f> > image_points;
			vector<vector<Point2f> > check_image_points;
			cv::Point check_image_point;

			vector<Point3f> obj;
			for (int j = 0; j < 36; j++)
				obj.push_back(Point3f(float(j / numSquares), float(j%numSquares), 0.0f));

			image_points.push_back(corners);
			object_points.push_back(obj);

	
			vector<Point3f> object_point = object_points[0];
			vector<Point2f> image_point = image_points[0];

			//calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
			Mat rvec;
			Mat tvec;
			solvePnP(object_point, image_point, cameraMatrix, distCoeffs, rvec, tvec);

			//draw axis 
			float length = 2.0f;
			vector<Point3f> axisPoints;
			axisPoints.push_back(Point3f(0, 0, 0));
			axisPoints.push_back(Point3f(length, 0, 0));
			axisPoints.push_back(Point3f(0, length, 0));
			axisPoints.push_back(Point3f(0, 0, length));
			vector< Point2f > imagePoints;
			projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

			double square_size = 0.029;
		

			/*
			double square_size = 0.029;
			float midlength_table = float(1.5 / square_size);
			//float width_table = 10.f;
			vector<Point3f> pocketPoints;
			pocketPoints.push_back(Point3f(0, 0, 0));
			pocketPoints.push_back(Point3f(0, midlength_table, 0));
			//pocketPoints.push_back(Point3f(2 * midlength_table, 0, 0));
			//pocketPoints.push_back(Point3f(2 * midlength_table, width_table, 0));
			//pocketPoints.push_back(Point3f(midlength_table, width_table, 0));
			//pocketPoints.push_back(Point3f(width_table, 0, 0));
			projectPoints(pocketPoints, rvec, tvec, cameraMatrix, distCoeffs, m_image_pocket_points);
			*/


			// draw axis lines
			line(frame, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);
			line(frame, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 3);
			line(frame, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);

			
			//Convert rotation vector into rotation matrix 
			Mat R;
			Rodrigues(rvec, R);

			vector<Point3f> spacePoints;
			vector<Point2f> imPoints;

			double xa = tvec.at<double>(0, 0)*square_size;
			double ya = tvec.at<double>(1, 0)*square_size;
			double za = tvec.at<double>(2, 0)*square_size;

			float3 Chess_position_camera_view_space = { float(xa), float(ya), float(za) };


			vector<Point3f> spaceP;
			spaceP.push_back(Point3f(Chess_position_camera_view_space.x, Chess_position_camera_view_space.y, Chess_position_camera_view_space.z));
			vector< Point2f > imageP;

			Mat tvec_0(3, 1, CV_64F, double(0)); // translation vector extrisics camera 
			Mat R_0(3, 3, CV_64F); //rotation vector extrisics camera 
			cv::setIdentity(R_0);
			projectPoints(spaceP, R_0, tvec_0, cameraMatrix, distCoeffs, imageP);
			
			//cv::Point2f final_point = (float(x), float(y));
			cv::Point2f final_point = imageP[0];
			cv::Point2f center_point = imagePoints[0];

			circle(frame, final_point, 30, Scalar(1, 1, 1), 3);

			double dist = pow(center_point.x - final_point.x, 2) + pow(center_point.y - final_point.y, 2);

			double dist_max = (0.01*cameraIntrinsics->FocalLength.x)*(0.01*cameraIntrinsics->FocalLength.y);
			
			if (dist < dist_max)
			{ 
				//create quaternion 

				//const float4x4 Rotation = float4x4(R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), float(0.) , R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), float(0.), R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), float(0.), float(0.), float(0.), float(0.), float(1.));
		
				//SpatialAnchor^ m_table_anchor = SpatialAnchor::TryCreateRelativeTo(CameraCoordinateSystem, Chess_position_camera_space);
			

				/*
				quaternion orientationn = make_quaternion_from_axis_angle(float3(0, 0, 1), 0.5);

				float w = float(std::sqrt(max(0.0, 1.0 + R.at<float>(0, 0) + R.at<float>(1, 1) + R.at<float>(2, 2))) * 0.5);
				float x = float(std::sqrt(max(0.0, 1.0 + R.at<float>(0, 0) - R.at<float>(1, 1) - R.at<float>(2, 2))) * 0.5);
				float y = float(std::sqrt(max(0.0, 1.0 - R.at<float>(0, 0) + R.at<float>(1, 1) - R.at<float>(2, 2))) * 0.5);
				float z = float(std::sqrt(max(0.0, 1.0 - R.at<float>(0, 0) - R.at<float>(1, 1) + R.at<float>(2, 2))) * 0.5);

				int resx;
				int resy;
				int resz;
				sign(R.at<float>(2, 1) - R.at<float>(1, 2), resx);
				x *= resx;
				sign(R.at<float>(0, 2) - R.at<float>(2, 0),resy);
				y *= resy;
				sign(R.at<float>(1, 0) - R.at<float>(0, 1),resz);
				z *= resz;

				quaternion orientation = quaternion(x, y, z, w);

				*/
				
				// Create the anchor at position.
				
				Mat Rot;

				//transpose(R_cam, R_cam_inv);

				Mat R_cam_inv = R_cam.inv();
				//Rot = R + R_cam_inv;

	
				/*
				double sy = sqrt(R.at<double>(0, 0) * Rot.at<double>(0, 0) + Rot.at<double>(1, 0) * Rot.at<double>(1, 0));

				bool singular = sy < 1e-6; // If

				double a, b, c;
				if (!singular)
				{
					a = atan2(Rot.at<double>(2, 1), Rot.at<double>(2, 2));
					b = atan2(-Rot.at<double>(2, 0), sy);
					c = atan2(Rot.at<double>(1, 0), Rot.at<double>(0, 0));
				}
				else
				{
					a = atan2(-Rot.at<double>(1, 2), Rot.at<double>(1, 1));
					b = atan2(-Rot.at<double>(2, 0), sy);
					c = 0;
				}
				*/
	
				
				float4x4 CameraViewTransformInv = float4x4 {
					R_cam_inv.at<float>(0, 0), R_cam_inv.at<float>(0, 1), R_cam_inv.at<float>(0, 2), 0.f, 
					R_cam_inv.at<float>(1, 0), R_cam_inv.at<float>(1, 1), R_cam_inv.at<float>(1, 2), 0.f, 
					R_cam_inv.at<float>(2, 0), R_cam_inv.at<float>(2, 1), R_cam_inv.at<float>(2, 2), 0.f, 
					0.f, 0.f, 0.f, 1 };
							
				
				float3 c2 = { Chess_position_camera_view_space.x - CameraViewTransform.m14, Chess_position_camera_view_space.y - CameraViewTransform.m24, Chess_position_camera_view_space.z - CameraViewTransform.m34 };
				
				float3 Chess_position_camera_space2 = {c2.x*CameraViewTransform.m11 + c2.y*CameraViewTransform.m21 + c2.z*CameraViewTransform.m31,
					c2.x*CameraViewTransform.m12 + c2.y*CameraViewTransform.m22 + c2.z*CameraViewTransform.m32,
					c2.x*CameraViewTransform.m13 + c2.y*CameraViewTransform.m23 + c2.z*CameraViewTransform.m33};

				float3 Chess_position_camera_space = {c2.x*R_cam_inv.at<float>(1, 1) + c2.y*R_cam_inv.at<float>(1, 2) + c2.z*R_cam_inv.at<float>(1, 3),
					c2.x*R_cam_inv.at<float>(2, 1) + c2.y*R_cam_inv.at<float>(2, 2) + c2.z*R_cam_inv.at<float>(2,3),
					c2.x*R_cam_inv.at<float>(3, 1) + c2.y*R_cam_inv.at<float>(3, 2) + c2.z*R_cam_inv.at<float>(3, 2) };

				//float3 c1 = (Chess_position_camera_space.x*CameraViewTransform.m11 + Chess_position_camera_space.y*CameraViewTransform.m12 + Chess_position_camera_space.z*CameraViewTransform.m13 + CameraViewTransform.m14,
					//Chess_position_camera_space.x*CameraViewTransform.m21 + Chess_position_camera_space.y*CameraViewTransform.m22 + Chess_position_camera_space.z*CameraViewTransform.m23 + CameraViewTransform.m24,
					//Chess_position_camera_space.x*CameraViewTransform.m31 + Chess_position_camera_space.y*CameraViewTransform.m32 + Chess_position_camera_space.z*CameraViewTransform.m33 + CameraViewTransform.m34);
				
				float3 c3 = transform(Chess_position_camera_space2, CameraViewTransform);

				float3 c5 = transform(Chess_position_camera_space, FrameToOrigin);

				float3 c6 = transform(Chess_position_camera_space, OriginToFrame);

				vector<Point3f> sp;
				sp.push_back(Point3f(c3.x, c3.y, c3.z));
				vector<Point2f> im;
				projectPoints(sp, R_0, tvec_0, cameraMatrix, distCoeffs, im);
				circle(frame, im[0], 100, Scalar(0, 0, 0), 2, 8, 0);

				



				//quaternion orientation = make_quaternion_from_yaw_pitch_roll(float(a), float(b), float(c));

				//m_table_anchor = SpatialAnchor::TryCreateRelativeTo(CameraCoordinateSystem, Chess_position_camera_space);
				
				//if (m_table_anchor != nullptr)
				//{	
		
				//	anchorSpace = m_table_anchor->CoordinateSystem;
		
				//	const auto tryTransform = m_WorldCoordinateSystem->TryGetTransformTo(anchorSpace);
				//	const auto tryTransform2 = anchorSpace->TryGetTransformTo(m_WorldCoordinateSystem);

				//	if (tryTransform != nullptr && tryTransform2 != nullptr)
				//	{
				//		_isPoolDetected = true;
				//
				//		WorldCoordinateSystemToAnchorSpace = tryTransform->Value;
				//		AnchorSpaceToWorldCoordinateSystem = tryTransform2->Value;

						//float midlength_table = 1.5f;
						//float width_table = 1.5f;
				//		m_world_pocket_points.push_back(transform(float3(0, 0, 0), AnchorSpaceToWorldCoordinateSystem));
						//m_world_pocket_points.push_back(transform(float3(0, width_table, 0), AnchorSpaceToWorldCoordinateSystem));
						//m_world_pocket_points.push_back(transform(float3(0, 0, midlength_table), AnchorSpaceToWorldCoordinateSystem));
						
						/*
						const auto tryTransform3 = m_WorldCoordinateSystem->TryGetTransformTo(CameraCoordinateSystem);
						Windows::Foundation::Numerics::float4x4 WorldCoordinateSystemToCameraSpace = tryTransform3->Value;


						float3 pocket_points_camera_space = transform(m_world_pocket_points[0], WorldCoordinateSystemToCameraSpace);
						float3 pocket_points_camera_view = transform(pocket_points_camera_space, CameraViewTransform);

						vector<Point3f> space;
						space.push_back(Point3f(pocket_points_camera_view.x, pocket_points_camera_view.y, pocket_points_camera_view.z));
						vector<Point2f> pocket_points_frame;
						
						projectPoints(space, R_0, tvec_0, cameraMatrix, distCoeffs, pocket_points_frame);
						circle(frame, pocket_points_frame[0], 100, Scalar (0,0,0) , 2, 8, 0);
						*/
				//	}
					
				}

			}
		}

	

	
	void AppMain::ProcessBalls(Mat frame, Windows::Media::Devices::Core::CameraIntrinsics^ cameraIntrinsics, Windows::Perception::Spatial::SpatialCoordinateSystem^ CameraCoordinateSystem, Windows::Foundation::Numerics::float4x4 CameraViewTransform)
	{

		Mat cameraMatrix(3, 3, CV_64FC1);
		Mat distCoeffs(5, 1, CV_64FC1);
	
		Mat HSVframe;
		vector<Mat> channels(3);

		cv::blur(frame, frame, cv::Size(5, 5));

		cv::cvtColor(frame, HSVframe, CV_BGR2HSV);
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
		double fy = cameraIntrinsics->FocalLength.y;
		double maxRadius = 0.03*(fx+fy/2.);
		double MinRadius = 0.01*(fx+fy/2.);
		Scalar color_ball = Scalar(255, 255, 255);
		vector<vector<cv::Point>>hull(contours.size());

		int bestball = 0;
		float bestDotProduct = -1.0f;
		bool ball_found = false;

		for (size_t i = 0; i < contours.size(); i++)
		{
			convexHull(Mat(contours[i]), hull[i]);
			approxPolyDP(hull[i], contours_poly[i], 3, true);
			minEnclosingCircle(contours_poly[i], center[i], radius[i]);
			
			// Calculate the vector towards the center of the ball.
			Windows::Foundation::Point centerOfBall_frame = { center[i].x,center[i].y };
			float2 const centerOfBall = cameraIntrinsics->UnprojectAtUnitDepth(centerOfBall_frame);
			float3 const vectorTowardscenter = normalize(float3{centerOfBall.x, centerOfBall.y, -1.0f });
			
			// Get the dot product between the vector towards the face and the gaze vector.
			// The closer the dot product is to 1.0, the closer the face is to the middle of the video image.
			float const dotFaceWithGaze = dot(vectorTowardscenter, -float3::unit_z());

			// Pick the ball that best matches the users gaze.
			if (dotFaceWithGaze > bestDotProduct && radius[i]<maxRadius && radius[i]>MinRadius)
			{
				bestDotProduct = dotFaceWithGaze;
				bestball = i;
				ball_found = true;
			}
		}

		
		const auto tryTransform = m_WorldCoordinateSystem->TryGetTransformTo(CameraCoordinateSystem);
		Windows::Foundation::Numerics::float4x4 WorldCoordinateSystemToCameraSpace = tryTransform->Value;


		float3 pocket_points_camera_space = transform(m_world_pocket_points[0], WorldCoordinateSystemToCameraSpace);
		float3 pocket_points_camera_view = transform(pocket_points_camera_space, CameraViewTransform);

		vector<Point3f> spaceP;
		spaceP.push_back(Point3f(pocket_points_camera_view.x, pocket_points_camera_view.y, pocket_points_camera_view.z));
		vector<Point2f> pocket_points_frame;
		Mat tvec_0(3, 1, CV_64F, double(0));
		Mat R_0(3, 3, CV_64F); 
		cv::setIdentity(R_0);
		projectPoints(spaceP, R_0, tvec_0, cameraMatrix, distCoeffs, pocket_points_frame);
		circle(frame, pocket_points_frame[0], 100, color_ball, 2, 8, 0);

		/*
		
		if (ball_found)
		{ 
			//draw circle
			circle(frame, center[bestball], (int)radius[bestball], color_ball, 2, 8, 0);
			// draw trajectory lines
			//line(frame, pocket_points_frame[0], center[bestball], Scalar(0, 0, 0), 3);
			//line(frame, m_image_pocket_points[1], center[bestball], Scalar(0, 0, 0), 3);
			//line(frame, image_pocket_points[2], center[bestball], Scalar(0, 0, 0), 3);
			//line(frame, image_pocket_points[3], center[bestball], Scalar(0, 0, 0), 3);
			//line(frame, image_pocket_points[4], center[bestball], Scalar(0, 0, 0), 3);
			//line(frame, image_pocket_points[5], center[bestball], Scalar(0, 0, 0), 3);
		}
		
		*/
		
	
		
	}


	void AppMain::OnUpdate(
		_In_ Windows::Graphics::Holographic::HolographicFrame^ holographicFrame,
		_In_ const Graphics::StepTimer& stepTimer)
	{
		UNREFERENCED_PARAMETER(holographicFrame);

		dbg::TimerGuard timerGuard(
			L"AppMain::OnUpdate",
			30.0 /* minimum_time_elapsed_in_milliseconds */);

		//
		// Update scene objects.
		//
		// Put time-based updates here. By default this code will run once per frame,
		// but if you change the StepTimer to use a fixed time step this code will
		// run as many times as needed to get to the current step.
		//



		for (auto& r : _slateRendererList)
		{
			r->Update(
				stepTimer);
		}


		//
		// Process sensor data received through the HoloLensForCV component !!!! 
		//

		if (!_holoLensMediaFrameSourceGroupStarted)
		{
			return;
		}


		

		// Get a prediction of where holographic cameras will be when this frame
		// is presented.
		//HolographicFramePrediction^ prediction = holographicFrame->CurrentPrediction;

		// Back buffers can change from frame to frame. Validate each buffer, and recreate
		// resource views and depth buffers as needed.
		//m_deviceResources->EnsureCameraResources(holographicFrame, prediction);

		//SpatialCoordinateSystem^ currentCoordinateSystem = m_referenceFrame->GetStationaryCoordinateSystemAtTimestamp(prediction->Timestamp);


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

		//cameraIntrinsics contains the camera parameters
		Windows::Media::Devices::Core::CameraIntrinsics^ cameraIntrinsics = latestFrame->CoreCameraIntrinsics;

		//Get the camera transforms
		Windows::Foundation::Numerics::float4x4 FrameToOrigin = latestFrame->FrameToOrigin;
		Windows::Foundation::Numerics::float4x4 OriginToFrame = latestFrame->OriginToFrame;
		Windows::Perception::Spatial::SpatialCoordinateSystem^ CameraCoordinateSystem = latestFrame->CameraCoordinateSystem;
		Windows::Foundation::Numerics::float4x4 CameraViewTransform = latestFrame->CameraViewTransform;
		Windows::Foundation::Numerics::float4x4 CameraProjectionTransform = latestFrame->CameraProjectionTransform;




		

		

		cv::Mat cameraMatrix(3, 3, CV_64FC1);
		cv::Mat distCoeffs(5, 1, CV_64FC1);

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


		cv::Mat frame;

		// WrapHoloLensSensorFrameWithCvMat defined in OpenCVHelpers 
		// In : holoLensSensorFrame (1st arg)
		// Out : cvMat wrappedImage (2nd arg)

		rmcv::WrapHoloLensSensorFrameWithCvMat(
			latestFrame,
			frame);

		Mat rvec;
		Mat tvec;

	
		
		//if (_isPoolDetected == false)
		//{
		//	DetectPoolTable(frame, CameraCoordinateSystem, cameraIntrinsics, CameraViewTransform);
		//}

		DetectPoolTable(frame, CameraCoordinateSystem, cameraIntrinsics, CameraViewTransform, FrameToOrigin, OriginToFrame);
		
		if (_isPoolDetected == false)
		{
		//	cv::blur(frame, frame, cv::Size(20, 20));
		}
		else
		{
			//ProcessBalls(frame, cameraIntrinsics, CameraCoordinateSystem, CameraViewTransform);
		}



		/*
		if (_isPoolDetected == true)
		{
			const auto tryTransform = anchorSpace->TryGetTransformTo(CameraCoordinateSystem);
			const auto tryTransform2 = anchorSpace->TryGetTransformTo(m_WorldCoordinateSystem);

			if (tryTransform != nullptr)
			{
				float4x4 AnchorSpaceToCameraCoordinateSystem;
				AnchorSpaceToCameraCoordinateSystem = tryTransform->Value;
				float3 point_anchor = (float(0.), float(0.), float(0.));
				float3 point_camera = transform(point_anchor, AnchorSpaceToCameraCoordinateSystem);
				Windows::Foundation::Point point_frame = cameraIntrinsics->ProjectOntoFrame(point_camera);
				cv::Point2f final_point = (point_frame.X, point_frame.Y);
				circle(wrappedImage, final_point, 20, Scalar(100, 200, 1), 5);
			}

			else
			{
				float4x4 AnchorSpaceToWorldCoordinateSystem;
				AnchorSpaceToWorldCoordinateSystem = tryTransform2->Value;
				float3 point_anchor = (float(0.), float(0.), float(0.));
				float3 point_world = transform(point_anchor, AnchorSpaceToWorldCoordinateSystem);
				float3 point_camera = transform(point_world, OriginToFrame);
				Windows::Foundation::Point point_frame = cameraIntrinsics->ProjectOntoFrame(point_camera);
				cv::Point2f final_point = (point_frame.X, point_frame.Y);
				circle(wrappedImage, final_point, 20, Scalar(100, 200, 1), 5);
			}

		}
		*/
		
		/*

		if (!_undistortMapsInitialized)
		{
			//Computes the undistortion and rectification transformation map

			cv::initUndistortRectifyMap(
				cameraMatrix,
				distCoeffs,
				cv::Mat_<double>::eye(3, 3),
				cameraMatrix,
				cv::Size(wrappedImage.cols, wrappedImage.rows),
				CV_32FC1,
				_undistortMap1,
				_undistortMap2);

			_undistortMapsInitialized = true;
		}



		if (_undistortMapsInitialized)
		{
			//if undistortion initialize, geometric transformation to undistort thanks to undistortmaps
			cv::remap(
				wrappedImage,
				_undistortedPVCameraImage,
				_undistortMap1,
				_undistortMap2,
				cv::INTER_LINEAR);

			//resize the image to Size(round(fx*src.cols), round(fy*src.rows))
			cv::resize(
				_undistortedPVCameraImage,
				_resizedPVCameraImage,
				cv::Size(),
				0.8,
				0.8,
				cv::INTER_AREA);
		}
		else
		{
			cv::resize(
				wrappedImage,
				_resizedPVCameraImage,
				cv::Size(),
				0.8,
				0.8,
				cv::INTER_AREA);
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
		for (size_t i = 0; i < _visualizationTextureList.size(); ++i)
		{
			_slateRendererList[i]->Render(
				_visualizationTextureList[i]);
		}

		if (_isActiveRenderer)
		{
			_currentSlateRenderer->Render(_currentVisualizationTexture);
		}
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
