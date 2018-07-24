
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
		, m_WorldCoordinateSystem_Set(false)
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

		// The simplest way to render world-locked holograms is to create a stationary reference frame
		// when the app is launched. This is roughly analogous to creating a "world" coordinate system
		// with the origin placed at the device's position as the app is launched.

		
		//m_referenceFrame = _spatialPerception->GetOriginFrameOfReference();
		m_referenceFrame = m_locator->CreateStationaryFrameOfReferenceAtCurrentLocation();


		m_WorldCoordinateSystem = m_referenceFrame->CoordinateSystem;

		
	}



	void AppMain::OnSpatialInput(
		_In_ Windows::UI::Input::Spatial::SpatialInteractionSourceState^ pointerState) //On a click 

	{

		//Get the coordinate system in use 
		//Windows::Perception::Spatial::SpatialCoordinateSystem^ currentCoordinateSystem =
		//	m_locator->CreateStationaryFrameOfReferenceAtCurrentLocation()->CoordinateSystem;

		m_reference_attached_Frame = m_locator->CreateAttachedFrameOfReferenceAtCurrentHeading();


		if (!_isActiveRenderer) //isActiveRendere is True when the Hologram visualisation window is activated but not frozen 
		{
			_currentSlateRenderer = std::make_shared<Rendering::SlateRenderer>(_deviceResources);
			m_markerRenderer = std::make_shared<Rendering::MarkerRenderer>(_deviceResources);

			_slateRendererList.push_back(_currentSlateRenderer);

			// When a Pressed gesture is detected, the sample hologram will be repositioned
			// two meters in front of the user.

			SpatialPointerPose^ pointerPose = pointerState->TryGetPointerPose(m_WorldCoordinateSystem);

			_currentSlateRenderer->PositionHologram(pointerPose);

			const float3 headPosition = pointerPose->Head->Position;
			const float3 headDirection = pointerPose->Head->ForwardDirection;
			float3 const headBack = -headDirection;
			float3 const headUp = pointerPose->Head->UpDirection;
			float3 const headRight = cross(headDirection, headUp);

			X_rendered_frame=headRigh;
			Y_rendered_frame=-headUp;

			constexpr float distanceFromUser = 2.0f; // meters
			const float3 gazeAtTwoMeters = headPosition + (distanceFromUser * headDirection);

			center_plane_rendered_frame = gazeAtTwoMeters;

			ConstructPlaneFromPointNormal(center_plane_rendered_frame, headDirection, plane_rendered_frame);
		
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

	
	void AppMain::ConstructPlaneFromPointNormal(_In_ Windows::Foundation::Numerics::float3 Point, Windows::Foundation::Numerics::float3 normal, _Out_ Windows::Foundation::Numerics::float4 plane)
	{
		// plane definded by ax+by+cz+d = 0 avec a=plane.x / b=plane.y / c= plane.z / d=plane.w
		float3 Normalized_normal = normalize(normal);
		plane.x = Normalized_normal.x;
		plane.y = Normalized_normal.y;
		plane.z = Normalized_normal.z;
		plane.w = -dot(Point, Normalized_normal);
	}

	void AppMain::IntersectionLinePlane(_In_ float3 p1, float3 p2, float4 plane, _Out_ float3 inters)
	{
		float3 diff = p1 - p2;
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
	}


	void AppMain::m_transform(double x_in, double y_in, double z_in, Windows::Foundation::Numerics::float4x4 Transform, double x_out, double y_out, double z_out)
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
		R.at<double>(1, 2) = Transform.m23;
		R.at<double>(2, 2) = Transform.m33;

		Mat mat_in(3, 1, cv::DataType<double>::type);
		mat_in.at<double>(0, 0) = x_in;
		mat_in.at<double>(1, 0) = y_in;
		mat_in.at<double>(2, 0) = z_in;

		Mat mat_out(3, 1, cv::DataType<double>::type);
		mat_out = R * mat_in + tvec;

		x_out = mat_out.at<double>(0, 0);
		y_out = mat_out.at<double>(1, 0);
		z_out = mat_out.at<double>(2, 0);

	}

	void AppMain::DetectPoolTable(Mat frame, SpatialCoordinateSystem^ CameraCoordinateSystem, Windows::Media::Devices::Core::CameraIntrinsics^ cameraIntrinsics, Mat tvec_cam, Mat R_cam, Windows::Foundation::Numerics::float4x4 CameraToWorld)
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


			cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
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


			vector<Point3f> spacePoints;
			vector<Point2f> imPoints;

			double xa = tvec.at<double>(0, 0)*square_size;
			double ya = tvec.at<double>(1, 0)*square_size;
			double za = tvec.at<double>(2, 0)*square_size;

			float3 Chess_position_camera_view_space = { float(xa), float(ya), float(za)};

			;

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

				Mat R_cam_inv(3, 3, cv::DataType<double>::type);

				cv::transpose(R_cam, R_cam_inv);

				Mat z1(3, 1, cv::DataType<double>::type);
				z1.at<double>(0, 0) = xa;
				z1.at<double>(1, 0) = -ya;
				z1.at<double>(2, 0) = -za;

			

				//Mat c2 = R_cam_inv * (c1 - tvec_cam);
				//Mat c2 = R_cam_inv * (c1);
				Mat z2 = z1;
			
				float3 Chess_position_camera_space = { float(z2.at<double>(0, 0)), float(z2.at<double>(1, 0)) , float(z2.at<double>(2, 0)) };

				Mat z3 = R_world * z2 + tvec_world;

				//m_transform(c2.at<double>(0, 0), c2.at<double>(0, 0), double z_in, Windows::Foundation::Numerics::float4x4 Transform, double x_out, double y_out, double z_out);
				
				Chess_position_world_space =  { float(z3.at<double>(0, 0)), float(z3.at<double>(1, 0)) , float(z3.at<double>(2, 0)) };

				
				

				_isPoolDetected = true;


			}

			//Convert rotation vector into rotation matrix 
			Mat R(3, 3, cv::DataType<double>::type);
			Rodrigues(rvec, R);
			

			float midlength_table = 1.17f;
			float width_table = 1.17f;

			//pocket1
			
			Mat pocket1_table_space(3, 1, cv::DataType<double>::type);
			pocket1_table_space.at<double>(0, 0) = -0.07/ square_size;
			pocket1_table_space.at<double>(1, 0) = -0.07/ square_size;
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

			

			//create quaternion s
			//float4x4 Rotation = float4x4{ float(Rot.at<double>(0, 0)), float(Rot.at<double>(0, 1)), float(Rot.at<double>(0, 2)), float(0.),
			//	float(Rot.at<double>(1, 0)), float(Rot.at<double>(1, 1)), float(Rot.at<double>(1, 2)), float(0.),
			//	float(Rot.at<double>(2, 0)), float(Rot.at<double>(2, 1)), float(Rot.at<double>(2, 2)), float(0.),
			//	float(0.), float(0.), float(0.), float(1.) };

			//quaternion q = make_quaternion_from_rotation_matrix(Rotation);

			//m_table_anchor = SpatialAnchor::TryCreateRelativeTo(CameraCoordinateSystem, Chess_position_camera_space, q);



			/*
			if (m_table_anchor != nullptr)
			{
			anchorSpace = m_table_anchor->CoordinateSystem;

			const auto tryTransform = m_WorldCoordinateSystem->TryGetTransformTo(anchorSpace);
			const auto tryTransform2 = anchorSpace->TryGetTransformTo(m_WorldCoordinateSystem);

			if (tryTransform != nullptr && tryTransform2 != nullptr)
			{
			_isPoolDetected = true;

			WorldCoordinateSystemToAnchorSpace = tryTransform->Value;
			AnchorSpaceToWorldCoordinateSystem = tryTransform2->Value;

			float midlength_table = 1.5f;
			float width_table = 1.5f;
			m_world_pocket_points.push_back(transform(float3{ 0, 0, 0 }, AnchorSpaceToWorldCoordinateSystem));
			m_world_pocket_points.push_back(transform(float3{ 0, width_table, 0 }, AnchorSpaceToWorldCoordinateSystem));
			m_world_pocket_points.push_back(transform(float3{ 0, 0, midlength_table }, AnchorSpaceToWorldCoordinateSystem));


			const auto tryTransform3 = m_WorldCoordinateSystem->TryGetTransformTo(CameraCoordinateSystem);
			Windows::Foundation::Numerics::float4x4 WorldCoordinateSystemToCameraSpace = tryTransform3->Value;

			float3 pocket_points_camera_space = transform(m_world_pocket_points[0], WorldCoordinateSystemToCameraSpace);
			float3 pocket_points_camera_view = transform(pocket_points_camera_space, CameraViewTransform);

			vector<Point3f> space;
			space.push_back(Point3f(pocket_points_camera_view.x, pocket_points_camera_view.y, pocket_points_camera_view.z));
			vector<Point2f> pocket_points_frame;

			projectPoints(space, R_0, tvec_0, cameraMatrix, distCoeffs, pocket_points_frame);
			circle(frame, pocket_points_frame[0], 100, Scalar(0, 0, 0), 2, 8, 0);

			}

			} 
			*/


		}
	}




	void AppMain::ProcessBalls(Mat frame, Windows::Media::Devices::Core::CameraIntrinsics^ cameraIntrinsics, Windows::Perception::Spatial::SpatialCoordinateSystem^ CameraCoordinateSystem, Mat tvec_world_i, Mat R_world_i, Windows::Foundation::Numerics::float4x4 CameraViewTransform, Windows::Foundation::Numerics::float4x4 CameraToWorld)
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

		//cv::blur(frame, frame, cv::Size(5, 5));

		cv::cvtColor(frame, HSVframe, CV_BGR2HSV);
		cv::split(HSVframe, channels);
		Mat hframe;
		hframe = channels[0];

		//calculate mean Hue channel
		Scalar tempval = mean(hframe);
		double Hmean = tempval.val[0];

		//Select dominqnt color with histograms
		/*
		// Quantize the hue to 30 levels
		// and the saturation to 32 levels
		int hbins = 30;
		int histSize[] = {hbins};
		// hue varies from 0 to 179, see cvtColor
		float hranges[] = { 0, 180 };
		const float* ranges[] = { hranges};
		MatND hist;
		// we compute the histogram from the 0-th and 1-st channels
		int channel[] = {0};

		calcHist(&HSVframe, 1, channel, Mat(), // do not use mask
			hist, 2, histSize, ranges,
			true, // the histogram is uniform
			false);

		double maxVal = 0;
		minMaxLoc(hist, 0, &maxVal, 0, 0);
		*/
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
		double maxRadius = 0.025*fx;
		double MinRadius = 0.01*fx;
		Scalar color_ball = Scalar(255, 255, 255);
		vector<vector<cv::Point>>hull(contours.size());

		int bestball = 0;
		float bestDotProduct = -1.0f;
		ball_found = false;
		float3 BallPositionInCameraSpace = { 0,0,0 };

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

		
		// Calculate the vector towards the center of the ball.
		Windows::Foundation::Point centerOfBall_frame = { center[bestball].x,center[bestball].y };
		float2 const centerOfBall = cameraIntrinsics->UnprojectAtUnitDepth(centerOfBall_frame);
		float3 const vectorTowardscenter = normalize(float3{ centerOfBall.x, centerOfBall.y, -1.0f });

		// Estimate depth anf ball position in world space
		constexpr float BallWidthInMeters = 0.045f;
		float const estimatedBallDepth = ((cameraIntrinsics->FocalLength.x+cameraIntrinsics->FocalLength.y)/2.0f)*BallWidthInMeters / static_cast<float>(2 * radius[bestball]);
		//float3 const BallPositionInCameraSpace = vectorTowardscenter * estimatedBallDepth;
		BallPositionInCameraSpace = vectorTowardscenter * estimatedBallDepth;

		dbg::trace(L"BallPositionCameraSpace");
		dbg::trace(L"%f %f %f", BallPositionInCameraSpace.x, BallPositionInCameraSpace.y, BallPositionInCameraSpace.z);

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


		Mat Ball1(3, 1, cv::DataType<double>::type);
		Ball1.at<double>(0, 0) = BallPositionInCameraSpace.x;
		Ball1.at<double>(1, 0) = BallPositionInCameraSpace.y;
		Ball1.at<double>(2, 0) = BallPositionInCameraSpace.z;

		Mat Ball2 = R_world * Ball1 + tvec_world;

		BallPositionInWorldSpace = { float(Ball2.at<double>(0, 0)), float(Ball2.at<double>(1, 0)) , float(Ball2.at<double>(2, 0)) };

		Mat tvec_cam(3, 1, cv::DataType<double>::type); // translation vector extrisics camera 
		Mat R_cam(3, 3, cv::DataType<double>::type); //rotation vector extrisics camera 

		tvec_cam.at<double>(0, 0) = CameraViewTransform.m41;
		tvec_cam.at<double>(1, 0) = CameraViewTransform.m42;
		tvec_cam.at<double>(2, 0) = CameraViewTransform.m43;

		R_cam.at<double>(0, 0) = CameraViewTransform.m11;
		R_cam.at<double>(1, 0) = CameraViewTransform.m12;
		R_cam.at<double>(2, 0) = CameraViewTransform.m13;
		R_cam.at<double>(0, 1) = CameraViewTransform.m21;
		R_cam.at<double>(1, 1) = CameraViewTransform.m22;
		R_cam.at<double>(2, 1) = CameraViewTransform.m23;
		R_cam.at<double>(0, 2) = CameraViewTransform.m31;
		R_cam.at<double>(1, 2) = CameraViewTransform.m32;
		R_cam.at<double>(2, 2) = CameraViewTransform.m33;


		Mat p1(3, 1, cv::DataType<double>::type);
		p1.at<double>(0, 0) = double(pocket_world_space[0].x);
		p1.at<double>(1, 0) = double(pocket_world_space[0].y);
		p1.at<double>(2, 0) = double(pocket_world_space[0].z);

		Mat p2(3, 1, cv::DataType<double>::type);
		p2.at<double>(0, 0) = double(pocket_world_space[1].x);
		p2.at<double>(1, 0) = double(pocket_world_space[1].y);
		p2.at<double>(2, 0) = double(pocket_world_space[1].z);

		Mat p3(3, 1, cv::DataType<double>::type);
		p3.at<double>(0, 0) = double(pocket_world_space[2].x);
		p3.at<double>(1, 0) = double(pocket_world_space[2].y);
		p3.at<double>(2, 0) = double(pocket_world_space[2].z);

		Mat p4(3, 1, cv::DataType<double>::type);
		p4.at<double>(0, 0) = double(pocket_world_space[3].x);
		p4.at<double>(1, 0) = double(pocket_world_space[3].y);
		p4.at<double>(2, 0) = double(pocket_world_space[3].z);

		Mat p5(3, 1, cv::DataType<double>::type);
		p5.at<double>(0, 0) = double(pocket_world_space[4].x);
		p5.at<double>(1, 0) = double(pocket_world_space[4].y);
		p5.at<double>(2, 0) = double(pocket_world_space[4].z);

		Mat p6(3, 1, cv::DataType<double>::type);
		p6.at<double>(0, 0) = double(pocket_world_space[5].x);
		p6.at<double>(1, 0) = double(pocket_world_space[5].y);
		p6.at<double>(2, 0) = double(pocket_world_space[5].z);

		
		Mat p1b = R_world_i * (p1 + tvec_world_i);
		Mat p2b = R_world_i * (p2 + tvec_world_i);
		Mat p3b = R_world_i * (p3 + tvec_world_i);
		Mat p4b = R_world_i * (p4 + tvec_world_i);
		Mat p5b = R_world_i * (p5 + tvec_world_i);
		Mat p6b = R_world_i * (p6 + tvec_world_i);
		

		Point3f p1c = { float(p1b.at<double>(0, 0)), -float(p1b.at<double>(1, 0)), -float(p1b.at<double>(2, 0)) };
		Point3f p2c = { float(p2b.at<double>(0, 0)), -float(p2b.at<double>(1, 0)), -float(p2b.at<double>(2, 0)) };
		Point3f p3c = { float(p3b.at<double>(0, 0)), -float(p3b.at<double>(1, 0)), -float(p3b.at<double>(2, 0)) };
		Point3f p4c = { float(p4b.at<double>(0, 0)), -float(p4b.at<double>(1, 0)), -float(p4b.at<double>(2, 0)) };
		Point3f p5c = { float(p5b.at<double>(0, 0)), -float(p5b.at<double>(1, 0)), -float(p5b.at<double>(2, 0)) };
		Point3f p6c = { float(p6b.at<double>(0, 0)), -float(p6b.at<double>(1, 0)), -float(p6b.at<double>(2, 0)) };

		Point3f b = { BallPositionInCameraSpace.x , -BallPositionInCameraSpace.y , -BallPositionInCameraSpace.z };

		vector <Point3f> pocket_points_camera_view_space;
		pocket_points_camera_view_space.push_back(p1c);
		pocket_points_camera_view_space.push_back(p2c);
		pocket_points_camera_view_space.push_back(p3c);
		pocket_points_camera_view_space.push_back(p4c);
		pocket_points_camera_view_space.push_back(p5c);
		pocket_points_camera_view_space.push_back(p6c);

		pocket_points_camera_view_space.push_back(b);

		vector<Point2f> pocket_points_frame;

		Mat tvec_0(3, 1, CV_64F, double(0)); // translation vector extrisics camera 
		Mat R_0(3, 3, CV_64F); //rotation vector extrisics camera 
		cv::setIdentity(R_0);
		projectPoints(pocket_points_camera_view_space, R_0, tvec_0, cameraMatrix, distCoeffs, pocket_points_frame);
		//circle(frame, pocket_points_frame[0], 50, Scalar(255, 255, 255), 2);
		//circle(frame, pocket_points_frame[1], 50, Scalar(255, 255, 255), 2);
		//circle(frame, pocket_points_frame[2], 50, Scalar(255, 255, 255), 2);
		//circle(frame, pocket_points_frame[3], 50, Scalar(255, 255, 255), 2);
		//circle(frame, pocket_points_frame[4], 50, Scalar(255, 255, 255), 2);
		//circle(frame, pocket_points_frame[5], 50, Scalar(255, 255, 255), 2);

		circle(frame, pocket_points_frame[6], 20, Scalar(255, 255, 255), 5);
		
	
		if (ball_found)
		{
			//draw circle
			circle(frame, center[bestball], (int)radius[bestball], color_ball, 2, 8, 0);
			// draw trajectory lines
			//line(frame, pocket_points_frame[0], center[bestball], Scalar(0, 0, 0), 3);
			//line(frame, pocket_points_frame[1], center[bestball], Scalar(0, 0, 0), 3);
			//line(frame, pocket_points_frame[2], center[bestball], Scalar(0, 0, 0), 3);
			//line(frame, pocket_points_frame[3], center[bestball], Scalar(0, 0, 0), 3);
			//line(frame, pocket_points_frame[4], center[bestball], Scalar(0, 0, 0), 3);
			//line(frame, pocket_points_frame[5], center[bestball], Scalar(0, 0, 0), 3);
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

		bool m_transform_ready = true;

	
		for (auto& r : _slateRendererList)
		{
			r->Update(
				stepTimer);
		}


		if (!_holoLensMediaFrameSourceGroupStarted)
		{
			return;
		}

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
		Windows::Perception::Spatial::SpatialCoordinateSystem^ CameraCoordinateSystem = latestFrame->CameraCoordinateSystem;
		Windows::Foundation::Numerics::float4x4 CameraViewTransform = latestFrame->CameraViewTransform;
		Windows::Foundation::Numerics::float4x4 CameraProjectionTransform = latestFrame->CameraProjectionTransform;

		Windows::Media::Capture::Frames::DepthMediaFrame^ DepthFrame = latestFrame->DepthFrame;
	

	
		Windows::Foundation::Numerics::float4x4 CameraToWorld;

		float3 CameraPositionWorldSpace = float3{ -CameraToWorld.m41, -CameraToWorld.m42, -CameraToWorld.m43 };
		dbg::trace(L"position");
		dbg::trace(L"%f %f %f", CameraPositionWorldSpace.x, CameraPositionWorldSpace.y, CameraPositionWorldSpace.z);

		HolographicFramePrediction^ prediction = holographicFrame->CurrentPrediction;
		if(_isActiveRenderer)
			AttachedCoordinateSystem = m_reference_attached_Frame->GetStationaryCoordinateSystemAtTimestamp(prediction->Timestamp);
		
		// Back buffers can change from frame to frame. Validate each buffer, and recreate
		// resource views and depth buffers as needed.
		//m_deviceResources->EnsureCameraResources(holographicFrame, prediction);

		Windows::Perception::PerceptionTimestamp^ Timestamp = prediction->Timestamp;

		SpatialLocation^ sp = m_locator->TryLocateAtTimestamp(Timestamp,m_WorldCoordinateSystem);
		
		float3 position2 = sp->Position;
		quaternion orientation = sp->Orientation;

		dbg::trace(L"position2");
		dbg::trace(L"%f %f %f", position2.x, position2.y, position2.z);



		/*
		const auto tryTransformm2 = CameraCoordinateSystem->TryGetTransformTo(AttachedCoordinateSystem);
		if (tryTransformm2 != nullptr)
		{
			Windows::Foundation::Numerics::float4x4 CamToAttached = tryTransformm2->Value;

			dbg::trace(L"CamToAttached");
			dbg::trace(L"%f %f %f", CamToAttached.m11, CamToAttached.m12, CamToAttached.m13);
			dbg::trace(L"%f %f %f", CamToAttached.m21, CamToAttached.m22, CamToAttached.m23);
			dbg::trace(L"%f %f %f", CamToAttached.m31, CamToAttached.m32, CamToAttached.m33);
			dbg::trace(L"%f %f %f", CamToAttached.m41, CamToAttached.m42, CamToAttached.m43);

		}
		*/
		
		const auto tryTransform = CameraCoordinateSystem->TryGetTransformTo(m_WorldCoordinateSystem);
		CameraToWorld = tryTransform->Value;
		if (tryTransform != nullptr)
		{
			CameraToWorld = tryTransform->Value;
		}
		else
		{


			SpatialPointerPose^ pose = SpatialPointerPose::TryGetAtTimestamp(m_WorldCoordinateSystem, prediction->Timestamp);

			if (pose != nullptr)
			{
				float3 headPosition = pose->Head->Position;
				float3 headUp = pose->Head->UpDirection;
				float3 headDirection = pose->Head->ForwardDirection;

				// To construct a rotation matrix, we need three vectors that are mutually orthogonal.
				// The first vector is the gaze vector.
				float3 negativeZAxis = normalize(headDirection);

				// The second vector should end up pointing away from the horizontal plane of the device.
				// We first guess by using the head "up" direction.
				float3 positiveYAxisGuess = normalize(headUp);

				// The third vector completes the set by being orthogonal to the other two.
				float3 positiveXAxis = normalize(cross(negativeZAxis, positiveYAxisGuess));

				// Now, we can correct our "up" vector guess by redetermining orthogonality.
				float3 positiveYAxis = normalize(cross(negativeZAxis, positiveXAxis));

				// The rotation matrix is formed as a standard basis rotation.
				float4x4 rotationTransform =
				{
					positiveXAxis.x, positiveYAxis.x, negativeZAxis.x, 0.f,
					positiveXAxis.y, positiveYAxis.y, negativeZAxis.y, 0.f,
					positiveXAxis.z, positiveYAxis.z, negativeZAxis.z, 0.f,
					0.f, 0.f, 0.f, 1.f,
				};

				// The translate transform can be constructed using the Windows::Foundation::Numerics API.
				float4x4 translationTransform = make_float4x4_translation(-headPosition);

				// Now, we have a basis transform from our spatial coordinate system to a device - relative
				// coordinate system.
				CameraToWorld = translationTransform * rotationTransform;
			}
			else
			{
				m_transform_ready = false;
			}

		}

		
		//SpatialPointerPose^ pose = SpatialPointerPose::TryGetAtTimestamp(m_WorldCoordinateSystem, prediction->Timestamp);
		//float3 headPosition = pose->Head->Position;
		//dbg::trace(L" headPosition");
		//dbg::trace(L"%f %f %f", headPosition.x, headPosition.y, headPosition);

		/*
		SpatialPointerPose^ pose = SpatialPointerPose::TryGetAtTimestamp(m_WorldCoordinateSystem, prediction->Timestamp);

		float3 headPosition = pose->Head->Position;
		float3 headUp = pose->Head->UpDirection;
		float3 headDirection = pose->Head->ForwardDirection;

		// To construct a rotation matrix, we need three vectors that are mutually orthogonal.
		// The first vector is the gaze vector.
		float3 negativeZAxis = normalize(headDirection);

		// The second vector should end up pointing away from the horizontal plane of the device.
		// We first guess by using the head "up" direction.
		float3 positiveYAxisGuess = normalize(headUp);

		// The third vector completes the set by being orthogonal to the other two.
		float3 positiveXAxis = normalize(cross(negativeZAxis, positiveYAxisGuess));

		// Now, we can correct our "up" vector guess by redetermining orthogonality.
		float3 positiveYAxis = normalize(cross(negativeZAxis, positiveXAxis));

		// The rotation matrix is formed as a standard basis rotation.
		float4x4 rotationTransform =
		{
			positiveXAxis.x, positiveYAxis.x, negativeZAxis.x, 0.f,
			positiveXAxis.y, positiveYAxis.y, negativeZAxis.y, 0.f,
			positiveXAxis.z, positiveYAxis.z, negativeZAxis.z, 0.f,
			0.f, 0.f, 0.f, 1.f,
		};

		// The translate transform can be constructed using the Windows::Foundation::Numerics API.
		float4x4 translationTransform = make_float4x4_translation(-headPosition);

		// Now, we have a basis transform from our spatial coordinate system to a device - relative
		// coordinate system.
		CameraToWorld = translationTransform * rotationTransform;
		*/

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

		Mat tvec_cam(3, 1, cv::DataType<double>::type); // translation vector extrisics camera 
		Mat R_cam(3, 3, cv::DataType<double>::type); //rotation vector extrisics camera 

		tvec_cam.at<double>(0, 0) = CameraViewTransform.m41;
		tvec_cam.at<double>(1, 0) = CameraViewTransform.m42;
		tvec_cam.at<double>(2, 0) = CameraViewTransform.m43;

		R_cam.at<double>(0, 0) = CameraViewTransform.m11;
		R_cam.at<double>(1, 0) = CameraViewTransform.m12;
		R_cam.at<double>(2, 0) = CameraViewTransform.m13;
		R_cam.at<double>(0, 1) = CameraViewTransform.m21;
		R_cam.at<double>(1, 1) = CameraViewTransform.m22;
		R_cam.at<double>(2, 1) = CameraViewTransform.m23;
		R_cam.at<double>(0, 2) = CameraViewTransform.m31;
		R_cam.at<double>(1, 2) = CameraViewTransform.m32;
		R_cam.at<double>(2, 2) = CameraViewTransform.m33;

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

		Mat tvec_world_i(3, 1, cv::DataType<double>::type);
		Mat R_world_i(3, 3, cv::DataType<double>::type);

		cv::transpose(R_world, R_world_i);
		tvec_world_i.at<double>(0, 0) = -tvec_world.at<double>(0, 0);
		tvec_world_i.at<double>(1, 0) = -tvec_world.at<double>(1, 0);
		tvec_world_i.at<double>(2, 0) = -tvec_world.at<double>(2, 0);

		cv::Mat frame;
		

		rmcv::WrapHoloLensSensorFrameWithCvMat(
			latestFrame, frame);

		
		if (m_transform_ready && _isActiveRenderer)
		{
			if (_isPoolDetected == false)
			{
				DetectPoolTable(frame, CameraCoordinateSystem, cameraIntrinsics, tvec_cam, R_cam, CameraToWorld);
			}
			else 
			{ 
				ProcessBalls(frame, cameraIntrinsics, CameraCoordinateSystem, tvec_world_i, R_world_i, CameraViewTransform, CameraToWorld);
				if (ball_found)
				{ 
					/*
					const auto tryTransformm = m_WorldCoordinateSystem->TryGetTransformTo(AttachedCoordinateSystem);
					if (tryTransformm != nullptr)
					{   
						

						
						Windows::Foundation::Numerics::float4x4 WorldToAttached = tryTransformm->Value;
						Mat tvec_attached(3, 1, cv::DataType<double>::type);
						Mat R_attached(3, 3, cv::DataType<double>::type);

						tvec_attached.at<double>(0, 0) = WorldToAttached.m41;
						tvec_attached.at<double>(1, 0) = WorldToAttached.m42;
						tvec_attached.at<double>(2, 0) = WorldToAttached.m43;

						R_attached.at<double>(0, 0) = WorldToAttached.m11;
						R_attached.at<double>(1, 0) = WorldToAttached.m12;
						R_attached.at<double>(2, 0) = WorldToAttached.m13;
						R_attached.at<double>(0, 1) = WorldToAttached.m21;
						R_attached.at<double>(1, 1) = WorldToAttached.m22;
						R_attached.at<double>(2, 1) = WorldToAttached.m23;
						R_attached.at<double>(0, 2) = WorldToAttached.m31;
						R_attached.at<double>(1, 2) = WorldToAttached.m32;
						R_attached.at<double>(2, 2) = WorldToAttached.m33;

						Mat Ball_world_space(3, 1, cv::DataType<double>::type);
						Ball_world_space.at<double>(0, 0) = BallPositionInWorldSpace.x;
						Ball_world_space.at<double>(1, 0) = BallPositionInWorldSpace.y;
						Ball_world_space.at<double>(2, 0) = BallPositionInWorldSpace.z;

						//Mat Ball_attached_space = R_attached * Ball_world_space + tvec_attached;

						float3 BallPositionAttachedCoordSystem = float3{ float(Ball_attached_space.at<double>(0, 0)), float(Ball_attached_space.at<double>(1, 0)), float(Ball_attached_space.at<double>(2, 0)) };
						dbg::trace(L"BallPositionAttachedCoordSystem");
						dbg::trace(L"%f %f %f", BallPositionAttachedCoordSystem.x, BallPositionAttachedCoordSystem.y, BallPositionAttachedCoordSystem.z);
						

						float x = - (DistanceRenderedFrameFromUser * BallPositionAttachedCoordSystem.x / BallPositionAttachedCoordSystem.z)*cameraIntrinsics->FocalLength.x + frame.cols/2.f ;
						float y = + (DistanceRenderedFrameFromUser * BallPositionAttachedCoordSystem.y / BallPositionAttachedCoordSystem.z)*cameraIntrinsics->FocalLength.y + frame.rows/2.f ;
						circle(frame, Point2f{x,y}, 30, Scalar(255, 255, 255),2);
						

						dbg::trace(L"BallPosition");
						dbg::trace(L"%f %f ", x, y);

					}
					*/

					frame = 0.f*frame;
					float3 BallPositionRenderedFrame3D;
					IntersectionLinePlane(CameraPositionWorldSpace, BallPositionInWorldSpace, plane_rendered_frame, BallPositionRenderedFrame3D);
					float x = dot(BallPositionRenderedFrame3D - center_plane_rendered_frame,X_rendered_frame);
					float y = dot(BallPositionRenderedFrame3D - center_plane_rendered_frame,Y_rendered_frame);
					float x_im = frame.cols / 2.0f + x*cameraIntrinsics->FocalLength.x;
					float y_im = frame.rows / 2.0f + y*cameraIntrinsics->FocalLength.y;
					

					float l = Norm(BallPositionRenderedFrame3D-CameraPositionWorldSpace);
					float L = Norm(BallPositionInWorldSpace-CameraPositionWorldSpace);
					float r = l*ball_real_diameter/(2.f*L);
					
					circle(frame, Point2f{ x_im,y_im}, r , Scalar(255, 255, 255), 2);

					dbg::trace(L"BallPosition");
					dbg::trace(L"%f %f ", x_im, y_im);
				}
			}
		}
		
		
		// Quantize the hue to 30 levels
		// and the saturation to 32 levels
		//int histSize[] = { hbins };
		// hue varies from 0 to 179, see cvtColor
		//int hbins = 30;
		//float hranges[] = { 0, 180 };
		//const float* ranges[] = { hranges };
		//MatND hist;
		// we compute the histogram from the 0-th and 1-st channels
		//int channel[] = { 0 };

		//calcHist(&frame, 1, channel, Mat(), // do not use mask
		//	hist, 2, histSize, ranges,
		//	true, // the histogram is uniform
		//	false);

		//double maxVal = 0;
		//minMaxLoc(hist, 0, &maxVal, 0, 0);
	

		
		

		if (!_undistortMapsInitialized)
		{
		//Computes the undistortion and rectification transformation map

		cv::initUndistortRectifyMap(
		cameraMatrix,
		distCoeffs,
		cv::Mat_<double>::eye(3, 3),
		cameraMatrix,
		cv::Size(frame.cols, frame.rows),
		CV_32FC1,
		_undistortMap1,
		_undistortMap2);

		_undistortMapsInitialized = true;
		}



		if (_undistortMapsInitialized)
		{
			//if undistortion initialize, geometric transformation to undistort thanks to undistortmaps
			cv::remap(
				frame,
				frame,
				_undistortMap1,
				_undistortMap2,
				cv::INTER_LINEAR);
		}

		/*
		//resize the image to Size(round(fx*src.cols), round(fy*src.rows))
		cv::resize(
		frame,
		frame,
		cv::Size(),
		0.1,
		0.1,
		cv::INTER_AREA);
		}
		else
		{
		cv::resize(
		frame,
		frame,
		cv::Size(),
		0.1,
		0.1,
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
			//m_markerRenderer->Render();
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
