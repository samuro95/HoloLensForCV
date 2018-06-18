
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
using namespace Eigen;


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


	bool _isPoolDetected = false;


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


	void AppMain::DetectPoolTable(_In_ Mat frame, Mat cameraMatrix, Mat distCoeffs, Windows::Foundation::Numerics::float4x4 FrameToOrigin, _Out_ SpatialCoordinateSystem^ anchorSpace)
	{

		//Use ChessBoardDetection to detect a corner and set a coordinate system linked with the plan of the pool table


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

			Mat rvec;
			Mat tvec;
			vector<Point3f> object_point = object_points[0];
			vector<Point2f> image_point = image_points[0];

			//calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
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

			// draw axis lines
			line(frame, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);
			line(frame, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 3);
			line(frame, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);

			//Convert rotation vector into rotation matrix 
			Mat R;
			Rodrigues(rvec, R);

			float3 Chess_position_camera_space = (tvec.at<float>(0,0), tvec.at<float>(1,0), tvec.at<float>(2,0));
			float3 Chess_position_world_space = transform(Chess_position_camera_space, FrameToOrigin);

			//create quaternion 

			float4x4 Rotation = float4x4(R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), float(0.) , R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), float(0.), R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), float(0.), float(0.), float(0.), float(0.), float(1.));
		
			orientation make_quaternion_from_rotation_matrix(Rotation);

			// Create the anchor at position.

			SpatialAnchor^ m_chess_anchor = SpatialAnchor::TryCreateRelativeTo(m_WorldCoordinateSystem,Chess_position_world_space,orientation);
			 if (anchor != nullptr)
			 {	
				float4x4 WorldCoordinateSystemToAnchorSpace;
				SpatialCoordinateSystem^ anchorSpace = m_table_anchor->CoordinateSystem;

				const auto tryTransform = m_WorldCoordinateSystem->TryGetTransformTo(anchorSpace);
				if (tryTransform != nullptr)
				{
					WorldCoordinateSystemToAnchorSpace = tryTransform->Value;
					_isPoolDetected = true;
				}

			 }

		}

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

		cv::Mat wrappedImage;

		// WrapHoloLensSensorFrameWithCvMat defined in OpenCVHelpers 
		// In : holoLensSensorFrame (1st arg)
		// Out : cvMat wrappedImage (2nd arg)

		rmcv::WrapHoloLensSensorFrameWithCvMat(
			latestFrame,
			wrappedImage);


		if (_isPoolDetected==false)
			{
			DetectPoolTable(wrappedImage, cameraMatrix, distCoeffs, FrameToOrigin, anchorSpace);
			}


		

		//DetectPoolTable(wrappedImage, cameraMatrix, distCoeffs, FrameToOrigin);


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

		Mat frame = _resizedPVCameraImage;

		/*

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
		int maxRadius = 60;
		int MinRadius = 8;
		Scalar color_ball = Scalar(255, 255, 255);
		Scalar color_table = Scalar(100, 10, 10);
		Mat drawing = Mat::zeros(hframe.size(), CV_8UC1);
		vector<vector<cv::Point>>hull(contours.size());
		double largest_area = 0;
		int largest_contour_index = 0;

		for (size_t i = 0; i < contours.size(); i++) {
		convexHull(Mat(contours[i]), hull[i]);
		approxPolyDP(hull[i], contours_poly[i], 3, true);
		minEnclosingCircle(contours_poly[i], center[i], radius[i]);
		double rad = radius[i];
		double area = contourArea(hull[i], false);
		if (area > largest_area) {
		largest_area = area;
		largest_contour_index = i;
		}
		if (rad<maxRadius && rad>MinRadius) {
		//drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0,Point() );
		//circle(drawing, center[i], (int)radius[i], color_ball, 2, 8, 0 );
		circle(frame, center[i], (int)radius[i], color_ball, 2, 8, 0);
		}
		}

		drawContours(frame, hull, largest_contour_index, color_table, 3, 8, vector<Vec4i>(), 0, cv::Point());
		//drawContours(drawing, hull, largest_contour_index, Scalar(255), 3, 8, vector<Vec4i>(), 0, cv::Point());




		/*


		//Blurs the image using the median filter with the ksize�ksize aperture
		cv::medianBlur(
		_resizedPVCameraImage,
		_blurredPVCameraImage,
		3);

		//Finds edges in an image using the Canny algorithm
		cv::Canny(
		_blurredPVCameraImage,
		_cannyPVCameraImage,
		50.0,
		200.0);

		//For each pxel of the edge map, if the pixel value is > 64, color the
		//corresponding pixel in blurredPVCameraImage


		for (int32_t y = 0; y < _blurredPVCameraImage.rows; ++y)
		{
		for (int32_t x = 0; x < _blurredPVCameraImage.cols; ++x)
		{
		if (_cannyPVCameraImage.at<uint8_t>(y, x) > 64)
		{
		//*(_blurredPVCameraImage.ptr<uint32_t>(y, x)) = 0xFFFF00FF;
		*(_blurredPVCameraImage.ptr<uint32_t>(y, x)) = 0x00FBFFFF;
		}
		}
		}



		*/
		/*


		//Linedetection

		//create LineFinder instance
		//LineFinder finder;

		//Set Hough parmeters
		//finder.setLineLenghAndGap(100, 20);
		//finder.setMinVote(80);

		//Detect lines and draw them
		//std::vector<cv::Vec4i> lines = finder.findLines(_cannyPVCameraImage);
		//finder.drawDetectedLines(_blurredPVCameraImage);

		//without using  class

		std::vector<cv::Vec4i> lines;
		cv::HoughLinesP(_cannyPVCameraImage, lines, 1, 3*3.14159 / 180, 70, 50, 80);

		std::vector<cv::Vec4i>::const_iterator it = lines.begin();
		cv::Scalar color = cv::Scalar(200, 30, 30);


		while (it!= lines.end()) {
		cv::Point pt1((*it)[0], (*it)[1]);
		cv::Point pt2((*it)[2], (*it)[3]);
		cv::line(_blurredPVCameraImage, pt1, pt2, color);
		++it;
		}

		imwrite("../../../result.jpg",_blurredPVCameraImage);


		*/

		//Update 2D texture to suit with blurredPVCameraImage

		OpenCVHelpers::CreateOrUpdateTexture2D(
			_deviceResources,
			_resizedPVCameraImage,
			_currentVisualizationTexture);

		//SpatialPointerPose^ pointerPose = SpatialPointerPose::TryGetAtTimestamp(currentCoordinateSystem, prediction->Timestamp);
	}



	/*

	// Renders the current frame to each holographic camera, according to the
	// current application and spatial positioning state. Returns true if the
	// frame was rendered to at least one camera.
	bool AppMain::Render(Windows::Graphics::Holographic::HolographicFrame^ holographicFrame)
	{
	if (!m_isReadyToRender)
	{
	return false;
	}

	SpatialCoordinateSystem^ currentCoordinateSystem = m_referenceFrame->GetStationaryCoordinateSystemAtTimestamp(holographicFrame->CurrentPrediction->Timestamp);

	// Lock the set of holographic camera resources, then draw to each camera
	// in this frame.
	return m_deviceResources->UseHolographicCameraResources<bool>(
	[this, holographicFrame, currentCoordinateSystem](std::map<UINT32, std::unique_ptr<DX::CameraResources>>& cameraResourceMap)
	{
	// Up-to-date frame predictions enhance the effectiveness of image stablization and
	// allow more accurate positioning of holograms.
	holographicFrame->UpdateCurrentPrediction();
	HolographicFramePrediction^ prediction = holographicFrame->CurrentPrediction;

	bool atLeastOneCameraRendered = false;
	for (auto cameraPose : prediction->CameraPoses)
	{
	// This represents the device-based resources for a HolographicCamera.
	DX::CameraResources* pCameraResources = cameraResourceMap[cameraPose->HolographicCamera->Id].get();

	// Get the device context.
	const auto context = m_deviceResources->GetD3DDeviceContext();
	const auto depthStencilView = pCameraResources->GetDepthStencilView();

	// Set render targets to the current holographic camera.
	ID3D11RenderTargetView *const targets[1] = { pCameraResources->GetBackBufferRenderTargetView() };
	context->OMSetRenderTargets(1, targets, depthStencilView);

	// Clear the back buffer and depth stencil view.
	context->ClearRenderTargetView(targets[0], DirectX::Colors::Transparent);
	context->ClearDepthStencilView(depthStencilView, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

	// The view and projection matrices for each holographic camera will change
	// every frame. This function refreshes the data in the constant buffer for
	// the holographic camera indicated by cameraPose.
	pCameraResources->UpdateViewProjectionBuffer(m_deviceResources, cameraPose, currentCoordinateSystem);

	// Attach the view/projection constant buffer for this camera to the graphics pipeline.
	bool cameraActive = pCameraResources->AttachViewProjectionBuffer(m_deviceResources);

	// Render world-locked content only when positional tracking is active.
	if (cameraActive)
	{
	_currentSlateRenderer->Render(_currentVisualizationTexture);
	}

	atLeastOneCameraRendered = true;
	}

	return atLeastOneCameraRendered;
	});

	}


	*/


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
