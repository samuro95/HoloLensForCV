//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#pragma once

#include "Common\StepTimer.h"
#include "Common\DeviceResources.h"

namespace ComputeOnDevice
{
	class AppMain : public Holographic::AppMainBase
	{
	public:
		AppMain(
			const std::shared_ptr<Graphics::DeviceResources>& deviceResources);

		// IDeviceNotify
		virtual void OnDeviceLost() override;

		virtual void OnDeviceRestored() override;

		// IAppMain
		virtual void OnHolographicSpaceChanged(
			_In_ Windows::Graphics::Holographic::HolographicSpace^ holographicSpace) override;


		virtual void OnSpatialInput(
			_In_ Windows::UI::Input::Spatial::SpatialInteractionSourceState^ pointerState) override;

		virtual void OnUpdate(
			_In_ Windows::Graphics::Holographic::HolographicFrame^ holographicFrame,
			_In_ const Graphics::StepTimer& stepTimer) override;

		// Renders holograms, including world-locked content.
		//bool Render(Windows::Graphics::Holographic::HolographicFrame^ holographicFrame);

		virtual void OnPreRender() override;

		virtual void OnRender() override;


	private:

		// Asynchronously creates resources for new holographic cameras.
		void OnCameraAdded(
			Windows::Graphics::Holographic::HolographicSpace^ sender,
			Windows::Graphics::Holographic::HolographicSpaceCameraAddedEventArgs^ args);

		// Synchronously releases resources for holographic cameras that are no longer
		// attached to the system.
		void OnCameraRemoved(
			Windows::Graphics::Holographic::HolographicSpace^ sender,
			Windows::Graphics::Holographic::HolographicSpaceCameraRemovedEventArgs^ args);

		// Used to notify the app when the positional tracking state changes.
		void OnLocatabilityChanged(
			Windows::Perception::Spatial::SpatialLocator^ sender,
			Platform::Object^ args);

		// Event registration tokens.
		Windows::Foundation::EventRegistrationToken                     m_cameraAddedToken;
		Windows::Foundation::EventRegistrationToken                     m_cameraRemovedToken;
		Windows::Foundation::EventRegistrationToken                     m_locatabilityChangedToken;

		// Initializes access to HoloLens sensors.
		void StartHoloLensMediaFrameSourceGroup();

		std::vector<std::shared_ptr<Rendering::SlateRenderer> >_slateRendererList;
		std::shared_ptr<Rendering::SlateRenderer> _currentSlateRenderer;
		std::shared_ptr<Rendering::MarkerRenderer>  m_markerRenderer;

		// Selected HoloLens media frame source group
		HoloLensForCV::MediaFrameSourceGroupType _selectedHoloLensMediaFrameSourceGroupType;
		HoloLensForCV::MediaFrameSourceGroup^ _holoLensMediaFrameSourceGroup;
		bool _holoLensMediaFrameSourceGroupStarted;

		// HoloLens media frame server manager
		HoloLensForCV::SensorFrameStreamer^ _sensorFrameStreamer;

		Windows::Foundation::DateTime _latestSelectedCameraTimestamp;

		cv::Mat _undistortMap1;
		cv::Mat _undistortMap2;
		bool _undistortMapsInitialized;

		cv::Mat _undistortedPVCameraImage;
		cv::Mat _resizedPVCameraImage;
		cv::Mat _blurredPVCameraImage;
		cv::Mat _cannyPVCameraImage;

		std::vector<Rendering::Texture2DPtr> _visualizationTextureList;
		Rendering::Texture2DPtr _currentVisualizationTexture;

		bool _isActiveRenderer;
		bool _isPoolDetected;

		// Represents the holographic space around the user.
		Windows::Graphics::Holographic::HolographicSpace^               m_holographicSpace;


		// SpatialLocator that is attached to the primary camera.
		Windows::Perception::Spatial::SpatialLocator^                   m_locator;
		// A reference frame attached to the holographic camera.
		
		//Windows::Perception::Spatial::SpatialLocatorAttachedFrameOfReference^ m_referenceFrame;
		Windows::Perception::Spatial::SpatialStationaryFrameOfReference^ m_referenceFrame;

		Windows::Perception::Spatial::SpatialLocatorAttachedFrameOfReference^ m_reference_attached_Frame;

		Windows::Perception::Spatial::SpatialCoordinateSystem^ m_WorldCoordinateSystem;

        void DetectPoolTable(cv::Mat frame, Windows::Perception::Spatial::SpatialCoordinateSystem^ CameraCoordinateSystem, Windows::Media::Devices::Core::CameraIntrinsics^ cameraIntrinsics, cv::Mat tvec_cam, cv::Mat R_cam, Windows::Foundation::Numerics::float4x4 CameraToWorld);

		void ProcessBalls(cv::Mat frame, Windows::Media::Devices::Core::CameraIntrinsics^ cameraIntrinsics, Windows::Perception::Spatial::SpatialCoordinateSystem^ CameraCoordinateSystem, cv::Mat tvec_world_i, cv::Mat R_world_i, Windows::Foundation::Numerics::float4x4 CameraViewTransform, Windows::Foundation::Numerics::float4x4 CameraToWorld);

		void sign(_In_ float x, _Out_ int res);

		// Cached pointer to device resources.
		std::shared_ptr<DX::DeviceResources>                            m_deviceResources;

		// Indicates whether all resources necessary for rendering are ready.
		bool m_isReadyToRender = false;

		Microsoft::WRL::ComPtr<ID3D11PixelShader>						 m_pixelShaderRGB;

		Windows::Perception::Spatial::SpatialCoordinateSystem^ anchorSpace;

		Windows::Foundation::Numerics::float4x4 WorldCoordinateSystemToAnchorSpace;

		Windows::Foundation::Numerics::float4x4 AnchorSpaceToWorldCoordinateSystem;

		Windows::Perception::Spatial::SpatialAnchor^ m_table_anchor;

		Windows::Foundation::Numerics::float3 Chess_position_world_space;

		void AppMain::m_transform(double x_in, double y_in, double z_in, Windows::Foundation::Numerics::float4x4 Transform, double x_out, double y_out, double z_out);

		bool m_WorldCoordinateSystem_Set;

		Graphics::StepTimer                                                  m_timer;

		std::vector<Windows::Foundation::Numerics::float3> pocket_world_space;
		Windows::Foundation::Numerics::float3 BallPositionInWorldSpace;
	
		bool ball_found;

		//distance from the user of the rendered frame in meters
		float DistanceRenderedFrameFromUser = 2.0f;

		Windows::Perception::Spatial::SpatialCoordinateSystem^ AttachedCoordinateSystem;

		void AppMain::ConstructPlaneFromPointNormal(_In_ Windows::Foundation::Numerics::float3 Point, Windows::Foundation::Numerics::float3 normal, _Out_ Windows::Foundation::Numerics::float4 plane);
		
		void AppMain::IntersectionLinePlane(_In_ Windows::Foundation::Numerics::float3 p1, Windows::Foundation::Numerics::float3 p2, Windows::Foundation::Numerics::float4 plane, _Out_ Windows::Foundation::Numerics::float3 inters);
	
		Windows::Foundation::Numerics::float4 plane_rendered_frame;

		Windows::Foundation::Numerics::float3 center_plane_rendered_frame;

		Windows::Foundation::Numerics::float3 X_rendered_frame;
		Windows::Foundation::Numerics::float3 Y_rendered_frame;

		float ball_real_diameter = 0.45f;

	};
}
	
