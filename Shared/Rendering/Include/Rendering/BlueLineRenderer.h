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

namespace Rendering
{
    class BlueLineRenderer
    {
    public:
		BlueLineRenderer(
            _In_ const Graphics::DeviceResourcesPtr& deviceResources);

        void CreateDeviceDependentResources();

        void ReleaseDeviceDependentResources();

		void Update(Windows::Foundation::Numerics::float3 positionA , Windows::Foundation::Numerics::float3 positionB, const Graphics::StepTimer& timer);

        void Render();

        // Property accessors.
        void SetPositionA(
            Windows::Foundation::Numerics::float3 posA)
        {
            _positionA = posA;
        }

		void SetPositionB(
			Windows::Foundation::Numerics::float3 posB)
		{
			_positionB = posB;
		}

        Windows::Foundation::Numerics::float3 GetPositionA()
        {
            return _positionA;
        }

		Windows::Foundation::Numerics::float3 GetPositionB()
		{
			return _positionB;
		}

    private:
        // Cached pointer to device resources.
        Graphics::DeviceResourcesPtr _deviceResources;

        // The material we'll use to render this slate.
        std::unique_ptr<SlateMaterial> _slateMaterial;

        // Direct3D resources for the slate geometry.
        std::vector<VertexPositionColorTexture> _vertices;
        Microsoft::WRL::ComPtr<ID3D11Buffer> _vertexBuffer;
        Microsoft::WRL::ComPtr<ID3D11Buffer> _modelConstantBuffer;

        // System resources for the slate geometry.
        SlateModelConstantBuffer _modelConstantBufferData;

        // Variables used with the rendering loop.
        bool _loadingComplete = false;

        Windows::Foundation::Numerics::float3 _positionA = { 0.f, 0.f, 0.f };
		Windows::Foundation::Numerics::float3 _positionB = { 0.f, 0.f, 0.f };


    };
}
