#pragma once

#include "Camera.h"
#include "CommandContext.h"
#include "GpuBuffer.h"
#include "PipelineState.h"
#include "PrimitiveTypes.h"
#include "RootSignature.h"
#include "VectorMath.h"

#include <cstdint>

class PrimitiveRenderer
{
public:
    PrimitiveRenderer();

    void Initialize();
    void Shutdown();

    void Render(
        GraphicsContext& context,
        const Math::Camera& camera,
        const RenderItem& renderItem);

private:
    void PrepareDraw(
        GraphicsContext& context,
        const Math::Camera& camera,
        const Math::Matrix4& worldMatrix,
        const Math::Vector4& color);

    void RenderCube(
        GraphicsContext& context,
        const Math::Camera& camera,
        const Math::Matrix4& worldMatrix,
        const Math::Vector4& color);

    void RenderCylinder(
        GraphicsContext& context,
        const Math::Camera& camera,
        const Math::Matrix4& worldMatrix,
        const Math::Vector4& color);

    void RenderSphere(
        GraphicsContext& context,
        const Math::Camera& camera,
        const Math::Matrix4& worldMatrix,
        const Math::Vector4& color);

private:
    RootSignature m_RootSignature;
    GraphicsPSO m_PipelineState;

    ByteAddressBuffer m_CylinderVertexBuffer;
    ByteAddressBuffer m_CylinderIndexBuffer;

    std::uint32_t m_CylinderIndexCount = 0;

    ByteAddressBuffer m_SphereVertexBuffer;
    ByteAddressBuffer m_SphereIndexBuffer;

    std::uint32_t m_SphereIndexCount = 0;

};