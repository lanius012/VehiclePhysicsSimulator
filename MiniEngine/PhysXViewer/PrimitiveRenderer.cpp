#include "pch.h"
#include "PrimitiveRenderer.h"

#include "BufferManager.h"
#include "Cube.h"
#include "GraphicsCommon.h"

#include "CompiledShaders/PrimitiveVS.h"
#include "CompiledShaders/PrimitivePS.h"

#include "PrimitiveMeshGenerator.h"
#include "PrimitiveTypes.h"

#include <cstddef>
#include <cstdint>

using namespace Math;
using namespace Graphics;

using CoreCubeVertex =
Graphics::Shapes::Cube::CubeVertex;

static_assert(
    sizeof(PrimitiveVertex) ==
    sizeof(CoreCubeVertex),
    "PrimitiveVertex and CubeVertex size mismatch.");

static_assert(
    offsetof(PrimitiveVertex, position) ==
    offsetof(CoreCubeVertex, position),
    "PrimitiveVertex position offset mismatch.");

static_assert(
    offsetof(PrimitiveVertex, normal) ==
    offsetof(CoreCubeVertex, normal),
    "PrimitiveVertex normal offset mismatch.");

namespace
{
    enum RootParameterIndex
    {
        kVertexConstants = 0,
        kPixelConstants,
        kRootParameterCount
    };

    // SetDynamicConstantBufferView()에 전달되는 주소는
    // 16바이트 정렬이어야 한다.
    struct alignas(16) VertexConstants
    {
        Matrix4 World;
        Matrix4 ViewProjection;
    };

    struct alignas(16) PixelConstants
    {
        Vector4 BaseColor;
    };
}

PrimitiveRenderer::PrimitiveRenderer()
    : m_PipelineState(L"Primitive Renderer PSO")
{
}

void PrimitiveRenderer::Initialize()
{
    //
    // 1. Root Signature
    //

    m_RootSignature.Reset(kRootParameterCount);

    // Vertex Shader의 b0
    m_RootSignature[kVertexConstants]
        .InitAsConstantBuffer(
            0,
            D3D12_SHADER_VISIBILITY_VERTEX);

    // Pixel Shader의 b0
    //
    // 같은 b0이지만 shader visibility가 서로 다르므로
    // 별개의 root parameter로 둘 수 있다.
    m_RootSignature[kPixelConstants]
        .InitAsConstantBuffer(
            0,
            D3D12_SHADER_VISIBILITY_PIXEL);


    m_RootSignature.Finalize(
        L"Primitive Renderer Root Signature",
        D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

    //
    // 2. Vertex Input Layout
    //

    D3D12_INPUT_ELEMENT_DESC inputLayout[] =
    {
        {
            "POSITION",
            0,
            DXGI_FORMAT_R32G32B32_FLOAT,
            0,
            static_cast<UINT>(
                offsetof(
                    PrimitiveVertex,
                    position)),
            D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA,
            0
        },
        {
            "NORMAL",
            0,
            DXGI_FORMAT_R32G32B32_FLOAT,
            0,
            static_cast<UINT>(
                offsetof(
                    PrimitiveVertex,
                    normal)),
            D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA,
            0
        }
    };

    //
    // 3. Graphics Pipeline State Object
    //

    const DXGI_FORMAT colorFormat =
        g_SceneColorBuffer.GetFormat();

    const DXGI_FORMAT depthFormat =
        g_SceneDepthBuffer.GetFormat();

    m_PipelineState.SetRootSignature(
        m_RootSignature);

    m_PipelineState.SetRasterizerState(
        RasterizerDefault);

    m_PipelineState.SetBlendState(
        BlendDisable);

    m_PipelineState.SetDepthStencilState(
        DepthStateReadWrite);

    m_PipelineState.SetInputLayout(
        _countof(inputLayout),
        inputLayout);

    m_PipelineState.SetPrimitiveTopologyType(
        D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE);

    m_PipelineState.SetRenderTargetFormats(
        1,
        &colorFormat,
        depthFormat);

    m_PipelineState.SetVertexShader(
        g_pPrimitiveVS,
        sizeof(g_pPrimitiveVS));

    m_PipelineState.SetPixelShader(
        g_pPrimitivePS,
        sizeof(g_pPrimitivePS));

    m_PipelineState.Finalize();

    


    //
// Core의 단위 Cube 초기화
//

    Graphics::Shapes::Cube::InitializeCubeBuffers();

    //
    // X축 단위 Cylinder 생성
    //

    constexpr std::uint32_t cylinderSegmentCount =
        32;

    const PrimitiveMeshData cylinderMesh =
        PrimitiveMeshGenerator::CreateCylinder(
            cylinderSegmentCount);

    if (cylinderMesh.vertices.empty() ||
        cylinderMesh.indices.empty())
    {
        throw std::runtime_error(
            "Failed to create cylinder mesh.");
    }

    m_CylinderVertexBuffer.Create(
        L"Primitive Cylinder Vertices",
        static_cast<std::uint32_t>(
            cylinderMesh.vertices.size()),
        sizeof(PrimitiveVertex),
        cylinderMesh.vertices.data());

    m_CylinderIndexBuffer.Create(
        L"Primitive Cylinder Indices",
        static_cast<std::uint32_t>(
            cylinderMesh.indices.size()),
        sizeof(std::uint16_t),
        cylinderMesh.indices.data());

    m_CylinderIndexCount =
        static_cast<std::uint32_t>(
            cylinderMesh.indices.size());

    //
// 단위 Sphere 생성
//

    constexpr std::uint32_t sphereSliceCount =
        24;

    constexpr std::uint32_t sphereStackCount =
        16;

    const PrimitiveMeshData sphereMesh =
        PrimitiveMeshGenerator::CreateSphere(
            sphereSliceCount,
            sphereStackCount);

    if (sphereMesh.vertices.empty() ||
        sphereMesh.indices.empty())
    {
        throw std::runtime_error(
            "Failed to create sphere mesh.");
    }

    m_SphereVertexBuffer.Create(
        L"Primitive Sphere Vertices",
        static_cast<std::uint32_t>(
            sphereMesh.vertices.size()),
        sizeof(PrimitiveVertex),
        sphereMesh.vertices.data());

    m_SphereIndexBuffer.Create(
        L"Primitive Sphere Indices",
        static_cast<std::uint32_t>(
            sphereMesh.indices.size()),
        sizeof(std::uint16_t),
        sphereMesh.indices.data());

    m_SphereIndexCount =
        static_cast<std::uint32_t>(
            sphereMesh.indices.size());

    
}

void PrimitiveRenderer::Shutdown()
{
    m_SphereIndexBuffer.Destroy();
    m_SphereVertexBuffer.Destroy();

    m_SphereIndexCount = 0;

    m_CylinderIndexBuffer.Destroy();
    m_CylinderVertexBuffer.Destroy();

    m_CylinderIndexCount = 0;

    Graphics::Shapes::Cube::DestroyCubeBuffers();

}

void PrimitiveRenderer::Render(
    GraphicsContext& context,
    const Camera& camera,
    const RenderItem& renderItem)
{
    if (!renderItem.visible)
    {
        return;
    }

    switch (renderItem.type)
    {
    case PrimitiveType::Cube:
        RenderCube(
            context,
            camera,
            renderItem.world,
            renderItem.color);
        break;

    case PrimitiveType::Cylinder:
        RenderCylinder(
            context,
            camera,
            renderItem.world,
            renderItem.color);
        break;

    case PrimitiveType::Sphere:
        RenderSphere(
            context,
            camera,
            renderItem.world,
            renderItem.color);
        break;

    default:
        break;
    }
}

void PrimitiveRenderer::PrepareDraw(
    GraphicsContext& context,
    const Camera& camera,
    const Matrix4& worldMatrix,
    const Vector4& color)
{
    alignas(16) VertexConstants vertexConstants;

    vertexConstants.World =
        worldMatrix;

    vertexConstants.ViewProjection =
        camera.GetViewProjMatrix();

    alignas(16) PixelConstants pixelConstants;

    pixelConstants.BaseColor =
        color;

    context.SetRootSignature(
        m_RootSignature);

    context.SetPipelineState(
        m_PipelineState);

    context.SetPrimitiveTopology(
        D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    context.SetDynamicConstantBufferView(
        kVertexConstants,
        sizeof(vertexConstants),
        &vertexConstants);

    context.SetDynamicConstantBufferView(
        kPixelConstants,
        sizeof(pixelConstants),
        &pixelConstants);


}

void PrimitiveRenderer::RenderCube(
    GraphicsContext& context,
    const Camera& camera,
    const Matrix4& worldMatrix,
    const Vector4& color)
{
    PrepareDraw(
        context,
        camera,
        worldMatrix,
        color);

    context.SetVertexBuffer(
        0,
        Graphics::Shapes::Cube::g_CubeVerts
        .VertexBufferView());

    context.Draw(
        Graphics::Shapes::Cube::g_NumVerts);
}

void PrimitiveRenderer::RenderCylinder(
    GraphicsContext& context,
    const Camera& camera,
    const Matrix4& worldMatrix,
    const Vector4& color)
{
    if (m_CylinderIndexCount == 0)
    {
        return;
    }

    PrepareDraw(
        context,
        camera,
        worldMatrix,
        color);

    context.SetVertexBuffer(
        0,
        m_CylinderVertexBuffer
        .VertexBufferView());

    context.SetIndexBuffer(
        m_CylinderIndexBuffer
        .IndexBufferView());

    context.DrawIndexed(
        m_CylinderIndexCount);
}

void PrimitiveRenderer::RenderSphere(
    GraphicsContext& context,
    const Camera& camera,
    const Matrix4& worldMatrix,
    const Vector4& color)
{
    if (m_SphereIndexCount == 0)
    {
        return;
    }

    PrepareDraw(
        context,
        camera,
        worldMatrix,
        color);

    context.SetVertexBuffer(
        0,
        m_SphereVertexBuffer
        .VertexBufferView());

    context.SetIndexBuffer(
        m_SphereIndexBuffer
        .IndexBufferView());

    context.DrawIndexed(
        m_SphereIndexCount);
}