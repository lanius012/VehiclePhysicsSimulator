#pragma once

#include "VectorMath.h"

#include <DirectXMath.h>

//
// 모든 절차적 도형이 공통으로 사용하는 정점 형식이다.
//
// 현재 Core의 CubeVertex와 같은 메모리 배치를 사용한다.
// 따라서 기존 Core Cube와 새 Cylinder가 같은 PSO를 공유할 수 있다.
//
struct PrimitiveVertex
{
    DirectX::XMFLOAT3 position;
    DirectX::XMFLOAT2 texcoord;
    DirectX::XMFLOAT3 normal;
    DirectX::XMFLOAT3 tangent;
    DirectX::XMFLOAT3 bitangent;
};

enum class PrimitiveType
{
    Cube,
    Cylinder,
    Sphere
};

struct RenderItem
{
    PrimitiveType type;
    Math::Matrix4 world;
    Math::Vector4 color;
    bool visible;

    RenderItem(
        PrimitiveType primitiveType,
        const Math::Vector4& primitiveColor)
        : type(primitiveType)
        , world(Math::kIdentity)
        , color(primitiveColor)
        , visible(true)
    {
    }
};