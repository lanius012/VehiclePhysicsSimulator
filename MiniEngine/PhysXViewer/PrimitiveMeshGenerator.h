#pragma once

#include "PrimitiveTypes.h"

#include <cstdint>
#include <vector>

struct PrimitiveMeshData
{
    std::vector<PrimitiveVertex> vertices;
    std::vector<std::uint16_t> indices;
};

class PrimitiveMeshGenerator
{
public:
    //
    // X축 방향의 단위 원기둥을 생성한다.
    //
    // 중심: (0, 0, 0)
    // X축 범위: -0.5 ~ +0.5
    // 전체 길이: 1
    // 반지름: 1
    //
    static PrimitiveMeshData CreateCylinder(
        std::uint32_t segmentCount);

    static PrimitiveMeshData CreateSphere(
        std::uint32_t sliceCount,
        std::uint32_t stackCount);
};