cbuffer PSConstants : register(b0)
{
    float4 BaseColor;
};

struct PSInput
{
    float4 Position : SV_POSITION;
    float3 Normal : NORMAL;
    float3 WorldPosition : TEXCOORD0;
    float3 LocalPosition : TEXCOORD1;
};


//
// =========================================================
// Grid
// =========================================================
//

float GetGridLine(
    float2 coordinate,
    float lineWidth)
{
    float2 fractionalPosition =
        frac(coordinate);

    float2 distanceToLine =
        min(
            fractionalPosition,
            1.0f - fractionalPosition);

    float minimumDistance =
        min(
            distanceToLine.x,
            distanceToLine.y);

    return
        1.0f -
        step(
            lineWidth,
            minimumDistance);
}


//
// =========================================================
// Center line dash
// =========================================================
//

float GetDashMask(
    float pathDistance)
{
    const float dashPeriod =
        5.0f;

    float dashPosition =
        frac(
            pathDistance /
            dashPeriod);

    return
        1.0f -
        step(
            0.55f,
            dashPosition);
}


//
// =========================================================
// Straight road
// =========================================================
//

void AccumulateStraightRoad(
    float2 position,
    float2 start,
    float2 end,
    float halfWidth,
    float pathOffset,
    inout float roadMask,
    inout float edgeMask,
    inout float centerLineMask)
{
    float2 segment =
        end -
        start;

    float segmentLengthSquared =
        dot(
            segment,
            segment);

    if (segmentLengthSquared <
        0.00001f)
    {
        return;
    }

    float segmentLength =
        sqrt(
            segmentLengthSquared);

    float2 direction =
        segment /
        segmentLength;

    float2 fromStart =
        position -
        start;

    float along =
        dot(
            fromStart,
            direction);

    float clampedAlong =
        clamp(
            along,
            0.0f,
            segmentLength);

    float2 closestPoint =
        start +
        direction *
        clampedAlong;

    float distanceToSegment =
        length(
            position -
            closestPoint);


    //
    // ---------------------------------------------------------
    // Road surface
    // ---------------------------------------------------------
    //

    float straightRoadMask =
        1.0f -
        smoothstep(
            halfWidth,
            halfWidth + 0.20f,
            distanceToSegment);

    roadMask =
        max(
            roadMask,
            straightRoadMask);


    //
    // ---------------------------------------------------------
    // Straight section only
    // ---------------------------------------------------------
    //

    float insideSegment =
        step(
            0.0f,
            along) *
        step(
            along,
            segmentLength);

    float perpendicularDistance =
        abs(
            fromStart.x *
            direction.y -
            fromStart.y *
            direction.x);


    //
    // ---------------------------------------------------------
    // Edge
    // ---------------------------------------------------------
    //

    float edgeDistance =
        abs(
            perpendicularDistance -
            (halfWidth - 0.25f));

    float straightEdgeMask =
        1.0f -
        smoothstep(
            0.03f,
            0.10f,
            edgeDistance);

    straightEdgeMask *=
        insideSegment;

    edgeMask =
        max(
            edgeMask,
            straightEdgeMask);


    //
    // ---------------------------------------------------------
    // Center dashed line
    // ---------------------------------------------------------
    //

    float straightCenterMask =
        1.0f -
        smoothstep(
            0.04f,
            0.09f,
            perpendicularDistance);

    float dashMask =
        GetDashMask(
            pathOffset +
            clampedAlong);

    straightCenterMask *=
        insideSegment *
        dashMask;

    centerLineMask =
        max(
            centerLineMask,
            straightCenterMask);
}


//
// =========================================================
// Quarter-circle road
// =========================================================
//
// quadrantX:
//   -1 = center 왼쪽
//   +1 = center 오른쪽
//
// quadrantZ:
//   -1 = center 아래
//   +1 = center 위
//
// startAngle은 도로 진행방향 기준 원호 시작각이다.
// 모든 코너는 clockwise 방향으로 진행한다.
//

void AccumulateQuarterCircleRoad(
    float2 position,
    float2 center,
    float radius,
    float halfWidth,
    float quadrantX,
    float quadrantZ,
    float startAngle,
    float pathOffset,
    inout float roadMask,
    inout float edgeMask,
    inout float centerLineMask)
{
    float2 offset =
        position -
        center;

    float distanceFromCenter =
        length(
            offset);

    //
    // 해당 1/4 원 영역인지 확인
    //

    float quadrantMask =
        step(
            0.0f,
            offset.x *
            quadrantX) *
        step(
            0.0f,
            offset.y *
            quadrantZ);


    //
    // ---------------------------------------------------------
    // Road surface
    // ---------------------------------------------------------
    //

    float distanceFromRoadCenter =
        abs(
            distanceFromCenter -
            radius);

    float arcRoadMask =
        1.0f -
        smoothstep(
            halfWidth,
            halfWidth + 0.20f,
            distanceFromRoadCenter);

    arcRoadMask *=
        quadrantMask;

    roadMask =
        max(
            roadMask,
            arcRoadMask);


    //
    // ---------------------------------------------------------
    // Edge
    // ---------------------------------------------------------
    //

    float innerEdgeRadius =
        radius -
        halfWidth +
        0.25f;

    float outerEdgeRadius =
        radius +
        halfWidth -
        0.25f;

    float innerEdge =
        1.0f -
        smoothstep(
            0.03f,
            0.10f,
            abs(
                distanceFromCenter -
                innerEdgeRadius));

    float outerEdge =
        1.0f -
        smoothstep(
            0.03f,
            0.10f,
            abs(
                distanceFromCenter -
                outerEdgeRadius));

    float arcEdgeMask =
        max(
            innerEdge,
            outerEdge);

    arcEdgeMask *=
        quadrantMask;

    edgeMask =
        max(
            edgeMask,
            arcEdgeMask);


    //
    // ---------------------------------------------------------
    // Center dashed line
    // ---------------------------------------------------------
    //

    float arcCenterMask =
        1.0f -
        smoothstep(
            0.04f,
            0.09f,
            distanceFromRoadCenter);

    float angle =
        atan2(
            offset.y,
            offset.x);

    //
    // 도로를 clockwise로 진행하므로
    // 시작각에서 현재각을 뺀다.
    //

    float localAngle =
        startAngle -
        angle;

    localAngle =
        clamp(
            localAngle,
            0.0f,
            1.57079633f);

    float arcDistance =
        localAngle *
        radius;

    float dashMask =
        GetDashMask(
            pathOffset +
            arcDistance);

    arcCenterMask *=
        quadrantMask *
        dashMask;

    centerLineMask =
        max(
            centerLineMask,
            arcCenterMask);
}


//
// =========================================================
// Ground pattern
// =========================================================
//

float3 ApplyGroundPattern(
    float3 baseColor,
    float3 worldPosition)
{
    float2 position =
        worldPosition.xz;


    //
    // =========================================================
    // Minor grid
    // =========================================================
    //

    float minorGrid =
        GetGridLine(
            position /
            4.0f,
            0.012f);

    float3 color =
        lerp(
            baseColor,
            baseColor * 0.65f,
            minorGrid * 0.35f);


    //
    // =========================================================
    // Track dimensions
    // =========================================================
    //
    // 시작 위치가 (0, 0) 부근이므로
    // 왼쪽 직선이 x = 0을 지나게 만들었다.
    //
    // 기존 점프대 (-1, 35)도 이 직선 위에 놓인다.
    //

    const float leftX =
        0.0f;

    const float rightX =
        360.0f;

    const float bottomZ =
        -180.0f;

    const float topZ =
        320.0f;

    //
    // 실제 차량이 따라가는 코너 반경
    //

    const float cornerRadius =
        60.0f;

    //
    // 전체 도로 폭 = 8m
    //

    const float roadHalfWidth =
        4.0f;

    //
// 첫 번째 좌회전 코너 기준 거리 표시선
//

    const float firstCornerStartZ =
        topZ - cornerRadius-50.0f;

    const float marker100mBeforeZ =
        firstCornerStartZ - 50.0f;


    //
    // =========================================================
    // Derived dimensions
    // =========================================================
    //

    const float verticalStraightLength =
        topZ -
        bottomZ -
        cornerRadius *
        2.0f;

    const float horizontalStraightLength =
        rightX -
        leftX -
        cornerRadius *
        2.0f;

    const float quarterCircleLength =
        1.57079633f *
        cornerRadius;


    //
    // =========================================================
    // Path offsets
    // =========================================================
    //
    // 중앙 점선이 코너를 지나도
    // 가능한 한 연속적으로 이어지도록
    // 누적 주행 거리를 사용한다.
    //

    const float leftStraightOffset =
        0.0f;

    const float topLeftCornerOffset =
        leftStraightOffset +
        verticalStraightLength;

    const float topStraightOffset =
        topLeftCornerOffset +
        quarterCircleLength;

    const float topRightCornerOffset =
        topStraightOffset +
        horizontalStraightLength;

    const float rightStraightOffset =
        topRightCornerOffset +
        quarterCircleLength;

    const float bottomRightCornerOffset =
        rightStraightOffset +
        verticalStraightLength;

    const float bottomStraightOffset =
        bottomRightCornerOffset +
        quarterCircleLength;

    const float bottomLeftCornerOffset =
        bottomStraightOffset +
        horizontalStraightLength;


    //
    // =========================================================
    // Masks
    // =========================================================
    //

    float roadMask =
        0.0f;

    float edgeMask =
        0.0f;

    float centerLineMask =
        0.0f;


    //
    // =========================================================
    // 1. Left straight
    // =========================================================
    //
    // 시작 지점이 포함된 직선
    //

    AccumulateStraightRoad(
        position,

        float2(
            leftX,
            bottomZ + cornerRadius),

        float2(
            leftX,
            topZ - cornerRadius),

        roadHalfWidth,
        leftStraightOffset,

        roadMask,
        edgeMask,
        centerLineMask);


    //
    // =========================================================
    // 2. Top-left corner
    // =========================================================
    //

    AccumulateQuarterCircleRoad(
        position,

        float2(
            leftX + cornerRadius,
            topZ - cornerRadius),

        cornerRadius,
        roadHalfWidth,

        -1.0f,
        +1.0f,

        3.14159265f,

        topLeftCornerOffset,

        roadMask,
        edgeMask,
        centerLineMask);


    //
    // =========================================================
    // 3. Top straight
    // =========================================================
    //

    AccumulateStraightRoad(
        position,

        float2(
            leftX + cornerRadius,
            topZ),

        float2(
            rightX - cornerRadius,
            topZ),

        roadHalfWidth,
        topStraightOffset,

        roadMask,
        edgeMask,
        centerLineMask);


    //
    // =========================================================
    // 4. Top-right corner
    // =========================================================
    //

    AccumulateQuarterCircleRoad(
        position,

        float2(
            rightX - cornerRadius,
            topZ - cornerRadius),

        cornerRadius,
        roadHalfWidth,

        +1.0f,
        +1.0f,

        1.57079633f,

        topRightCornerOffset,

        roadMask,
        edgeMask,
        centerLineMask);


    //
    // =========================================================
    // 5. Right straight
    // =========================================================
    //

    AccumulateStraightRoad(
        position,

        float2(
            rightX,
            topZ - cornerRadius),

        float2(
            rightX,
            bottomZ + cornerRadius),

        roadHalfWidth,
        rightStraightOffset,

        roadMask,
        edgeMask,
        centerLineMask);


    //
    // =========================================================
    // 6. Bottom-right corner
    // =========================================================
    //

    AccumulateQuarterCircleRoad(
        position,

        float2(
            rightX - cornerRadius,
            bottomZ + cornerRadius),

        cornerRadius,
        roadHalfWidth,

        +1.0f,
        -1.0f,

        0.0f,

        bottomRightCornerOffset,

        roadMask,
        edgeMask,
        centerLineMask);


    //
    // =========================================================
    // 7. Bottom straight
    // =========================================================
    //

    AccumulateStraightRoad(
        position,

        float2(
            rightX - cornerRadius,
            bottomZ),

        float2(
            leftX + cornerRadius,
            bottomZ),

        roadHalfWidth,
        bottomStraightOffset,

        roadMask,
        edgeMask,
        centerLineMask);


    //
    // =========================================================
    // 8. Bottom-left corner
    // =========================================================
    //

    AccumulateQuarterCircleRoad(
        position,

        float2(
            leftX + cornerRadius,
            bottomZ + cornerRadius),

        cornerRadius,
        roadHalfWidth,

        -1.0f,
        -1.0f,

        -1.57079633f,

        bottomLeftCornerOffset,

        roadMask,
        edgeMask,
        centerLineMask);


    //
    // =========================================================
    // Road surface
    // =========================================================
    //

    const float3 roadColor =
        float3(
            0.09f,
            0.10f,
            0.11f);

    color =
        lerp(
            color,
            roadColor,
            roadMask);


    //
    // =========================================================
    // Road edges
    // =========================================================
    //

    edgeMask *=
        roadMask;

    const float3 edgeColor =
        float3(
            0.90f,
            0.90f,
            0.90f);

    color =
        lerp(
            color,
            edgeColor,
            edgeMask);


    //
    // =========================================================
    // Center dashed line
    // =========================================================
    //

    centerLineMask *=
        roadMask;

    const float3 centerLineColor =
        float3(
            0.95f,
            0.95f,
            0.95f);

    color =
        lerp(
            color,
            centerLineColor,
            centerLineMask);

    //
// =========================================================
// Two marker lines on the first straight
// 1) 100m before first corner
// 2) first corner start
// =========================================================
//

// 도로 폭 안에서만 선을 보이게 하는 마스크
    float markerRoadWidthMask =
        1.0f -
        smoothstep(
            roadHalfWidth - 0.30f,
            roadHalfWidth,
            abs(position.x - leftX));


    // ---------------------------------------------------------
    // 1. 100m before first corner
    // ---------------------------------------------------------

    float marker100mDistance =
        abs(
            position.y -
            marker100mBeforeZ);

    float marker100mMask =
        1.0f -
        smoothstep(
            0.15f,
            0.25f,
            marker100mDistance);

    marker100mMask *=
        markerRoadWidthMask;

    marker100mMask *=
        roadMask;


    // ---------------------------------------------------------
    // 2. First corner start
    // ---------------------------------------------------------

    float cornerStartDistance =
        abs(
            position.y -
            firstCornerStartZ);

    float cornerStartMask =
        1.0f -
        smoothstep(
            0.15f,
            0.25f,
            cornerStartDistance);

    cornerStartMask *=
        markerRoadWidthMask;

    cornerStartMask *=
        roadMask;


    // ---------------------------------------------------------
    // Color
    // ---------------------------------------------------------

    const float3 marker100mColor =
        float3(
            1.00f,
            0.75f,
            0.10f);

    const float3 cornerStartColor =
        float3(
            1.00f,
            0.0f,
            0.0f);

    color =
        lerp(
            color,
            marker100mColor,
            marker100mMask);

    color =
        lerp(
            color,
            cornerStartColor,
            cornerStartMask);

    return
        color;
}


//
// =========================================================
// Wheel pattern
// =========================================================
//

float3 ApplyWheelPattern(
    float3 baseColor,
    float3 localPosition)
{
    float stripe =
        1.0f -
        smoothstep(
            0.015f,
            0.035f,
            abs(
                localPosition.z));

    float outerMask =
        step(
            0.06f,
            length(
                localPosition.yz));

    stripe *=
        outerMask;

    const float3 stripeColor =
        float3(
            0.95f,
            0.95f,
            0.95f);

    return
        lerp(
            baseColor,
            stripeColor,
            stripe);
}


//
// =========================================================
// Pixel Shader
// =========================================================
//

float4 main(
    PSInput input)
    : SV_TARGET
{
    //
    // =========================================================
    // Ground
    // =========================================================
    //

    if (BaseColor.a > 1.5f &&
        BaseColor.a <= 2.5f)
    {
        float3 groundColor =
            ApplyGroundPattern(
                BaseColor.rgb,
                input.WorldPosition);

        return
            float4(
                groundColor,
                1.0f);
    }


//
// =========================================================
// Lighting
// =========================================================
//

float3 normal =
    normalize(
        input.Normal);

float3 toLight =
    normalize(
        float3(
            -0.4f,
            1.0f,
            -0.3f));

float diffuse =
    saturate(
        dot(
            normal,
            toLight));

float ambient =
    0.20f;

float lighting =
    ambient +
    diffuse *
    0.80f;

float3 surfaceColor =
    BaseColor.rgb;


//
// =========================================================
// Wheel
// =========================================================
//

if (BaseColor.a > 2.5f)
{
    surfaceColor =
        ApplyWheelPattern(
            BaseColor.rgb,
            input.LocalPosition);
}


return
    float4(
        surfaceColor *
        lighting,
        1.0f);
}