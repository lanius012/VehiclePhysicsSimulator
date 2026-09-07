#include "pch.h"

#include "GameCore.h"
#include "GraphicsCore.h"
#include "SystemTime.h"
#include "TextRenderer.h"
#include "GameInput.h"
#include "CommandContext.h"
#include "RootSignature.h"
#include "PipelineState.h"
#include "BufferManager.h"

#include "Camera.h"
#include "PrimitiveRenderer.h"
#include "PhysicsSystem.h"

#include "PhysicsRenderBridge.h"
#include "PrimitiveTypes.h"

#include "VehicleInput.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <stdexcept>
#include <vector>

using namespace GameCore;
using namespace Graphics;
using namespace Math;

namespace
{

    const bool kEnableMiniEngineRendering = true;

    VehicleInput ReadVehicleInput()
    {
        VehicleInput input;

        const bool steerLeft =
            GameInput::IsPressed(
                GameInput::kKey_left);

        const bool steerRight =
            GameInput::IsPressed(
                GameInput::kKey_right);

        const bool driveForward =
            GameInput::IsPressed(
                GameInput::kKey_up);

        const bool driveReverse =
            GameInput::IsPressed(
                GameInput::kKey_down);

            // bool은 정수 변환 시
            // false = 0
            // true  = 1
            //
            // A만 누름:
            // 0 - 1 = -1
            //
            // D만 누름:
            // 1 - 0 = +1
            //
            // 둘 다 누름:
            // 1 - 1 = 0
        input.steer =
            static_cast<int>(steerRight) -
            static_cast<int>(steerLeft);

            // W만 누름:
            // 1 - 0 = +1
            //
            // S만 누름:
            // 0 - 1 = -1
            //
            // 둘 다 누름:
            // 1 - 1 = 0
        input.drive =
            static_cast<int>(driveForward) -
            static_cast<int>(driveReverse);

            // R을 누른 첫 프레임에만 true
        input.reset =
            GameInput::IsFirstPressed(
                GameInput::kKey_r);

        return input;
    }
}

class PhysXViewer : public GameCore::IGameApp
{
public:
    PhysXViewer()=default;

    virtual void Startup() override;
    virtual void Cleanup() override;
    virtual void Update(float deltaT) override;
    virtual void RenderScene() override;

private:
    enum class CameraMode
    {
        Chase = 0,
        CloseChase,
        Hood,
        Side,
        Front,
        Top,

        Count
    };

    void UpdateCamera(float deltaTime);

    const char* GetCameraModeName() const;

private:
    Camera m_Camera;

    CameraMode m_CameraMode =
        CameraMode::Chase;

    Vector3 m_CameraEye =
        Vector3(0.0f, 0.0f, 0.0f);

    Vector3 m_CameraTarget =
        Vector3(0.0f, 0.0f, 0.0f);

    bool m_CameraInitialized = false;

    PrimitiveRenderer m_PrimitiveRenderer;
    PhysicsSystem m_PhysicsSystem;
    PhysicsRenderBridge m_RenderBridge;

    std::vector<RenderItem> m_RenderItems;

    std::size_t m_GroundRenderItemIndex = 0;
    std::size_t m_ChassisRenderItemIndex = 0;
    std::size_t m_BodyRenderItemIndex = 0;

    std::array<
        std::size_t,
        PhysicsSystem::kWheelCount>
        m_WheelRenderItemIndices{};

    std::vector<std::size_t>
        m_AdditionalGroundRenderItemIndices;

    float m_PhysicsAccumulator = 0.0f;

    static constexpr float kPhysicsFixedDeltaTime =
        1.0f / 60.0f;
};

CREATE_APPLICATION(PhysXViewer)


void PhysXViewer::Startup()
{

    const bool physicsInitialized =
        m_PhysicsSystem.Initialize();

    if (!physicsInitialized)
    {
        throw std::runtime_error(
            "PhysicsSystem initialization failed.");
    }

    m_PhysicsAccumulator = 0.0f;

    if (!kEnableMiniEngineRendering) {
        return;
    }

    //
    // 1. 카메라 위치 설정
    //

    m_Camera.SetEyeAtUp(
        Vector3(8.0f, 6.0f, 12.0f),
        Vector3(0.0f, 2.0f, 0.0f),
        Vector3(0.0f, 1.0f, 0.0f));

    const float width =
        static_cast<float>(
            g_SceneColorBuffer.GetWidth());

    const float height =
        static_cast<float>(
            g_SceneColorBuffer.GetHeight());

    // MiniEngine Camera는 width / height가 아니라
    // height / width를 받는다.
    const float aspectHeightOverWidth =
        height / width;

    m_Camera.SetPerspectiveMatrix(
        XM_PIDIV4,             // 수직 FOV 45도
        aspectHeightOverWidth,
        0.1f,                  // near plane
        1000.0f);              // far plane

    m_Camera.Update();

    //
    // 2. Primitive Renderer 초기화
    //

    m_PrimitiveRenderer.Initialize();

    

    m_RenderItems.clear();
    m_RenderBridge.Clear();

    // -------------------------------------------------------------
    // Ground
    // -------------------------------------------------------------

    m_GroundRenderItemIndex =
        m_RenderItems.size();

    m_RenderItems.emplace_back(
        PrimitiveType::Cube,
        Vector4(
            0.35f,
            0.38f,
            0.42f,
            2.0f));

    if (!m_RenderBridge.RegisterBox(
        m_PhysicsSystem.GetGroundActor(),
        m_PhysicsSystem.GetGroundShape(),
        m_GroundRenderItemIndex))
    {
        throw std::runtime_error(
            "Failed to register ground.");
    }

    // -------------------------------------------------------------
// Rough road spheres and jump ramp
// -------------------------------------------------------------

    m_AdditionalGroundRenderItemIndices.clear();

    const std::size_t groundObjectCount =
        m_PhysicsSystem
        .GetAdditionalGroundObjectCount();

    m_AdditionalGroundRenderItemIndices.reserve(
        groundObjectCount);

    for (std::size_t i = 0;
        i < groundObjectCount;
        ++i)
    {
        const PhysicsSystem::GroundObject* object =
            m_PhysicsSystem
            .GetAdditionalGroundObject(i);

        if (object == nullptr ||
            object->actor == nullptr ||
            object->shape == nullptr)
        {
            throw std::runtime_error(
                "Invalid additional ground object.");
        }

        const physx::PxGeometryType::Enum geometryType =
            object->shape
            ->getGeometry()
            .getType();

        const std::size_t renderItemIndex =
            m_RenderItems.size();

        bool registered = false;

        switch (geometryType)
        {
        case physx::PxGeometryType::eSPHERE:
            // 비포장도로 돌기
            m_RenderItems.emplace_back(
                PrimitiveType::Sphere,
                Vector4(
                    0.38f,
                    0.24f,
                    0.12f,
                    1.0f));

            registered =
                m_RenderBridge.RegisterSphere(
                    object->actor,
                    object->shape,
                    renderItemIndex);
            break;

        case physx::PxGeometryType::eBOX:
            // 점프대
            m_RenderItems.emplace_back(
                PrimitiveType::Cube,
                Vector4(
                    0.48f,
                    0.30f,
                    0.14f,
                    1.0f));

            registered =
                m_RenderBridge.RegisterBox(
                    object->actor,
                    object->shape,
                    renderItemIndex);
            break;

        default:
            // 현재 렌더러가 지원하지 않는 형상
            continue;
        }

        if (!registered)
        {
            throw std::runtime_error(
                "Failed to register additional ground object.");
        }

        m_AdditionalGroundRenderItemIndices.push_back(
            renderItemIndex);
    }

    // -------------------------------------------------------------
    // Chassis
    // -------------------------------------------------------------

    m_ChassisRenderItemIndex =
        m_RenderItems.size();

    m_RenderItems.emplace_back(
        PrimitiveType::Cube,
        Vector4(
            0.15f,
            0.35f,
            0.90f,
            1.0f));

    if (!m_RenderBridge.RegisterBox(
        m_PhysicsSystem.GetChassisActor(),
        m_PhysicsSystem.GetChassisShape(),
        m_ChassisRenderItemIndex))
    {
        throw std::runtime_error(
            "Failed to register chassis.");
    }

    // -------------------------------------------------------------
// Upper Body Collision Box
// -------------------------------------------------------------

    m_BodyRenderItemIndex =
        m_RenderItems.size();

    m_RenderItems.emplace_back(
        PrimitiveType::Cube,
        Vector4(
            0.75f,
            0.15f,
            0.12f,
            1.0f));

    if (!m_RenderBridge.RegisterBox(
        m_PhysicsSystem.GetChassisActor(),
        m_PhysicsSystem.GetBodyShape(),
        m_BodyRenderItemIndex))
    {
        throw std::runtime_error(
            "Failed to register vehicle body box.");
    }

    // -------------------------------------------------------------
    // Wheels
    // -------------------------------------------------------------

    for (std::size_t i = 0;
        i < PhysicsSystem::kWheelCount;
        ++i)
    {
        m_WheelRenderItemIndices[i] =
            m_RenderItems.size();

        m_RenderItems.emplace_back(
            PrimitiveType::Cylinder,
            Vector4(
                0.08f,
                0.08f,
                0.08f,
                3.0f));

        const PhysicsSystem::WheelState* wheelState =
            m_PhysicsSystem.GetWheelState(i);

        if (wheelState == nullptr)
        {
            throw std::runtime_error(
                "Failed to get wheel state.");
        }

        const bool registered =
            m_RenderBridge.RegisterPoseCylinder(
                &wheelState->visualPoseWorld,
                m_WheelRenderItemIndices[i],
                m_PhysicsSystem.GetWheelRadius(),
                m_PhysicsSystem.GetWheelHalfWidth());

        if (!registered)
        {
            throw std::runtime_error(
                "Failed to register virtual wheel.");
        }
    }

    // 최초 pose를 RenderItem에 반영한다.
    m_RenderBridge.Sync(
        m_RenderItems);


}

void PhysXViewer::Cleanup()
{
    //
    // Bridge는 PhysX actor/shape 포인터를 빌려 쓰므로
    // PhysX 객체를 해제하기 전에 먼저 연결을 제거한다.
    //

    m_RenderBridge.Clear();

    m_AdditionalGroundRenderItemIndices.clear();

    m_RenderItems.clear();

    m_PhysicsSystem.Shutdown();
    if (!kEnableMiniEngineRendering) {
        return;
    }

    m_PrimitiveRenderer.Shutdown();
}
void PhysXViewer::Update(float deltaT)
{
    ScopedTimer timer(L"Update State");

    //
    // 1. 렌더 프레임에서 키 입력을 한 번 읽는다.
    //

    const VehicleInput vehicleInput =
        ReadVehicleInput();

    m_PhysicsSystem.SetVehicleInput(
        vehicleInput);

    //
    // 2. 렌더 deltaT 제한 및 누적
    //

    const float clampedDeltaTime =
        (std::min)(deltaT, 0.1f);

    m_PhysicsAccumulator +=
        clampedDeltaTime;

    //
    // 3. 고정 물리 스텝
    //

    while (m_PhysicsAccumulator >=
        kPhysicsFixedDeltaTime)
    {
        m_PhysicsSystem.Step(
            kPhysicsFixedDeltaTime);

        m_PhysicsAccumulator -=
            kPhysicsFixedDeltaTime;
    }

    if (!kEnableMiniEngineRendering) {
        return;
    }

    //
    // 4. fetchResults 이후 pose 동기화
    //

    m_RenderBridge.Sync(
        m_RenderItems);

    UpdateCamera(clampedDeltaTime);

    const float width =
        static_cast<float>(
            g_SceneColorBuffer.GetWidth());

    const float height =
        static_cast<float>(
            g_SceneColorBuffer.GetHeight());

    if (width > 0.0f &&
        height > 0.0f)
    {
        m_Camera.SetAspectRatio(
            height / width);
    }

    m_Camera.Update();
}

const char* PhysXViewer::GetCameraModeName() const
{
    switch (m_CameraMode)
    {
    case CameraMode::Chase:
        return "CHASE";

    case CameraMode::CloseChase:
        return "CLOSE CHASE";

    case CameraMode::Hood:
        return "HOOD";

    case CameraMode::Side:
        return "SIDE";

    case CameraMode::Front:
        return "FRONT";

    case CameraMode::Top:
        return "TOP";

    default:
        return "UNKNOWN";
    }
}

void PhysXViewer::UpdateCamera(
    float deltaTime)
{
    //
    // C 키를 누르면 다음 카메라 모드로 전환
    //

    if (GameInput::IsFirstPressed(
        GameInput::kKey_c))
    {
        const int currentMode =
            static_cast<int>(
                m_CameraMode);

        const int modeCount =
            static_cast<int>(
                CameraMode::Count);

        m_CameraMode =
            static_cast<CameraMode>(
                (currentMode + 1) %
                modeCount);

        //
        // 모드 변경 시 이전 카메라 위치에서
        // 자동차 내부를 통과하며 이동하지 않도록
        // 새 위치로 즉시 초기화한다.
        //

        m_CameraInitialized = false;
    }

    physx::PxRigidDynamic* chassis =
        m_PhysicsSystem.GetChassisActor();

    if (chassis == nullptr)
    {
        return;
    }

    const physx::PxTransform chassisPose =
        chassis->getGlobalPose();

    const physx::PxVec3 chassisPosition =
        chassisPose.p;

    const physx::PxVec3 worldUp(
        0.0f,
        1.0f,
        0.0f);

    //
    // 차체의 실제 Forward와 Up
    //
    // 보닛 시점에서는 차체의 피치와 롤도
    // 카메라에 반영하기 위해 사용한다.
    //

    physx::PxVec3 chassisForward =
        chassisPose.q.rotate(
            physx::PxVec3(
                0.0f,
                0.0f,
                1.0f));

    physx::PxVec3 chassisUp =
        chassisPose.q.rotate(
            physx::PxVec3(
                0.0f,
                1.0f,
                0.0f));

    if (chassisForward.magnitudeSquared() <
        1.0e-6f)
    {
        chassisForward =
            physx::PxVec3(
                0.0f,
                0.0f,
                1.0f);
    }
    else
    {
        chassisForward.normalize();
    }

    if (chassisUp.magnitudeSquared() <
        1.0e-6f)
    {
        chassisUp = worldUp;
    }
    else
    {
        chassisUp.normalize();
    }

    //
    // 일반 추적 카메라는 차체의 피치와 롤을
    // 그대로 따라가지 않도록 수평 Forward를 사용한다.
    //

    physx::PxVec3 flatForward =
        chassisForward;

    flatForward.y = 0.0f;

    if (flatForward.magnitudeSquared() <
        1.0e-6f)
    {
        flatForward =
            physx::PxVec3(
                0.0f,
                0.0f,
                1.0f);
    }
    else
    {
        flatForward.normalize();
    }

    physx::PxVec3 flatRight =
        worldUp.cross(
            flatForward);

    if (flatRight.magnitudeSquared() <
        1.0e-6f)
    {
        flatRight =
            physx::PxVec3(
                1.0f,
                0.0f,
                0.0f);
    }
    else
    {
        flatRight.normalize();
    }

    physx::PxVec3 desiredEyePx =
        chassisPosition;

    physx::PxVec3 desiredTargetPx =
        chassisPosition;

    physx::PxVec3 desiredUpPx =
        worldUp;

    float followSpeed = 6.0f;

    switch (m_CameraMode)
    {
    case CameraMode::Chase:
        //
        // 기존과 비슷한 기본 추적 시점
        //
        desiredEyePx =
            chassisPosition
            - flatForward * 7.0f
            + worldUp * 2.8f;

        desiredTargetPx =
            chassisPosition
            + flatForward * 3.0f
            + worldUp * 1.0f;

        desiredUpPx = worldUp;
        followSpeed = 6.0f;
        break;

    case CameraMode::CloseChase:
        //
        // 자동차에 조금 더 가까운 추적 시점
        //
        desiredEyePx =
            chassisPosition
            - flatForward * 4.0f
            + worldUp * 1.7f;

        desiredTargetPx =
            chassisPosition
            + flatForward * 4.0f
            + worldUp * 0.7f;

        desiredUpPx = worldUp;
        followSpeed = 9.0f;
        break;

    case CameraMode::Hood:
        //
        // 차체 앞쪽 위에 부착된 보닛 시점
        //
        // 이 모드는 차체의 롤과 피치를 함께 따라간다.
        //
        desiredEyePx =
            chassisPosition
            + chassisForward * 1.05f
            + chassisUp * 0.45f;

        desiredTargetPx =
            desiredEyePx
            + chassisForward * 15.0f;

        desiredUpPx = chassisUp;
        followSpeed = 20.0f;
        break;

    case CameraMode::Side:
        //
        // 자동차 오른쪽 측면에서 바라보는 시점
        //
        // 반대쪽에서 보고 싶으면
        // + flatRight를 - flatRight로 바꾸면 된다.
        //
        desiredEyePx =
            chassisPosition
            + flatRight * 7.0f
            + worldUp * 2.5f;

        desiredTargetPx =
            chassisPosition
            + worldUp * 0.7f;

        desiredUpPx = worldUp;
        followSpeed = 5.0f;
        break;

    case CameraMode::Front:
        //
        // 자동차 앞에서 뒤쪽을 바라보는 시점
        //
        desiredEyePx =
            chassisPosition
            + flatForward * 7.0f
            + worldUp * 2.5f;

        desiredTargetPx =
            chassisPosition
            + worldUp * 0.7f;

        desiredUpPx = worldUp;
        followSpeed = 6.0f;
        break;

    case CameraMode::Top:
        //
        // 자동차 위에서 내려다보는 시점
        //
        desiredEyePx =
            chassisPosition
            + worldUp * 14.0f;

        desiredTargetPx =
            chassisPosition
            + flatForward * 2.0f;

        //
        // 화면 위쪽이 자동차 전방을 향하도록 한다.
        //
        desiredUpPx = flatForward;
        followSpeed = 5.0f;
        break;

    default:
        break;
    }

    const Vector3 desiredEye(
        desiredEyePx.x,
        desiredEyePx.y,
        desiredEyePx.z);

    const Vector3 desiredTarget(
        desiredTargetPx.x,
        desiredTargetPx.y,
        desiredTargetPx.z);

    const Vector3 desiredUp(
        desiredUpPx.x,
        desiredUpPx.y,
        desiredUpPx.z);

    if (!m_CameraInitialized)
    {
        m_CameraEye =
            desiredEye;

        m_CameraTarget =
            desiredTarget;

        m_CameraInitialized = true;
    }
    else
    {
        const float followT =
            (std::min)(
                followSpeed * deltaTime,
                1.0f);

        m_CameraEye +=
            (desiredEye - m_CameraEye) *
            followT;

        m_CameraTarget +=
            (desiredTarget - m_CameraTarget) *
            followT;
    }

    m_Camera.SetEyeAtUp(
        m_CameraEye,
        m_CameraTarget,
        desiredUp);
}

void PhysXViewer::RenderScene()
{

    if (!kEnableMiniEngineRendering) {
        return;
    }

    GraphicsContext& context =
        GraphicsContext::Begin(L"Scene Render");

    //
    // 1. Color/Depth 버퍼 상태 전환
    //

    context.TransitionResource(
        g_SceneColorBuffer,
        D3D12_RESOURCE_STATE_RENDER_TARGET);

    context.TransitionResource(
        g_SceneDepthBuffer,
        D3D12_RESOURCE_STATE_DEPTH_WRITE,
        true);

    //
    // 2. 이전 프레임 제거
    //

    context.ClearColor(
        g_SceneColorBuffer);

    context.ClearDepth(
        g_SceneDepthBuffer);

    //
    // 3. 렌더 타깃 설정
    //

    context.SetRenderTarget(
        g_SceneColorBuffer.GetRTV(),
        g_SceneDepthBuffer.GetDSV());

    context.SetViewportAndScissor(
        0,
        0,
        g_SceneColorBuffer.GetWidth(),
        g_SceneColorBuffer.GetHeight());

    //
    // 4. 단위 Cube 렌더링
    //

    //
    for (const RenderItem& renderItem :
        m_RenderItems)
    {
        m_PrimitiveRenderer.Render(
            context,
            m_Camera,
            renderItem);
    }

    const VehicleInput input =
        ReadVehicleInput();

    const float screenWidth =
        static_cast<float>(
            g_SceneColorBuffer.GetWidth());

    const float screenHeight =
        static_cast<float>(
            g_SceneColorBuffer.GetHeight());

    TextContext text(
        context,
        screenWidth,
        screenHeight);

    text.Begin();

    //
    // ---------------------------------------------------------
    // Left : Input / Camera
    // ---------------------------------------------------------
    //

    text.ResetCursor(
        20.0f,
        20.0f);

    text.SetTextSize(
        24.0f);

    text.DrawFormattedString(
        "UP    : %s\n",
        input.drive > 0 ? "PRESSED" : "-");

    text.DrawFormattedString(
        "DOWN  : %s\n",
        input.drive < 0 ? "PRESSED" : "-");

    text.DrawFormattedString(
        "LEFT  : %s\n",
        input.steer < 0 ? "PRESSED" : "-");

    text.DrawFormattedString(
        "RIGHT : %s\n",
        input.steer > 0 ? "PRESSED" : "-");

    text.DrawFormattedString(
        "\nCAMERA [C] : %s\n",
        GetCameraModeName());


    //
    // ---------------------------------------------------------
    // Right : Vehicle Debug Information
    // ---------------------------------------------------------
    //

    const float debugPanelWidth =
        550.0f;

    text.ResetCursor(
        screenWidth - debugPanelWidth,
        20.0f);

    text.SetTextSize(
        24.0f);

    physx::PxRigidDynamic* chassis =
        m_PhysicsSystem.GetChassisActor();

    if (chassis != nullptr)
    {
        const physx::PxTransform chassisPose =
            chassis->getGlobalPose();

        const physx::PxVec3 velocity =
            chassis->getLinearVelocity();

        physx::PxVec3 forward =
            chassisPose.q.rotate(
                physx::PxVec3(
                    0.0f,
                    0.0f,
                    1.0f));

        const float speedKmh =
            velocity.magnitude() *
            3.6f;

        const float forwardSpeedKmh =
            velocity.dot(forward) *
            3.6f;

        text.DrawFormattedString(
            "=== VEHICLE ===\n");

        text.DrawFormattedString(
            "Speed       : %7.2f km/h\n",
            speedKmh);

        text.DrawFormattedString(
            "Forward     : %7.2f km/h\n",
            forwardSpeedKmh);

        text.DrawFormattedString(
            "\n");
    }


    //
    // Wheel names correspond to PhysicsSystem::WheelIndex:
    //
    // 0 = RearLeft
    // 1 = RearRight
    // 2 = FrontLeft
    // 3 = FrontRight
    //

    const char* wheelNames[
        PhysicsSystem::kWheelCount] =
        {
            "REAR LEFT",
            "REAR RIGHT",
            "FRONT LEFT",
            "FRONT RIGHT"
        };

        for (std::size_t i = 0;
            i < PhysicsSystem::kWheelCount;
            ++i)
        {
            const PhysicsSystem::WheelState* wheel =
                m_PhysicsSystem.GetWheelState(i);

            if (wheel == nullptr)
            {
                continue;
            }

            const float wheelSurfaceSpeedKmh =
                wheel->angularVelocity *
                m_PhysicsSystem.GetWheelRadius() *
                3.6f;

            const float slipAngleDeg =
                wheel->slipAngle *
                180.0f /
                physx::PxPi;

            text.DrawFormattedString(
                "[%s]  %s\n",
                wheelNames[i],
                wheel->isGrounded
                ? "GROUND"
                : "AIR");

            text.DrawFormattedString(
                " w:%7.2f rad/s"
                "  V:%7.2f km/h\n",
                wheel->angularVelocity,
                wheelSurfaceSpeedKmh);

            text.DrawFormattedString(
                " Fx:%8.1f"
                "  Fy:%8.1f"
                "  Fz:%8.1f\n",
                wheel->longitudinalForce,
                wheel->lateralForce,
                wheel->normalLoad);

            text.DrawFormattedString(
                " Slip:%7.3f"
                "  Angle:%6.2f deg\n",
                wheel->longitudinalSlip,
                slipAngleDeg);

            text.DrawFormattedString(
                " DriveT:%7.1f"
                "  BrakeT:%7.1f\n\n",
                wheel->driveTorque,
                wheel->brakeTorque);
        }

        text.End();


    context.Finish();
}