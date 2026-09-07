#include "pch.h"
#include "PhysicsSystem.h"

#include <iostream>
#include <algorithm>
#include <array>
#include <cmath>


using namespace physx;

namespace
{

    void DebugPrint(
        const char* format,
        ...)
    {
        char buffer[1024];

        va_list arguments;
        va_start(arguments, format);

        _vsnprintf_s(
            buffer,
            sizeof(buffer),
            _TRUNCATE,
            format,
            arguments);

        va_end(arguments);

        OutputDebugStringA(buffer);
    }


    // PhysX의 PxBoxGeometry는 전체 크기가 아니라 half extents를 사용한다.
    constexpr float kGroundHalfWidth = 10.0f;
    constexpr float kGroundHalfHeight = 0.25f;
    constexpr float kGroundHalfDepth = 10.0f;

    float MoveToward(
        float current,
        float target,
        float maxDelta)
    {
        if (current < target)
        {
            return PxMin(
                current + maxDelta,
                target);
        }

        if (current > target)
        {
            return PxMax(
                current - maxDelta,
                target);
        }

        return target;
    }

    float DegToRad(float degrees)
    {
        return degrees * PxPi / 180.0f;
    }

    float Lerp(
        float from,
        float to,
        float t)
    {
        return from + (to - from) * t;
    }

    // 기존 코드의 속도별 최대 조향각을 사용한다.
    //
    // 기존 코드에는 30~60 km/h 구간이 빠져 있어서
    // 30 km/h 지점에서 조향각이 갑자기 변하는 문제가 있었다.
    // 여기서는 그 구간을 연속적으로 보간한다.
    float GetMaxSteerAngleBySpeed(
        float speedMetersPerSecond)
    {
        const float speedKmh =
            std::abs(speedMetersPerSecond) * 3.6f;

        if (speedKmh < 10.0f)
        {
            return DegToRad(30.0f);
        }

        if (speedKmh < 30.0f)
        {
            const float t =
                (speedKmh - 10.0f) / 20.0f;

            return DegToRad(
                Lerp(30.0f, 20.0f, t));
        }

        if (speedKmh < 60.0f)
        {
            const float t =
                (speedKmh - 30.0f) / 30.0f;

            return DegToRad(
                Lerp(20.0f, 6.0f, t));
        }

        if (speedKmh < 100.0f)
        {
            const float t =
                (speedKmh - 60.0f) / 40.0f;

            return DegToRad(
                Lerp(6.0f, 2.0f, t));
        }

        if (speedKmh < 140.0f)
        {
            const float t =
                (speedKmh - 100.0f) / 40.0f;

            return DegToRad(
                Lerp(2.0f, 1.0f, t));
        }
        /*
        if (speedKmh < 200.0f)
        {
            const float t =
                (speedKmh - 140.0f) / 60.0f;

            return DegToRad(
                Lerp(2.5f, 1.0f, t));
        }*/

        return DegToRad(1.0f);
    }


    float GetSteeringSpeedByVehicleSpeed(float speedKmh) {
        if (speedKmh < 60.0f) return DegToRad(120.0f);
        if (speedKmh < 100.0f) return DegToRad(80.0f);
        if (speedKmh < 140.0f) return DegToRad(40.0f);
        return DegToRad(15.0f);
    }

    float longitudinalModeling(
        float slipratio, int ground) {
        float key = std::abs(slipratio);
        float result = 0;
        if (ground == 0) {
            if (key < 0.0225) result = 17.11 * key;
            else if (key < 0.14247) result = 5.127 * key + 0.2695;
            else if (key < 0.9141) result = -0.511925 * key + 1.073;
            else result = 0.60498833;
        }
        else if (ground == 1) {//wet
            if (key < 0.02368000) result = 15.44303649 * key;
            else if (key < 0.14997000) result = 4.62672338 * key + 0.25613029;
            else if (key < 0.91613000) result = -0.47577523 * key + 1.02135201;
            else result = 0.58548005;
        }
        else if (ground == 2) {//packedsnow
            if (key < 0.07262000) result = 3.00067212 * key;
            else if (key < 0.20311000) result = 1.16553905 * key + 0.13326736;
            else if (key < 0.81043000) result = -0.13424098 * key + 0.39726569;
            else result = 0.28847277;
        }
        else if (ground == 3) {//drysnow
            if (key < 0.07316000) result = 2.08418385 * key;
            else if (key < 0.20303000) result = 0.82791337 * key + 0.09190875;
            else if (key < 0.82564000) result = -0.10294727 * key + 0.28090138;
            else result = 0.19590400;
        }
        else if (ground == 4) {//ice
            if (key < 0.07322000) result = 0.88055424 * key;
            else if (key < 0.20299000) result = 0.35081928 * key + 0.03878719;
            else if (key < 0.82757000) result = -0.04407023 * key + 0.11894582;
            else result = 0.08247462;
        }
        else if (ground == 5) {//dry
            if (key < 0.02428000) result = 23.50332033 * key;
            else if (key < 0.14867000) result = 5.70254347 * key + 0.43220286;
            else if (key < 0.37800000) result = -0.14097377 * key + 1.30095857;
            else result = 1.24767049;
        }
        else if (ground == 6) {//wet
            if (key < 0.01260000) result = 32.89187393 * key;
            else if (key < 0.14238000) result = 5.74481729 * key + 0.34205291;
            else if (key < 0.86803000) result = -0.26556620 * key + 1.19781131;
            else result = 0.96729189;
        }
        
        
        if (slipratio >= 0) return result;
        else return -result;
        
    }

    float lateralModeling(
        float slipangle, int ground) {
        float key = std::abs(slipangle);
        float result = 0;
        if (ground == 0) {
        if (key < 0.05) result = 10.16355 * key;
        else if (key < 0.25) result = 3.5 * key + 0.3329;
        else if (key < 0.43321) result = -0.310657 * key + 1.113678;
        else result = 0.9791;
        }
        else if (ground == 1) {//wet
            if (key < 0.04162610) result = 9.87801466 * key;
            else if (key < 0.22424688) result = 2.34812679 * key + 0.31343989;
            else if (key < 0.22425212) result = -0.00000305 * key + 0.84000068;
            else result = 0.84000000;
        }
        else if (ground == 2) {//packedsnow
            if (key < 0.02236290) result = 6.22984273 * key;
            else if (key < 0.25219659) result = 1.12116999 * key + 0.11424476;
            else if (key < 0.39152622) result = -0.04025333 * key + 0.40715175;
            else result = 0.39139152;
        }
        else if (ground == 3) {//sandsnow
            if (key < 0.02362478) result = 4.87937792 * key;
            else if (key < 0.28367534) result = 0.82955322 * key + 0.09567620;
            else if (key < 0.34813559) result = -0.01581734 * key + 0.33548699;
            else result = 0.32998041;
        }
        else if (ground == 4) {//ice
            if (key < 0.01390678) result = 4.73301709 * key;
            else if (key < 0.07775442) result = 1.56903160 * key + 0.04400086;
            else if (key < 0.49643971) result = -0.22178627 * key + 0.18324486;
            else result = 0.07314135;
        }
        else if (ground == 5) {//dry
            if (key < 0.01977633) result = 28.43501296 * key;
            else if (key < 0.09317964) result = 10.40361663 * key + 0.35659477;
            else if (key < 0.44825291) result = -1.04485638 * key + 1.42335934;
            else result = 0.95499943;
        }
        else if (ground == 6) {//wet
            if (key < 0.01524196) result = 17.07787670 * key;
            else if (key < 0.10661518) result = 4.69174305 * key + 0.18878896;
            else if (key < 0.44236766) result = -0.33895753 * key + 0.72513802;
            else result = 0.57519417;
        }

        if (slipangle >= 0) return -result;
        else return result;
    }
}

bool PhysicsSystem::InitializePhysX()
{
    using namespace physx;

    m_Foundation = PxCreateFoundation(
        PX_PHYSICS_VERSION,
        m_Allocator,
        m_ErrorCallback);

    if (m_Foundation == nullptr)
    {
        return false;
    }

    // ---------------------------------------------------------
    // 선택 사항: 기존 코드에서 PVD를 사용했다면 이식
    // ---------------------------------------------------------

    m_Pvd = PxCreatePvd(
        *m_Foundation);

    if (m_Pvd != nullptr)
    {
        m_PvdTransport =
            PxDefaultPvdSocketTransportCreate(
                "127.0.0.1",
                5425,
                10);

        if (m_PvdTransport != nullptr)
        {
            m_Pvd->connect(
                *m_PvdTransport,
                PxPvdInstrumentationFlag::eALL);
        }
    }

    PxTolerancesScale toleranceScale;

    m_Physics = PxCreatePhysics(
        PX_PHYSICS_VERSION,
        *m_Foundation,
        toleranceScale,
        true,
        m_Pvd);

    if (m_Physics == nullptr)
    {
        return false;
    }

    // D6 Joint 등 PhysX Extensions를 사용하므로 초기화한다.
    m_ExtensionsInitialized =
        PxInitExtensions(
            *m_Physics,
            m_Pvd);

    if (!m_ExtensionsInitialized)
    {
        return false;
    }

    PxSceneDesc sceneDesc(
        m_Physics->getTolerancesScale());

    sceneDesc.gravity =
        PxVec3(0.0f, -9.81f, 0.0f);

    m_Dispatcher =
        PxDefaultCpuDispatcherCreate(2);

    if (m_Dispatcher == nullptr)
    {
        return false;
    }

    sceneDesc.cpuDispatcher =
        m_Dispatcher;

    sceneDesc.filterShader =
        PxDefaultSimulationFilterShader;

    // 기존 프로젝트에서 eTGS를 사용했다면 여기로 이동한다.
    //
    sceneDesc.solverType =
         PxSolverType::eTGS;

    sceneDesc.flags |= PxSceneFlag::eENABLE_BODY_ACCELERATIONS;//for accelerations



    m_Scene =
        m_Physics->createScene(
            sceneDesc);

    if (m_Scene == nullptr)
    {
        return false;
    }

    m_Scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
    m_Scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);

    PxPvdSceneClient* pvdClient = m_Scene->getScenePvdClient();
    if (pvdClient)
    {
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }

    // 기존 마찰값으로 교체
    m_Material =
        m_Physics->createMaterial(
            0.5f,
            0.5f,
            0.8f);

    if (m_Material == nullptr)
    {
        return false;
    }

    return true;
}

bool PhysicsSystem::CreateGround()
{
    using namespace physx;

    if (m_Physics == nullptr ||
        m_Scene == nullptr ||
        m_Material == nullptr)
    {
        return false;
    }

    const PxVec3 groundHalfExtents(
        10000.0f,
        0.25f,
        10000.0f);

    const PxTransform groundPose(
        PxVec3(
            0.0f,
            -groundHalfExtents.y,
            0.0f));

    m_Ground =
        m_Physics->createRigidStatic(
            groundPose);

    if (m_Ground == nullptr)
    {
        return false;
    }

    PxShape* groundShape =
        m_Physics->createShape(
            PxBoxGeometry(
                groundHalfExtents),
            *m_Material);

    if (groundShape == nullptr)
    {
        return false;
    }

    m_Ground->attachShape(
        *groundShape);

    // 비소유 포인터로 기억한다.
    m_GroundShape = groundShape;

    // createShape()로 받은 로컬 참조만 반환한다.
    // Actor는 부착된 Shape의 참조를 유지한다.
    groundShape->release();

    m_Scene->addActor(
        *m_Ground);

    // -------------------------------------------------------------
// 추가 지형을 Scene과 렌더링 목록에 등록하는 보조 함수
// -------------------------------------------------------------

    auto registerGroundObject =
        [this](PxRigidStatic* actor) -> bool
    {
        if (actor == nullptr)
        {
            return false;
        }

        PxShape* shape = nullptr;

        if (actor->getShapes(
            &shape,
            1) != 1 ||
            shape == nullptr)
        {
            actor->release();
            return false;
        }

        // PhysX Scene에 들어감
        m_Scene->addActor(
            *actor);

        // 렌더링 대상 목록에도 들어감
        GroundObject object;

        object.actor = actor;
        object.shape = shape;

        m_AdditionalGroundObjects.push_back(
            object);

        return true;
    };

    // -------------------------------------------------------------
// 비포장도로
// 필요 없으면 이 블록 전체를 주석 처리
// -------------------------------------------------------------
    
    /*for (int i = 10; i < 30; ++i)
    {
        const std::array<float, 13> xPositions =
        {
            -1.5f,
            -1.25f,
            -1.0f,
            -0.75f,
            -0.5f,
            -0.25f,
             0.0f,
             0.25f,
             0.5f,
             0.75f,
             1.0f,
             1.25f,
             1.5f,
        };

        for (float x : xPositions)
        {
            PxRigidStatic* sphere =
                PxCreateStatic(
                    *m_Physics,
                    PxTransform(
                        PxVec3(
                            x,
                            0.0f,
                            static_cast<float>(i))),
                    PxSphereGeometry(
                        0.05f),
                    *m_Material);

            if (!registerGroundObject(
                sphere))
            {
                return false;
            }
        }
    }*/
    
    // -------------------------------------------------------------
// 점프대
// 필요 없으면 이 블록 전체를 주석 처리
// -------------------------------------------------------------

    PxRigidStatic* jumpRamp =
        PxCreateStatic(
            *m_Physics,
            PxTransform(
                PxVec3(
                    -1.0f,
                    0.0f,
                    35.0f),
                PxQuat(
                    -20.0f *
                    PxPi /
                    180.0f,
                    PxVec3(
                        1.0f,
                        0.0f,
                        0.0f))),
            PxBoxGeometry(
                3.0f,
                0.5f,
                6.0f),
            *m_Material);

    if (!registerGroundObject(
        jumpRamp))
    {
        return false;
    }
    
    //큰 구 경사로를 표현

    /*PxRigidStatic* sphere =
        PxCreateStatic(
            *m_Physics,
            PxTransform(
                PxVec3(
                    0.0f,
                    -17.0f,
                    20.0f)),
            PxSphereGeometry(
                20.0f),
            *m_Material);

    if (!registerGroundObject(
        sphere))
    {
        return false;
    }*/



    return true;
}

bool PhysicsSystem::CreateChassis()
{
    using namespace physx;

    if (m_Physics == nullptr ||
        m_Scene == nullptr ||
        m_Material == nullptr)
    {
        return false;
    }

    // 기존 프로젝트의 초기 차체 pose로 교체
    m_InitialChassisPose =
        PxTransform(
            PxVec3(0.0f, m_WheelRadius, 0.0f));

    // 아래 생성 부분은 기존 프로젝트에서
    // 정상 동작하던 방식으로 교체한다.
    m_Chassis =
        PxCreateDynamic(
            *m_Physics,
            m_InitialChassisPose,
            PxBoxGeometry(
                m_ChassisHalfExtents),
            *m_Material,
            1.0f);

    PxRigidBodyExt::setMassAndUpdateInertia(*m_Chassis, m_ChassisMass);


    if (m_Chassis == nullptr)
    {
        return false;
    }



    if (m_Chassis->getNbShapes() == 0)
    {
        return false;
    }


    if (m_Chassis->getShapes(
        &m_ChassisShape,
        1) != 1)
    {
        return false;
    }
    

    PxShape* bodyShape = m_Physics->createShape(PxBoxGeometry(m_BodyHalfExtents), *m_Material);

    if (bodyShape == nullptr) {
        return false;
    }

    bodyShape->setLocalPose(m_BodyLocalPose);

    m_Chassis->attachShape(*bodyShape);

    m_BodyShape = bodyShape;

    bodyShape->release();

    PxRigidBodyExt::setMassAndUpdateInertia(*m_Chassis, m_ChassisMass);

    m_Scene->addActor(*m_Chassis);



    return true;
}

bool PhysicsSystem::InitializeWheels()
{

    if (m_Chassis == nullptr) {
        return false;
    }

    const std::array<PxVec3, kWheelCount>
        mountPositionsLocal =
    {
        // RearLeft
        PxVec3(
            -m_ChassisHalfExtents.x,
            0.0f,
            -m_ChassisHalfExtents.z),

        // RearRight
        PxVec3(
            +m_ChassisHalfExtents.x,
            0.0f,
            -m_ChassisHalfExtents.z),

        // FrontLeft
        PxVec3(
            -m_ChassisHalfExtents.x,
            0.0f,
            +m_ChassisHalfExtents.z),

        // FrontRight
        PxVec3(
            +m_ChassisHalfExtents.x,
            0.0f,
            +m_ChassisHalfExtents.z)
    };

    const PxTransform chassisPose =
        m_Chassis->getGlobalPose();

    for (std::size_t i = 0; i < kWheelCount; ++i) {

        m_Wheels[i] = WheelState{};


        m_Wheels[i].mountPositionLocal = mountPositionsLocal[i];
        m_Wheels[i].centerPositionWorld =
            chassisPose.transform(
                m_Wheels[i].mountPositionLocal);
    }

    return true;
}

bool PhysicsSystem::CreateVehicle()
{

    if (!CreateChassis())
    {
        return false;
    }

    if (!InitializeWheels()) {
        return false;
    }


    return true;
}

bool PhysicsSystem::Initialize()
{
    Shutdown();

    if (!InitializePhysX())
    {
        Shutdown();
        return false;
    }

    if (!CreateGround())
    {
        Shutdown();
        return false;
    }

    if (!CreateVehicle())
    {
        Shutdown();
        return false;
    }

    m_VehicleInput = {};
    m_ResetRequested = false;
    m_TargetSteeringAngle = 0.0f;

    return true;
}

void PhysicsSystem::SetVehicleInput(
    const VehicleInput& input)
{
    m_VehicleInput = input;

    if (input.reset)
    {
        m_ResetRequested = true;
    }
}

void PhysicsSystem::ApplySteering(
    float fixedDeltaTime)
{
    if (m_Chassis == nullptr ||
        fixedDeltaTime <= 0.0f)
    {
        return;
    }

    const PxTransform chassisPose =
        m_Chassis->getGlobalPose();

    const float chassisSpeed =
        m_Chassis->getLinearVelocity().magnitude();

    // 속도가 높을수록 최대 조향각을 줄인다.
    const float maxAllowedSteeringAngle =
        PxMin(
            GetMaxSteerAngleBySpeed(
                chassisSpeed),
            m_MaxSteeringAngle);

    /*
     * 기존 main.cpp:
     *
     * 왼쪽 키  -> steerInput = +1
     * 오른쪽 키 -> steerInput = -1
     *
     * 현재 VehicleInput:
     *
     * 왼쪽  -> steer = -1
     * 오른쪽 -> steer = +1
     *
     * 기존 코드와 동일한 방향을 유지하기 위해
     * 입력 부호를 반대로 적용한다.
     */
    const float requestedSteeringAngle =
        -static_cast<float>(
            m_VehicleInput.steer) *
        maxAllowedSteeringAngle;

    
    const float steeringSpeed =
        GetSteeringSpeedByVehicleSpeed(chassisSpeed*3.6f);

    // 키를 누르는 즉시 45도가 되는 것이 아니라,
    // 초당 m_SteeringSpeed만큼 점진적으로 움직인다.
    m_TargetSteeringAngle =
        MoveToward(
            m_TargetSteeringAngle,
            requestedSteeringAngle,
            steeringSpeed *
            fixedDeltaTime);

    // 속도가 올라가 최대 허용 조향각이 작아졌을 경우,
    // 기존 조향각도 새 제한 안으로 줄인다.
    m_TargetSteeringAngle =
        PxClamp(
            m_TargetSteeringAngle,
            -maxAllowedSteeringAngle,
            maxAllowedSteeringAngle);
    
    m_Wheels[FrontLeft].steeringAngle = m_TargetSteeringAngle;
    m_Wheels[FrontRight].steeringAngle = m_TargetSteeringAngle;


}

void PhysicsSystem::ApplyDrive(
    float fixedDeltaTime)
{
    if (m_Chassis == nullptr ||
        fixedDeltaTime <= 0.0f)
    {
        return;
    }

    const float driveInput = PxClamp(m_VehicleInput.drive, -1, 1);

    for (WheelState& wheel : m_Wheels) {
        wheel.driveTorque = 0.0f;
        wheel.coastTorque = 0.0f;
        wheel.brakeTorque = 0.0f;
    }

    //현재 차체 전방 속도

    const PxTransform chassisPose =
        m_Chassis->getGlobalPose();

    const PxVec3 chassisForward =
        chassisPose.q.rotate(
            PxVec3(0.0f, 0.0f, 1.0f));

    const float forwardSpeed =
        m_Chassis->getLinearVelocity()
        .dot(chassisForward);

    //최고속도 근처에서 구동 토크 줄이기

    float speedLimiterScale = 1.0f;

    const float inputEpsilon = 0.01f;

    const bool isBraking = PxAbs(driveInput) > inputEpsilon &&
        PxAbs(forwardSpeed) > m_StopSpeedThreshold &&
        forwardSpeed * driveInput < 0.0f;

    if (isBraking) {
        for (WheelState& wheel : m_Wheels) {
            wheel.brakeTorque = m_BrakeTorque;
        }
        return;
    }


    const float limiterStartRatio = 0.90f;

    if (PxAbs(driveInput) > inputEpsilon) {
        const float speedInDriveDirection = forwardSpeed * driveInput;

        const float limiterStartSpeed = m_MaxVehicleSpeed * limiterStartRatio;

        if (speedInDriveDirection > limiterStartSpeed) {
            const float limiterRange = m_MaxVehicleSpeed - limiterStartSpeed;
            const float limiterProgress = (speedInDriveDirection - limiterStartSpeed) / limiterRange;

            speedLimiterScale = 1.0f - PxClamp(limiterProgress, 0.0f, 1.0f);
        }
    }

    const float totalDriveTorque = driveInput * m_DriveTorque * speedLimiterScale;


    //torque vectoring
    
    //실제 차량 wheelbase/trackwidth

    const float wheelBase = m_ChassisHalfExtents.z * 2.0f;

    const float trackWidth = m_ChassisHalfExtents.x * 2.0f;

    //차체 up 방향
    const PxVec3 chassisUp =
        chassisPose.q.rotate(
            PxVec3(0.0f, 1.0f, 0.0f));
    
    //실제 앞바퀴 조향각
    const float actualSteerAngle =
        m_Wheels[FrontRight].steeringAngle;

    //조향각과 차량 속도로부터 목표 yaw rate 계산
    float targetYawRate = 0.0f;

    if (PxAbs(wheelBase) > 1.0e-4f) {
        targetYawRate =
            forwardSpeed *
            std::tan(actualSteerAngle) /
            wheelBase;
    }

    //실제 yaw rate
    const float actualYawRate =
        chassisUp.dot(
            m_Chassis->getAngularVelocity());

    //yaw rate 오차

    const float yawError =
        targetYawRate - actualYawRate;

    //실제 yaw angular acceleration
    const float yawAcceleration =
        chassisUp.dot(
            m_Chassis->getAngularAcceleration());

    //PD 제어기로 필요한 yaw moment 계산

    float torqueVectoringMoment =
        m_VectorKp * yawError -
        m_VectorKd * yawAcceleration;

    //drive 입력이 없을때는 torque vectoring으로 바퀴를 구동하지 않음
    if (PxAbs(driveInput) <= inputEpsilon) {
        torqueVectoringMoment = 0.0f;
    }

    //yawmoment를 좌우 바퀴의 구동 토크 차이로 변환
    //deltaT=Mz*wheelRadius/trackWidth

    float torqueVectoring = 0.0f;

    if (PxAbs(trackWidth) > 1.0e-4f) {
        torqueVectoring = torqueVectoringMoment * m_WheelRadius / trackWidth;
    }

    //앞/뒤 구동 토크 분배

    const float frontAxleTorque = totalDriveTorque * m_FrontDriveRate;

    const float rearAxleTorque = totalDriveTorque * (1.0f - m_FrontDriveRate);

    //앞/뒤 torque vectoring 분배

    const float frontVectorTorque = torqueVectoring * m_FrontYawMomentRate;

    const float rearVectorTorque = torqueVectoring * (1.0f - m_FrontYawMomentRate);

    //좌우 바퀴에 토크 전달

    m_Wheels[FrontLeft].driveTorque = frontAxleTorque * 0.5f-frontVectorTorque;

    m_Wheels[FrontRight].driveTorque = frontAxleTorque * 0.5f+frontVectorTorque;

    m_Wheels[RearLeft].driveTorque = rearAxleTorque * 0.5f-rearVectorTorque;

    m_Wheels[RearRight].driveTorque = rearAxleTorque * 0.5f+rearVectorTorque;

    //가속 입력이 없을때 약한 엔진 브레이크

    if (PxAbs(driveInput) <= inputEpsilon) {
        const float totalCoastTorque =
            m_DriveTorque *
            m_CoastTorqueRate;

        const float coastTorquePerWheel =
            totalCoastTorque /
            static_cast<float>(kWheelCount);

        for (WheelState& wheel : m_Wheels) {
            if (PxAbs(wheel.angularVelocity) > 0.01f) {
                if (wheel.angularVelocity > 0.0f) {
                    wheel.coastTorque =
                        -coastTorquePerWheel;
                }
                else {
                    wheel.coastTorque =
                        coastTorquePerWheel;
                }
                
            }
            else {
                wheel.coastTorque = 0.0f;
                wheel.angularVelocity = 0.0f;
            }
        }
    }

}

void PhysicsSystem::RayCasting() {

    PxTransform chassisPose = m_Chassis->getGlobalPose();
    PxVec3 chassisDown = chassisPose.q.rotate(PxVec3(0.0f, -1.0f, 0.0f));
    

    for (std::size_t i = 0; i < kWheelCount; ++i) {

        WheelState *wheel = &m_Wheels[i];

        PxVec3 mountPositionWorld = chassisPose.transform(wheel->mountPositionLocal);

        physx::PxRaycastBuffer hitBuffer;

        bool hasHit = m_Scene->raycast(
            mountPositionWorld,
            chassisDown,
            m_maxRaycastDistance,
            hitBuffer,
            PxHitFlag::ePOSITION|PxHitFlag::eNORMAL,
            PxQueryFilterData(PxQueryFlag::eSTATIC)
        );

        if (hasHit && hitBuffer.hasBlock) {

            wheel->isGrounded = true;

            const physx::PxRaycastHit& hit = hitBuffer.block;
            wheel->contactPointWorld = hit.position;
            wheel->contactNormalWorld = hit.normal;

            wheel->suspensionLength =
                -PxClamp(
                    hit.distance - m_WheelRadius,
                    -m_SuspensionMaxLength,
                    m_SuspensionMaxLength
                );
        }
        else {
            wheel->isGrounded = false;
            wheel->suspensionLength = m_SuspensionRestLengthDelta;
        }
    }


}
/*
void PhysicsSystem::CylinderSweep() {

    PxTransform chassisPose = m_Chassis->getGlobalPose();
    PxVec3 chassisDown = chassisPose.q.rotate(PxVec3(0.0f, -1.0f, 0.0f));

    const PxConvexCore::Cylinder wheelCylinderCore(
        m_WheelHalfWidth,
        m_WheelRadius);

    const PxConvexCoreGeometry wheelCylinderGeometry(
        wheelCylinderCore);


    for (std::size_t i = 0; i < kWheelCount; ++i) {

        WheelState* wheel = &m_Wheels[i];

        const PxVec3 mountPositionWorld = chassisPose.transform(wheel->mountPositionLocal);

        const PxQuat steeringRotation(wheel->steeringAngle, PxVec3(0.0f, 1.0f, 0.0f));

        const PxTransform sweepStartPose(mountPositionWorld - chassisDown * m_maxCylinderSweepDistance, chassisPose.q * steeringRotation);

        physx::PxSweepBuffer hitBuffer;

        bool hasHit = m_Scene->sweep(
            wheelCylinderGeometry,
            sweepStartPose,
            chassisDown,
            m_maxCylinderSweepDistance * 2,
            hitBuffer,
            PxHitFlag::ePOSITION | PxHitFlag::eNORMAL,
            PxQueryFilterData(PxQueryFlag::eSTATIC)
        );

        if (hasHit && hitBuffer.hasBlock) {

            wheel->isGrounded = true;

            const physx::PxSweepHit& hit = hitBuffer.block;

            wheel->contactPointWorld = hit.position;
            wheel->contactNormalWorld = hit.normal;

            float expectsuspensionLength = PxClamp(
                m_maxCylinderSweepDistance - hit.distance,
                -m_SuspensionMaxLength,
                m_SuspensionMaxLength
            );


            if (expectsuspensionLength < wheel->suspensionLength - 0.03) {
                if (wheel->suspensionLength > -m_SuspensionRestLengthDelta + 0.03) {
                    wheel->suspensionLength -= 0.03;
                }
                else if (wheel->suspensionLength < -m_SuspensionRestLengthDelta - 0.03) {
                    wheel->suspensionLength += 0.03;
                }
                else {
                    wheel->suspensionLength = -m_SuspensionRestLengthDelta;
                }

                if (wheel->suspensionLength <= expectsuspensionLength) {
                    wheel->suspensionLength = expectsuspensionLength;
                }
                else {
                    wheel->isGrounded = false;
                }
            }
            else {
                wheel->suspensionLength = expectsuspensionLength;
            }

            if (wheel->suspensionLength == m_SuspensionMaxLength) {
                wheel->isSuspensionLimited = true;
                wheel->suspensionExcess = m_maxCylinderSweepDistance - hit.distance - m_SuspensionMaxLength;
            }
            else {
                wheel->isSuspensionLimited = false;
                wheel->suspensionExcess = 0.0f;
                wheel->directLoadtoChassis = 0.0f;
            }


        }
        else {
            wheel->isGrounded = false;
            wheel->isSuspensionLimited = false;
            wheel->suspensionExcess = 0.0f;
            wheel->directLoadtoChassis = 0.0f;
            if (wheel->suspensionLength > -m_SuspensionRestLengthDelta + 0.03) {
                wheel->suspensionLength -= 0.03;
            }
            else if (wheel->suspensionLength < -m_SuspensionRestLengthDelta - 0.03) {
                wheel->suspensionLength += 0.03;
            }
            else {
                wheel->suspensionLength = -m_SuspensionRestLengthDelta;
            }
        }



    }

}

void PhysicsSystem::CalculateNormalLoads(
    float fixedDeltaTime) {

    const PxTransform chassisPose = m_Chassis->getGlobalPose();

    const PxVec3 ChassisUpWorld = chassisPose.q.rotate(PxVec3(0.0f, 1.0f, 0.0f));

    for (std::size_t i = 0; i < kWheelCount; ++i) {

        WheelState& wheel = m_Wheels[i];

        if (wheel.isGrounded) {
            wheel.compressionVelocity = -(wheel.suspensionLength - wheel.prevsuspensionLength) / fixedDeltaTime;

            //wheel.locallinearVelocity = PxRigidBodyExt::getLocalVelocityAtLocalPos(*m_Chassis, wheel.mountPositionLocal);
            //wheel.compressionVelocity = wheel.locallinearVelocity.y;

            wheel.suspensionForce = m_staticnormalLoad + m_SuspensionSpringStrength * wheel.suspensionLength - m_SuspensionDamperRate * wheel.compressionVelocity;
            if (wheel.isSuspensionLimited) {
                wheel.directLoadtoChassis = 10 * m_SuspensionSpringStrength * wheel.suspensionExcess;

                wheel.suspensionForce += wheel.directLoadtoChassis;
            }
            wheel.normalLoad = PxClamp(wheel.suspensionForce, 0.0f, m_MaxSuspensionForce) * PxMax(ChassisUpWorld.dot(wheel.contactNormalWorld), 0.0f);
        }
        else {
            wheel.suspensionForce = 0.0f;
            wheel.normalLoad = 0.0f;
        }

        wheel.prevsuspensionLength = wheel.suspensionLength;

    }
}*/

void PhysicsSystem::CylinderSweep(
    float fixedDeltaTime) {

    m_HasZeroDistanceSweep = false;
    PxTransform chassisPose = m_Chassis->getGlobalPose();
    PxVec3 chassisDown = chassisPose.q.rotate(PxVec3(0.0f, -1.0f, 0.0f));
    const PxVec3 chassisUpWorld = chassisPose.q.rotate(PxVec3(0.0f, 1.0f, 0.0f));

    const PxConvexCore::Cylinder wheelCylinderCore(
        m_WheelHalfWidth,
        m_WheelRadius);

    const PxConvexCoreGeometry wheelCylinderGeometry(
        wheelCylinderCore);


    for (std::size_t i = 0; i < kWheelCount; ++i) {

        WheelState* wheel = &m_Wheels[i];

        const PxVec3 mountPositionWorld = chassisPose.transform(wheel->mountPositionLocal);

        const PxQuat steeringRotation(wheel->steeringAngle, PxVec3(0.0f, 1.0f, 0.0f));

        const PxTransform sweepStartPose(mountPositionWorld - chassisDown * m_maxCylinderSweepDistance, chassisPose.q * steeringRotation);

        physx::PxSweepBuffer hitBuffer;

        bool hasHit = m_Scene->sweep(
            wheelCylinderGeometry,
            sweepStartPose,
            chassisDown,
            m_maxCylinderSweepDistance * 2,
            hitBuffer,
            PxHitFlag::ePOSITION | PxHitFlag::eNORMAL,
            PxQueryFilterData(PxQueryFlag::eSTATIC)
        );

        if (hasHit && hitBuffer.hasBlock) {

            wheel->isGrounded = true;

            const physx::PxSweepHit& hit = hitBuffer.block;

            wheel->contactPointWorld = hit.position;
            wheel->contactNormalWorld = hit.normal;

            DebugPrint("%.3f, %.3f, %.3f: normal, %.3f, %.3f, %.3f: chassisup\n", hit.normal.x, hit.normal.y, hit.normal.z, chassisUpWorld.x, chassisUpWorld.y, chassisUpWorld.z);

            if (hit.distance == 0) {

                m_HasZeroDistanceSweep = true;
                wheel->isGrounded = false;
                wheel->isSuspensionLimited = false;
                wheel->suspensionExcess = 0.0f;
                wheel->directLoadtoChassis = 0.0f;
                if (wheel->suspensionLength > -m_SuspensionRestLengthDelta + 0.03) {
                    wheel->suspensionLength -= 0.03;
                }
                else if (wheel->suspensionLength < -m_SuspensionRestLengthDelta - 0.03) {
                    wheel->suspensionLength += 0.03;
                }
                else {
                    wheel->suspensionLength = -m_SuspensionRestLengthDelta;
                }
            }
            else {
                float expectsuspensionLength = PxClamp(
                    m_maxCylinderSweepDistance - hit.distance,
                    -m_SuspensionMaxLength,
                    m_SuspensionMaxLength
                );


                if (expectsuspensionLength < wheel->suspensionLength - 0.03) {
                    if (wheel->suspensionLength > -m_SuspensionRestLengthDelta + 0.03) {
                        wheel->suspensionLength -= 0.03;
                    }
                    else if (wheel->suspensionLength < -m_SuspensionRestLengthDelta - 0.03) {
                        wheel->suspensionLength += 0.03;
                    }
                    else {
                        wheel->suspensionLength = -m_SuspensionRestLengthDelta;
                    }

                    if (wheel->suspensionLength <= expectsuspensionLength) {
                        wheel->suspensionLength = expectsuspensionLength;
                    }
                    else {
                        wheel->isGrounded = false;
                    }
                }
                else {
                    wheel->suspensionLength = expectsuspensionLength;
                }

                if (wheel->suspensionLength == m_SuspensionMaxLength) {
                    wheel->isSuspensionLimited = true;
                    wheel->suspensionExcess = m_maxCylinderSweepDistance - hit.distance - m_SuspensionMaxLength;
                }
                else {
                    wheel->isSuspensionLimited = false;
                    wheel->suspensionExcess = 0.0f;
                    wheel->directLoadtoChassis = 0.0f;
                }
            }



        }
        else {
            wheel->isGrounded = false;
            wheel->isSuspensionLimited = false;
            wheel->suspensionExcess = 0.0f;
            wheel->directLoadtoChassis = 0.0f;
            if (wheel->suspensionLength > -m_SuspensionRestLengthDelta + 0.03) {
                wheel->suspensionLength -= 0.03;
            }
            else if (wheel->suspensionLength < -m_SuspensionRestLengthDelta - 0.03) {
                wheel->suspensionLength += 0.03;
            }
            else {
                wheel->suspensionLength = -m_SuspensionRestLengthDelta;
            }
        }



    }

}

void PhysicsSystem::CalculateNormalLoads(
    float fixedDeltaTime) {

    const PxTransform chassisPose = m_Chassis->getGlobalPose();

    const PxVec3 ChassisUpWorld = chassisPose.q.rotate(PxVec3(0.0f, 1.0f, 0.0f));

    for (std::size_t i = 0; i < kWheelCount; ++i) {

        WheelState& wheel = m_Wheels[i];

        if (wheel.isGrounded) {
            wheel.compressionVelocity = (wheel.suspensionLength - wheel.prevsuspensionLength) / fixedDeltaTime;

            //wheel.locallinearVelocity = PxRigidBodyExt::getLocalVelocityAtLocalPos(*m_Chassis, wheel.mountPositionLocal);
            //wheel.compressionVelocity = wheel.locallinearVelocity.y;

            wheel.suspensionForce = m_staticnormalLoad + m_SuspensionSpringStrength * wheel.suspensionLength + m_SuspensionDamperRate * wheel.compressionVelocity;
            if (wheel.isSuspensionLimited) {
                const float excessVelocity = (wheel.suspensionExcess - wheel.prevSuspensionExcess) / fixedDeltaTime;

                const float bumpCompressionVelocity = PxMax(excessVelocity, 0.0f);

                const float bumpSpringForce = m_BumpStopStrength * wheel.suspensionExcess;

                const float bumpDamperForce = m_BumpStopDamperRate * bumpCompressionVelocity;

                wheel.directLoadtoChassis = bumpSpringForce + bumpDamperForce;

                wheel.directLoadtoChassis = PxClamp(wheel.directLoadtoChassis, 0.0f, m_MaxBumpStopForce);

                wheel.suspensionForce += wheel.directLoadtoChassis;
            }
            wheel.normalLoad = PxClamp(wheel.suspensionForce, 0.0f, m_MaxSuspensionForce) * PxMax(ChassisUpWorld.dot(wheel.contactNormalWorld), 0.0f);
        }
        else {
            wheel.suspensionForce = 0.0f;
            wheel.normalLoad = 0.0f;
            wheel.directLoadtoChassis = 0.0f;
        }

        wheel.prevsuspensionLength = wheel.suspensionLength;
        wheel.prevSuspensionExcess = wheel.suspensionExcess;

    }
}

void PhysicsSystem::CalculateAntiRollForces() {

    if (m_Chassis == nullptr) {
        return;
    }

    for (WheelState& wheel : m_Wheels) {
        wheel.antiRollForce = 0.0f;
    }

    //rear anti roll bar

    WheelState& rearLeft = m_Wheels[RearLeft];

    WheelState& rearRight = m_Wheels[RearRight];

    if (rearLeft.isGrounded && rearRight.isGrounded) {
        const float suspensionDifference = rearRight.suspensionLength - rearLeft.suspensionLength;

        const float antiRollForce = m_AntiRollStiffness * suspensionDifference;

        rearLeft.antiRollForce = -antiRollForce;

        rearRight.antiRollForce = +antiRollForce;
    }

    //front anti roll bar

    WheelState& frontLeft = m_Wheels[FrontLeft];
    WheelState& frontRight = m_Wheels[FrontRight];

    if (frontLeft.isGrounded && frontRight.isGrounded) {
        const float suspensionDifference = frontRight.suspensionLength - frontLeft.suspensionLength;

        const float antiRollForce = m_AntiRollStiffness * suspensionDifference;

        frontLeft.antiRollForce = -antiRollForce;

        frontRight.antiRollForce = +antiRollForce;
    }

    //anti roll bar 반영된 normal load 계산

    const PxTransform chassisPose = m_Chassis->getGlobalPose();

    const PxVec3 chassisUpWorld = chassisPose.q.rotate(PxVec3(0.0f, 1.0f, 0.0f));

    for (WheelState& wheel : m_Wheels) {
        if (!wheel.isGrounded) {
            wheel.normalLoad = 0.0f;
            continue;
        }

        const float totalSuspensionForce = wheel.suspensionForce + wheel.antiRollForce;

        wheel.normalLoad = PxClamp(totalSuspensionForce, 0.0f, m_MaxSuspensionForce) * PxMax(chassisUpWorld.dot(wheel.contactNormalWorld), 0.0f);
    }

}

void PhysicsSystem::CalculateTireForces(
    float fixedDeltaTime) {

    const PxTransform chassisPose = m_Chassis->getGlobalPose();

    for (std::size_t i = 0; i < kWheelCount; ++i) {
        WheelState& wheel = m_Wheels[i];

        wheel.longitudinalSlip = 0.0f;
        wheel.slipAngle = 0.0f;

        wheel.longitudinalForce = 0.0f;
        wheel.lateralForce = 0.0f;

        //접지 하지 않아 지면으로부터 힘을 받지 않는경우
        if (!wheel.isGrounded || wheel.normalLoad <= 0.0f) continue;

        const PxQuat steeringRotation(wheel.steeringAngle, PxVec3(0.0f, 1.0f, 0.0f));

        const PxVec3 tireForwardLocal = steeringRotation.rotate(PxVec3(0.0f, 0.0f, 1.0f));

        PxVec3 tireForwardWorld = chassisPose.q.rotate(tireForwardLocal);

        //Fx, Fz 방향 구하기
        tireForwardWorld -= wheel.contactNormalWorld * tireForwardWorld.dot(wheel.contactNormalWorld);

        tireForwardWorld.normalize();

        wheel.longitudinalDirectionWorld = tireForwardWorld;

        PxVec3 tireRightWorld = wheel.contactNormalWorld.cross(tireForwardWorld);

        tireRightWorld.normalize();

        wheel.lateralDirectionWorld = tireRightWorld;

        //바퀴 중심의 속도(바퀴의 병진속도) 좌표계에 투영하기
        const PxVec3 wheelCenterLocal = wheel.mountPositionLocal + PxVec3(0.0f, wheel.suspensionLength, 0.0f);

        wheel.centerPositionWorld = chassisPose.transform(wheelCenterLocal);

        const PxVec3 wheelCenterVelocityWorld = PxRigidBodyExt::getVelocityAtPos(*m_Chassis, wheel.centerPositionWorld);


        //바퀴의 접촉점 좌표계에서의 속도 계산, 접촉점에서의 바퀴 속도 계산
        const float longitudinalSpeed = wheelCenterVelocityWorld.dot(tireForwardWorld);

        const float lateralSpeed = wheelCenterVelocityWorld.dot(tireRightWorld);

        const float wheelSurfaceSpeed = wheel.angularVelocity * m_WheelRadius;


        //슬립비 계산
        const float slipDenominator = PxMax(std::abs(longitudinalSpeed), m_MinSlipSpeed);

        wheel.longitudinalSlip = (wheelSurfaceSpeed - longitudinalSpeed) / slipDenominator;

        wheel.longitudinalSlip = PxClamp(wheel.longitudinalSlip, -m_MaxSlipRatio, m_MaxSlipRatio);


        //슬립각 계산
        const float slipAngleDenominator = PxMax(std::abs(longitudinalSpeed), m_MinSlipSpeed);

        wheel.slipAngle = std::atan2(lateralSpeed, slipAngleDenominator);

        wheel.slipAngle = PxClamp(wheel.slipAngle, -m_MaxSlipAngle, m_MaxSlipAngle);

        //종력, 횡력 계산

        //선형 모델

        //float longitudinalForce = m_LongitudinalStiffness * wheel.longitudinalSlip;
       
        //float lateralForce = -m_LateralStiffness * wheel.slipAngle;

        //pacejka 곡선 선형 근사 모델

        float longitudinalForce = longitudinalModeling(wheel.longitudinalSlip,5)*wheel.normalLoad;

        float lateralForce = lateralModeling(wheel.slipAngle,5)*wheel.normalLoad;

        //마찰제한

        const float maximumTireForce = m_TireFrictionCoefficient * wheel.normalLoad;

        const float forceMagnitudeSquared = longitudinalForce * longitudinalForce + lateralForce * lateralForce;

        const float maximumForceSquared = maximumTireForce * maximumTireForce;

        if (forceMagnitudeSquared > maximumForceSquared && forceMagnitudeSquared > 1.0e-8f) {
            const float forceMagnitude = std::sqrt(forceMagnitudeSquared);

            const float forceScale = maximumTireForce / forceMagnitude;
            
            longitudinalForce *= forceScale;
            
            lateralForce *= forceScale;

        }

        wheel.longitudinalForce = longitudinalForce;

        wheel.lateralForce = lateralForce;

        //저속 정지 처리

        if (PxAbs(m_VehicleInput.drive) <= 0.05f) {
            const PxTransform pose = m_Chassis->getGlobalPose();

            PxVec3 forward = pose.q.rotate(PxVec3(0.0f, 0.0f, 1.0f));

            forward.y = 0.0f;

            if (forward.normalize()) {
                PxVec3 velocity = m_Chassis->getLinearVelocity();

                const float forwardSpeed = velocity.dot(forward);

                if (PxAbs(forwardSpeed) < 0.1f) {
                    velocity -= forward * forwardSpeed;

                    m_Chassis->setLinearVelocity(velocity);

                    for (WheelState& wheel : m_Wheels) {
                        wheel.angularVelocity = 0.0f;
                        wheel.longitudinalSlip = 0.0f;
                        wheel.longitudinalForce = 0.0f;
                        wheel.coastTorque = 0.0f;
                    }
                }
            }
        }

        DebugPrint(
            "[%zu] %.3f degree, %.3f longi, %.3f later, %.3f normal, %.3f sus\n",
            i,
            wheel.slipAngle * 180.0f / PxPi,
            longitudinalForce,
            lateralForce,
            wheel.normalLoad,
            wheel.suspensionForce);

    }
}

void PhysicsSystem::UpdateWheelAngularDynamics(
    float fixedDeltaTime) {

    if (m_Chassis == nullptr ||
        fixedDeltaTime <= 0.0f ||
        m_WheelMomentOfInertia <= 0.0f)
    {
        return;
    }
    
    const float angularVelocityEpsilon = 0.01f;

    const PxTransform chassisPose =
        m_Chassis->getGlobalPose();

    const PxVec3 chassisForward =
        chassisPose.q.rotate(
            PxVec3(0.0f, 0.0f, 1.0f));

    const float vehicleSpeed =
        PxAbs(
            m_Chassis->getLinearVelocity()
            .dot(chassisForward));

    for (WheelState& wheel : m_Wheels) {

        //1. 타이어 종력의 바퀴 반작용 토크
        const float tireReactionTorque = -wheel.longitudinalForce * m_WheelRadius;

        //2. 바퀴 회전 저항 토크
        const float resistanceTorque = -m_WheelMomentOfInertia * m_WheelAngularResistance * wheel.angularVelocity;

        //3. coastTorque가 바퀴회전을 반대방향으로 만들지 않도록 제한

        float coastTorque = wheel.coastTorque;

        if (PxAbs(wheel.angularVelocity) > angularVelocityEpsilon) {
            const float maximumStoppingTorque = PxAbs(wheel.angularVelocity) * m_WheelMomentOfInertia / fixedDeltaTime;

            coastTorque = PxClamp(coastTorque, -maximumStoppingTorque, maximumStoppingTorque);
        }
        else {
            coastTorque = 0.0f;
        }

        //+마찬가지로 brakeTorque가 바퀴회전을 반대방향으로 만들지 않도록 제한
        //했었지만 brake는 lock이 걸릴 수 있어야함
        /*float limitedBrakeTorque = wheel.brakeTorque;

        if (PxAbs(wheel.angularVelocity) > angularVelocityEpsilon) {
            const float maximumStoppingTorque = PxAbs(wheel.angularVelocity) * m_WheelMomentOfInertia / fixedDeltaTime;

            limitedBrakeTorque = PxClamp(limitedBrakeTorque, -maximumStoppingTorque, maximumStoppingTorque);
        }
        else {
            limitedBrakeTorque = 0.0f;
        }*/

        //3. ABS

        if (wheel.brakeTorque > 0.0f && wheel.isGrounded && vehicleSpeed > m_ABSMinSpeed) {
            const float brakingSlip = PxAbs(wheel.longitudinalSlip);

            //wheel lock 방향 -> 빠르게 감압
            if (brakingSlip > m_ABSReleaseSlip) {
                wheel.absBrakeScale -= m_ABSReleaseRate * fixedDeltaTime;
            }
            //wheel 회복 -> 다시 가압
            else if (brakingSlip < m_ABSReapplySlip) {
                wheel.absBrakeScale += m_ABSReapplyRate * fixedDeltaTime;
            }

            //중간 영역에서는 현재 압력 유지
            wheel.absBrakeScale = PxClamp(wheel.absBrakeScale, 0.0f, 1.0f);

        }
        else {
            wheel.absBrakeScale = 1.0f;
        }

        wheel.absBrakeScale = 1.0f;

        //4. Brake를 제외한 모든 토크
        const float torqueWithoutBrake = wheel.driveTorque + coastTorque + tireReactionTorque + resistanceTorque;

        //5. 실제 brake torque 크기
        const float brakeTorqueMagnitude = wheel.brakeTorque * wheel.absBrakeScale;

        float actualBrakeTorque = 0.0f;

        //6. Brake는 항상 현재 회전을 방해

        if (brakeTorqueMagnitude > 0.0f) {
            if (wheel.angularVelocity > 0.0f) {
                actualBrakeTorque = -brakeTorqueMagnitude;
            }
            else if (wheel.angularVelocity < -angularVelocityEpsilon) {
                actualBrakeTorque = brakeTorqueMagnitude;
            }
            else {
                //wheel이 이미 정지한 경우
                //brake가 다른 모든 torque를 버틸 수 있다면
                //wheel lock 상태 유지

                if (PxAbs(torqueWithoutBrake) <= brakeTorqueMagnitude) {
                    wheel.angularVelocity = 0.0f;

                    continue;
                }

                //외부 torque가 brake보다 강하면 그 torque의 반대 방향으로 brake 작용

                if (torqueWithoutBrake > 0.0f) {
                    actualBrakeTorque = -brakeTorqueMagnitude;
                }
                else {
                    actualBrakeTorque = brakeTorqueMagnitude;
                }
            }
        }

        //7. 그후 모든 torque를 동등하게 합산

        const float totalTorque = torqueWithoutBrake + actualBrakeTorque;

        const float angularAcceleration = totalTorque / m_WheelMomentOfInertia;

        const float previousAngularVelocity = wheel.angularVelocity;

        float nextAngularVelocity = previousAngularVelocity + angularAcceleration * fixedDeltaTime;

        //8.Brake 때문에 0을 통과하려는 경우
        //brake가 다른 torque를 버틸 수 있으면
        //역회전이 아니라 wheel lock

        if (brakeTorqueMagnitude > 0.0f && previousAngularVelocity * nextAngularVelocity < 0.0f && PxAbs(torqueWithoutBrake) <= brakeTorqueMagnitude) {
            nextAngularVelocity = 0.0f;
        }

        //9. 안전제한, 실제 제한은 vehicle speed로 이미 제한하나 안정장치
        const float safetyAngularVelocity = m_MaxWheelAngularVelocity * 1.2f;

        wheel.angularVelocity = PxClamp(nextAngularVelocity, -safetyAngularVelocity, safetyAngularVelocity);

    }
}

void PhysicsSystem::ApplyWheelForces() {
    const PxTransform chassisPose = m_Chassis->getGlobalPose();

    const PxVec3 chassisUpWorld = chassisPose.q.rotate(PxVec3(0.0f, 1.0f, 0.0f));

    for (const WheelState& wheel : m_Wheels) {

        const PxVec3 mountPositionWorld = chassisPose.transform(wheel.mountPositionLocal);

        const PxVec3 suspensionForceWorld = chassisUpWorld * (wheel.suspensionForce+wheel.antiRollForce+wheel.recoveryForce);

        PxRigidBodyExt::addForceAtPos(*m_Chassis, suspensionForceWorld, mountPositionWorld, PxForceMode::eFORCE);

        if (!wheel.isGrounded) {
            continue;
        }
        
        const PxVec3 longitudinalForceWorld = wheel.longitudinalDirectionWorld * wheel.longitudinalForce;
        const PxVec3 lateralForceWorld = wheel.lateralDirectionWorld * wheel.lateralForce;

        const PxVec3 totalTireForceWorld = longitudinalForceWorld +lateralForceWorld;

        //PxRigidBodyExt::addForceAtPos(*m_Chassis, totalTireForceWorld, wheel.centerPositionWorld, PxForceMode::eFORCE);

        PxRigidBodyExt::addForceAtPos(*m_Chassis, totalTireForceWorld, wheel.contactPointWorld, PxForceMode::eFORCE);
    }
}

void PhysicsSystem::CalculateOverturnRecovery() {
    if (m_Chassis == nullptr) {
        return;
    }

    for (WheelState& wheel : m_Wheels) {
        wheel.recoveryForce = 0.0f;
    }

    const PxTransform chassisPose = m_Chassis->getGlobalPose();

    const PxVec3 worldUp(0.0f, 1.0f, 0.0f);

    const PxVec3 chassisUp = chassisPose.q.rotate(PxVec3(0.0f, 1.0f, 0.0f));

    const PxVec3 chassisForward = chassisPose.q.rotate(PxVec3(0.0f, 0.0f, 1.0f));

    const float upDot = PxClamp(chassisUp.dot(worldUp), -1.0f, 1.0f);

    //chassis up과 world up 사이의 각도
    const float tiltAngle = std::acos(upDot);

    if (tiltAngle <= m_RecoveryStartAngle) {
        return;
    }

    float recoveryRatio = (tiltAngle - m_RecoveryStartAngle) / (m_RecoveryFullAngle - m_RecoveryStartAngle);

    recoveryRatio = PxClamp(recoveryRatio, 0.0f, 1.0f);

    recoveryRatio = recoveryRatio * recoveryRatio * (3.0f - 2.0f * recoveryRatio);


    //chassisUp와 worldUp 사이의 회전 오차 벡터값

    PxVec3 errorAxis = chassisUp.cross(worldUp);

    const float axisLength = errorAxis.magnitude();

    if (axisLength > 1.0e-5f) {
        errorAxis /= axisLength;
    }
    else {
        //완전히 뒤집힌 180도 상태에서는 crossproduct로 방향을 정할수 없으므로 roll 방향을 기본 복원축으로 사용
        if (upDot > 0.0f) return;

        errorAxis = chassisForward;
    }

    const PxVec3 orientationErrorWorld = errorAxis * tiltAngle;

    //world->chassis local
    const PxQuat inverseRotation = chassisPose.q.getConjugate();

    const PxVec3 orientationErrorLocal = inverseRotation.rotate(orientationErrorWorld);

    const PxVec3 angularVelocityLocal = inverseRotation.rotate(m_Chassis->getAngularVelocity());

    //roll+pitch PD 계산
    float pitchAcceleration = m_RecoveryKp *
        orientationErrorLocal.x -
        m_RecoveryKd *
        angularVelocityLocal.x;

    float rollAcceleration = m_RecoveryKp *
        orientationErrorLocal.z -
        m_RecoveryKd *
        angularVelocityLocal.z;

    pitchAcceleration *= recoveryRatio;
    rollAcceleration *= recoveryRatio;

    pitchAcceleration = PxClamp(pitchAcceleration, -m_MaxRecoveryAngularAcceleration, m_MaxRecoveryAngularAcceleration);

    rollAcceleration = PxClamp(rollAcceleration, -m_MaxRecoveryAngularAcceleration, m_MaxRecoveryAngularAcceleration);

    const PxVec3 inertia = m_Chassis->getMassSpaceInertiaTensor();

    const float pitchTorque = inertia.x * pitchAcceleration;

    const float rollTorque = inertia.z * rollAcceleration;

    //Torque->wheel force
    const float pitchForce = pitchTorque / (4.0f * m_ChassisHalfExtents.z);

    const float rollForce = rollTorque / (4.0f * m_ChassisHalfExtents.x);

    m_Wheels[RearLeft].recoveryForce = pitchForce - rollForce;
    m_Wheels[RearRight].recoveryForce = pitchForce + rollForce;
    m_Wheels[FrontLeft].recoveryForce = -pitchForce - rollForce;
    m_Wheels[FrontRight].recoveryForce = -pitchForce + rollForce;

    for (WheelState& wheel : m_Wheels) {
        wheel.recoveryForce = PxClamp(wheel.recoveryForce, -m_MaxRecoveryForcePerWheel, m_MaxRecoveryForcePerWheel);
    }

}

void PhysicsSystem::UpdateWheelRotation(
    float fixedDeltaTime) {

    const PxTransform chassisPose = m_Chassis->getGlobalPose();

    for (WheelState& wheel : m_Wheels) {
        if (fixedDeltaTime > 0.0f) {
            wheel.rotationAngle += wheel.angularVelocity * fixedDeltaTime;

            wheel.rotationAngle = std::fmod(wheel.rotationAngle, PxTwoPi);

        }

        const PxVec3 wheelCenterLocal = wheel.mountPositionLocal + PxVec3(0.0f, wheel.suspensionLength, 0.0f);

        wheel.centerPositionWorld = chassisPose.transform(wheelCenterLocal);


        const PxQuat steeringRotation(wheel.steeringAngle, PxVec3(0.0f, 1.0f, 0.0f));

        const PxQuat rollingRotation(wheel.rotationAngle, PxVec3(1.0f, 0.0f, 0.0f));

        PxQuat wheelRotationWorld = chassisPose.q * steeringRotation * rollingRotation;

        wheel.visualPoseWorld =
            PxTransform(
                wheel.centerPositionWorld,
                wheelRotationWorld);
    }
}

void PhysicsSystem::ApplyVehicleControl(
    float fixedDeltaTime)
{
    if (m_Chassis == nullptr)
    {
        return;
    }

    ApplySteering(
        fixedDeltaTime);

    ApplyDrive(
        fixedDeltaTime);

    //RayCasting();

    CylinderSweep(fixedDeltaTime);

    CalculateNormalLoads(fixedDeltaTime);

    CalculateAntiRollForces();
    
    CalculateTireForces(fixedDeltaTime);

    UpdateWheelAngularDynamics(fixedDeltaTime);

    CalculateOverturnRecovery();

    ApplyWheelForces();

    
   

}

void PhysicsSystem::ResetRigidBodyState(
    physx::PxRigidDynamic* body,
    const physx::PxTransform& pose)
{
    if (body == nullptr)
    {
        return;
    }

    body->setGlobalPose(
        pose);

    body->setLinearVelocity(
        physx::PxVec3(0.0f));

    body->setAngularVelocity(
        physx::PxVec3(0.0f));

    body->clearForce();
    body->clearTorque();

    body->wakeUp();
}

void PhysicsSystem::ResetVehicle()
{
    ResetRigidBodyState(
        m_Chassis,
        m_InitialChassisPose);

    for (WheelState& wheel : m_Wheels) {
        wheel.angularVelocity = 0.0f;
        wheel.rotationAngle = 0.0f;
        wheel.longitudinalSlip = 0.0f;
        wheel.slipAngle = 0.0f;
        wheel.longitudinalForce = 0.0f;
        wheel.lateralForce = 0.0f;
        wheel.driveTorque = 0.0f;
        wheel.coastTorque = 0.0f;
        wheel.brakeTorque = 0.0f;
        wheel.absBrakeScale = 1.0f;
        wheel.prevsuspensionLength = wheel.suspensionLength;
    }

    m_TargetSteeringAngle =
        0.0f;
}

void PhysicsSystem::Step(
    float fixedDeltaTime)
{
    if (m_Scene == nullptr ||
        fixedDeltaTime <= 0.0f)
    {
        return;
    }

    if (m_ResetRequested)
    {
        ResetVehicle();
        m_ResetRequested = false;
    }

    ApplyVehicleControl(
        fixedDeltaTime);

    m_Scene->simulate(
        fixedDeltaTime);

    m_Scene->fetchResults(
        true);

    UpdateWheelRotation(fixedDeltaTime);
}

void PhysicsSystem::Shutdown()
{
    if (m_Chassis != nullptr)
    {
        m_Chassis->release();
        m_Chassis = nullptr;
    }

    m_ChassisShape = nullptr;

    // ---------------------------------------------------------
// 추가 지형
// ---------------------------------------------------------

    for (GroundObject& object :
        m_AdditionalGroundObjects)
    {
        if (object.actor != nullptr)
        {
            object.actor->release();
            object.actor = nullptr;
        }

        // Shape는 Actor가 소유하므로 별도로 release하지 않는다.
        object.shape = nullptr;
    }

    m_AdditionalGroundObjects.clear();

    // ---------------------------------------------------------
    // Ground
    // ---------------------------------------------------------

    if (m_Ground != nullptr)
    {
        m_Ground->release();
        m_Ground = nullptr;
    }

    m_GroundShape = nullptr;

    // ---------------------------------------------------------
    // Material / Scene
    // ---------------------------------------------------------

    if (m_Material != nullptr)
    {
        m_Material->release();
        m_Material = nullptr;
    }

    if (m_Scene != nullptr)
    {
        m_Scene->release();
        m_Scene = nullptr;
    }

    if (m_Dispatcher != nullptr)
    {
        m_Dispatcher->release();
        m_Dispatcher = nullptr;
    }

    // ---------------------------------------------------------
    // Extensions
    // ---------------------------------------------------------

    if (m_ExtensionsInitialized)
    {
        PxCloseExtensions();
        m_ExtensionsInitialized = false;
    }

    // ---------------------------------------------------------
    // Physics / PVD / Foundation
    // ---------------------------------------------------------

    if (m_Physics != nullptr)
    {
        m_Physics->release();
        m_Physics = nullptr;
    }

    if (m_Pvd != nullptr)
    {
        m_Pvd->release();
        m_Pvd = nullptr;
    }

    if (m_PvdTransport != nullptr)
    {
        m_PvdTransport->release();
        m_PvdTransport = nullptr;
    }

    if (m_Foundation != nullptr)
    {
        m_Foundation->release();
        m_Foundation = nullptr;
    }

    m_VehicleInput = {};
    m_ResetRequested = false;
    m_TargetSteeringAngle = 0.0f;
}

physx::PxRigidStatic*
PhysicsSystem::GetGroundActor() const
{
    return m_Ground;
}

physx::PxShape*
PhysicsSystem::GetGroundShape() const
{
    return m_GroundShape;
}

physx::PxRigidDynamic*
PhysicsSystem::GetChassisActor() const
{
    return m_Chassis;
}

physx::PxShape*
PhysicsSystem::GetChassisShape() const
{
    return m_ChassisShape;
}

physx::PxShape*
PhysicsSystem::GetBodyShape() const
{
    return m_BodyShape;
}
