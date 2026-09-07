#pragma once

#include "PxPhysicsAPI.h"

#include "extensions/PxDefaultCpuDispatcher.h"
#include "extensions/PxD6Joint.h"
#include "extensions/PxExtensionsAPI.h"

#include "VehicleInput.h"

#include <array>
#include <cstddef>
#include <vector>
#include <cmath>

class PhysicsSystem
{
public:
    static constexpr std::size_t kWheelCount = 4;

    enum WheelIndex : std::size_t
    {
        RearLeft=0,
        RearRight,
        FrontLeft,
        FrontRight,
        
    };

    struct WheelState
    {
        // ---------------------------------------------------------
        // 차체에 고정된 바퀴/서스펜션 장착 위치
        // ---------------------------------------------------------

        // 차체 로컬 좌표계에서의 서스펜션 장착점
        physx::PxVec3 mountPositionLocal{
            0.0f,
            0.0f,
            0.0f
        };

        // ---------------------------------------------------------
        // Raycast 접지 정보
        // ---------------------------------------------------------

        // Ray가 유효한 지면에 닿았는가
        bool isGrounded = false;



        // 지면과 타이어가 만나는 월드 접촉점
        physx::PxVec3 contactPointWorld{
            0.0f,
            0.0f,
            0.0f
        };

        // 접촉 지면의 월드 법선
        physx::PxVec3 contactNormalWorld{
            0.0f,
            1.0f,
            0.0f
        };

        // ---------------------------------------------------------
        // 서스펜션 상태
        // ---------------------------------------------------------


        // 현재 서스펜션 길이.
        //
        // suspensionLength =
        //     raycastDistance - wheelRadius
        float suspensionLength = 0.0f;

        float prevsuspensionLength = 0.0f;

        // 서스펜션 압축 속도.
        //
        // 양수: 압축 중
        // 음수: 신장 중
        float compressionVelocity = 0.0f;

        bool isSuspensionLimited = false;

        float suspensionExcess = 0.0f;

        float prevSuspensionExcess = 0.0f;

        float directLoadtoChassis = 0.0f;


        float suspensionForce = 0.0f;

        float antiRollForce = 0.0f;

        float recoveryForce = 0.0f;

        // 스프링과 댐퍼로 계산한 수직하중
        //
        // normalLoad =
        //     springStrength * compression
        //   + damperRate * compressionVelocity
        float normalLoad = 0.0f;

        //종력
        float longitudinalForce = 0.0f;

        //횡력
        float lateralForce = 0.0f;

        // ---------------------------------------------------------
        // 타이어 좌표계
        // ---------------------------------------------------------

        // 지면 접평면 위의 타이어 전방 방향
        physx::PxVec3 longitudinalDirectionWorld{
            0.0f,
            0.0f,
            1.0f
        };

        // 지면 접평면 위의 타이어 오른쪽 방향
        physx::PxVec3 lateralDirectionWorld{
            1.0f,
            0.0f,
            0.0f
        };

        // ---------------------------------------------------------
        // 타이어 슬립 상태
        // ---------------------------------------------------------

        // 종방향 슬립비
        float longitudinalSlip = 0.0f;

        // 횡방향 슬립각, rad
        float slipAngle = 0.0f;

        float relaxedSlipAngle = 0.0f;

        // ---------------------------------------------------------
        // 가상 바퀴 위치와 회전 상태
        // ---------------------------------------------------------

        // 렌더링과 이후 타이어 힘 계산에 사용할 바퀴 중심 위치
        physx::PxVec3 centerPositionWorld{
            0.0f,
            0.0f,
            0.0f
        };

        // 해당 바퀴의 현재 조향각
        // 후륜은 일반적으로 0
        float steeringAngle = 0.0f;

        // 바퀴 회전 각도.
        // Actor가 없으므로 렌더링 회전을 직접 누적한다.
        float rotationAngle = 0.0f;

        float driveTorque = 0.0f;

        float coastTorque = 0.0f;

        float brakeTorque = 0.0f;

        float absBrakeScale = 1.0f;


        // 바퀴의 선속도
        physx::PxVec3 locallinearVelocity{
            0.0f,
            0.0f,
            0.0f
        };

        // 바퀴 회전 각속도 rad/s.
        // 이후 슬립비 및 구동 토크 계산에 사용한다.
        float angularVelocity = 0.0f;

        physx::PxTransform visualPoseWorld{
            physx::PxIdentity
        };


    };

    struct GroundObject
    {
        physx::PxRigidStatic* actor = nullptr;

        // Actor가 소유하는 Shape를 가리키는 비소유 포인터
        physx::PxShape* shape = nullptr;
    };

public:
    PhysicsSystem() = default;
    ~PhysicsSystem() = default;

    bool Initialize();

    void SetVehicleInput(
        const VehicleInput& input);

    void Step(float fixedDeltaTime);

    void Shutdown();

    // ---------------------------------------------------------
    // PhysicsRenderBridge에서 사용할 객체 getter
    // ---------------------------------------------------------

    physx::PxRigidStatic*
        GetGroundActor() const;

    physx::PxShape*
        GetGroundShape() const;

    physx::PxRigidDynamic*
        GetChassisActor() const;

    physx::PxShape*
        GetChassisShape() const;

    physx::PxShape*
        GetBodyShape() const;


    float GetWheelRadius() const
    {
        return m_WheelRadius;
    }

    float GetWheelHalfWidth() const
    {
        return m_WheelHalfWidth;
    }

    const WheelState* GetWheelState(
        std::size_t wheelIndex) const
    {
        return &m_Wheels[wheelIndex];
    }

    std::size_t GetAdditionalGroundObjectCount() const
    {
        return m_AdditionalGroundObjects.size();
    }

    const GroundObject* GetAdditionalGroundObject(
        std::size_t index) const
    {
        if (index >=
            m_AdditionalGroundObjects.size())
        {
            return nullptr;
        }

        return &m_AdditionalGroundObjects[index];
    }

private:
    // ---------------------------------------------------------
    // PhysX 환경과 객체 생성
    // ---------------------------------------------------------

    bool InitializePhysX();

    bool CreateGround();

    bool CreateVehicle();

    bool CreateChassis();

    bool InitializeWheels();


    // ---------------------------------------------------------
    // 차량 제어
    // ---------------------------------------------------------

    void ApplyVehicleControl(
        float fixedDeltaTime);

    void ApplySteering(
        float fixedDeltaTime);

    void ApplyDrive(
        float fixedDeltaTime);

    void RayCasting();

    void CylinderSweep(
        float fixedDeltaTime);

    void CalculateNormalLoads(
        float fixedDeltaTime);

    void CalculateAntiRollForces();

    void CalculateTireForces(
        float fixedDeltaTime);

    void UpdateWheelAngularDynamics(
        float fixedDeltaTime);

    void ApplyWheelForces();

    void CalculateOverturnRecovery();

    void UpdateWheelRotation(
        float fixedDeltaTime);


    void ResetVehicle();

    // ---------------------------------------------------------
    // 보조 함수
    // ---------------------------------------------------------

    static bool IsValidWheelIndex(
        std::size_t wheelIndex);

    static void ResetRigidBodyState(
        physx::PxRigidDynamic* body,
        const physx::PxTransform& pose);

private:
    // ---------------------------------------------------------
    // 입력 상태
    // ---------------------------------------------------------

    VehicleInput m_VehicleInput{};

    // reset은 버튼 이벤트이므로 별도로 저장한다.
    bool m_ResetRequested = false;

    // ---------------------------------------------------------
    // PhysX SDK 객체
    // ---------------------------------------------------------

    physx::PxDefaultAllocator m_Allocator;
    physx::PxDefaultErrorCallback m_ErrorCallback;

    physx::PxFoundation* m_Foundation = nullptr;
    physx::PxPhysics* m_Physics = nullptr;
    physx::PxScene* m_Scene = nullptr;

    physx::PxDefaultCpuDispatcher*
        m_Dispatcher = nullptr;

    physx::PxMaterial* m_Material = nullptr;

    bool m_ExtensionsInitialized = false;

    // ---------------------------------------------------------
    // 선택 사항: 기존 프로젝트에서 PVD를 사용했다면 유지
    // ---------------------------------------------------------

    physx::PxPvd* m_Pvd = nullptr;
    physx::PxPvdTransport* m_PvdTransport = nullptr;

    // ---------------------------------------------------------
    // 지면
    // ---------------------------------------------------------

    physx::PxRigidStatic* m_Ground = nullptr;

    // Shape 포인터는 비소유 참조다.
    // 실제 Shape 수명은 Actor의 참조가 유지한다.
    physx::PxShape* m_GroundShape = nullptr;
    
    //기타 지면 object들
    std::vector<GroundObject>
        m_AdditionalGroundObjects;

    // ---------------------------------------------------------
    // 자동차
    // ---------------------------------------------------------

    physx::PxRigidDynamic* m_Chassis = nullptr;
    physx::PxShape* m_ChassisShape = nullptr;
    physx::PxShape* m_BodyShape = nullptr;
    float m_ChassisMass = 800.f;

    std::array<
        WheelState,
        kWheelCount> m_Wheels{};

    // ---------------------------------------------------------
    // 차량 크기
    // 기존 자동차 코드에서 사용하던 값으로 교체
    // ---------------------------------------------------------
    /*
    physx::PxVec3 m_ChassisHalfExtents{
0.5f,
0.05f,
1.0f
    };

    physx::PxVec3 m_BodyHalfExtents{
    0.7f,
    0.25f,
    1.4f
    };

    physx::PxTransform m_BodyLocalPose{
        physx::PxVec3(
            0.0f,
            0.25f,
            0.0f)
    };*/
    
    physx::PxVec3 m_ChassisHalfExtents{
        0.9f,
        0.05f,
        1.6f
    };

    physx::PxVec3 m_BodyHalfExtents{
    1.0f,
    0.25f,
    2.0f
    };

    physx::PxTransform m_BodyLocalPose{
        physx::PxVec3(
            0.0f,
            0.25f,
            0.0f)
    };
    
    // ---------------------------------------------------------
    // 바퀴 형상
    // ---------------------------------------------------------

    float m_WheelRadius = 0.3f;
    float m_WheelHalfWidth = 0.1f;

    // ---------------------------------------------------------
    // 서스펜션 설정
    // ---------------------------------------------------------

    // spring의 일반상태와 평형상태의 길이차
    float m_SuspensionRestLengthDelta = 0.05f;

    // Raycast로 허용하는 최대 서스펜션 길이
    float m_SuspensionMaxLength = 0.05f;

    // Raycast 최대 길이
    float m_maxRaycastDistance = m_WheelRadius + m_SuspensionMaxLength;

    float m_maxCylinderSweepDistance = m_SuspensionMaxLength+0.1f;

    float m_SuspensionMoveSpeed = 1.8f;

    bool m_HasZeroDistanceSweep = false;

    //roll recvoery PD
    //
    //Kp
    //Kd

    float m_RecoveryKp = 10.0f;
    float m_RecoveryKd = 4.0f;

    //recovery가 개입하기 시작하는 roll angle
    float m_RecoveryStartAngle = physx::PxPi / 3.0f;

    //이 각도부터 PD를 100%적용
    float m_RecoveryFullAngle = physx::PxPi / 2.0f;

    //지나치게 큰 회전 가속도를 막는다.
    float m_MaxRecoveryAngularAcceleration = 10.0f;

    //한 바퀴에서 recovery가 만들어낼 수 있는 최대 힘
    float m_MaxRecoveryForcePerWheel = 5000.0f;


    // 계산 폭주를 막기 위한 선택적 상한
    float m_MaxSuspensionForce = 100000.0f;
    
    float m_staticnormalLoad = m_ChassisMass * 9.81f / 4;

    // N/m
    float m_SuspensionSpringStrength = m_staticnormalLoad/m_SuspensionRestLengthDelta;

    float m_SuspensionDampingRatio = 1.0f;

    // N·s/m
    float m_SuspensionDamperRate = 2.0f*m_SuspensionDampingRatio*std::sqrt(m_SuspensionSpringStrength*m_ChassisMass/4.0f);

    float m_BumpStopDampingRatio = 1.0f;

    float m_BumpStopStrength =  m_SuspensionSpringStrength * 10.0f;

    float m_BumpStopDamperRate = 2.0f * m_BumpStopDampingRatio * std::sqrt(m_BumpStopStrength * m_ChassisMass / 4.0f);

    //최대 압축점에서 몇 m 전부터 bumpdamper가 작동할지
    float m_BumpStopDamperRange = 0.02f;

    float m_MaxBumpStopForce = 25000.0f;

    float m_AntiRollStiffness = m_SuspensionSpringStrength * 0.7f;

    // ---------------------------------------------------------
    // 초기 pose
    // ResetVehicle()에서 사용
    // ---------------------------------------------------------

    physx::PxTransform m_InitialChassisPose{
        physx::PxIdentity
    };

    std::array<
        physx::PxTransform,
        kWheelCount> m_InitialWheelPoses{};


    // ---------------------------------------------------------
// 차량 제어 상태 및 튜닝 값
// ---------------------------------------------------------

// 실제로는 점진적으로 변화하는 현재 조향 명령각
    float m_TargetSteeringAngle = 0.0f;

    // 저속 최대 조향각 45도
    float m_MaxSteeringAngle =
        physx::PxPi / 6.0f;

    // 초당 120도
    float m_SteeringSpeed =
        physx::PxPi * 2.0f / 3.0f;

    float m_SteeringReturnSpeed = physx::PxPi;

    /*
    float m_SteeringHoldTime = 0.0f;
    float m_TimeToMaxSteeringSpeed = 0.5f;
    int m_PreviousSteerInput = 0;
    */


    float m_DriveTorque = 1200.0f;
    float m_MaxVehicleSpeed = 100.0f/3.6f;

    // 원본 main.cpp 값
    float m_SteerKp = 100.0f;
    float m_SteerKd = 20.0f;
    float m_MaxSteerTorque = 800.0f;

    // 토크 벡터링 PD
    float m_VectorKp = 40.0f;
    float m_VectorKd = 20.0f;

    // 전체 구동 토크 중 앞바퀴 비율
    //
    // 0.0 = 완전 후륜
    // 0.5 = 전후 50:50
    // 1.0 = 완전 전륜
    float m_FrontDriveRate = 0.5f;

    // 토크 벡터링 yaw moment 중 앞바퀴 분배 비율
    float m_FrontYawMomentRate = 0.5f;

    // drive == 0일 때 구동 토크 대비 감쇠 비율
    //
    // 0.0 = 완전한 관성 주행
    // 0.15 = 약한 엔진 브레이크
    // 1.0 = 기존 코드가 의도했던 강한 감속
    float m_CoastTorqueRate = 0.15f;

    // ---------------------------------------------------------
    // 타이어 힘 모델
    // ---------------------------------------------------------

    // 기준 하중에서의 종방향 강성
    //
    // 단위:
    // N / slip ratio
    //
    // slip ratio가 0.1이고 하중이 기준 하중과 같으면
    // 약 800 N의 종력이 발생한다.
    float m_LongitudinalStiffness = 2000.0f;

    // 기준 하중에서의 코너링 강성
    //
    // 단위:
    // N / rad
    //
    // slip angle이 약 5도라면
    // 15000 * 0.087 ≒ 1300 N
    float m_LateralStiffness = 3000.0f;

    // 타이어와 지면 사이의 마찰계수
    float m_TireFrictionCoefficient = 0.8f;

    // 저속에서 슬립 계산의 분모가 0에 가까워지는 것을 방지
    float m_MinSlipSpeed = 5.0f;

    // 선형 모델에 지나치게 큰 슬립이 들어오는 것을 방지
    float m_MaxSlipRatio = 5.0f;

    // 최대 45도까지만 선형 슬립각 계산에 사용
    float m_MaxSlipAngle =
        physx::PxPi / 4.0f;

    // ---------------------------------------------------------
// 가상 바퀴 회전 동역학
// ---------------------------------------------------------

// 바퀴 회전관성, kg·m²
    float m_WheelMomentOfInertia =
        1.5f;

    // drive == 0일 때 바퀴를 멈추는 브레이크 토크
    float m_BrakeTorque =
        6000.0f;

    float m_ABSReleaseSlip = 0.2f;
    float m_ABSReapplySlip = 0.1f;

    float m_ABSReleaseRate = 20.0f;
    float m_ABSReapplyRate = 6.0f;

    float m_ABSMinSpeed = 2.0f;

    // 베어링 및 구동계 저항의 단순 근사
    float m_WheelAngularResistance =
        5.0f/m_MaxVehicleSpeed;

    // 최대 바퀴 각속도, rad/s
//
// 최대 차량 속도에 맞추려면:
// maxWheelAngularVelocity = maxVehicleSpeed / wheelRadius
    float m_MaxWheelAngularVelocity =
        m_MaxVehicleSpeed / m_WheelRadius;

    // 입력 중 바퀴 각속도가 변하는 속도
    // 단위: rad/s²
    /*float m_WheelAngularAcceleration =
        20.0f;*/

    float m_StopSpeedThreshold = 0.1f;
    float m_StopAngularThreshold = 0.2f;

};