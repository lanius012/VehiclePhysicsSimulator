#pragma once

#include "PxPhysicsAPI.h"
#include "PrimitiveTypes.h"

#include <cstddef>
#include <vector>

class PhysicsRenderBridge
{
public:
    PhysicsRenderBridge() = default;

    bool RegisterBox(
        physx::PxRigidActor* actor,
        physx::PxShape* shape,
        std::size_t renderItemIndex);

    bool RegisterSphere(
        physx::PxRigidActor* actor,
        physx::PxShape* shape,
        std::size_t renderItemIndex);

    // PhysX에는 범용 내장 Cylinder geometry가 없고,
    // CustomGeometry의 실제 치수는 애플리케이션 정의에 의존한다.
    //
    // 따라서 바퀴의 반지름과 폭을 차량 설정에서 명시적으로 전달한다.
    bool RegisterPoseCylinder(
        const physx::PxTransform* worldPose,
        std::size_t renderItemIndex,
        float radius,
        float halfWidth,
        const physx::PxTransform& visualAxisCorrection =
        physx::PxTransform(physx::PxIdentity));

    void Sync(
        std::vector<RenderItem>& renderItems) const;

    void Clear();

private:
    enum class PoseSource
    {
        ActorShape,
        ExternalTransform
    };

    struct Binding
    {
        PoseSource poseSource =
            PoseSource::ActorShape;

        // 지면과 차체에서 사용
        physx::PxRigidActor* actor = nullptr;
        physx::PxShape* shape = nullptr;

        // WheelState::visualPoseWorld에서 사용
        const physx::PxTransform* externalWorldPose =
            nullptr;

        std::size_t renderItemIndex = 0;

        physx::PxTransform visualLocalPose{
            physx::PxIdentity
        };

        physx::PxVec3 visualScale{
            1.0f,
            1.0f,
            1.0f
        };
    };

    bool RegisterActorShapeBinding(
        physx::PxRigidActor* actor,
        physx::PxShape* shape,
        std::size_t renderItemIndex,
        const physx::PxTransform& visualLocalPose,
        const physx::PxVec3& visualScale);

    bool RegisterExternalTransformBinding(
        const physx::PxTransform* worldPose,
        std::size_t renderItemIndex,
        const physx::PxTransform& visualLocalPose,
        const physx::PxVec3& visualScale);

    static bool IsValidScale(
        const physx::PxVec3& scale);

    static Math::Matrix4 BuildWorldMatrix(
        const physx::PxTransform& worldPose,
        const physx::PxVec3& scale);

private:
    std::vector<Binding> m_Bindings;
};