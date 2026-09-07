#include "pch.h"
#include "PhysicsRenderBridge.h"

#include <DirectXMath.h>

using namespace physx;
using namespace Math;

bool PhysicsRenderBridge::IsValidScale(
    const PxVec3& scale)
{
    return scale.isFinite() &&
        scale.x > 0.0f &&
        scale.y > 0.0f &&
        scale.z > 0.0f;
}

bool PhysicsRenderBridge::RegisterActorShapeBinding(
    PxRigidActor* actor,
    PxShape* shape,
    std::size_t renderItemIndex,
    const PxTransform& visualLocalPose,
    const PxVec3& visualScale)
{
    if (actor == nullptr ||
        shape == nullptr ||
        !visualLocalPose.isValid() ||
        !IsValidScale(visualScale))
    {
        return false;
    }

    Binding binding;

    binding.poseSource =
        PoseSource::ActorShape;

    binding.actor = actor;
    binding.shape = shape;
    binding.renderItemIndex = renderItemIndex;
    binding.visualLocalPose = visualLocalPose;
    binding.visualScale = visualScale;

    m_Bindings.push_back(binding);

    return true;
}

bool PhysicsRenderBridge::RegisterExternalTransformBinding(
    const PxTransform* worldPose,
    std::size_t renderItemIndex,
    const PxTransform& visualLocalPose,
    const PxVec3& visualScale)
{
    if (worldPose == nullptr ||
        !worldPose->isValid() ||
        !visualLocalPose.isValid() ||
        !IsValidScale(visualScale))
    {
        return false;
    }

    Binding binding;

    binding.poseSource =
        PoseSource::ExternalTransform;

    binding.externalWorldPose = worldPose;
    binding.renderItemIndex = renderItemIndex;
    binding.visualLocalPose = visualLocalPose;
    binding.visualScale = visualScale;

    m_Bindings.push_back(binding);

    return true;
}

bool PhysicsRenderBridge::RegisterBox(
    PxRigidActor* actor,
    PxShape* shape,
    std::size_t renderItemIndex)
{
    if (actor == nullptr || shape == nullptr)
    {
        return false;
    }

    const PxGeometry& geometry =
        shape->getGeometry();

    if (geometry.getType() !=
        PxGeometryType::eBOX)
    {
        return false;
    }

    const PxBoxGeometry& boxGeometry =
        static_cast<const PxBoxGeometry&>(
            geometry);

    // PhysX는 half extents,
    // 단위 Cube 렌더 메시에는 전체 크기가 필요하다.
    const PxVec3 visualScale(
        boxGeometry.halfExtents.x * 2.0f,
        boxGeometry.halfExtents.y * 2.0f,
        boxGeometry.halfExtents.z * 2.0f);

    return RegisterActorShapeBinding(
        actor,
        shape,
        renderItemIndex,
        PxTransform(PxIdentity),
        visualScale);
}

bool PhysicsRenderBridge::RegisterSphere(
    PxRigidActor* actor,
    PxShape* shape,
    std::size_t renderItemIndex)
{
    if (actor == nullptr ||
        shape == nullptr)
    {
        return false;
    }

    const PxGeometry& geometry =
        shape->getGeometry();

    if (geometry.getType() !=
        PxGeometryType::eSPHERE)
    {
        return false;
    }

    const PxSphereGeometry& sphereGeometry =
        static_cast<const PxSphereGeometry&>(
            geometry);

    if (sphereGeometry.radius <= 0.0f)
    {
        return false;
    }

    // 생성한 단위 Sphere의 반지름은 1이다.
    const PxVec3 visualScale(
        sphereGeometry.radius,
        sphereGeometry.radius,
        sphereGeometry.radius);

    return RegisterActorShapeBinding(
        actor,
        shape,
        renderItemIndex,
        PxTransform(PxIdentity),
        visualScale);
}

bool PhysicsRenderBridge::RegisterPoseCylinder(
    const PxTransform* worldPose,
    std::size_t renderItemIndex,
    float radius,
    float halfWidth,
    const PxTransform& visualAxisCorrection)
{
    if (radius <= 0.0f ||
        halfWidth <= 0.0f)
    {
        return false;
    }

    // Unit Cylinder:
    // X축 전체 길이 1
    // Y/Z축 반지름 1
    const PxVec3 visualScale(
        halfWidth * 2.0f,
        radius,
        radius);

    return RegisterExternalTransformBinding(
        worldPose,
        renderItemIndex,
        visualAxisCorrection,
        visualScale);
}

void PhysicsRenderBridge::Sync(
    std::vector<RenderItem>& renderItems) const
{
    for (const Binding& binding : m_Bindings)
    {
        if (binding.renderItemIndex >=
            renderItems.size())
        {
            continue;
        }

        PxTransform visualWorldPose(
            PxIdentity);

        bool hasValidPose = false;

        switch (binding.poseSource)
        {
        case PoseSource::ActorShape:
        {
            if (binding.actor == nullptr ||
                binding.shape == nullptr)
            {
                break;
            }

            const PxTransform actorWorldPose =
                binding.actor->getGlobalPose();

            const PxTransform shapeLocalPose =
                binding.shape->getLocalPose();

            visualWorldPose =
                actorWorldPose *
                shapeLocalPose *
                binding.visualLocalPose;

            hasValidPose =
                visualWorldPose.isValid();

            break;
        }

        case PoseSource::ExternalTransform:
        {
            if (binding.externalWorldPose == nullptr ||
                !binding.externalWorldPose->isValid())
            {
                break;
            }

            visualWorldPose =
                *binding.externalWorldPose *
                binding.visualLocalPose;

            hasValidPose =
                visualWorldPose.isValid();

            break;
        }
        }

        if (!hasValidPose)
        {
            continue;
        }

        renderItems[
            binding.renderItemIndex
        ].world =
            BuildWorldMatrix(
                visualWorldPose,
                binding.visualScale);
    }
}

void PhysicsRenderBridge::Clear()
{
    //
    // actor와 shape를 release하지 않는다.
    //
    // 실제 소유자는 PhysicsSystem이다.
    //

    m_Bindings.clear();
}

Matrix4 PhysicsRenderBridge::BuildWorldMatrix(
    const PxTransform& worldPose,
    const PxVec3& scale)
{
    //
    // 1. PhysX Quaternion → MiniEngine Quaternion
    //

    const Quaternion rotation(
        DirectX::XMVectorSet(
            worldPose.q.x,
            worldPose.q.y,
            worldPose.q.z,
            worldPose.q.w));

    //
    // 2. PhysX 위치 → MiniEngine 위치
    //

    const Vector3 translation(
        worldPose.p.x,
        worldPose.p.y,
        worldPose.p.z);

    //
    // 3. Quaternion을 회전 basis로 변환
    //

    Matrix3 basis(rotation);

    //
    // 4. 회전된 로컬 축에 Scale 적용
    //

    basis.SetX(
        basis.GetX() * scale.x);

    basis.SetY(
        basis.GetY() * scale.y);

    basis.SetZ(
        basis.GetZ() * scale.z);

    //
    // 5. 회전 + 스케일 + 위치를 합친 Affine Transform
    //

    const AffineTransform worldTransform(
        basis,
        translation);

    return Matrix4(worldTransform);
}