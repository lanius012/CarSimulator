#include "pch.h"
#include "PhysicsRenderBridge.h"

#include <DirectXMath.h>

using namespace physx;
using namespace Math;

bool PhysicsRenderBridge::RegisterBinding(
    PxRigidActor* actor,
    PxShape* shape,
    std::size_t renderItemIndex,
    const PxTransform& visualLocalPose,
    const PxVec3& visualScale)
{
    if (actor == nullptr || shape == nullptr)
    {
        return false;
    }

    if (!visualLocalPose.isValid())
    {
        return false;
    }

    if (visualScale.x <= 0.0f ||
        visualScale.y <= 0.0f ||
        visualScale.z <= 0.0f)
    {
        return false;
    }

    Binding binding;

    binding.actor = actor;
    binding.shape = shape;
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

    return RegisterBinding(
        actor,
        shape,
        renderItemIndex,
        PxTransform(PxIdentity),
        visualScale);
}

bool PhysicsRenderBridge::RegisterCylinder(
    PxRigidActor* actor,
    PxShape* shape,
    std::size_t renderItemIndex,
    float radius,
    float halfWidth,
    const PxTransform& visualAxisCorrection)
{
    if (radius <= 0.0f || halfWidth <= 0.0f)
    {
        return false;
    }

    // 현재 Unit Cylinder 규격:
    //
    // 중심축: X
    // X 전체 길이: 1
    // Y/Z 반지름: 1
    //
    // 실제 바퀴 크기로 만들기 위한 스케일:
    //
    // X = 전체 폭 = 2 * halfWidth
    // Y = radius
    // Z = radius
    const PxVec3 visualScale(
        halfWidth * 2.0f,
        radius,
        radius);

    return RegisterBinding(
        actor,
        shape,
        renderItemIndex,
        visualAxisCorrection,
        visualScale);
}

void PhysicsRenderBridge::Sync(
    std::vector<RenderItem>& renderItems) const
{
    for (const Binding& binding : m_Bindings)
    {
        if (binding.actor == nullptr ||
            binding.shape == nullptr)
        {
            continue;
        }

        if (binding.renderItemIndex >=
            renderItems.size())
        {
            continue;
        }

        //
        // Actor의 월드 pose
        //

        const PxTransform actorWorldPose =
            binding.actor->getGlobalPose();

        //
        // Shape는 Actor 중심과 다르게 배치될 수 있다.
        //
        // 예:
        // 차체 Actor 하나에 범퍼 Shape가 앞쪽으로 이동되어 부착된 경우
        //

        const PxTransform shapeLocalPose =
            binding.shape->getLocalPose();

        //
        // 최종 시각적 pose:
        //
        // Actor world
        // × Shape local
        // × 렌더 축 보정
        //

        const PxTransform visualWorldPose =
            actorWorldPose *
            shapeLocalPose *
            binding.visualLocalPose;

        renderItems[binding.renderItemIndex].world =
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