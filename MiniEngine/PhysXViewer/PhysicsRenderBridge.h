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

    // PhysX에는 범용 내장 Cylinder geometry가 없고,
    // CustomGeometry의 실제 치수는 애플리케이션 정의에 의존한다.
    //
    // 따라서 바퀴의 반지름과 폭을 차량 설정에서 명시적으로 전달한다.
    bool RegisterCylinder(
        physx::PxRigidActor* actor,
        physx::PxShape* shape,
        std::size_t renderItemIndex,
        float radius,
        float halfWidth,
        const physx::PxTransform& visualAxisCorrection =
        physx::PxTransform(physx::PxIdentity));

    void Sync(
        std::vector<RenderItem>& renderItems) const;

    void Clear();

private:
    struct Binding
    {
        physx::PxRigidActor* actor = nullptr;
        physx::PxShape* shape = nullptr;

        std::size_t renderItemIndex = 0;

        // PhysX 형상 축과 렌더 메시 축 사이의 고정 회전 보정
        physx::PxTransform visualLocalPose{
            physx::PxIdentity
        };

        // 단위 Primitive를 실제 크기로 만드는 값
        physx::PxVec3 visualScale{
            1.0f,
            1.0f,
            1.0f
        };
    };

    bool RegisterBinding(
        physx::PxRigidActor* actor,
        physx::PxShape* shape,
        std::size_t renderItemIndex,
        const physx::PxTransform& visualLocalPose,
        const physx::PxVec3& visualScale);

    static Math::Matrix4 BuildWorldMatrix(
        const physx::PxTransform& worldPose,
        const physx::PxVec3& scale);

private:
    std::vector<Binding> m_Bindings;
};