#pragma once

#include "PxPhysicsAPI.h"

#include "extensions/PxDefaultCpuDispatcher.h"
#include "extensions/PxD6Joint.h"
#include "extensions/PxExtensionsAPI.h"

#include "VehicleInput.h"

#include <array>
#include <cstddef>

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

    physx::PxRigidDynamic*
        GetWheelActor(
            std::size_t wheelIndex) const;

    physx::PxShape*
        GetWheelShape(
            std::size_t wheelIndex) const;

    float GetWheelRadius() const
    {
        return m_WheelRadius;
    }

    float GetWheelHalfWidth() const
    {
        return m_WheelHalfWidth;
    }

private:
    // ---------------------------------------------------------
    // PhysX 환경과 객체 생성
    // ---------------------------------------------------------

    bool InitializePhysX();

    bool CreateGround();

    bool CreateVehicle();

    bool CreateChassis();

    bool CreateWheels();

    bool CreateWheelJoints();

    // ---------------------------------------------------------
    // 차량 제어
    // ---------------------------------------------------------

    void ApplyVehicleControl(
        float fixedDeltaTime);

    void ApplySteering(
        float fixedDeltaTime);

    void ApplyDrive(
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

    // ---------------------------------------------------------
    // 자동차
    // ---------------------------------------------------------

    physx::PxRigidDynamic* m_Chassis = nullptr;
    physx::PxShape* m_ChassisShape = nullptr;

    std::array<
        physx::PxRigidDynamic*,
        kWheelCount> m_Wheels{};

    std::array<
        physx::PxShape*,
        kWheelCount> m_WheelShapes{};

    std::array<
        physx::PxD6Joint*,
        kWheelCount> m_WheelJoints{};

    physx::PxD6Joint* m_FrontSteeringLinkJoint = nullptr;

    // ---------------------------------------------------------
    // 차량 크기
    // 기존 자동차 코드에서 사용하던 값으로 교체
    // ---------------------------------------------------------

    physx::PxVec3 m_ChassisHalfExtents{
        0.5f,
        0.05f,
        1.0f
    };

    float m_WheelRadius = 0.5f;
    float m_WheelHalfWidth = 0.15f;

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
        physx::PxPi / 4.0f;

    // 초당 120도
    float m_SteeringSpeed =
        physx::PxPi * 2.0f / 3.0f;

    float m_DriveTorque = 200.0f;
    float m_MaxVehicleSpeed = 90.0f;

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
};