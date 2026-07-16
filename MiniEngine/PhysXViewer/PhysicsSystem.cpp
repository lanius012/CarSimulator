#include "pch.h"
#include "PhysicsSystem.h"

#include <algorithm>
#include <array>
#include <cmath>


using namespace physx;

namespace
{
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
            return DegToRad(45.0f);
        }

        if (speedKmh < 30.0f)
        {
            const float t =
                (speedKmh - 10.0f) / 20.0f;

            return DegToRad(
                Lerp(45.0f, 35.0f, t));
        }

        if (speedKmh < 60.0f)
        {
            const float t =
                (speedKmh - 30.0f) / 30.0f;

            return DegToRad(
                Lerp(35.0f, 20.0f, t));
        }

        if (speedKmh < 100.0f)
        {
            const float t =
                (speedKmh - 60.0f) / 40.0f;

            return DegToRad(
                Lerp(20.0f, 10.0f, t));
        }

        if (speedKmh < 140.0f)
        {
            const float t =
                (speedKmh - 100.0f) / 40.0f;

            return DegToRad(
                Lerp(10.0f, 6.0f, t));
        }

        return DegToRad(6.0f);
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
        10.0f,
        0.25f,
        10.0f);

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

    PxRigidBodyExt::updateMassAndInertia(*m_Chassis, 800.0f);

    if (m_Chassis == nullptr)
    {
        return false;
    }

    // 기존 코드의 질량 및 관성 설정을 그대로 이식
    //
    // m_Chassis->setMass(...);
    // m_Chassis->setMassSpaceInertiaTensor(...);
    // m_Chassis->setCMassLocalPose(...);

    // 기존 solver 설정
    //
    // m_Chassis->setSolverIterationCounts(16, 4);

    if (m_Chassis->getNbShapes() == 0)
    {
        return false;
    }

    PxShape* chassisShape = nullptr;

    if (m_Chassis->getShapes(
        &chassisShape,
        1) != 1)
    {
        return false;
    }

    m_ChassisShape =
        chassisShape;

    m_ChassisShape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
    m_ChassisShape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);

    m_Scene->addActor(
        *m_Chassis);

    return true;
}

bool PhysicsSystem::CreateWheels()
{
    using namespace physx;

    if (m_Physics == nullptr ||
        m_Scene == nullptr ||
        m_Material == nullptr)
    {
        return false;
    }

    // 이 값들은 기존 프로젝트의 실제 초기 위치로 교체한다.
    const std::array<PxVec3, kWheelCount>
        initialWheelPositions =
    {
        PxVec3(-m_ChassisHalfExtents.x, m_WheelRadius, -m_ChassisHalfExtents.z),
        PxVec3(+m_ChassisHalfExtents.x, m_WheelRadius, -m_ChassisHalfExtents.z),
        PxVec3(-m_ChassisHalfExtents.x, m_WheelRadius, +m_ChassisHalfExtents.z),
        PxVec3(+m_ChassisHalfExtents.x, m_WheelRadius, +m_ChassisHalfExtents.z)
    };

    for (std::size_t i = 0;
        i < kWheelCount;
        ++i)
    {
        m_InitialWheelPoses[i] =
            PxTransform(
                initialWheelPositions[i]);

        // -----------------------------------------------------
        // 이 부분을 기존 바퀴 생성 코드로 교체
        // -----------------------------------------------------
        //
        // CylinderCallbacks
        // CustomGeometry
        // PxConvexMeshGeometry
        // 기존 32각기둥
        //
        // 어떤 방식이든 그대로 사용할 수 있다.

        PxCustomGeometryExt::CylinderCallbacks* cylinder = new PxCustomGeometryExt::CylinderCallbacks(m_WheelHalfWidth, m_WheelRadius);

        PxRigidDynamic* wheel =
            m_Physics->createRigidDynamic(PxTransform(m_InitialWheelPoses[i]));

        PxShape* wheelShape =
            m_Physics->createShape(PxCustomGeometry(*cylinder), *m_Material);

        wheel->attachShape(*wheelShape);

        // 예:
        //
        // wheel = m_Physics->createRigidDynamic(...);
        // wheelShape = m_Physics->createShape(...);
        // wheel->attachShape(*wheelShape);
        // wheelShape->release();

        if (wheel == nullptr ||
            wheelShape == nullptr)
        {
            return false;
        }

        // 기존 코드의 질량, 관성, 감쇠, solver 설정
        //
        // wheel->setMass(30.0f);
        // wheel->setMassSpaceInertiaTensor(...);
        // wheel->setSolverIterationCounts(16, 4);
        // wheel->setAngularDamping(...);

        PxRigidBodyExt::updateMassAndInertia(*wheel, 30.0f);

        wheel->setSolverIterationCounts(16, 4);

        m_Wheels[i] =
            wheel;

        m_WheelShapes[i] =
            wheelShape;

        m_Scene->addActor(
            *m_Wheels[i]);

        wheelShape->release();
    }

    return true;
}

bool PhysicsSystem::CreateWheelJoints()
{
    using namespace physx;

    if (m_Physics == nullptr ||
        m_Chassis == nullptr)
    {
        return false;
    }

    PxJointLinearLimitPair ylimit(m_Physics->getTolerancesScale(), -0.3f, 0.3f);
    PxD6JointDrive drive(1000.0f, 100.0f, PX_MAX_F32, true);

    for (std::size_t i = 0;
        i < kWheelCount;
        ++i)
    {
        if (m_Wheels[i] == nullptr)
        {
            return false;
        }

        // 기존 프로젝트에서 계산했던 local frame을 사용한다.
        PxTransform chassisLocalFrame=m_InitialWheelPoses[i];

        PxTransform wheelLocalFrame(
            0.0f,0.0f,0.0f);

        // 예:
        //
        // chassisLocalFrame =
        //     PxTransform(chassisLocalAnchor[i]);
        //
        // wheelLocalFrame =
        //     PxTransform(wheelLocalAnchor);

        m_WheelJoints[i] =
            PxD6JointCreate(
                *m_Physics,
                m_Chassis,
                chassisLocalFrame,
                m_Wheels[i],
                wheelLocalFrame);

        if (m_WheelJoints[i] == nullptr)
        {
            return false;
        }

        // 아래에는 기존 D6 설정을 그대로 이동한다.
        //
        // m_WheelJoints[i]->setMotion(
        //     PxD6Axis::eTWIST,
        //     PxD6Motion::eFREE);
        //
        // m_WheelJoints[i]->setMotion(
        //     PxD6Axis::eSWING1,
        //     PxD6Motion::eLIMITED);
        //
        // m_WheelJoints[i]->setMotion(
        //     PxD6Axis::eSWING2,
        //     PxD6Motion::eLOCKED);
        //
        // m_WheelJoints[i]->setTwistLimit(...);
        // m_WheelJoints[i]->setSwingLimit(...);
        // m_WheelJoints[i]->setDrive(...);

        m_WheelJoints[i]->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
        if (i>=2) m_WheelJoints[i]->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
        m_WheelJoints[i]->setMotion(PxD6Axis::eY, PxD6Motion::eLIMITED);
        m_WheelJoints[i]->setLinearLimit(PxD6Axis::eY, ylimit);
        m_WheelJoints[i]->setDrive(PxD6Drive::eY, drive);

    }

    m_FrontSteeringLinkJoint =
        PxD6JointCreate(*m_Physics, m_Wheels[2], PxTransform(PxVec3(0.0f, 0.0f, 0.0f)), m_Wheels[3], PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));
    m_FrontSteeringLinkJoint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
    m_FrontSteeringLinkJoint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
    m_FrontSteeringLinkJoint->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
    m_FrontSteeringLinkJoint->setMotion(PxD6Axis::eY, PxD6Motion::eFREE);
    m_FrontSteeringLinkJoint->setMotion(PxD6Axis::eZ, PxD6Motion::eFREE);

    return true;
}

bool PhysicsSystem::CreateVehicle()
{

    if (!CreateChassis())
    {
        return false;
    }

    if (!CreateWheels())
    {
        return false;
    }

    if (!CreateWheelJoints())
    {
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

    const PxVec3 chassisForward =
        chassisPose.q.rotate(
            PxVec3(0.0f, 0.0f, 1.0f));

    const PxVec3 chassisRight =
        chassisPose.q.rotate(
            PxVec3(1.0f, 0.0f, 0.0f));

    const PxVec3 chassisUp =
        chassisPose.q.rotate(
            PxVec3(0.0f, 1.0f, 0.0f));

    const float forwardSpeed =
        m_Chassis->getLinearVelocity()
        .dot(chassisForward);

    // 속도가 높을수록 최대 조향각을 줄인다.
    const float maxAllowedSteeringAngle =
        PxMin(
            GetMaxSteerAngleBySpeed(
                forwardSpeed),
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

    // 키를 누르는 즉시 45도가 되는 것이 아니라,
    // 초당 m_SteeringSpeed만큼 점진적으로 움직인다.
    m_TargetSteeringAngle =
        MoveToward(
            m_TargetSteeringAngle,
            requestedSteeringAngle,
            m_SteeringSpeed *
            fixedDeltaTime);

    // 속도가 올라가 최대 허용 조향각이 작아졌을 경우,
    // 기존 조향각도 새 제한 안으로 줄인다.
    m_TargetSteeringAngle =
        PxClamp(
            m_TargetSteeringAngle,
            -maxAllowedSteeringAngle,
            maxAllowedSteeringAngle);

    const PxQuat desiredSteeringRotation(
        m_TargetSteeringAngle,
        chassisUp);

    const PxVec3 desiredWheelRight =
        desiredSteeringRotation.rotate(
            chassisRight);

    const PxVec3 chassisAngularVelocity =
        m_Chassis->getAngularVelocity();

    const std::array<std::size_t, 2>
        frontWheelIndices =
    {
        FrontLeft,
        FrontRight
    };

    for (const std::size_t wheelIndex :
    frontWheelIndices)
    {
        PxRigidDynamic* wheel =
            m_Wheels[wheelIndex];

        if (wheel == nullptr)
        {
            continue;
        }

        const PxTransform wheelPose =
            wheel->getGlobalPose();

        const PxVec3 currentWheelRight =
            wheelPose.q.rotate(
                PxVec3(1.0f, 0.0f, 0.0f));

        /*
         * 현재 wheel right에서 desired wheel right까지
         * chassisUp 축을 중심으로 얼마나 회전해야 하는지 계산한다.
         */
        const float sinValue =
            chassisUp.dot(
                currentWheelRight.cross(
                    desiredWheelRight));

        const float cosValue =
            PxClamp(
                currentWheelRight.dot(
                    desiredWheelRight),
                -1.0f,
                1.0f);

        const float angleError =
            std::atan2(
                sinValue,
                cosValue);

        // 바퀴의 차체 대비 상대 조향 각속도
        const PxVec3 relativeAngularVelocity =
            wheel->getAngularVelocity() -
            chassisAngularVelocity;

        const float steeringAngularVelocity =
            relativeAngularVelocity.dot(
                chassisUp);

        // PD 조향 제어
        float steeringTorque =
            m_SteerKp * angleError -
            m_SteerKd *
            steeringAngularVelocity;

        steeringTorque =
            PxClamp(
                steeringTorque,
                -m_MaxSteerTorque,
                m_MaxSteerTorque);

        wheel->addTorque(
            chassisUp * steeringTorque,
            PxForceMode::eFORCE);
    }
}

void PhysicsSystem::ApplyDrive(
    float fixedDeltaTime)
{
    if (m_Chassis == nullptr ||
        fixedDeltaTime <= 0.0f)
    {
        return;
    }

    const PxTransform chassisPose =
        m_Chassis->getGlobalPose();

    const PxVec3 chassisForward =
        chassisPose.q.rotate(
            PxVec3(0.0f, 0.0f, 1.0f));

    const PxVec3 chassisUp =
        chassisPose.q.rotate(
            PxVec3(0.0f, 1.0f, 0.0f));

    const float forwardSpeed =
        m_Chassis->getLinearVelocity()
        .dot(chassisForward);

    // ---------------------------------------------------------
    // 1. 속도 제한
    // ---------------------------------------------------------

    const float absoluteForwardSpeed =
        std::abs(forwardSpeed);

    const float speedRatio =
        absoluteForwardSpeed /
        m_MaxVehicleSpeed;

    /*
     * 기존 식:
     *
     * 1 - (v / maxSpeed)^2
     *
     * 기존 코드는 maxSpeed를 넘으면 음수가 되어
     * 반대 방향 토크가 생길 수 있으므로 0~1로 clamp한다.
     */
    const float torqueScale =
        PxClamp(
            1.0f -
            speedRatio * speedRatio,
            0.0f,
            1.0f);

    const float scaledDriveTorque =
        m_DriveTorque *
        torqueScale;

    // ---------------------------------------------------------
    // 2. 목표 yaw rate 계산
    // ---------------------------------------------------------

    const float wheelBase =
        m_ChassisHalfExtents.z * 2.0f;

    const float trackWidth =
        m_ChassisHalfExtents.x * 2.0f;

    float targetYawRate = 0.0f;

    if (wheelBase > 0.0001f)
    {
        targetYawRate =
            forwardSpeed *
            std::tan(
                m_TargetSteeringAngle) /
            wheelBase;
    }

    const float actualYawRate =
        chassisUp.dot(
            m_Chassis->getAngularVelocity());

    const float yawError =
        targetYawRate -
        actualYawRate;

    /*
     * eENABLE_BODY_ACCELERATIONS가 Scene에 설정되어 있으므로
     * 이전 물리 스텝에서 계산된 각가속도를 읽을 수 있다.
     */
    const float yawAcceleration =
        chassisUp.dot(
            m_Chassis->getAngularAcceleration());

    // ---------------------------------------------------------
    // 3. PD 토크 벡터링
    // ---------------------------------------------------------

    float torqueVectoring =
        m_VectorKp * yawError -
        m_VectorKd * yawAcceleration;

    /*
     * yaw moment를 좌우 wheel torque 차이로 변환한다.
     *
     * 기존 코드:
     *
     * torqueVectoring *= wheelRadius;
     * torqueVectoring /= trackWidth;
     */
    if (trackWidth > 0.0001f)
    {
        torqueVectoring *=
            m_WheelRadius /
            trackWidth;
    }
    else
    {
        torqueVectoring = 0.0f;
    }

    // 지나치게 큰 토크 벡터링으로 한쪽 바퀴 토크가
    // 완전히 뒤집히는 것을 방지한다.
    const float maxVectoringTorque =
        PxMax(
            scaledDriveTorque,
            0.0f);

    torqueVectoring =
        PxClamp(
            torqueVectoring,
            -maxVectoringTorque,
            maxVectoringTorque);

    // ---------------------------------------------------------
    // 4. 앞뒤 및 좌우 토크 분배
    // ---------------------------------------------------------

    const float rearDriveRate =
        1.0f -
        m_FrontDriveRate;

    const float rearYawMomentRate =
        1.0f -
        m_FrontYawMomentRate;

    const std::array<float, kWheelCount>
        finalDriveTorque =
    {
        // RearLeft
        scaledDriveTorque *
            rearDriveRate -
        torqueVectoring *
            rearYawMomentRate,

        // RearRight
        scaledDriveTorque *
            rearDriveRate +
        torqueVectoring *
            rearYawMomentRate,

        // FrontLeft
        scaledDriveTorque *
            m_FrontDriveRate -
        torqueVectoring *
            m_FrontYawMomentRate,

        // FrontRight
        scaledDriveTorque *
            m_FrontDriveRate +
        torqueVectoring *
            m_FrontYawMomentRate
    };

    // ---------------------------------------------------------
    // 5. 바퀴에 구동 토크 적용
    // ---------------------------------------------------------

    const PxVec3 wheelRollAxisLocal(
        1.0f,
        0.0f,
        0.0f);

    constexpr float stopThreshold =
        0.1f;

    for (std::size_t i = 0;
        i < kWheelCount;
        ++i)
    {
        PxRigidDynamic* wheel =
            m_Wheels[i];

        if (wheel == nullptr)
        {
            continue;
        }

        const PxTransform wheelPose =
            wheel->getGlobalPose();

        const PxVec3 rollAxisWorld =
            wheelPose.q.rotate(
                wheelRollAxisLocal);

        const float wheelAngularSpeed =
            wheel->getAngularVelocity()
            .dot(rollAxisWorld);

        float driveDirection =
            static_cast<float>(
                m_VehicleInput.drive);

        /*
         * drive == 0이면 별도 브레이크 입력 없이
         * 현재 회전 반대 방향으로 엔진 브레이크성 감쇠를 준다.
         *
         * 기존 코드에서는 torqueMagnitude를 먼저 계산한 뒤
         * forwardInput을 바꾸고 있어서 실제 감쇠 토크가
         * 정상적으로 적용되지 않았다.
         */
        if (m_VehicleInput.drive == 0)
        {
            if (wheelAngularSpeed >
                stopThreshold)
            {
                driveDirection = -1.0f;
            }
            else if (wheelAngularSpeed <
                -stopThreshold)
            {
                driveDirection = 1.0f;
            }
            else
            {
                driveDirection = 0.0f;
            }
        }

        float torqueMagnitude =
            driveDirection *
            finalDriveTorque[i];

        // 입력이 없을 때 감속이 지나치게 강하지 않도록
        // 엔진 브레이크 토크를 줄인다.
        if (m_VehicleInput.drive == 0)
        {
            torqueMagnitude *=
                m_CoastTorqueRate;
        }

        wheel->addTorque(
            rollAxisWorld *
            torqueMagnitude,
            PxForceMode::eFORCE);
    }
}

void PhysicsSystem::ApplyVehicleControl(
    float fixedDeltaTime)
{
    if (m_Chassis == nullptr)
    {
        return;
    }

    for (physx::PxRigidDynamic* wheel :
        m_Wheels)
    {
        if (wheel == nullptr)
        {
            return;
        }
    }
    //ApplySteering과 ApplyDrive 구현 필요 일단은 차량 출력되는지 확인하자
    ApplySteering(
        fixedDeltaTime);

    ApplyDrive(
        fixedDeltaTime);
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

    for (std::size_t i = 0;
        i < kWheelCount;
        ++i)
    {
        ResetRigidBodyState(
            m_Wheels[i],
            m_InitialWheelPoses[i]);
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
}

void PhysicsSystem::Shutdown()
{
    // ---------------------------------------------------------
    // Joint
    // ---------------------------------------------------------

    for (physx::PxD6Joint*& joint :
        m_WheelJoints)
    {
        if (joint != nullptr)
        {
            joint->release();
            joint = nullptr;
        }
    }

    // ---------------------------------------------------------
    // Dynamic Actors
    // ---------------------------------------------------------

    for (physx::PxRigidDynamic*& wheel :
        m_Wheels)
    {
        if (wheel != nullptr)
        {
            wheel->release();
            wheel = nullptr;
        }
    }

    m_WheelShapes.fill(
        nullptr);

    if (m_Chassis != nullptr)
    {
        m_Chassis->release();
        m_Chassis = nullptr;
    }

    m_ChassisShape = nullptr;

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

physx::PxRigidDynamic*
PhysicsSystem::GetWheelActor(
    std::size_t wheelIndex) const
{
    if (!IsValidWheelIndex(
        wheelIndex))
    {
        return nullptr;
    }

    return m_Wheels[
        wheelIndex];
}

physx::PxShape*
PhysicsSystem::GetWheelShape(
    std::size_t wheelIndex) const
{
    if (!IsValidWheelIndex(
        wheelIndex))
    {
        return nullptr;
    }

    return m_WheelShapes[
        wheelIndex];
}

bool PhysicsSystem::IsValidWheelIndex(
    std::size_t wheelIndex)
{
    return wheelIndex <
        kWheelCount;
}