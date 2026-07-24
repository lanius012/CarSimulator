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
        1000.0f,
        0.25f,
        1000.0f);

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

    PxRigidBodyExt::setMassAndUpdateInertia(*m_Chassis, m_ChassisMass);

    if (m_Chassis == nullptr)
    {
        return false;
    }

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


    m_Scene->addActor(
        *m_Chassis);

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

    const PxVec3 chassisForward =
        chassisPose.q.rotate(
            PxVec3(0.0f, 0.0f, 1.0f));

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

    const float targetAngularVelocity = driveInput * m_MaxWheelAngularVelocity;

    const float maximumAngularVelocityChange = m_WheelAngularAcceleration * fixedDeltaTime;

    for (WheelState& wheel : m_Wheels) {
        wheel.angularVelocity =
            MoveToward(
                wheel.angularVelocity,
                targetAngularVelocity,
                maximumAngularVelocityChange);

        const float reactionAngularVelocityChange = m_WheelAngularResistance * PxAbs(wheel.angularVelocity) * fixedDeltaTime;

        wheel.angularVelocity=MoveToward(wheel.angularVelocity, 0.0f, reactionAngularVelocityChange);
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

void PhysicsSystem::CylinderSweep() {

    PxTransform chassisPose = m_Chassis->getGlobalPose();
    PxVec3 chassisDown = chassisPose.q.rotate(PxVec3(0.0f, -1.0f, 0.0f));

    const PxConvexCore::Cylinder wheelCylinderCore(
        m_WheelHalfWidth,
        m_WheelRadius);

    const PxConvexCoreGeometry wheelCylinderGeometry(
        wheelCylinderCore);

    const PxReal maxSweepDistance =
        PxMax(0.0f, m_maxRaycastDistance - m_WheelRadius);

    for (std::size_t i = 0; i < kWheelCount; ++i) {

        WheelState* wheel = &m_Wheels[i];

        const PxVec3 mountPositionWorld = chassisPose.transform(wheel->mountPositionLocal);

        const PxQuat steeringRotation(wheel->steeringAngle, PxVec3(0.0f, 1.0f, 0.0f));

        const PxTransform sweepStartPose(mountPositionWorld-chassisDown*m_SuspensionMaxLength, chassisPose.q*steeringRotation);

        physx::PxSweepBuffer hitBuffer;

        bool hasHit = m_Scene->sweep(
            wheelCylinderGeometry,
            sweepStartPose,
            chassisDown,
            maxSweepDistance+m_SuspensionMaxLength,
            hitBuffer,
            PxHitFlag::ePOSITION | PxHitFlag::eNORMAL,
            PxQueryFilterData(PxQueryFlag::eSTATIC)
        );

        if (hasHit && hitBuffer.hasBlock) {

            wheel->isGrounded = true;

            const physx::PxSweepHit& hit = hitBuffer.block;

            wheel->contactPointWorld = hit.position;
            wheel->contactNormalWorld = hit.normal;

            wheel->suspensionLength =
                -PxClamp(
                    hit.distance-m_SuspensionMaxLength,
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

void PhysicsSystem::CalculateNormalLoads() {

    const PxTransform chassisPose = m_Chassis->getGlobalPose();

    const PxVec3 ChassisUpWorld = chassisPose.q.rotate(PxVec3(0.0f, 1.0f, 0.0f));

    for (std::size_t i = 0; i < kWheelCount; ++i) {

        WheelState& wheel = m_Wheels[i];

        if (wheel.isGrounded) {
            wheel.locallinearVelocity = PxRigidBodyExt::getLocalVelocityAtLocalPos(*m_Chassis, wheel.mountPositionLocal);
            wheel.compressionVelocity = wheel.locallinearVelocity.y;
            wheel.suspensionForce = m_staticnormalLoad+m_SuspensionSpringStrength * wheel.suspensionLength - m_SuspensionDamperRate * wheel.compressionVelocity;
            wheel.normalLoad = PxClamp(wheel.suspensionForce, 0.0f,m_MaxSuspensionForce) * PxMax(ChassisUpWorld.dot(wheel.contactNormalWorld),0.0f);
        }
        else {
            wheel.suspensionForce = 0.0f;
            wheel.normalLoad = 0.0f;
        }
        
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

        PxVec3 tireRightWorld = wheel.contactNormalWorld.cross(tireForwardWorld);

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

        //종력, 횡력 계산(현재는 선형 모델)

        float longitudinalForce = m_LongitudinalStiffness * wheel.longitudinalSlip;

        float lateralForce = -m_LateralStiffness * wheel.slipAngle;

        //마찰제한

        const float maximumTireForce = m_TireFrictionCoefficient * wheel.normalLoad;

        const float forceMagnitudeSquared = longitudinalForce * longitudinalForce + lateralForce * lateralForce;

        const float maximumForceSquared = maximumTireForce * maximumTireForce;

        if (forceMagnitudeSquared > maximumForceSquared && forceMagnitudeSquared > 1.0e-8f) {
            const float forceMagnitude = std::sqrt(maximumForceSquared);

            const float forceScale = maximumTireForce / forceMagnitude;
            
            longitudinalForce *= forceScale;
            
            lateralForce *= forceScale;

        }

        wheel.longitudinalForce = longitudinalForce;

        wheel.lateralForce = lateralForce;

        DebugPrint(
            "[%zu] %.3f degree, %.3f longi, %.3f later, %.3f normal\n",
            i,
            wheel.slipAngle * 180.0f / PxPi,
            longitudinalForce,
            lateralForce,
            wheel.normalLoad);

    }
}

void PhysicsSystem::ApplyWheelForces() {
    const PxTransform chassisPose = m_Chassis->getGlobalPose();

    const PxVec3 chassisUpWorld = chassisPose.q.rotate(PxVec3(0.0f, 1.0f, 0.0f));

    for (const WheelState& wheel : m_Wheels) {
        if (!wheel.isGrounded) {
            continue;
        }

        const PxVec3 suspensionForceWorld = chassisUpWorld * wheel.suspensionForce;
        const PxVec3 longitudinalForceWorld = wheel.longitudinalDirectionWorld * wheel.longitudinalForce;
        const PxVec3 lateralForceWorld = wheel.lateralDirectionWorld * wheel.lateralForce;

        const PxVec3 totalTireForceWorld = longitudinalForceWorld +lateralForceWorld;

        const PxVec3 mountPositionWorld = chassisPose.transform(wheel.mountPositionLocal);

        PxRigidBodyExt::addForceAtPos(*m_Chassis, suspensionForceWorld, mountPositionWorld, PxForceMode::eFORCE);

        PxRigidBodyExt::addForceAtPos(*m_Chassis, totalTireForceWorld, wheel.centerPositionWorld, PxForceMode::eFORCE);
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

    RayCasting();

    //CylinderSweep();

    CalculateNormalLoads();
    for (WheelState& wheel : m_Wheels) {
        DebugPrint("after Normal wheel force: %.3f, %d, %.3f\n", wheel.suspensionForce, wheel.isGrounded, wheel.suspensionLength);
    }
    
    CalculateTireForces(fixedDeltaTime);

    for (WheelState& wheel : m_Wheels) {
    }
    ApplyWheelForces();
    for (WheelState& wheel : m_Wheels) {
    }
   

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
