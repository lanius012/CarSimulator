#include "PxPhysicsAPI.h"
#include "extensions/PxCustomGeometryExt.h"

#include <iostream>
#include <windows.h>

using namespace physx;

static PxDefaultAllocator		gAllocator;
static PxDefaultErrorCallback	gErrorCallback;
static PxFoundation* gFoundation = NULL;
static PxPhysics* gPhysics = NULL;
static PxDefaultCpuDispatcher* gDispatcher = NULL;
static PxScene* gScene = NULL;
static PxMaterial* gMaterial = NULL;
static PxPvd* gPvd = NULL;

static PxArray<PxCustomGeometryExt::BaseConvexCallbacks*> gConvexes;
static PxArray<PxRigidDynamic*> gActors;

static PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 1.0f);
	PxRigidBodyExt::updateMassAndInertia(*dynamic, 800.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	gActors.pushBack(dynamic);
	return dynamic;
}

static void createWheel(const PxTransform& t, PxReal halfx, PxReal halfz, PxReal radius, PxReal height)
{

	PxCustomGeometryExt::CylinderCallbacks* cylinder = new PxCustomGeometryExt::CylinderCallbacks(height, radius);

	gConvexes.pushBack(cylinder);

	PxShape* wheelshape = gPhysics->createShape(PxCustomGeometry(*cylinder), *gMaterial);

	//PxShape* wheelshape = gPhysics->createShape(PxSphereGeometry(radius), *gMaterial);

	wheelshape->setFlag(PxShapeFlag::eVISUALIZATION, true);

	//PxShape* wheelshape = gPhysics->createShape(PxSphereGeometry(5.0f), *gMaterial);

	PxVec3 wheelOffsets[4] =
	{
		//PxVec3(0,radius,0),
		PxVec3(t.p.x - halfx, radius, t.p.z - halfz),
		PxVec3(t.p.x + halfx, radius, t.p.z - halfz),
		PxVec3(t.p.x - halfx, radius, t.p.z + halfz),
		PxVec3(t.p.x + halfx, radius, t.p.z + halfz)
	};

	for (PxU32 i = 0; i < 4; i++)
	{
		PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(wheelOffsets[i]));
		body->setActorFlag(PxActorFlag::eVISUALIZATION, true);

		body->attachShape(*wheelshape);
		PxRigidBodyExt::updateMassAndInertia(*body, 30.0f);

		//body->setAngularVelocity(PxVec3(50.0f, 0.0f, 0.0f));

		gScene->addActor(*body);
		gActors.pushBack(body);
	}

	wheelshape->release();
}

void initPhysics(bool interactive)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
	PxInitExtensions(*gPhysics, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;

	sceneDesc.solverType=PxSolverType::eTGS;
	gScene = gPhysics->createScene(sceneDesc);

	gScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
	gScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.8f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	/*
	if (!interactive)
		createDynamic(PxTransform(PxVec3(0, 40, 100)), PxSphereGeometry(10), PxVec3(0, -50, -100));
	*/

	
	/*
	for (int i = 10; i < 30; i++) {
		PxRigidStatic* groundSphere0 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-1.0f, 0.0f, i * 1.0f)), PxSphereGeometry(0.1f), *gMaterial);
		gScene->addActor(*groundSphere0);
		PxRigidStatic* groundSphere1 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-0.5f, 0.0f, i*1.0f)), PxSphereGeometry(0.1f), *gMaterial);
		gScene->addActor(*groundSphere1);
		PxRigidStatic* groundSphere2 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.0f, 0.0f, i * 1.0f)), PxSphereGeometry(0.1f), *gMaterial);
		gScene->addActor(*groundSphere2);
		PxRigidStatic* groundSphere3 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.5f, 0.0f, i*1.0f)), PxSphereGeometry(0.1f), *gMaterial);
		gScene->addActor(*groundSphere3);
		PxRigidStatic* groundSphere4 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(1.0f, 0.0f, i * 1.0f)), PxSphereGeometry(0.1f), *gMaterial);
		gScene->addActor(*groundSphere4);
		
	}*/

	/*PxRigidStatic* groundSphere1 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-0.5f, 0.0f, 3.0f)), PxSphereGeometry(0.4f), *gMaterial);
	gScene->addActor(*groundSphere1);
	PxRigidStatic* groundSphere2 = PxCreateStatic(*gPhysics, PxTransform(PxVec3(0.5f, 0.0f, 3.0f)), PxSphereGeometry(0.4f), *gMaterial);
	gScene->addActor(*groundSphere2);*/

	createWheel(PxTransform(PxVec3(0, 0, 0)), 0.5f, 1.0f, 0.5f, 0.35f);


	PxRigidDynamic* ghost = createDynamic(PxTransform(PxVec3(0.0f,0.5f,0.0f)), PxBoxGeometry(0.5f,0.05f,1.0f));

	PxShape* shape = NULL;
	ghost->getShapes(&shape, 1);

	shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
	shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);



	PxD6Joint* joint1 = PxD6JointCreate(*gPhysics, gActors[4], PxTransform(PxVec3(-0.5f, 0.0f, -1.0f)), gActors[0], PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));
	PxD6Joint* joint2 = PxD6JointCreate(*gPhysics, gActors[4], PxTransform(PxVec3(0.5f,0.0f, -1.0f)), gActors[1], PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));
	PxD6Joint* joint3 = PxD6JointCreate(*gPhysics, gActors[4], PxTransform(PxVec3(-0.5f,0.0f, 1.0f)), gActors[2], PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));
	PxD6Joint* joint4 = PxD6JointCreate(*gPhysics, gActors[4], PxTransform(PxVec3(0.5f, 0.0f, 1.0f)), gActors[3], PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));

	joint1->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	joint2->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	joint3->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	joint4->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);

	joint3->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	joint4->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);

	/*PxJointLimitCone swingLimit(PxPi / 4.0f, PxPi);

	swingLimit.stiffness = 0.0f;
	swingLimit.damping = 0.0f;

	joint3->setSwingLimit(swingLimit);
	joint4->setSwingLimit(swingLimit);*/

	//joint3->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
	//joint4->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

	joint1->setMotion(PxD6Axis::eY, PxD6Motion::eLIMITED);
	joint2->setMotion(PxD6Axis::eY, PxD6Motion::eLIMITED);
	joint3->setMotion(PxD6Axis::eY, PxD6Motion::eLIMITED);
	joint4->setMotion(PxD6Axis::eY, PxD6Motion::eLIMITED);


	PxJointLinearLimitPair ylimit(gPhysics->getTolerancesScale(),-0.3f, 0.3f);
	joint1->setLinearLimit(PxD6Axis::eY,ylimit);
	joint2->setLinearLimit(PxD6Axis::eY, ylimit);
	joint3->setLinearLimit(PxD6Axis::eY, ylimit);
	joint4->setLinearLimit(PxD6Axis::eY, ylimit);

	PxD6JointDrive drive(1000.0f, 100.0f, PX_MAX_F32, true);
	joint1->setDrive(PxD6Drive::eY, drive);
	joint2->setDrive(PxD6Drive::eY, drive);
	joint3->setDrive(PxD6Drive::eY, drive);
	joint4->setDrive(PxD6Drive::eY, drive);

	//left and right wheel joint
	//PxD6Joint* joint5 = PxD6JointCreate(*gPhysics, gActors[0], PxTransform(PxVec3(0.0f, 0.0f, 0.0f)), gActors[1], PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));
	PxD6Joint* joint6 = PxD6JointCreate(*gPhysics, gActors[2], PxTransform(PxVec3(0.0f, 0.0f, 0.0f)), gActors[3], PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));
	/*joint5->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	//joint5->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	joint5->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
	joint5->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
	joint5->setMotion(PxD6Axis::eY, PxD6Motion::eFREE);
	joint5->setMotion(PxD6Axis::eZ, PxD6Motion::eFREE);*/

	joint6->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	//joint6->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	joint6->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
	joint6->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
	joint6->setMotion(PxD6Axis::eY, PxD6Motion::eFREE);
	joint6->setMotion(PxD6Axis::eZ, PxD6Motion::eFREE);
	

}

static PxReal DegToRad(PxReal deg) {
	return deg * PxPi / 180.0f;
}

//max steer angle per speed
//inspired by Unity/PhysX vehicle guidance
static PxReal GetMaxSteerAngleBySpeed(PxReal speedMps) {
	PxReal speedKmh = std::abs(speedMps) * 3.6f;

	if (speedKmh < 10.0f)
		return DegToRad(45.0f);

	if (speedKmh < 30.0f) {
		PxReal t = (speedKmh - 10.0f) / 20.0f;
		return DegToRad(45.0f + (35.0f - 45.0f) * t);
	}

	if (speedKmh < 100.0f) {
		PxReal t = (speedKmh - 60.0f) / 40.0f;
		return DegToRad(20.0f + (10.0f - 20.0f) * t);
	}

	if (speedKmh < 140.0f) {
		PxReal t = (speedKmh - 100.0f) / 40.0f;
		return DegToRad(10.0f + (6.0f - 10.0f) * t);
	}

	return DegToRad(6.0f);

}


void UpdateVehicleFromKeyboard(float dt) {

	PxReal driveTorque = 100.0f;
	const PxReal maxSpeed = 45.0f;

	const PxReal maxMechanicalSteerAngle = DegToRad(45.0f);

	const PxReal maxSteerRate = DegToRad(120.0f);

	const PxReal steerKp = 250.0f;
	const PxReal steerKd = 35.0f;
	const PxReal maxSteerTorque = 800.0f;


	const PxVec3 wheelRollAxisLocal(1.0f, 0.0f, 0.0f);

	const PxVec3 wheelForwardLocal(0.0f, 0.0f, 1.0f);

	static PxReal actualSteerAngle = 0.0f;

	//read key board input
	int forwardInput = 0, steerInput = 0;

	if (GetAsyncKeyState(VK_UP)) {
		forwardInput += 1;
	}
	if (GetAsyncKeyState(VK_DOWN)) {
		forwardInput -= 1;
	}
	if (GetAsyncKeyState(VK_LEFT)) {
		steerInput += 1;
	}
	if (GetAsyncKeyState(VK_RIGHT)) {
		steerInput -= 1;
	}

	//chassis base

	PxTransform chassisPose = gActors[4]->getGlobalPose();

	PxVec3 chassisForward = chassisPose.q.rotate(PxVec3(0.0f, 0.0f, 1.0f));

	PxVec3 chassisUp = chassisPose.q.rotate(PxVec3(0.0f, 1.0f, 0.0f));

	PxReal forwardSpeed = gActors[4]->getLinearVelocity().dot(chassisForward);

	// A/D input -> targetSteerAngle
	PxReal maxAllowedSteerAngle = GetMaxSteerAngleBySpeed(forwardSpeed);

	PxReal targetSteerAngle = 0.0f;
	if (steerInput == 1) {
		targetSteerAngle = maxAllowedSteerAngle;
	}
	else if (steerInput == -1) {
		targetSteerAngle = -maxAllowedSteerAngle;
	}


	//targetSteerAngle->actualSteerAngle

	PxReal maxSteerDelta = maxSteerRate * dt;

	PxReal delta = targetSteerAngle - actualSteerAngle;
	if (std::abs(delta) <= maxSteerDelta) {
		actualSteerAngle = targetSteerAngle;
	}
	else if(delta<0) {
		actualSteerAngle = actualSteerAngle - maxSteerDelta;
	}
	else if (delta > 0) {
		actualSteerAngle = actualSteerAngle + maxSteerDelta;
	}

	if (actualSteerAngle > maxAllowedSteerAngle) {
		actualSteerAngle = maxAllowedSteerAngle;
	}
	else if (actualSteerAngle < -maxAllowedSteerAngle) {
		actualSteerAngle = -maxAllowedSteerAngle;
	}



	//steering torque to front wheels
	for (int i = 2; i < 4; i++) {
		PxTransform wheelPose = gActors[i]->getGlobalPose();

		PxVec3 chassisRight = chassisPose.q.rotate(PxVec3(1.0f, 0.0f, 0.0f));
		PxVec3 chassisUp = chassisPose.q.rotate(PxVec3(0.0f, 1.0f, 0.0f));

		PxVec3 currentWheelRight = wheelPose.q.rotate(PxVec3(1.0f, 0.0f, 0.0f));

		PxQuat desiredSteerRotation(actualSteerAngle, chassisUp);
		PxVec3 desiredWheelRight = desiredSteerRotation.rotate(chassisRight);

		PxReal sinValue = chassisUp.dot(currentWheelRight.cross(desiredWheelRight));
		PxReal cosValue = currentWheelRight.dot(desiredWheelRight);

		PxReal angleError = std::atan2(sinValue, cosValue);


		PxVec3 relativeAngularVelocity = gActors[i]->getAngularVelocity() - gActors[4]->getAngularVelocity();

		PxReal steerAngularVelocity = relativeAngularVelocity.dot(chassisUp);

		PxReal steerTorque = steerKp * angleError - steerKd * steerAngularVelocity;

		if (steerTorque > maxSteerTorque) {
			steerTorque = maxSteerTorque;
		}
		else if (steerTorque < -maxSteerTorque) {
			steerTorque = -maxSteerTorque;
		}

		gActors[i]->addTorque(chassisUp * steerTorque, PxForceMode::eFORCE);

	}


	//velocity limit
	PxVec3 forward = chassisPose.q.rotate(PxVec3(0.0f, 0.0f, 1.0f));
	PxVec3 velocity = gActors[4]->getLinearVelocity();

	PxReal forwardv = forward.dot(velocity);

	forwardv = std::abs(forwardv);

	PxReal torqueScale = 1.0f-(forwardv / maxSpeed)*(forwardv/maxSpeed);
	driveTorque = driveTorque * torqueScale;


	
	//basic torque vectoring
	PxReal torqueVectoringGain = 40.0f;
	PxReal carL = 2.0f;
	PxReal targetYawRate = forwardSpeed * std::tan(actualSteerAngle) / carL;
	PxReal actualYawRate = chassisUp.dot(gActors[4]->getAngularVelocity());

	PxReal yawError = targetYawRate - actualYawRate;

	PxReal torqueCorrection = yawError * torqueVectoringGain;


	PxReal torqueVectoring = torqueCorrection * 0.5f;

	PxReal finaldriveTorque[4] =
	{
		driveTorque - torqueVectoring,
		driveTorque + torqueVectoring,
		driveTorque - torqueVectoring,
		driveTorque+torqueVectoring
		
	};
	
	//drive torque to wheel
	for (int i = 0; i < 4; i++) {
		PxTransform wheelPose = gActors[i]->getGlobalPose();

		PxVec3 rollAxisWorld = wheelPose.q.rotate(wheelRollAxisLocal);

		PxReal wheelSpeed = (gActors[i]->getAngularVelocity()).dot(rollAxisWorld);

		const PxReal stopThreshold = 0.1f;

		//PxReal torqueMagnitude = forwardInput * driveTorque;
		PxReal torqueMagnitude = forwardInput*finaldriveTorque[i];

		if (forwardInput == 0) {
			if (wheelSpeed > stopThreshold) forwardInput = -1;
			else if (wheelSpeed < -stopThreshold) forwardInput = 1;
		}

		gActors[i]->addTorque(rollAxisWorld * torqueMagnitude, PxForceMode::eFORCE);

	}


	//adtional torque correction
	


}


void stepPhysics(bool /*interactive*/)
{
	const float dt = 1.0f / 60.0f;
	
	UpdateVehicleFromKeyboard(dt);
	gScene->simulate(dt);
	gScene->fetchResults(true);
}

void cleanupPhysics(bool /*interactive*/)
{

	while (!gConvexes.empty()) {
		delete gConvexes.back();
		gConvexes.popBack();
	}
	gConvexes.reset();

	
	PX_RELEASE(gScene);

	while (!gActors.empty()) {
		PX_RELEASE(gActors.back());
		gActors.popBack();
	}
	gActors.reset();

	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if (gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		PX_RELEASE(gPvd);
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);

	printf("SnippetHelloWorld done.\n");
}

int main()
{
	static const PxU32 frameCount = 1000;
	initPhysics(false);
	for (PxU32 i = 0; i < frameCount; i++) {
		stepPhysics(false);
		Sleep(16);
	}
	cleanupPhysics(false);

	return 0;
}