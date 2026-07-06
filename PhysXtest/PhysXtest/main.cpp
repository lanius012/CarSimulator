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
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
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

	wheelshape->setFlag(PxShapeFlag::eVISUALIZATION, true);

	PxVec3 wheelOffsets[4] =
	{
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
		PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);

		body->setAngularVelocity(PxVec3(50.0f, 0.0f, 0.0f));

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
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	/*
	if (!interactive)
		createDynamic(PxTransform(PxVec3(0, 40, 100)), PxSphereGeometry(10), PxVec3(0, -50, -100));
	*/

	PxRigidStatic* groundSphere = PxCreateStatic(*gPhysics, PxTransform(PxVec3(-1.0f, 0.0f, 3.0f)), PxSphereGeometry(1.0f), *gMaterial);
	gScene->addActor(*groundSphere);

	createWheel(PxTransform(PxVec3(0, 0, 0)), 0.5f, 1.0f, 0.5f, 0.2f);

	



	PxRigidDynamic* ghost = createDynamic(PxTransform(PxVec3(0.0f,0.5f,0.0f)), PxBoxGeometry(0.5f,0.05f,1.0f));

	ghost->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);
	PxShape* shape = NULL;
	ghost->getShapes(&shape, 1);

	shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
	shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);



	PxD6Joint* joint1 = PxD6JointCreate(*gPhysics, ghost, PxTransform(PxVec3(-0.5f, 0.0f, -1.0f)), gActors[0], PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));
	PxD6Joint* joint2 = PxD6JointCreate(*gPhysics, ghost, PxTransform(PxVec3(0.5f,0.0f, -1.0f)), gActors[1], PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));
	PxD6Joint* joint3 = PxD6JointCreate(*gPhysics, ghost, PxTransform(PxVec3(-0.5f,0.0f, 1.0f)), gActors[2], PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));
	PxD6Joint* joint4 = PxD6JointCreate(*gPhysics, ghost, PxTransform(PxVec3(0.5f, 0.0f, 1.0f)), gActors[3], PxTransform(PxVec3(0.0f, 0.0f, 0.0f)));

	joint1->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	joint2->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	joint3->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	joint4->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);

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


}

void stepPhysics(bool /*interactive*/)
{
	gScene->simulate(1.0f / 60.0f);
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
	static const PxU32 frameCount = 600;
	initPhysics(false);
	for (PxU32 i = 0; i < frameCount; i++)
		/*if (GetAsyncKeyState(0x20) & 0x8000) {
			std::cout<<"key Pressed\n";
			//gActors[0]->addTorque(PxVec3(20.0f, 0.0f, 0.0f), PxForceMode::eFORCE);
			//gActors[1]->addTorque(PxVec3(20.0f, 0.0f, 0.0f), PxForceMode::eFORCE);
			//gActors[2]->addTorque(PxVec3(20.0f, 0.0f, 0.0f), PxForceMode::eFORCE);
			//gActors[3]->addTorque(PxVec3(20.0f, 0.0f, 0.0f), PxForceMode::eFORCE);
		}*/
		stepPhysics(false);
	cleanupPhysics(false);

	return 0;
}