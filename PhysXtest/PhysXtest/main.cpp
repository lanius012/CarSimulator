#include "PxPhysicsAPI.h"
#include "extensions/PxCustomGeometryExt.h"

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
static PxArray<PxRigidActor*> gActors;

static PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
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

		body->setAngularVelocity(PxVec3(20.0f, 0.0f, 0.0f));
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

	createDynamic(PxTransform(PxVec3(0, 1.2f, 0)), PxBoxGeometry(1.0f, 0.2f, 2.0f), PxVec3(0.0f, 1.0f, 0.0f));
	createWheel(PxTransform(PxVec3(0, 0, 0)), 0.5f, 1.0f, 0.5f, 0.2f);
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


/*void keyPress(unsigned char key, const PxTransform& camera)
{
	switch (toupper(key))
	{
	case 'B':	createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);						break;
	case ' ':	createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0, 0, -1)) * 200);	break;
	}
}*/

int main()
{
	static const PxU32 frameCount = 600;
	initPhysics(false);
	for (PxU32 i = 0; i < frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);

	return 0;
}