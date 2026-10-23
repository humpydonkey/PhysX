// Repro: GPU convex-vs-plane contact manifold regeneration depends on PxMeshScale.
//
// One dynamic convex prism (regular n-gon, physical radius R and height H) rests on a PxPlane under GPU dynamics,
// TGS, PCM and friction. The same physical hull is created two ways: "unscaled" (cooked at physical size, identity
// scale) and "scaled" (cooked 1/s times larger and scaled by PxMeshScale(s)). Both have the same shape, mass and inertia.
//
// For every run the body starts at rest, tilted a few degrees so that one rim edge touches the plane, and is released.
// Gravity, the plane and friction cannot add mechanical energy, so the energy can only fall.
// The program prints the largest energy above the start ("gain") and the number of steps in which the set of hull
// vertices in contact changed ("contact set changes", a public-API lower bound for manifold regeneration).
//
// Build (PhysX built on Linux with GPU support, L = physx/bin/linux.x86_64/release, run from the physx directory):
//   g++ -O2 -std=c++14 -DNDEBUG -DPX_PHYSX_STATIC_LIB -Iinclude -I/usr/local/cuda/include -o scaled_hull_plane_repro scaled_hull_plane_repro.cpp \
//     -Wl,--start-group $L/libPhysX_static_64.a $L/libPhysXExtensions_static_64.a $L/libPhysXCommon_static_64.a \
//     $L/libPhysXFoundation_static_64.a $L/libPhysXPvdSDK_static_64.a -Wl,--end-group -L/usr/local/cuda/lib64/stubs -lcuda -lpthread -ldl -lrt
// Run: LD_LIBRARY_PATH=$L ./scaled_hull_plane_repro tilt=1 azimuth=20 scale=0.0625
#include "PxPhysicsAPI.h"
#include "gpu/PxGpu.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace physx;

namespace
{
struct Options
{
	double scale = 1.0;      // PxMeshScale factor s; points are cooked 1/s times larger than the physical hull
	double azimuthDeg = 20.0; // direction of the initial tilt
	double tiltDeg = 3.0;
	double friction = 0.6;
	int sides = 8;
	double radius = 0.0584;  // physical hull radius [m]
	double height = 0.279;   // physical hull height [m]
	int steps = 1200;
	double hz = 240.0;
	int posIters = 16;
	int velIters = 1;
	double speed = 0.0;      // initial horizontal speed along +x [m/s]
	double spin = 0.0;       // initial angular speed about the vertical axis [rad/s]
	int bodies = 1;          // more than one: performance mode, bodies in a grid, no energy output
};

double argValue(int argc, char** argv, const char* key, double def)
{
	const size_t n = strlen(key);
	for (int i = 1; i < argc; i++)
		if (!strncmp(argv[i], key, n) && argv[i][n] == '=')
			return atof(argv[i] + n + 1);
	return def;
}

// Contact reports give the set of hull vertices in contact; the key is a bit mask over the hull's vertices.
struct ContactSetRecorder : PxSimulationEventCallback
{
	PxTransform poseAtStepStart;
	const std::vector<PxVec3>* vertices = nullptr; // physical size, hull space
	PxU64 mask = 0;

	void onConstraintBreak(PxConstraintInfo*, PxU32) override {}
	void onWake(PxActor**, PxU32) override {}
	void onSleep(PxActor**, PxU32) override {}
	void onTrigger(PxTriggerPair*, PxU32) override {}
	void onAdvance(const PxRigidBody* const*, const PxTransform*, const PxU32) override {}
	void onContact(const PxContactPairHeader&, const PxContactPair* pairs, PxU32 nbPairs) override
	{
		for (PxU32 i = 0; i < nbPairs; i++)
		{
			PxContactStreamIterator it(pairs[i].contactPatches, pairs[i].contactPoints, pairs[i].getInternalFaceIndices(), pairs[i].patchCount, pairs[i].contactCount);
			while (it.hasNextPatch())
			{
				it.nextPatch();
				while (it.hasNextContact())
				{
					it.nextContact();
					const PxVec3 local = poseAtStepStart.transformInv(it.getContactPoint());
					size_t best = 0;
					float bestD = PX_MAX_F32;
					for (size_t v = 0; v < vertices->size(); v++)
					{
						const float d = (local - (*vertices)[v]).magnitudeSquared();
						if (d < bestD)
						{
							bestD = d;
							best = v;
						}
					}
					mask |= PxU64(1) << best;
				}
			}
		}
	}
};

PxFilterFlags filterShader(PxFilterObjectAttributes, PxFilterData, PxFilterObjectAttributes, PxFilterData, PxPairFlags& pairFlags, const void*, PxU32)
{
	pairFlags = PxPairFlag::eCONTACT_DEFAULT | PxPairFlag::eNOTIFY_TOUCH_FOUND | PxPairFlag::eNOTIFY_TOUCH_PERSISTS | PxPairFlag::eNOTIFY_CONTACT_POINTS;
	return PxFilterFlag::eDEFAULT;
}
} // namespace

int main(int argc, char** argv)
{
	Options o;
	o.scale = argValue(argc, argv, "scale", o.scale);
	o.azimuthDeg = argValue(argc, argv, "azimuth", o.azimuthDeg);
	o.tiltDeg = argValue(argc, argv, "tilt", o.tiltDeg);
	o.friction = argValue(argc, argv, "friction", o.friction);
	o.sides = int(argValue(argc, argv, "sides", o.sides));
	o.radius = argValue(argc, argv, "radius", o.radius);
	o.height = argValue(argc, argv, "height", o.height);
	o.steps = int(argValue(argc, argv, "steps", o.steps));
	o.hz = argValue(argc, argv, "hz", o.hz);
	o.posIters = int(argValue(argc, argv, "pos_iters", o.posIters));
	o.velIters = int(argValue(argc, argv, "vel_iters", o.velIters));
	o.speed = argValue(argc, argv, "speed", o.speed);
	o.spin = argValue(argc, argv, "spin", o.spin);
	o.bodies = int(argValue(argc, argv, "bodies", o.bodies));
	const bool useGpu = argValue(argc, argv, "gpu", 1) > 0;
	const double pi = 3.14159265358979323846;

	PxDefaultAllocator allocator;
	PxDefaultErrorCallback errorCallback;
	PxFoundation* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback);
	PxTolerancesScale tolerances;
	PxPhysics* physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, tolerances);

	// Hull points: physical size divided by the scale, so that PxMeshScale(s) restores the physical size.
	std::vector<PxVec3> physicalVertices, cookedPoints;
	for (int layer = 0; layer < 2; layer++)
		for (int i = 0; i < o.sides; i++)
		{
			const double a = 2.0 * pi * i / o.sides;
			const PxVec3 v(float(o.radius * cos(a)), float(layer ? o.height : 0.0), float(o.radius * sin(a)));
			physicalVertices.push_back(v);
			cookedPoints.push_back(v * float(1.0 / o.scale));
		}

	PxCookingParams cooking(tolerances);
	cooking.buildGPUData = true;
	PxConvexMeshDesc desc;
	desc.points.count = PxU32(cookedPoints.size());
	desc.points.stride = sizeof(PxVec3);
	desc.points.data = cookedPoints.data();
	desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
	PxConvexMesh* mesh = PxCreateConvexMesh(cooking, desc, physics->getPhysicsInsertionCallback());
	if (!mesh)
	{
		fprintf(stderr, "convex cooking failed\n");
		return 2;
	}

	ContactSetRecorder recorder;
	recorder.vertices = &physicalVertices;
	PxSceneDesc sceneDesc(tolerances);
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	sceneDesc.cpuDispatcher = PxDefaultCpuDispatcherCreate(0);
	sceneDesc.filterShader = filterShader;
	sceneDesc.simulationEventCallback = &recorder;
	sceneDesc.solverType = PxSolverType::eTGS;
	sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;
	sceneDesc.bounceThresholdVelocity = 0.0f;
	PxCudaContextManager* cuda = nullptr;
	if (useGpu)
	{
		PxCudaContextManagerDesc cudaDesc;
		cuda = PxCreateCudaContextManager(*foundation, cudaDesc);
		if (!cuda || !cuda->contextIsValid())
		{
			fprintf(stderr, "no valid CUDA context\n");
			return 3;
		}
		sceneDesc.cudaContextManager = cuda;
		sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
		sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
	}
	PxScene* scene = physics->createScene(sceneDesc);

	PxMaterial* bodyMaterial = physics->createMaterial(float(o.friction), float(o.friction), 0.0f);
	PxMaterial* planeMaterial = physics->createMaterial(float(o.friction), float(o.friction), 0.0f);
	scene->addActor(*PxCreatePlane(*physics, PxPlane(0, 1, 0, 0), *planeMaterial));

	const float s = float(o.scale);
	const double tilt = o.tiltDeg * pi / 180.0, az = o.azimuthDeg * pi / 180.0;
	const PxQuat tiltRotation(float(tilt), PxVec3(float(-sin(az)), 0.0f, float(cos(az))));
	float lowest = PX_MAX_F32;
	for (const PxVec3& v : physicalVertices)
		lowest = PxMin(lowest, tiltRotation.rotate(v).y);

	std::vector<PxRigidDynamic*> bodies;
	const int grid = int(ceil(sqrt(double(o.bodies))));
	for (int b = 0; b < o.bodies; b++)
	{
		const PxVec3 position(float(b % grid) * 0.5f, -lowest, float(b / grid) * 0.5f);
		PxRigidDynamic* body = physics->createRigidDynamic(PxTransform(position, tiltRotation));
		PxShape* shape = physics->createShape(PxConvexMeshGeometry(mesh, PxMeshScale(PxVec3(s, s, s))), *bodyMaterial, true);
		shape->setContactOffset(0.002f);
		shape->setRestOffset(0.0f);
		body->attachShape(*shape);
		PxRigidBodyExt::updateMassAndInertia(*body, 1000.0f);
		body->setSolverIterationCounts(PxU32(o.posIters), PxU32(o.velIters));
		body->setSleepThreshold(0.0f);
		body->setAngularDamping(0.05f);
		body->setLinearVelocity(PxVec3(float(o.speed), 0.0f, 0.0f));
		body->setAngularVelocity(PxVec3(0.0f, float(o.spin), 0.0f));
		scene->addActor(*body);
		bodies.push_back(body);
	}

	PxRigidDynamic* body = bodies[0];
	const double mass = body->getMass();
	const PxVec3 inertia = body->getMassSpaceInertiaTensor();
	auto energy = [&]() {
		const PxTransform com = body->getGlobalPose() * body->getCMassLocalPose();
		const PxVec3 v = body->getLinearVelocity();
		const PxVec3 w = com.q.rotateInv(body->getAngularVelocity());
		return mass * 9.81 * com.p.y + 0.5 * mass * double(v.dot(v)) + 0.5 * (double(inertia.x) * w.x * w.x + double(inertia.y) * w.y * w.y + double(inertia.z) * w.z * w.z);
	};

	const float dt = float(1.0 / o.hz);
	const auto wallStart = std::chrono::steady_clock::now();
	if (o.bodies > 1)
	{
		for (int i = 0; i < o.steps; i++)
		{
			scene->simulate(dt);
			scene->fetchResults(true);
		}
		const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
		printf("bodies=%d scale=%g steps=%d wall_s=%.3f ms_per_step=%.3f\n", o.bodies, o.scale, o.steps, seconds, 1000.0 * seconds / o.steps);
		return 0;
	}

	const double e0 = energy();
	double maxEnergy = e0, maxTiltDeg = 0.0;
	PxU64 previousMask = 0;
	int contactSetChanges = 0;
	for (int i = 0; i < o.steps; i++)
	{
		recorder.poseAtStepStart = body->getGlobalPose();
		recorder.mask = 0;
		scene->simulate(dt);
		scene->fetchResults(true);
		if (i > 0 && recorder.mask != previousMask)
			contactSetChanges++;
		previousMask = recorder.mask;
		maxEnergy = PxMax(maxEnergy, energy());
		const PxVec3 up = body->getGlobalPose().q.rotate(PxVec3(0, 1, 0));
		maxTiltDeg = PxMax(maxTiltDeg, acos(double(PxClamp(up.y, -1.0f, 1.0f))) * 180.0 / pi);
	}
	printf("scale=%g azimuth=%g steps=%d gain_J=%.4f max_tilt_deg=%.2f end_minus_start_J=%.4f contact_set_changes=%d\n", o.scale, o.azimuthDeg, o.steps,
	       maxEnergy - e0, maxTiltDeg, energy() - e0, contactSetChanges);
	fflush(stdout);
	return 0;
}
