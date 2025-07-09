// BulletWorld.cpp

#if USE_BULLET_PHYSICS

#include "precompiled.h"

BulletWorld::BulletWorld() {}
BulletWorld::~BulletWorld() {}

BulletWorld& BulletWorld::Instance() 
{
	static BulletWorld instance;
	return instance;
}

void BulletWorld::Init() 
{
	collisionConfig = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfig);
	broadphase = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver();

	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
	dynamicsWorld->setGravity(btVector3(0, 0, BULLET2GOLDSRC_GRAVITY));
}

void BulletWorld::StepSimulation(float deltaTime) 
{
	if (dynamicsWorld)
		dynamicsWorld->stepSimulation(deltaTime, 10);
}

void BulletWorld::RegisterRigidBody(const edict_t* ent, BulletEntity* entity)
{
	int entnum = NUM_FOR_EDICT(ent);

	m_bulletEntities[entnum] = entity;

	//entity->rigidBody->setUserPointer((void*)ent);
	dynamicsWorld->addRigidBody(entity->rigidBody);
}

BulletEntity* BulletWorld::GetEntity(const edict_t* ent) const
{
	auto it = m_bulletEntities.find(NUM_FOR_EDICT(ent));
	return (it != m_bulletEntities.end()) ? it->second : nullptr;
}

void BulletWorld::RemoveRigidBody(const edict_t* ent)
{
	auto it = m_bulletEntities.find(NUM_FOR_EDICT(ent));
	if (it != m_bulletEntities.end())
	{
		dynamicsWorld->removeRigidBody(it->second->rigidBody);
		delete it->second;
		m_bulletEntities.erase(it);
	}
}

void BulletWorld::SetPhysicsKeyValue(const edict_t* pEntity, const char* key, const char* value)
{
	if (!pEntity || !key || !value)
		return;

	int entIndex = NUM_FOR_EDICT(pEntity);

	// Sanity check
	if (entIndex < g_psvs.maxclients || entIndex > g_psv.num_edicts)
	{
		Con_Printf("%s: Invalid entity index %d\n", __func__, entIndex);
		return;
	}

	// Match key
	if (strcmp(key, "rigidbody") == 0)
	{
		float mass = 1.0f;
		btCollisionShape* shape = nullptr;

		if (getBulletShape(value, &shape, &mass))
		{
			btVector3 inertia(0, 0, 0);
			shape->calculateLocalInertia(mass, inertia);

			btTransform transform;
			transform.setIdentity();
			transform.setOrigin(btVector3(
				pEntity->v.origin[0] * UNIT_SCALE,
				pEntity->v.origin[1] * UNIT_SCALE,
				pEntity->v.origin[2] * UNIT_SCALE
			));

			auto motionState = new btDefaultMotionState(transform);
			auto bodyCI = btRigidBody::btRigidBodyConstructionInfo(mass, motionState, shape, inertia);
			auto rigidBody = new btRigidBody(bodyCI);

			auto wrapper = new BulletEntity{
				.rigidBody = rigidBody,
				.shape = shape,
				.motionState = motionState,
				.owner = pEntity
			};

			RegisterRigidBody(pEntity, wrapper);
		}
	}
	else if (strcmp(key, "gravity") == 0)
	{
		// TODO: parse and set per-entity gravity if needed
	}
	else if (strcmp(key, "kinematic") == 0)
	{
		// TODO: make rigidbody kinematic if value is "1"
	}
	else
	{
		Con_Printf("%s: Unknown physics key '%s'\n", __func__, key);
	}
}


void BulletWorld::BuildBulletCollisionFromBSP(model_t* model)
{
	if (!model || model->numsurfaces <= 0)
		return;

	std::vector<btVector3> vertices;
	std::vector<int> indices;

	for (int i = 0; i < model->numsurfaces; ++i) {
		msurface_t* surf = &model->surfaces[i];
		if (!(surf->flags & SURF_DRAWTILED)) {
			for (int j = 1; j < surf->numedges - 1; ++j) {
				int idx0 = model->surfedges[surf->firstedge];
				int idx1 = model->surfedges[surf->firstedge + j];
				int idx2 = model->surfedges[surf->firstedge + j + 1];

				auto getVertIndex = [&](int edgeIndex) -> int {
					bool reversed = edgeIndex < 0;
					int realEdge = reversed ? -edgeIndex : edgeIndex;
					medge_t* edge = &model->edges[realEdge];
					return reversed ? edge->v[1] : edge->v[0];
				};

				auto getVertex = [&](int vertIndex) -> btVector3 {
					mvertex_t* vert = &model->vertexes[vertIndex];
					return btVector3(vert->position[0], vert->position[1], vert->position[2]) / 32.0f;
				};

				int vi0 = getVertIndex(idx0);
				int vi1 = getVertIndex(idx1);
				int vi2 = getVertIndex(idx2);

				// Înlocuim duplicatele brute cu mapă sau adăugăm direct:
				btVector3 v0 = getVertex(vi0);
				btVector3 v1 = getVertex(vi1);
				btVector3 v2 = getVertex(vi2);

				int baseIndex = static_cast<int>(vertices.size());
				vertices.push_back(v0);
				vertices.push_back(v1);
				vertices.push_back(v2);

				indices.push_back(baseIndex + 0);
				indices.push_back(baseIndex + 1);
				indices.push_back(baseIndex + 2);
			}
		}
	}

	auto vertexArray = new btTriangleIndexVertexArray(
		static_cast<int>(indices.size() / 3), indices.data(), sizeof(int) * 3,
		static_cast<int>(vertices.size()), reinterpret_cast<btScalar*>(vertices.data()), sizeof(btVector3)
	);

	btBvhTriangleMeshShape* shape = new btBvhTriangleMeshShape(vertexArray, true);
	btDefaultMotionState* motionState = new btDefaultMotionState(btTransform::getIdentity());

	btRigidBody::btRigidBodyConstructionInfo info(0.0f, motionState, shape);
	btRigidBody* staticBody = new btRigidBody(info);

	BulletWorld::Instance().GetWorld()->addRigidBody(staticBody);

	Con_Printf("[Bullet] BSP collision loaded: %d triangles, %d verts\n", (int)(indices.size() / 3), (int)vertices.size());
}

#endif // USE_BULLET_PHYSICS
