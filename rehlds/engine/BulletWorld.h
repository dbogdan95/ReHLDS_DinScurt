#pragma once
// BulletWorld.h

#if USE_BULLET_PHYSICS

#include <btBulletDynamicsCommon.h>
#include <unordered_map>

#define BULLET2GOLDSRC_GRAVITY -525 //-10m/s^2 -> -1000cm/s^2 -> convert to game units (1/1.905f)

struct BulletEntity
{
	btRigidBody* rigidBody = nullptr;
	btCollisionShape* shape = nullptr;
	btMotionState* motionState = nullptr;

	const edict_t* owner = nullptr;

	~BulletEntity()
	{
		if (rigidBody)
		{
			if (rigidBody->getMotionState())
				delete rigidBody->getMotionState();
			delete rigidBody;
		}

		delete shape;
	}
};


class BulletWorld 
{
public:
	static BulletWorld& Instance();

	void Init();
	void StepSimulation(float deltaTime);
	void SetPhysicsKeyValue(const edict_t* pEntity, const char* key, const char* value);
	
	void BulletWorld::RegisterRigidBody(const edict_t* ent, BulletEntity* entity);
	void RemoveRigidBody(const edict_t* ent);

	BulletEntity* BulletWorld::GetEntity(const edict_t* ent) const;
	btDiscreteDynamicsWorld* GetWorld() { 
		return dynamicsWorld; 
	}

	std::unordered_map<int, BulletEntity*> GetEntitiesMap()
	{
		return m_bulletEntities;
	}

	static void BuildBulletCollisionFromBSP(model_t* model);

private:
	BulletWorld();
	~BulletWorld();

	btDefaultCollisionConfiguration* collisionConfig = nullptr;
	btCollisionDispatcher* dispatcher = nullptr;
	btBroadphaseInterface* broadphase = nullptr;
	btSequentialImpulseConstraintSolver* solver = nullptr;
	btDiscreteDynamicsWorld* dynamicsWorld = nullptr;

	std::unordered_map<int, BulletEntity*> m_bulletEntities;
};

#endif // USE_BULLET_PHYSICS
