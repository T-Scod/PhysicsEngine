#pragma once

#include <vector>
#include "PhysicsObject.h"

class PhysicsScene
{
public:
	PhysicsScene();
	PhysicsScene(const glm::vec2& gravity, const float timeStep);
	~PhysicsScene();

	// adds an actor
	void AddActor(PhysicsObject* actor);
	// removes an actor
	bool RemoveActor(PhysicsObject* actor);
	// calls the update function on all actors
	void Update(const float dt);
	// updates all the actor's gizmos
	void UpdateGizmos();
	// calls the debug function of each actor
	void DebugScene();

	// sets and gets the gravity
	void SetGravity(const glm::vec2& gravity) { m_gravity = gravity; }
	glm::vec2 GetGravity() const { return m_gravity; }
	
	// sets and gets the time step
	void SetTimeStep(const float timeStep) { m_timeStep = timeStep; }
	float GetTimeStep() const { return m_timeStep; }

	void CheckForCollision();

	static bool Plane2Plane(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool Plane2Sphere(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool Plane2Box(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool Sphere2Plane(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool Sphere2Sphere(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool Sphere2Box(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool Box2Plane(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool Box2Sphere(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool Box2Box(PhysicsObject* obj1, PhysicsObject* obj2);

protected:
	// the value of gravity in this physics scene
	glm::vec2 m_gravity;
	// used to customise prioitisation of accuracy and speed
	float m_timeStep;
	// contains all the actors in the scene
	std::vector<PhysicsObject*> m_actors;
};