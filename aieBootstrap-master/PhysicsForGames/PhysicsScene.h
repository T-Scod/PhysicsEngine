#pragma once

#include <vector>
#include "PhysicsObject.h"
#include "Rigidbody.h"

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

	void SetGravity(const glm::vec2& gravity) { m_gravity = gravity; }
	glm::vec2 GetGravity() const { return m_gravity; }
	void SetTimeStep(const float timeStep) { m_timeStep = timeStep; }
	float GetTimeStep() const { return m_timeStep; }

	// checks if any actors are colliding with each other
	void CheckForCollision();

	// checks for collision between plane and plane
	static bool Plane2Plane(PhysicsObject* obj1, PhysicsObject* obj2, const glm::vec2& gravity, const float timeStep);
	// does the reverse of a circle to plane collision check
	static bool Plane2Sphere(PhysicsObject* obj1, PhysicsObject* obj2, const glm::vec2& gravity, const float timeStep);
	// does the reverse of a box to plane collision check
	static bool Plane2Box(PhysicsObject* obj1, PhysicsObject* obj2, const glm::vec2& gravity, const float timeStep);
	// checks for collision between circle and plane
	static bool Sphere2Plane(PhysicsObject* obj1, PhysicsObject* obj2, const glm::vec2& gravity, const float timeStep);
	// checks for collision between box and plane
	static bool Sphere2Sphere(PhysicsObject* obj1, PhysicsObject* obj2, const glm::vec2& gravity, const float timeStep);
	// checks for collision between circle and box
	static bool Sphere2Box(PhysicsObject* obj1, PhysicsObject* obj2, const glm::vec2& gravity, const float timeStep);
	// checks for collision between box and plane
	static bool Box2Plane(PhysicsObject* obj1, PhysicsObject* obj2, const glm::vec2& gravity, const float timeStep);
	// does the reverse of a cirlce to box collision check
	static bool Box2Sphere(PhysicsObject* obj1, PhysicsObject* obj2, const glm::vec2& gravity, const float timeStep);
	// cheks for collision between box and box
	static bool Box2Box(PhysicsObject* obj1, PhysicsObject* obj2, const glm::vec2& gravity, const float timeStep);

	/* PARAMS:
	obj - the object that the friction force is being applied to
	force - the calculated resolution force
	contact - the point of contact
	gravity - force due to gravity
	timeStep - fixed time step
	탎 - the static friction coefficient of the object that obj is colliding with
	탃 - the kinetic friction coefficient of the object that obj is colliding with*/
	static void ApplyFriction(Rigidbody* obj, const glm::vec2& force, const glm::vec2& contact, const glm::vec2 gravity, const float timeStep, const float 탎, const float 탃);

protected:
	// the value of gravity in this physics scene
	glm::vec2 m_gravity;
	// used to customise prioitisation of accuracy and speed
	float m_timeStep;
	// contains all the actors in the scene
	std::vector<PhysicsObject*> m_actors;
};