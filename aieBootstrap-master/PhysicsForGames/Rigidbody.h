#pragma once

#include "PhysicsObject.h"

class Rigidbody : public PhysicsObject
{
public:
	Rigidbody(const ShapeType& shapeID, const glm::vec2& position, const glm::vec2& velocity, const float rotation, const float mass, const bool collision = true);
	~Rigidbody();

	// updates with a fixed time step
	virtual void FixedUpdate(const glm::vec2& gravity, const float timeStep);
	// used to check the variable values
	virtual void Debug();
	// applys a force to the object
	void ApplyForce(const glm::vec2& force);
	// applys a force on another object from this object
	void ApplyForceToActor(Rigidbody* actor2, const glm::vec2& force);

	void SetPosition(const glm::vec2 position);
	glm::vec2 GetPosition() const { return m_position; }
	float GetRotation() const { return m_rotation; }
	glm::vec2 GetVelocity() const { return m_velocity; }
	bool GetCollision() const { return m_collision; }
	float GetMass() const { return m_mass; }

	//// use for when mass is lost, i.e. when a rocket uses up fuel
	//void SetMass(const float mass);

protected:
	// stores the location
	glm::vec2 m_position;
	// stores the directional speed
	glm::vec2 m_velocity;
	// stores the weight of the object
	float m_mass;
	// stores the object's rotation and is 2D so we only need a single float to represent our rotation
	float m_rotation;
	// determines if the object should collide with other objects
	bool m_collision;
};