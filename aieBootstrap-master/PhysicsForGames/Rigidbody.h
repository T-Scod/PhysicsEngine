#pragma once

#include "PhysicsObject.h"

class Rigidbody : public PhysicsObject
{
public:
	Rigidbody(const ShapeType& shapeID, const glm::vec2& position, const glm::vec2& velocity, const float rotation, const float angularVelocity, const float mass,
		const glm::vec4& colour = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f), const bool isKinematic = false, const float elasticity = 1.0f,
		const float linearDrag = 0.0f, const float angularDrag = 0.0f, const float µs = 0.0f, const float µk = 0.0f);
	~Rigidbody();

	// updates with a fixed time step
	virtual void FixedUpdate(const glm::vec2& gravity, const float timeStep);
	// used to check the variable values
	virtual void Debug();
	// applys a force to the object
	void ApplyForce(const glm::vec2& force, const glm::vec2& pos);

	void SetPosition(const glm::vec2& position);
	glm::vec2 GetPosition() const { return m_position; }
	void SetVelocity(const glm::vec2& velocity);
	glm::vec2 GetVelocity() const { return m_velocity; }
		
	void SetRotation(const float rotation);
	float GetRotation() const { return m_rotation; }
	void SetAngularVelocity(const float angularVelocity);
	float GetAngularVelocity() const { return m_angularVelocity; }
	float GetMoment() const { return m_moment; }
	
	// use for when mass is lost, i.e. when a rocket uses up fuel
	void SetMass(const float mass);
	float GetMass() const { return m_mass; }
	float GetElasticity() const { return m_elasticity; }

	void SetLinearDrag(const float linearDrag);
	float GetLinearDrag() const { return m_linearDrag; }
	void SetAngularDrag(const float angularDrag);
	float GetAngularDrag() const { return m_angularDrag; }

protected:
	// stores the object's location
	glm::vec2 m_position;
	// stores the directional speed
	glm::vec2 m_velocity;
	// stores the weight of the object
	float m_mass;
	// stores the how much the object is rotated in radians
	float m_rotation;
	// change in rotation over time
	float m_angularVelocity;
	// represents the moment of inertia
	float m_moment;
	// the elasticity coefficient of the object
	float m_elasticity;
	// scalar values that reduces velocity each frame due to "air resistance"
	float m_linearDrag;
	float m_angularDrag;
};