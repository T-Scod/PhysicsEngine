#include "Rigidbody.h"
#include <iostream>

Rigidbody::Rigidbody(const ShapeType& shapeID, const glm::vec2& position, const glm::vec2& velocity, const float rotation, const float mass,
	const glm::vec4& colour, const float elasticity) :
	PhysicsObject(shapeID, colour) // passes the shapeID to the physics object constructor
{
	m_position = position;
	m_velocity = velocity;
	m_rotation = rotation;
	m_mass = mass;
	m_elasticity = elasticity
}
Rigidbody::~Rigidbody()
{
}

void Rigidbody::FixedUpdate(const glm::vec2& gravity, const float timeStep)
{
	// applies the force due to gravity
	ApplyForce(gravity * m_mass * timeStep);
	// moves the object based on the displacement created by the velocity
	m_position += m_velocity * timeStep;
}

void Rigidbody::Debug()
{
	std::cout << "Shape ID: " << m_shapeID << std::endl;
	std::cout << "Position: " << m_position.x << ", " << m_position.y << std::endl;
	std::cout << "Velocity: " << m_velocity.x << ", " << m_velocity.y << std::endl;
	std::cout << "Rotation: " << m_rotation << std::endl;
	std::cout << "Mass: " << m_mass << std::endl;
}

void Rigidbody::ApplyForce(const glm::vec2& force)
{
	// adds the instantaneous acceleration to the current velocity
	m_velocity += force / m_mass;
}
// works based off of Newton's 3rd law
void Rigidbody::ApplyForceToActor(Rigidbody * actor2, const glm::vec2& force)
{
	// applys a force on the other object
	actor2->ApplyForce(force);
	// applys an opposite force on this object
	this->ApplyForce(-force);
}

void Rigidbody::SetPosition(const glm::vec2& position)
{
	m_position = position;
}

//void Rigidbody::SetMass(const float mass)
//{
//	m_mass = mass;
//}