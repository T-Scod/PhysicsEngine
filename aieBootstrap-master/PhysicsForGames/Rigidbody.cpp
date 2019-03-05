#include "Rigidbody.h"
#include <iostream>

#define MIN_LINEAR_THRESHOLD 0.0001f
#define MIN_ROTATION_THRESHOLD 0.00001f

Rigidbody::Rigidbody(const ShapeType& shapeID, const glm::vec2& position, const glm::vec2& velocity, const float rotation, const float angularVelocity, const float mass,
	const glm::vec4& colour, const bool isKinematic, const float elasticity, const float linearDrag, const float angularDrag, const float 탎, const float 탃) :
	PhysicsObject(shapeID, colour, isKinematic, 탎, 탃) // passes the shapeID to the physics object constructor
{
	m_position = position;
	m_velocity = velocity;
	m_rotation = rotation;
	m_angularVelocity = angularVelocity;
	m_mass = mass;
	m_elasticity = elasticity;
	m_linearDrag = linearDrag;
	m_angularDrag = angularDrag;
}
Rigidbody::~Rigidbody()
{
}

void Rigidbody::FixedUpdate(const glm::vec2& gravity, const float timeStep)
{
	// applies the force due to gravity
	m_velocity += gravity * timeStep;
	// moves the object based on the displacement created by the velocity
	m_position += m_velocity * timeStep;
	// rotates the object based on the angular velocity
	m_rotation += m_angularVelocity * timeStep;

	// decreases the velocities based on the drag
	m_velocity -= m_velocity * m_linearDrag * timeStep;
	m_angularVelocity -= m_angularVelocity * m_angularDrag * timeStep;

	// if the velocity is small enough to be below the threshold then it is set to 0
	if (glm::length(m_velocity) < MIN_LINEAR_THRESHOLD)
	{
		m_velocity = glm::vec2(0.0f, 0.0f);
	}
	// if the velocity is small enough to be below the threshold then it is set to 0
	if (fabsf(m_angularVelocity) < MIN_ROTATION_THRESHOLD)
	{
		m_angularVelocity = 0.0f;
	}
}

void Rigidbody::Debug()
{
	//std::cout << "Shape ID: " << m_shapeID << std::endl;
	std::cout << "Position: " << m_position.x << ", " << m_position.y << std::endl;
	std::cout << "Velocity: " << m_velocity.x << ", " << m_velocity.y << std::endl;
	//std::cout << "Rotation: " << m_rotation << std::endl;
	//std::cout << "Angular Velocity: " << m_angularVelocity << std::endl;
	//std::cout << "Mass: " << m_mass << std::endl;
	//std::cout << "Elasticity: " << m_elasticity << std::endl;
	//std::cout << "Linear Drag: " << m_linearDrag << std::endl;
	//std::cout << "Angular Drag: " << m_angularDrag << std::endl;
	//std::cout << "Kinematic: " << m_isKinematic << std::endl;
}

void Rigidbody::ApplyForce(const glm::vec2& force, const glm::vec2& pos)
{
	// adds the instantaneous acceleration to the current velocity
	m_velocity += force / m_mass;
	// adds the instantaneous acceleration to the angular velocity based on the position where the force is applied
	m_angularVelocity += ((force.y * pos.x) - (force.x * pos.y)) / (m_moment);
}

void Rigidbody::SetPosition(const glm::vec2& position)
{
	m_position = position;
}
void Rigidbody::SetVelocity(const glm::vec2 & velocity)
{
	m_velocity = velocity;
}

void Rigidbody::SetRotation(const float rotation)
{
	m_rotation = rotation;
}
void Rigidbody::SetAngularVelocity(const float angularVelocity)
{
	m_angularVelocity = angularVelocity;
}

void Rigidbody::SetMass(const float mass)
{
	m_mass = mass;
}

void Rigidbody::SetLinearDrag(const float linearDrag)
{
	m_linearDrag = linearDrag;
}
void Rigidbody::SetAngularDrag(const float angularDrag)
{
	m_angularDrag = angularDrag;
}