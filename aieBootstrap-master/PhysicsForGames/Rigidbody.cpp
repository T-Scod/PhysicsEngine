#include "Rigidbody.h"
#include <iostream>
#define MIN_LINEAR_THRESHOLD 0.0001f
#define MIN_ROTATION_THRESHOLD 0.00001f

Rigidbody::Rigidbody(const ShapeType& shapeID, const glm::vec2& position, const glm::vec2& velocity, const float rotation, const float mass,
	const glm::vec4& colour, const float elasticity, const float linearDrag, const float angularDrag, const float 탎, const float 탃) :
	PhysicsObject(shapeID, colour, 탎, 탃) // passes the shapeID to the physics object constructor
{
	m_position = position;
	m_velocity = velocity;
	m_rotation = rotation;
	m_angularVelocity = 0.0f;
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
	std::cout << "Shape ID: " << m_shapeID << std::endl;
	std::cout << "Position: " << m_position.x << ", " << m_position.y << std::endl;
	std::cout << "Velocity: " << m_velocity.x << ", " << m_velocity.y << std::endl;
	std::cout << "Rotation: " << m_rotation << std::endl;
	std::cout << "Mass: " << m_mass << std::endl;
}

void Rigidbody::ApplyForce(const glm::vec2& force, const glm::vec2& pos)
{
	// adds the instantaneous acceleration to the current velocity
	m_velocity += force / m_mass;
	float av = ((force.y * pos.x) - (force.x * pos.y)) / (m_moment);
	m_angularVelocity += av;
}

void Rigidbody::SetPosition(const glm::vec2& position)
{
	m_position = position;
}

void Rigidbody::SetVelocity(const glm::vec2 & velocity)
{
	m_velocity = velocity;
}

//void Rigidbody::SetMass(const float mass)
//{
//	m_mass = mass;
//}