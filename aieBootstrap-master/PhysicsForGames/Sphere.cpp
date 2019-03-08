#include "Sphere.h"

Sphere::Sphere(const glm::vec2& position, const glm::vec2& velocity, const float radius, const float mass,
	const glm::vec4& colour, const bool kinematic, const bool staticRigidbody,
	const float elasticity, const float linearDrag, const float angularDrag, const float 탎, const float 탃) :
	Rigidbody(SPHERE, position, velocity, 0.0f, 0.0f, mass, // specified rotation and angular velocity is 0 because the drawn circle does not rotate
		colour, kinematic, staticRigidbody, elasticity, linearDrag, angularDrag, 탎, 탃) // passes the relative information into the rigidbody constructor
{
	m_radius = radius;
	m_moment = 0.5f * mass * radius * radius;
}
Sphere::Sphere(const glm::vec2& position, const float inclination, const float speed, const float radius, const float mass,
	const glm::vec4& colour, const bool kinematic, const bool staticRigidbody,
	const float elasticity, const float linearDrag, const float angularDrag, const float 탎, const float 탃) :
	Rigidbody(SPHERE, position, glm::vec2(cosf(inclination) * speed, sinf(inclination) * speed), 0.0f, 0.0f, mass, // specified rotation and angular velocity is 0 because the drawn circle does not rotate
		colour, kinematic, staticRigidbody, elasticity, linearDrag, angularDrag, 탎, 탃) // velocity is worked out using inclination and speed
{
	m_radius = radius;
	m_moment = 0.5f * mass * radius * radius;
}
Sphere::~Sphere()
{
}

void Sphere::MakeGizmo()
{
	// uses gizmos to draw a circle
	aie::Gizmos::add2DCircle(m_position, m_radius, 24, m_colour);
}

void Sphere::SetRadius(const float radius)
{
	m_radius = radius;
}