#include "Sphere.h"

Sphere::Sphere(const glm::vec2& position, const glm::vec2& velocity, const float radius, const float mass,
	const glm::vec4& colour, const float elasticity, const float linearDrag, const float angularDrag, const float 탎, const float 탃) :
	Rigidbody(SPHERE, position, velocity, 0, mass,
		colour, elasticity, linearDrag, angularDrag, 탎, 탃) // passes the relative information into the rigidbody constructor
{
	m_radius = radius;
	m_moment = 0.5f * mass * radius * radius;
}
Sphere::Sphere(const glm::vec2& position, const float inclination, const float speed, const float radius, const float mass,
	const glm::vec4& colour, const float elasticity, const float linearDrag, const float angularDrag, const float 탎, const float 탃) :
	Rigidbody(SPHERE, position, glm::vec2(cosf(inclination) * speed, sinf(inclination) * speed), 0, mass,
		colour, elasticity, linearDrag, angularDrag, 탎, 탃) // velocity is worked out using inclination and speed
{
	m_radius = radius;
	m_moment = 0.5f * mass * radius * radius;
}
Sphere::~Sphere()
{
}

void Sphere::MakeGizmo()
{
	glm::vec2 end = glm::vec2(std::cos(m_rotation), std::sin(m_rotation)) * m_radius;

	// uses gizmos to draw a circle
	aie::Gizmos::add2DCircle(m_position, m_radius, 24, m_colour);
	aie::Gizmos::add2DLine(m_position, m_position + end, glm::vec4(1, 0, 0, 1));
}