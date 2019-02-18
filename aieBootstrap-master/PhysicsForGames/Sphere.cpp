#include "Sphere.h"

Sphere::Sphere(const glm::vec2& position, const glm::vec2& velocity, const float radius, const float mass,
	const glm::vec4& colour, const float elasticity) :
	Rigidbody(SPHERE, position, velocity, 0, mass, colour, elasticity) // passes the relative information into the rigidbody constructor
{
	m_radius = radius;
}
Sphere::Sphere(const glm::vec2& position, const float inclination, const float speed, const float radius, const float mass,
	const glm::vec4& colour, const float elasticity) :
	Rigidbody(SPHERE, position, glm::vec2(cosf(inclination) * speed, sinf(inclination) * speed), 0, mass, colour, elasticity) // velocity is worked out using inclination and speed
{
	m_radius = radius;
}
Sphere::~Sphere()
{
}

void Sphere::MakeGizmo()
{
	// uses gizmos to draw a circle
	aie::Gizmos::add2DCircle(m_position, m_radius, 24, m_colour);
}