#include "Sphere.h"

Sphere::Sphere(const glm::vec2& position, const glm::vec2& velocity, const float mass, const float radius, const glm::vec4& colour, const bool collision) :
	Rigidbody(SPHERE, position, velocity, 0, mass, collision) // passes the relative information into the rigidbody constructor
{
	m_radius = radius;
	m_colour = colour;
}
Sphere::Sphere(const glm::vec2 & position, const float inclination, const float speed, const float mass, const float radius, const glm::vec4 & colour, const bool collision) :
	Rigidbody(SPHERE, position, glm::vec2(cosf(inclination) * speed, sinf(inclination) * speed), 0, mass, collision) // velocity is worked out using inclination and speed
{
	m_radius = radius;
	m_colour = colour;
}
Sphere::~Sphere()
{
}

void Sphere::MakeGizmo()
{
	// uses gizmos to draw a circle
	aie::Gizmos::add2DCircle(m_position, m_radius, 24, m_colour);
}