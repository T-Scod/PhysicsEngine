#include "Poly.h"

Poly::Poly(const glm::vec2& position, const std::vector<glm::vec2>& vertices, const glm::vec2 & velocity, const float mass,
	const glm::vec4 & colour, const float elasticity, const float linearDrag, const float angularDrag, const float 탎, const float 탃) :
	Rigidbody(POLY, position, velocity, 0.0f, mass,
		colour, elasticity, linearDrag, angularDrag, 탎, 탃) // passes the relative information into the rigidbody constructor
{
	m_vertices = vertices;
	m_radius = 0.0f;
	for each (glm::vec2 vertex in m_vertices)
	{
		float distance = glm::distance(vertex, position + vertex);
		if (distance > m_radius)
		{
			m_radius = distance;
		}
	}
}

Poly::Poly(const glm::vec2& position, const std::vector<glm::vec2>& vertices, const float inclination, const float speed, const float mass,
	const glm::vec4 & colour, const float elasticity, const float linearDrag, const float angularDrag, const float 탎, const float 탃) :
	Rigidbody(POLY, position, glm::vec2(cosf(inclination) * speed, sinf(inclination) * speed), 0.0f, mass,
		colour, elasticity, linearDrag, angularDrag, 탎, 탃) // passes the relative information into the rigidbody constructor
{
	m_vertices = vertices;
	m_radius = 0.0f;
	for each (glm::vec2 vertex in m_vertices)
	{
		float distance = glm::distance(vertex, position + vertex);
		if (distance > m_radius)
		{
			m_radius = distance;
		}
	}
}

Poly::~Poly()
{
}

glm::vec2 Poly::Project(const glm::vec2 & axis) const
{
	return glm::vec2();
}

std::vector<glm::vec2> Poly::GetAxis() const
{
	return std::vector<glm::vec2>();
}

Sphere Poly::GetBroadCollider() const
{
	return Sphere(m_position, m_velocity, m_radius, m_mass, m_colour, m_elasticity, m_linearDrag, m_angularDrag, m_탎, m_탃);
}
