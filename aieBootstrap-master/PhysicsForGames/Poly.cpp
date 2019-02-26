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
			m_radius = distance + 0.1f;
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
			m_radius = distance + 0.1f;
		}
	}
}
Poly::Poly(const std::vector<glm::vec2>& vertices, const glm::vec2 & velocity, const float mass,
	const glm::vec4 & colour, const float elasticity, const float linearDrag, const float angularDrag, const float 탎, const float 탃) :
	Rigidbody(POLY, glm::vec2(0.0f, 0.0f), velocity, 0.0f, mass,
		colour, elasticity, linearDrag, angularDrag, 탎, 탃) // passes the relative information into the rigidbody constructor
{
	glm::vec2 position = glm::vec2(0.0f, 0.0f);
	for each (glm::vec2 vertex in vertices)
	{
		position += vertex;
	}
	position /= vertices.size();
	m_position = position;

	m_radius = 0.0f;
	for each (glm::vec2 vertex in vertices)
	{
		m_vertices.push_back(vertex - position);
		float distance = glm::distance(vertex, position);
		if (distance > m_radius)
		{
			m_radius = distance + 0.1f;
		}
	}
}
Poly::Poly(const std::vector<glm::vec2>& vertices, const float inclination, const float speed, const float mass,
	const glm::vec4 & colour, const float elasticity, const float linearDrag, const float angularDrag, const float 탎, const float 탃) :
	Rigidbody(POLY, glm::vec2(0.0f, 0.0f), glm::vec2(cosf(inclination) * speed, sinf(inclination) * speed), 0.0f, mass,
		colour, elasticity, linearDrag, angularDrag, 탎, 탃) // passes the relative information into the rigidbody constructor
{
	glm::vec2 position = glm::vec2(0.0f, 0.0f);
	for each (glm::vec2 vertex in vertices)
	{
		position += vertex;
	}
	position /= vertices.size();
	m_position = position;

	m_radius = 0.0f;
	for each (glm::vec2 vertex in vertices)
	{
		m_vertices.push_back(vertex - position);
		float distance = glm::distance(vertex, position);
		if (distance > m_radius)
		{
			m_radius = distance + 0.1f;
		}
	}
}
Poly::~Poly()
{
}

void Poly::MakeGizmo()
{
	for (int i = 0; i < m_vertices.size(); i++)
	{
		int j = 0;
		if (i + 1 < m_vertices.size())
		{
			j = i + 1;
		}
		aie::Gizmos::add2DLine(m_vertices[i] + m_position, m_vertices[j] + m_position, m_colour);
	}
}

glm::vec2 Poly::Project(const glm::vec2 & axis) const
{
	float min = glm::dot(axis, m_vertices[0]);
	float max = min;
	for (int i = 1; i < m_vertices.size(); i++)
	{
		float proj = glm::dot(axis, m_vertices[i]);
		if (proj < min)
		{
			min = proj;
		}
		else if (proj > max)
		{
			max = proj;
		}
	}

	return glm::vec2(min, max);
}

float Poly::GetOverlap(const glm::vec2 & proj1, const glm::vec2 & proj2) const
{
	float overlap = proj1.y - proj2.x;
	if (overlap > proj2.y - proj1.x)
	{
		overlap = proj2.y - proj1.x;
	}

	return overlap;
}
bool Poly::Overlap(const glm::vec2 & proj1, const glm::vec2 & proj2) const
{
	if (proj1.x > proj2.y || proj2.x > proj1.y)
	{
		return false;
	}

	return true;
}

glm::vec2 Poly::GetContact(const Poly * other, const glm::vec2 & normal, const float overlap) const
{
	return glm::vec2();
}

std::vector<glm::vec2> Poly::GetAxis() const
{
	std::vector<glm::vec2> axis;
	for (int i = 0; i < m_vertices.size(); i++)
	{
		glm::vec2 vert1 = m_vertices[i];
		glm::vec2 vert2;
		if (i + 1 == m_vertices.size())
		{
			vert2 = m_vertices[0];
		}
		else
		{
			vert2 = m_vertices[i + 1];
		}
		glm::vec2 edge = vert2 - vert1;
		axis.push_back(glm::normalize(glm::vec2(-edge.y, edge.x)));
	}

	return axis;
}