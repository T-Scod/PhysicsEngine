#include "Poly.h"
#include <limits>

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

std::vector<glm::vec2> Poly::ContactPoints(const Poly * other, const glm::vec2 & normal) const
{
	std::vector<glm::vec2> clippedPoints;

	Edge edge1 = this->BestEdge(normal);
	Edge edge2 = other->BestEdge(-normal);

	Edge ref, inc;
	bool flip = false;
	if (fabsf(glm::dot(edge1.edge, normal)) <= fabsf(glm::dot(edge2.edge, normal)))
	{
		ref = edge1;
		inc = edge2;
	}
	else
	{
		ref = edge2;
		inc = edge1;

		flip = true;
	}

	glm::vec2 refEdge = ref.edge;
	refEdge = glm::normalize(refEdge);

	float overlap1 = glm::dot(refEdge, ref.vert1);
	Clip(clippedPoints, inc.vert1, inc.vert2, refEdge, overlap1);
	if (clippedPoints.size() < 2)
	{
		return clippedPoints;
	}

	float overlap2 = glm::dot(refEdge, ref.vert2);
	Clip(clippedPoints, clippedPoints[0], clippedPoints[1], -refEdge, -overlap2);
	if (clippedPoints.size() < 2)
	{
		return clippedPoints;
	}

	glm::vec2 refNorm = Cross(ref.edge, -1.0f);
	if (flip)
	{
		refNorm *= -1.0f;
	}

	float max = glm::dot(refNorm, ref.max);

	if (glm::dot(refNorm, clippedPoints[0]) - max < 0.0f)
	{
		clippedPoints.erase(clippedPoints.begin());
	}
	if (glm::dot(refNorm, clippedPoints[1]) - max < 0.0f)
	{
		clippedPoints.erase(clippedPoints.begin() + 1);
	}

	return clippedPoints;
}

Edge Poly::BestEdge(const glm::vec2 & normal) const
{
	float max = std::numeric_limits<float>::min();
	unsigned int index = 0;
	for (int i = 0; i < m_vertices.size(); i++)
	{
		float projection = glm::dot(normal, m_vertices[i]);
		if (projection > max)
		{
			max = projection;
			index = i;
		}
	}

	glm::vec2 vert = m_vertices[index];
	glm::vec2 vertNext = m_vertices[(index + 1 == m_vertices.size()) ? 0 : index + 1];
	glm::vec2 vertPrev = m_vertices[(index - 1 < 0) ? m_vertices.size() : index - 1];

	glm::vec2 leftEdge = vert - vertPrev;
	glm::vec2 rightEdge = vert - vertNext;
	leftEdge = glm::normalize(leftEdge);
	rightEdge = glm::normalize(rightEdge);

	if (glm::dot(normal, rightEdge) <= glm::dot(normal, leftEdge))
	{
		Edge bestEdge;
		bestEdge.max = vert;
		bestEdge.edge = vertNext - vert;
		bestEdge.vert1 = vert;
		bestEdge.vert2 = vertNext;
		return bestEdge;
	}
	else
	{
		Edge bestEdge;
		bestEdge.max = vert;
		bestEdge.edge = vert - vertPrev;
		bestEdge.vert1 = vertPrev;
		bestEdge.vert2 = vert;
		return bestEdge;
	}
}

void Poly::Clip(std::vector<glm::vec2> clippedPoints, const glm::vec2 & vert1, const glm::vec2 & vert2, const glm::vec2 & normal, const float overlap) const
{
	float dot1 = glm::dot(normal, vert1) - overlap;
	float dot2 = glm::dot(normal, vert2) - overlap;

	if (dot1 >= 0.0f)
	{
		clippedPoints.push_back(vert1);
	}
	if (dot2 >= 0.0f)
	{
		clippedPoints.push_back(vert2);
	}

	if (dot1 * dot2 < 0.0f)
	{
		glm::vec2 edge = vert2 - vert1;

		float location = dot1 / (dot1 - dot2);
		edge *= location;
		edge += vert1;

		clippedPoints.push_back(edge);
	}
}

glm::vec2 Poly::Cross(const glm::vec2 & vect, const float scalar) const
{
	return glm::vec2(-1.0f * vect.y * scalar, vect.x * scalar);
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