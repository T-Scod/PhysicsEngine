#include "Poly.h"
#include <limits>

Poly::Poly(const glm::vec2& position, const std::vector<glm::vec2>& vertices, const glm::vec2 & velocity, const float mass,
	const glm::vec4 & colour, const bool isKinematic, const float elasticity, const float linearDrag, const float angularDrag, const float 탎, const float 탃) :
	Rigidbody(POLY, position, velocity, 0.0f, 0.0f, mass,
		colour, isKinematic, elasticity, linearDrag, angularDrag, 탎, 탃) // passes the relative information into the rigidbody constructor
{
	m_vertices = vertices;
	m_radius = 0.0f;
	// compares the distances of all the vertices from the position of the poly
	for each (glm::vec2 vertex in m_vertices)
	{
		float distance = glm::distance(vertex, position + vertex);
		if (distance > m_radius)
		{
			// stores the current largest distance as the radius
			m_radius = distance + 0.1f;
		}
	}
}
Poly::Poly(const glm::vec2& position, const std::vector<glm::vec2>& vertices, const float inclination, const float speed, const float mass,
	const glm::vec4 & colour, const bool isKinematic, const float elasticity, const float linearDrag, const float angularDrag, const float 탎, const float 탃) :
	Rigidbody(POLY, position, glm::vec2(cosf(inclination) * speed, sinf(inclination) * speed), 0.0f, 0.0f, mass,
		colour, isKinematic, elasticity, linearDrag, angularDrag, 탎, 탃) // passes the relative information into the rigidbody constructor
{
	m_vertices = vertices;
	m_radius = 0.0f;
	// compares the distances of all the vertices from the position of the poly
	for each (glm::vec2 vertex in m_vertices)
	{
		float distance = glm::distance(vertex, position + vertex);
		if (distance > m_radius)
		{
			// stores the current largest distance as the radius
			m_radius = distance + 0.1f;
		}
	}
}
Poly::Poly(const std::vector<glm::vec2>& vertices, const glm::vec2 & velocity, const float mass,
	const glm::vec4 & colour, const bool isKinematic, const float elasticity, const float linearDrag, const float angularDrag, const float 탎, const float 탃) :
	Rigidbody(POLY, glm::vec2(0.0f, 0.0f), velocity, 0.0f, 0.0f, mass,
		colour, isKinematic, elasticity, linearDrag, angularDrag, 탎, 탃) // passes the relative information into the rigidbody constructor
{
	// the average position of all the vertices
	glm::vec2 position = glm::vec2(0.0f, 0.0f);
	for each (glm::vec2 vertex in vertices)
	{
		position += vertex;
	}
	position /= vertices.size();
	m_position = position;

	m_radius = 0.0f;
	// compares the distances of all the vertices from the position of the poly
	for each (glm::vec2 vertex in vertices)
	{
		// adds the vertex to the collection as a vector relative to the position
		m_vertices.push_back(vertex - position);
		float distance = glm::distance(vertex, position);
		if (distance > m_radius)
		{
			// stores the current largest distance as the radius
			m_radius = distance + 0.1f;
		}
	}
}
Poly::Poly(const std::vector<glm::vec2>& vertices, const float inclination, const float speed, const float mass,
	const glm::vec4 & colour, const bool isKinematic, const float elasticity, const float linearDrag, const float angularDrag, const float 탎, const float 탃) :
	Rigidbody(POLY, glm::vec2(0.0f, 0.0f), glm::vec2(cosf(inclination) * speed, sinf(inclination) * speed), 0.0f, 0.0f, mass,
		colour, isKinematic, elasticity, linearDrag, angularDrag, 탎, 탃) // passes the relative information into the rigidbody constructor
{
	// the average position of all the vertices
	glm::vec2 position = glm::vec2(0.0f, 0.0f);
	for each (glm::vec2 vertex in vertices)
	{
		position += vertex;
	}
	position /= vertices.size();
	m_position = position;

	m_radius = 0.0f;
	// compares the distances of all the vertices from the position of the poly
	for each (glm::vec2 vertex in vertices)
	{
		// adds the vertex to the collection as a vector relative to the position
		m_vertices.push_back(vertex - position);
		float distance = glm::distance(vertex, position);
		if (distance > m_radius)
		{
			// stores the current largest distance as the radius
			m_radius = distance + 0.1f;
		}
	}
}
Poly::~Poly()
{
}

void Poly::MakeGizmo()
{
	/*unfilled poly*/
	//for (int i = 0; i < m_vertices.size(); i++)
	//{
	//	int j = 0;
	//	if (i + 1 < m_vertices.size())
	//	{
	//		j = i + 1;
	//	}
	//	// draws a line between the vertices
	//	aie::Gizmos::add2DLine(m_vertices[i] + m_position, m_vertices[j] + m_position, m_colour);
	//}

	/*filled poly*/
	if (m_vertices.size() > 2)
	{
		glm::vec2 v0 = m_vertices[0];
		glm::vec2 v1 = m_vertices[1];
		glm::vec2 v2 = m_vertices[2];
		for (int i = 2; i < m_vertices.size(); i++)
		{
			v2 = m_vertices[i];
			aie::Gizmos::add2DTri(v0, v1, v2, m_colour);
			v1 = v2;
		}
	}
}

// returns the min and max projection of vertices on the axis
glm::vec2 Poly::Project(const glm::vec2 & axis) const
{
	// projects the first vertex onto the axis
	float min = glm::dot(axis, m_vertices[0]);
	float max = min;
	// iterates through each vertex storing the projection if it is less than or greater than the current min or max
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

// assumes that there is an overlap
float Poly::GetOverlap(const glm::vec2 & proj1, const glm::vec2 & proj2) const
{
	// stores the smaller of the difference between min and max projections from each poly
	float overlap = proj1.y - proj2.x;
	if (overlap > proj2.y - proj1.x)
	{
		overlap = proj2.y - proj1.x;
	}

	return overlap;
}
bool Poly::Overlap(const glm::vec2 & proj1, const glm::vec2 & proj2) const
{
	// checks if there is a space between the projections which would imply that there is no overlap
	if (proj1.x > proj2.y || proj2.x > proj1.y)
	{
		return false;
	}

	return true;
}

std::vector<glm::vec2> Poly::ContactPoints(const Poly * other, const glm::vec2 & normal) const
{
	// the contact manifold
	std::vector<glm::vec2> clippedPoints;

	// gets the most relevant edges from the two polys
	Edge edge1 = this->BestEdge(normal);
	Edge edge2 = other->BestEdge(-normal);

	// the reference and incident edges
	Edge ref, inc;
	bool flip = false;
	// the reference edge is the edge that is more perpendicular to the collision normal
	if (fabsf(glm::dot(edge1.edge, normal)) <= fabsf(glm::dot(edge2.edge, normal)))
	{
		ref = edge1;
		inc = edge2;
	}
	else
	{
		ref = edge2;
		inc = edge1;
		// the reference and incident edges were flipped so a flag is set to ensure that the correct edge normal is used
		flip = true;
	}

	// stores the reference edge
	glm::vec2 refEdge = ref.edge;
	refEdge = glm::normalize(refEdge);

	// the offset of the reference edge's first vertex along the reference edge
	float offset1 = glm::dot(refEdge, ref.vert1);
	// clips the incident edge by the first vertex in the reference edge
	Clip(clippedPoints, inc.vert1, inc.vert2, refEdge, offset1);
	// ensures that two points were returned
	if (clippedPoints.size() < 2)
	{
		return std::vector<glm::vec2>();
	}

	// the offset of the reference edge's second vertex alon gthe reference edge
	float offset2 = glm::dot(refEdge, ref.vert2);
	// clips what is left of the incident edge by the second vertex in the reference edge
	// the clipping will be done in the opposite direction
	Clip(clippedPoints, clippedPoints[0], clippedPoints[1], -refEdge, -offset2);
	if (clippedPoints.size() < 2)
	{
		return std::vector<glm::vec2>();
	}

	// get the reference edge normal
	glm::vec2 refNorm = Cross(ref.edge, -1.0f);
	// if the incident and reference edges had to be flipped then the reference edge needs to be flipped
	if (flip)
	{
		refNorm *= -1.0f;
	}
	// get the largest depth
	float max = glm::dot(refNorm, ref.max);
	// make sure that the final points are not past this maximum
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
	// sets the initial maximum number as the lowest possible float value
	float max = std::numeric_limits<float>::lowest();
	unsigned int index = 0;
	// finds the vertex furthest along the collision normal
	for (int i = 0; i < m_vertices.size(); i++)
	{
		// projects the vertex onto the normal
		float projection = glm::dot(normal, m_vertices[i]);
		// checks if it is greater than the current maximum projection
		if (projection > max)
		{
			// stores it as the new max
			max = projection;
			// stores the index of the vertex in the collection
			index = i;
		}
	}

	// gets the max vertex
	glm::vec2 vert = m_vertices[index];
	// gets the vertex to the right of the max
	glm::vec2 vertNext = m_vertices[(index + 1 == m_vertices.size()) ? 0 : index + 1];
	// gets the vertex to the left of the max
	glm::vec2 vertPrev = m_vertices[(index - 1 < 0) ? m_vertices.size() : index - 1];

	// gets the vector between the max vertex and left vertex to get the left edge
	glm::vec2 leftEdge = vert - vertPrev;
	leftEdge = glm::normalize(leftEdge);
	// gets the vector between the max vertex and right vertex to get the right edge
	glm::vec2 rightEdge = vert - vertNext;
	rightEdge = glm::normalize(rightEdge);

	// returns the edge that is most perpendicular to the collision normal
	if (glm::dot(normal, rightEdge) <= glm::dot(normal, leftEdge))
	{
		Edge bestEdge;
		bestEdge.max = vert;
		// stores the edge with winding correctness
		bestEdge.edge = vertNext - vert;
		bestEdge.vert1 = vert;
		bestEdge.vert2 = vertNext;
		return bestEdge;
	}
	else
	{
		Edge bestEdge;
		bestEdge.max = vert;
		// stores the edge with winding correctness
		bestEdge.edge = vert - vertPrev;
		bestEdge.vert1 = vertPrev;
		bestEdge.vert2 = vert;
		return bestEdge;
	}
}

// clips the line segment between vert1 and vert2 if they are past the offset along the normal
void Poly::Clip(std::vector<glm::vec2> clippedPoints, const glm::vec2 & vert1, const glm::vec2 & vert2, const glm::vec2 & normal, const float offset) const
{
	// checks if the points are past the offset along the normal
	float dot1 = glm::dot(normal, vert1) - offset;
	float dot2 = glm::dot(normal, vert2) - offset;
	// if so then add them to the collection
	if (dot1 >= 0.0f)
	{
		clippedPoints.push_back(vert1);
	}
	if (dot2 >= 0.0f)
	{
		clippedPoints.push_back(vert2);
	}

	// checks if they are on opposing sides so that the correct point is computed
	if (dot1 * dot2 < 0.0f)
	{
		// get the vector for the edge that is being clipped
		glm::vec2 edge = vert2 - vert1;
		// compute the location along the edge
		float location = dot1 / (dot1 - dot2);
		edge *= location;
		edge += vert1;
		// add the point
		clippedPoints.push_back(edge);
	}
}

glm::vec2 Poly::Cross(const glm::vec2 & vect, const float scalar) const
{
	// returns the perpendicular vector and scales it
	return glm::vec2(-vect.y * scalar, vect.x * scalar);
}

std::vector<glm::vec2> Poly::GetAxis() const
{
	// collection of vectors between vertices
	std::vector<glm::vec2> axis;
	for (int i = 0; i < m_vertices.size(); i++)
	{
		glm::vec2 vert1 = m_vertices[i];
		glm::vec2 vert2;
		// checks if it is at the end of the container and should wrap around to the start
		if (i + 1 == m_vertices.size())
		{
			vert2 = m_vertices[0];
		}
		else
		{
			vert2 = m_vertices[i + 1];
		}
		// gets the vector betweem the vertices
		glm::vec2 edge = vert2 - vert1;
		// adds the potential collision normal to the collection
		axis.push_back(glm::normalize(glm::vec2(-edge.y, edge.x)));
	}

	return axis;
}