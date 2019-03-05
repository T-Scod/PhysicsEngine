#pragma once

#include "Rigidbody.h"
#include <vector>

// vector between two vertices
struct Edge
{
	// the maximum of the two ends of the edge
	glm::vec2 max;
	glm::vec2 edge;
	glm::vec2 vert1;
	glm::vec2 vert2;
};

// object that contains more than 2 vertices
class Poly : public Rigidbody
{
public:
	// takes in a position and a collection of vertices around the position (i.e. the verts will be around the origin)
	Poly(const glm::vec2& position, const std::vector<glm::vec2>& vertices, const glm::vec2& velocity, /*const float rotation, const float angularVelocity,*/ const float mass,
		const glm::vec4& colour = glm::vec4(1, 1, 1, 1), const bool isKinematic = false, const float elasticity = 1.0f,
		const float linearDrag = 0.0f, const float angularDrag = 0.0f, const float 탎 = 0.0f, const float 탃 = 0.0f);
	// determines the velocity of the object using an incline (in radians) and a scalar (the speed)
	Poly(const glm::vec2& position, const std::vector<glm::vec2>& vertices, const float inclination, const float speed, /*const float rotation, const float angularVelocity,*/ const float mass,
		const glm::vec4& colour = glm::vec4(1, 1, 1, 1), const bool isKinematic = false, const float elasticity = 1.0f,
		const float linearDrag = 0.0f, const float angularDrag = 0.0f, const float 탎 = 0.0f, const float 탃 = 0.0f);
	// takes in a collection of positions of vertices in the world
	Poly(const std::vector<glm::vec2>& vertices, const glm::vec2& velocity, /*const float rotation, const float angularVelocity,*/ const float mass,
		const glm::vec4& colour = glm::vec4(1, 1, 1, 1), const bool isKinematic = false, const float elasticity = 1.0f,
		const float linearDrag = 0.0f, const float angularDrag = 0.0f, const float 탎 = 0.0f, const float 탃 = 0.0f);
	// determines the velocity of the object using an incline (in radians) and a scalar (the speed)
	Poly(const std::vector<glm::vec2>& vertices, const float inclination, /*const float rotation, const float angularVelocity,*/ const float speed, const float mass,
		const glm::vec4& colour = glm::vec4(1, 1, 1, 1), const bool isKinematic = false, const float elasticity = 1.0f,
		const float linearDrag = 0.0f, const float angularDrag = 0.0f, const float 탎 = 0.0f, const float 탃 = 0.0f);
	~Poly();

	// draws lines between each of the vertices
	virtual void MakeGizmo();
	// projects all vertices onto an axis and returns the smallest and largest projection
	glm::vec2 Project(const glm::vec2& axis) const;
	// gets the overlap amount between two projections
	float GetOverlap(const glm::vec2& proj1, const glm::vec2& proj2) const;
	// determines if there is an overlap between two projections
	bool Overlap(const glm::vec2& proj1, const glm::vec2& proj2) const;
	// gets a collection of points where two polys collide
	std::vector<glm::vec2> ContactPoints(const Poly* other, const glm::vec2& normal) const;

	// gets all potential collision normals of the poly
	std::vector<glm::vec2> GetAxis() const;
	std::vector<glm::vec2> GetVertices() const { return m_vertices; }
	float GetRadius() const { return m_radius; }

protected:
	// determines which edge of the poly is best for checking for collision points
	Edge BestEdge(const glm::vec2& normal) const;
	// clips the edge based on an offset along the normal
	void Clip(std::vector<glm::vec2> clippedPoints, const glm::vec2& vert1, const glm::vec2& vert2, const glm::vec2& normal, const float offset) const;
	// cross product
	glm::vec2 Cross(const glm::vec2& vect, const float scalar) const;

protected:
	// the vertex furthest from the position of the poly
	float m_radius;
	// collection of vectors of vertices from the position of the poly
	std::vector<glm::vec2> m_vertices;
};