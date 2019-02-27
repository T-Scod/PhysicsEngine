#pragma once

#include "Rigidbody.h"
#include <vector>

struct Edge
{
	glm::vec2 max;
	glm::vec2 edge;
	glm::vec2 vert1;
	glm::vec2 vert2;
};

class Poly : public Rigidbody
{
public:
	Poly(const glm::vec2& position, const std::vector<glm::vec2>& vertices, const glm::vec2& velocity, const float mass,
		const glm::vec4& colour = glm::vec4(1, 1, 1, 1), const float elasticity = 1.0f,
		const float linearDrag = 0.0f, const float angularDrag = 0.0f, const float 탎 = 0.0f, const float 탃 = 0.0f);
	Poly(const glm::vec2& position, const std::vector<glm::vec2>& vertices, const float inclination, const float speed, const float mass,
		const glm::vec4& colour = glm::vec4(1, 1, 1, 1), const float elasticity = 1.0f,
		const float linearDrag = 0.0f, const float angularDrag = 0.0f, const float 탎 = 0.0f, const float 탃 = 0.0f);
	Poly(const std::vector<glm::vec2>& vertices, const glm::vec2& velocity, const float mass,
		const glm::vec4& colour = glm::vec4(1, 1, 1, 1), const float elasticity = 1.0f,
		const float linearDrag = 0.0f, const float angularDrag = 0.0f, const float 탎 = 0.0f, const float 탃 = 0.0f);
	Poly(const std::vector<glm::vec2>& vertices, const float inclination, const float speed, const float mass,
		const glm::vec4& colour = glm::vec4(1, 1, 1, 1), const float elasticity = 1.0f,
		const float linearDrag = 0.0f, const float angularDrag = 0.0f, const float 탎 = 0.0f, const float 탃 = 0.0f);
	~Poly();

	virtual void MakeGizmo();
	glm::vec2 Project(const glm::vec2& axis) const;
	float GetOverlap(const glm::vec2& proj1, const glm::vec2& proj2) const;
	bool Overlap(const glm::vec2& proj1, const glm::vec2& proj2) const;
	std::vector<glm::vec2> ContactPoints(const Poly* other, const glm::vec2& normal) const;
	Edge BestEdge(const glm::vec2& normal) const;
	void Clip(std::vector<glm::vec2> clippedPoints, const glm::vec2& vert1, const glm::vec2& vert2, const glm::vec2& normal, const float overlap) const;
	glm::vec2 Cross(const glm::vec2& vect, const float scalar) const;

	std::vector<glm::vec2> GetAxis() const;
	std::vector<glm::vec2> GetVertices() const { return m_vertices; }
	float GetRadius() const { return m_radius; }

protected:
	float m_radius;
	std::vector<glm::vec2> m_vertices;
};