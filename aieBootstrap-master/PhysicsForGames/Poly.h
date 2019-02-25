#pragma once

#include "Rigidbody.h"
#include "Sphere.h"
#include <vector>

class Poly : public Rigidbody
{
public:
	Poly(const glm::vec2& position, const std::vector<glm::vec2>& vertices, const glm::vec2& velocity, const float mass,
		const glm::vec4& colour = glm::vec4(1, 1, 1, 1), const float elasticity = 1.0f,
		const float linearDrag = 0.0f, const float angularDrag = 0.0f, const float 탎 = 0.0f, const float 탃 = 0.0f);
	Poly(const glm::vec2& position, const std::vector<glm::vec2>& vertices, const float inclination, const float speed, const float mass,
		const glm::vec4& colour = glm::vec4(1, 1, 1, 1), const float elasticity = 1.0f,
		const float linearDrag = 0.0f, const float angularDrag = 0.0f, const float 탎 = 0.0f, const float 탃 = 0.0f);
	~Poly();

	glm::vec2 Project(const glm::vec2& axis) const;

	std::vector<glm::vec2> GetVertices() const { return m_vertices; }
	std::vector<glm::vec2> GetAxis() const;
	Sphere GetBroadCollider() const;

protected:
	float m_radius;
	std::vector<glm::vec2> m_vertices;
};