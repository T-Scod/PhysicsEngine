#pragma once

#include "Rigidbody.h"

class AABB : public Rigidbody
{
public:
	AABB(const glm::vec2& position, const glm::vec2& velocity, const float mass, const float width, const float height, const glm::vec4& colour, const bool collision = true);
	AABB(const glm::vec2& position, const float inclination, const float speed, const float mass, const float width, const float height, const glm::vec4& colour, const bool collision = true);
	~AABB();

	virtual void MakeGizmo();

	glm::vec2 GetMin() const { return glm::vec2(m_position.x - (m_width / 2.0f), m_position.y - (m_height / 2.0f)); }
	glm::vec2 GetMax() const { return glm::vec2(m_position.x + (m_width / 2.0f), m_position.y + (m_height / 2.0f)); }
	float GetWidth() const { return m_width; }
	float GetHeight() const { return m_height; }
	glm::vec2 GetExtents() const { return glm::vec2(m_width * 0.5f, m_height * 0.5f); }
	glm::vec4 GetColour() const { return m_colour; }

protected:
	float m_width;
	float m_height;
	glm::vec4 m_colour;
};