#pragma once

#include "Rigidbody.h"
#include <vector>

class AABB : public Rigidbody
{
public:
	AABB(const glm::vec2& position, const glm::vec2& velocity, const float width, const float height, const float mass,
		const glm::vec4& colour = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f), const float elasticity = 1.0f,
		const float linearDrag = 0.0f, const float angularDrag = 0.0f, const float 탎 = 0.0f, const float 탃 = 0.0f);
	AABB(const glm::vec2& position, const float inclination, const float speed, const float width, const float height, const float mass,
		const glm::vec4& colour = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f), const float elasticity = 1.0f,
		const float linearDrag = 0.0f, const float angularDrag = 0.0f, const float 탎 = 0.0f, const float 탃 = 0.0f);
	~AABB();

	// draws the box
	virtual void MakeGizmo();

	// subtracts half the width from the x position and half the height from the y position to get the minimum position that lies within the box
	glm::vec2 GetMin() const { return glm::vec2(m_position.x - (m_width / 2.0f), m_position.y - (m_height / 2.0f)); } // bottom left corner
	// adds half the width to the x position and half the height to the y position to get the maximum position that lies within the box
	glm::vec2 GetMax() const { return glm::vec2(m_position.x + (m_width / 2.0f), m_position.y + (m_height / 2.0f)); } // top right corner
	float GetWidth() const { return m_width; }
	float GetHeight() const { return m_height; }
	// returns half of the width and height
	glm::vec2 GetExtents() const { return glm::vec2(m_width * 0.5f, m_height * 0.5f); }
	std::vector<glm::vec2> GetCorners() const;

protected:
	// stores the width of the box
	float m_width;
	// stores the height of the box
	float m_height;
};