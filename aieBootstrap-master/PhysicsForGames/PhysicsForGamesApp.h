#pragma once

#include "Application.h"
#include "Renderer2D.h"
#include "PhysicsScene.h"
#include "Sphere.h"

class PhysicsForGamesApp : public aie::Application
{
public:
	PhysicsForGamesApp();
	virtual ~PhysicsForGamesApp();

	virtual bool startup();
	virtual void shutdown();

	virtual void update(float deltaTime);
	virtual void draw();

	void SetupContinuousDemo(const glm::vec2& startPos, const float inclination, const float speed, const float gravity);

protected:
	aie::Renderer2D* m_2dRenderer;
	aie::Font* m_font;
	// the scene where all physics take place
	PhysicsScene* m_physicsScene;

	//// rocket object
	//Sphere* m_rocket;
	//// container of fuel
	//std::vector<Sphere*> m_fuel;
};