#pragma once

#include "Application.h"
#include "Renderer2D.h"
#include "PhysicsScene.h"

class CollisionApp : public aie::Application
{
public:
	CollisionApp();
	virtual ~CollisionApp();

	virtual bool startup();
	virtual void shutdown();

	virtual void update(float deltaTime);
	virtual void draw();

protected:
	aie::Renderer2D* m_2dRenderer;
	aie::Font* m_font;
	// the scene where all physics take place
	PhysicsScene* m_physicsScene;
};