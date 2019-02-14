#include "PhysicsForGamesApp.h"
#include "Texture.h"
#include "Font.h"
#include "Input.h"
#include <iostream>

PhysicsForGamesApp::PhysicsForGamesApp()
{
}

PhysicsForGamesApp::~PhysicsForGamesApp()
{
}

bool PhysicsForGamesApp::startup()
{
	// increase the 2D line count to maximise the number of objects we can draw
	aie::Gizmos::create(255U, 255U, 65535U, 65535U);

	m_2dRenderer = new aie::Renderer2D();
	m_font = new aie::Font("../bin/font/consolas.ttf", 32);

	m_physicsScene = new PhysicsScene();
	m_physicsScene->SetGravity(glm::vec2(0, -9.8f)); // gravity on
	m_physicsScene->SetTimeStep(0.5f);

	//// two spheres that will move together and collide
	//Sphere* ball1 = new Sphere(glm::vec2(-20, 0), glm::vec2(0, 0), 4.0f, 4, glm::vec4(1, 0, 0, 1));
	//Sphere* ball2 = new Sphere(glm::vec2(10, 0), glm::vec2(0, 0), 4.0f, 4, glm::vec4(0, 1, 0, 1));
	//m_physicsScene->AddActor(ball1);
	//m_physicsScene->AddActor(ball2);
	//ball1->ApplyForce(glm::vec2(30, 0));
	//ball2->ApplyForce(glm::vec2(-15, 0));

	//// initialise the rocket object
	//m_rocket = new Sphere(glm::vec2(0, 0), glm::vec2(0, 0), 10.0f, 5.0f, glm::vec4(1, 0, 0, 1), false);
	//m_physicsScene->AddActor(m_rocket);

	float radius = 1.0f;
	float speed = 30.0f;
	glm::vec2 startPos(-40.0f, 0.0f);
	float inclination = 0.785398f; // 45 degrees

	m_physicsScene->AddActor(new Sphere(startPos, inclination, speed, 1, radius, glm::vec4(1, 0, 0, 1)));
	SetupContinuousDemo(startPos, inclination, speed, -9.8f);

	return true;
}

void PhysicsForGamesApp::shutdown()
{
	delete m_font;
	delete m_2dRenderer;

	aie::Gizmos::destroy();

	delete m_physicsScene;
	m_physicsScene = nullptr;

	//m_fuel.clear();
}

void PhysicsForGamesApp::update(float deltaTime)
{
	// input example
	aie::Input* input = aie::Input::getInstance();

	//aie::Gizmos::clear();

	//static float timer = 0.0f;
	//timer += deltaTime;

	//// checks that the rocket still has some mass and uses a fixed time step of 0.05
	//if (timer >= 0.05f && m_rocket->GetMass() > 5.0f)
	//{
	//	// decreases the total mass of the rocket
	//	m_rocket->SetMass(m_rocket->GetMass() - 0.1f);
	//	// adds a new exhaust fume underneath the rocket
	//	m_fuel.push_back(new Sphere(m_rocket->GetPosition() - glm::vec2(0, 2.5f), glm::vec2(0, 0), 0.1f, 0.2f, glm::vec4(0, 1, 0, 1), false));

	//	// adds the exhaust fume to the fuel container
	//	m_physicsScene->AddActor(m_fuel.back());
	//	// applies a force seperating the fume and the rocket
	//	m_rocket->ApplyForceToActor(m_fuel.back(), glm::vec2(0, -10.0f));

	//	timer -= 0.05f;
	//}

	//// deallocates an exhaust fume after a while
	//if (m_fuel.size() == 20)
	//{
	//	m_physicsScene->RemoveActor(m_fuel[0]);
	//	Sphere* temp = m_fuel[0];
	//	m_fuel.erase(m_fuel.begin());
	//	delete temp;
	//	temp = nullptr;
	//}

	m_physicsScene->Update(deltaTime);
	m_physicsScene->UpdateGizmos();

	// exit the application
	if (input->isKeyDown(aie::INPUT_KEY_ESCAPE))
		quit();
}

void PhysicsForGamesApp::draw()
{
	// wipe the screen to the background colour
	clearScreen();

	// begin drawing sprites
	m_2dRenderer->begin();

	// draw your stuff here!
	static float aspectRatio = 16.0f / 9.0f;
	aie::Gizmos::draw2D(glm::ortho<float>(-100.0f, 100.0f, -100.0f / aspectRatio, 100.0f / aspectRatio, -1.0f, 1.0f));

	// output some text, uses the last used colour
	m_2dRenderer->drawText(m_font, "Press ESC to quit", 0, 0);

	// done drawing sprites
	m_2dRenderer->end();
}

void PhysicsForGamesApp::SetupContinuousDemo(const glm::vec2 & startPos, const float inclination, const float speed, const float gravity)
{
	float t = 0.0f;
	float tStep = 0.5f;
	float radius = 1.0f;
	int segments = 12;
	glm::vec4 colour = glm::vec4(1, 1, 0, 1);

	glm::vec2 velocity = glm::vec2(cosf(inclination) * speed, sinf(inclination) * speed);

	while (t <= 5.0f)
	{
		// x = x0 + u.x * t
		float x = startPos.x + (velocity.x * t);
		// y = y0 + u.y * t + 0.5 * g * t^2
		float y = startPos.y + (velocity.y * t) + (0.5f * gravity * powf(t, 2));

		aie::Gizmos::add2DCircle(glm::vec2(x, y), radius, segments, colour);
		t += tStep;
	}
}