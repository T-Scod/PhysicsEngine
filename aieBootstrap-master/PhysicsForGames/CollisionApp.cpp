#include "CollisionApp.h"
#include "Texture.h"
#include "Font.h"
#include "Input.h"
#include <iostream>
#include "Plane.h"
#include "Sphere.h"
#include "AABB.h"
#include "Poly.h"

CollisionApp::CollisionApp()
{
}

CollisionApp::~CollisionApp()
{
}

bool CollisionApp::startup()
{
	// increase the 2D line count to maximise the number of objects we can draw
	aie::Gizmos::create(255U, 255U, 65535U, 65535U);

	m_2dRenderer = new aie::Renderer2D();
	m_font = new aie::Font("../bin/font/consolas.ttf", 32);

	m_physicsScene = new PhysicsScene();
	m_physicsScene->SetGravity(glm::vec2(0, -10.0f));
	m_physicsScene->SetTimeStep(0.001f);

	Plane* top = new Plane({ 0.0f, -1.0f }, -55.0f);
	m_physicsScene->AddActor(top);
	Plane* bottom = new Plane({ 0.0f, 1.0f }, -55.0f);
	m_physicsScene->AddActor(bottom);
	Plane* plane1 = new Plane(glm::normalize(glm::vec2(0.707f, 0.707f)), -70.0f, { 1.0f, 1.0f, 1.0f, 1.0f }, false, 1.0f, 1.0f);
	m_physicsScene->AddActor(plane1);
	Plane* plane2 = new Plane(glm::normalize(glm::vec2(0.707f, -0.707f)), -70.0f, { 1.0f, 1.0f, 1.0f, 1.0f }, false, 1.0f, 1.0f);
	m_physicsScene->AddActor(plane2);
	Plane* plane3 = new Plane(glm::normalize(glm::vec2(-0.707f, -0.707f)), -70.0f, { 1.0f, 1.0f, 1.0f, 1.0f }, false, 1.0f, 1.0f);
	m_physicsScene->AddActor(plane3);
	Plane* plane4 = new Plane(glm::normalize(glm::vec2(-0.707f, 0.707f)), -70.0f, { 1.0f, 1.0f, 1.0f, 1.0f }, false, 1.0f, 1.0f);
	m_physicsScene->AddActor(plane4);

	AABB* staticBox = new AABB({ 40.0f, 0.0f }, { 0.0f, 0.0f }, 15.0f, 15.0f, 1.0f, { 1.0f, 1.0f, 1.0f, 1.0f }, false, true);
	m_physicsScene->AddActor(staticBox);
	Sphere* staticCircle = new Sphere({ -40.0f, 0.0f }, { 0.0f, 0.0f }, 10.0f, 1.0f, { 1.0f, 1.0f, 1.0f, 1.0f }, false, true);
	m_physicsScene->AddActor(staticCircle);

	AABB* box1 = new AABB({ 60.0f, 20.0f }, { 0.0f, 10.0f }, 7.0f, 7.0f, 2.0f, { 1.0f, 0.0f, 0.0f, 1.0f }, false, false);
	m_physicsScene->AddActor(box1);
	AABB* box2 = new AABB({ -65.0f, 7.0f }, { 3.0f, 3.0f }, 7.0f, 7.0f, 5.0f, { 0.0f, 1.0f, 1.0f, 1.0f }, false, false);
	m_physicsScene->AddActor(box2);
	AABB* box3 = new AABB({ 60.0f, -7.0f }, { 10.0f, 0.0f }, 7.0f, 7.0f, 2.0f, { 1.0f, 1.0f, 0.0f, 1.0f }, false, false);
	m_physicsScene->AddActor(box3);
	AABB* box4 = new AABB({ -65.0f, -20.0f }, { -3.0f, 3.0f }, 7.0f, 7.0f, 5.0f, { 1.0f, 0.0f, 1.0f, 1.0f }, false, false);
	m_physicsScene->AddActor(box4);
	Sphere* sphere1 = new Sphere({ -60.0f, 20.0f }, { 0.0f, -10.0f }, 5.0f, 2.0f, { 1.0f, 0.0f, 0.0f, 1.0f }, false, false);
	m_physicsScene->AddActor(sphere1);
	Sphere* sphere2 = new Sphere({ 65.0f, 7.0f }, { -3.0f, -3.0f }, 5.0f, 5.0f, { 0.0f, 1.0f, 1.0f, 1.0f }, false, false);
	m_physicsScene->AddActor(sphere2);
	Sphere* sphere3 = new Sphere({ -60.0f, -7.0f }, { -10.0f, 0.0f }, 5.0f, 2.0f, { 1.0f, 1.0f, 0.0f, 1.0f }, false, false);
	m_physicsScene->AddActor(sphere3);
	Sphere* sphere4 = new Sphere({ 65.0f, -20.0f }, { -3.0f, -3.0f }, 5.0f, 5.0f, { 1.0f, 0.0f, 1.0f, 1.0f }, false, false);
	m_physicsScene->AddActor(sphere4);

	return true;
}

void CollisionApp::shutdown()
{
	delete m_font;
	delete m_2dRenderer;

	aie::Gizmos::destroy();

	delete m_physicsScene;
	m_physicsScene = nullptr;
}

void CollisionApp::update(float deltaTime)
{
	// input example
	aie::Input* input = aie::Input::getInstance();

	aie::Gizmos::clear();

	m_physicsScene->Update(deltaTime);
	m_physicsScene->UpdateGizmos();

	// exit the application
	if (input->isKeyDown(aie::INPUT_KEY_ESCAPE))
		quit();
}

void CollisionApp::draw()
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