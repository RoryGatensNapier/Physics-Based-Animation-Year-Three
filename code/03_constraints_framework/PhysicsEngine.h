#pragma once


#include <glm/glm.hpp>

#include "PhysicsObject.h"

// Fwd declaration
class MeshDb;
class ShaderDb;
class Camera;

class PhysicsEngine
{
public:
	void Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb);
	void Update(float deltaTime, float totalTime);
	void Display(const glm::mat4& viewMatrix, const glm::mat4& projMatrix);
	void HandleInputKey(int keyCode, bool pressed);

	void Task1Init();
	void Task1Update(float deltaTime, float totalTime); // 5-particle chain
	// ... rest of the tasks here

private:


	PhysicsBody ground;
	Particle particles[5] = {};

	glm::vec4 particle_colour[5] = { glm::vec4(1,0,0,1), glm::vec4(0,1,0,1), glm::vec4(0,0,1,1), glm::vec4(1,0,1,1), glm::vec4(1,1,0,1) };

	int prt_len = sizeof(particles)/sizeof(particles[0]);
};