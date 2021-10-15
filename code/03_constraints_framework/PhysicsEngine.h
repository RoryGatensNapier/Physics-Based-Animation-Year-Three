#pragma once


#include <glm/glm.hpp>

#include "PhysicsObject.h"

#define LEN 10

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

	void Task1Init(MeshDb& meshDb, const Shader* defaultShader);
	void Task1Update(float deltaTime, float totalTime); // 5-particle chain
	// ... rest of the tasks here

	void TaskClothSim(float deltaTime, float totalTime);
	void InitClothSim(MeshDb& meshDb, const Shader* defaultShader);

private:


	PhysicsBody ground;
	int prt_len = LEN;
	int prt_lim = LEN - 1;
	Particle particles[LEN] = {};
	Particle pt_2d[LEN][LEN] = {};
	node_Particle p_nodes[LEN][LEN] = {};
	bool toggleSim = false;
	bool toggleBlowDryer = false;
	int simMode = 1;

	//glm::vec4 particle_colour[5] = { glm::vec4(1,0,0,1), glm::vec4(0,1,0,1), glm::vec4(0,0,1,1), glm::vec4(1,0,1,1), glm::vec4(1,1,0,1) };
};