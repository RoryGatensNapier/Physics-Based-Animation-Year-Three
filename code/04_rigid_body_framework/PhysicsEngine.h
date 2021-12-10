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

	void RigidBodyInit(const Shader* rbShader, const Mesh* rbMesh, glm::vec3 pos, glm::vec3 scale, glm::vec3 initVel, glm::vec3 initRotVel);
	RigidBody SpheresInit(const Shader* rbShader, const Mesh* rbMesh, glm::vec3 pos, glm::vec3 initVel, glm::vec3 initRotVel);
	void Task1Update(float deltaTime, float totalTime);
	void BroadPhaseChunkAssignment();
	
	// ... rest of the tasks here

private:

	PhysicsBody ground;


	RigidBody rbody1;

	//RigidBody Balls[10];
	std::vector<RigidBody> Balls;
	int ballCount = 30;
	float ballElasticity = 0.8f;
	float wallElasticity = 0.4f;

	bool toggleSim = false;
	int simMode = 1;
};