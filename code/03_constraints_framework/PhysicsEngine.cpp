#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"
#include "Force.h"

using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

double physTime = 0.0;
double physDeltaTime = 0.01;
double currentTime = glfwGetTime();
double physAcca = 0.0;

Particle particles[9];
const int prt_len = sizeof(particles) / sizeof(particles[0]);

vec3 phys_log_Pos[prt_len], phys_log_Vel[prt_len], p_arr[prt_len], v_arr[prt_len] = { vec3(0) };

void ExplicitEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement
	vel = vel + (accel * dt) + (impulse);
	pos = pos + (vel * dt);
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

vec3 CollisionImpulse(Particle& pobj, const glm::vec3& cubeCentre, float cubeHalfExtent, float coefficientOfRestitution = 0.9f)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate collision impulse
	vec3 impulse{ 0.0f };
	int signage = 0;
	if (pobj.Position().x >= cubeCentre.x + (cubeHalfExtent) || pobj.Position().x <= cubeCentre.x - (cubeHalfExtent - 1))
	{
		impulse += vec3(2 * (pobj.Velocity().x * -coefficientOfRestitution), pobj.Velocity().y, pobj.Velocity().z);
		//pobj.SetPosition(vec3(cubeCentre.x + (signage * cubeHalfExtent) - 1, pobj.Position().y, pobj.Position().z));
	}
	if (pobj.Position().y >= cubeCentre.y + (cubeHalfExtent) || pobj.Position().y <= cubeCentre.y - (cubeHalfExtent - 1))
	{
		impulse += vec3(pobj.Velocity().x, 2 * (pobj.Velocity().y * -coefficientOfRestitution), pobj.Velocity().z);
		//pobj.SetPosition(vec3(pobj.Position().x, cubeCentre.y + (signage * cubeHalfExtent) - 1, pobj.Position().z));
	}
	if (pobj.Position().z >= cubeCentre.z + (cubeHalfExtent) || pobj.Position().z <= cubeCentre.z - (cubeHalfExtent - 1))
	{
		impulse += vec3(pobj.Velocity().x, pobj.Velocity().y, 2 * (pobj.Velocity().z * -coefficientOfRestitution));
		//pobj.SetPosition(vec3(pobj.Position().x, pobj.Position().y, cubeCentre.z + (signage * cubeHalfExtent) - 1));
	}
	return impulse;
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

Particle InitParticle(const Mesh* particleMesh, const Shader* particleShader, vec4 Colour, vec3 Position, vec3 Scale, vec3 initVelocity)
{
	Particle newParticle;
	newParticle.SetMesh(particleMesh);
	newParticle.SetShader(particleShader);
	newParticle.SetColor(Colour);
	newParticle.SetPosition(Position);
	newParticle.SetScale(Scale);
	newParticle.SetVelocity(initVelocity);
	return newParticle;
}

void Update_TimestepAlphaEval(Particle particle, vec3& loggedPos, vec3& loggedVel, float alpha)
{
	vec3 newState_Pos = particle.Position() * (float)alpha + loggedPos * (float)(1.0 - alpha);
	vec3 newState_Vel = particle.Velocity() * (float)alpha + loggedVel * (float)(1.0 - alpha);
	particle.SetPosition(newState_Pos);
	particle.SetVelocity(newState_Vel);
	loggedPos = particle.Position();
	loggedVel = particle.Velocity();
}

// This is called once
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");
	auto groundMesh = meshDb.Get("plane");

	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(10.0f));

	camera = Camera(vec3(0, 5, 10));
	for (int x = 0; x < prt_len; x++)
	{
		particles[x] = InitParticle(meshDb.Get("cube"), defaultShader, vec4(0,0,0,1), vec3(-4 + x, 3, 0), vec3(0.1), vec3(0));
		p_arr[x] = particles[x].Position();
		v_arr[x] = particles[x].Velocity();
	}
}

void PhysicsEngine::Task1Init()
{
}

void PhysicsEngine::Task1Update(float deltaTime, float totalTime)
{
	// Calculate forces, then acceleration, then integrate
}


// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	double newTime = glfwGetTime();
	double frameTime = newTime - currentTime;
	if (frameTime > 0.25)
	{
		frameTime = 0.25;
	}
	currentTime = newTime;
	physAcca += frameTime;
	while (physAcca >= physDeltaTime)
	{
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// TODO: Handle collisions and calculate impulse
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		vec3 impulse[prt_len];
		vec3 acceleration = GRAVITY;
		for (int x = 0; x < prt_len; x++)
		{
			particles[x].ClearForcesImpulses();
			impulse[x] = CollisionImpulse(particles[x], glm::vec3(0.0f, 7.0f, 0.0f), 7.0f);// , 1.0f);
			p_arr[x] = particles[x].Position(), v_arr[x] = particles[x].Velocity();
			SymplecticEuler(p_arr[x], v_arr[x], particles[x].Mass(), acceleration, impulse[x], physDeltaTime);
			particles[x].SetPosition(p_arr[x]);
			particles[x].SetVelocity(v_arr[x]);
		}
		physTime += physDeltaTime;
		physAcca -= physDeltaTime;
	}
	for (int x = 0; x < prt_len; x++)
	{
		Update_TimestepAlphaEval(particles[x], p_arr[x], v_arr[x], physDeltaTime);
	}
}

// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	ground.Draw(viewMatrix, projMatrix);
	for (auto x : particles)
	{
		x.Draw(viewMatrix, projMatrix);
	}
}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
	default:
		break;
	}
}