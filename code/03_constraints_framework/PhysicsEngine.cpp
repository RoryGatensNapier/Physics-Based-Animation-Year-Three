#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"
#include "Force.h"

using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

double totalTime = 0.0;
double physAcca = 0.0;

Particle particles[5];
const int prt_len = sizeof(particles) / sizeof(particles[0]);

//vec3 phys_log_Pos[prt_len], phys_log_Vel[prt_len], p_arr[prt_len], v_arr[prt_len] = { vec3(0) };

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

void SymplecticEuler(Particle& p, float mass, const vec3& force, const vec3& impulse, float dt)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement
	vec3 accel = force / mass;
	p.SetVelocity(p.Velocity() + (accel * dt) + (impulse));
	p.SetPosition(p.Position() + (p.Velocity() * dt));
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

vec3 CollisionImpulse(Particle& pobj, const glm::vec3& cubeCentre, float cubeHalfExtent, float coefficientOfRestitution = 0.9f)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate collision impulse
	vec3 impulse{ 0.0f };
	int signage = 0;
	float P_axisVals[3] = { pobj.Position().x, pobj.Position().y, pobj.Position().z };
	float cube_axisVals[3] = { cubeCentre.x, cubeCentre.y, cubeCentre.z };
	for (auto x = 0; x < sizeof(P_axisVals)/sizeof(P_axisVals[0]); x++)
	{
		if (P_axisVals[x] >= 0)
		{
			signage = 1;
		}
		else
		{
			signage = -1;
		}
		if (P_axisVals[x] >= cube_axisVals[x] + (cubeHalfExtent) || P_axisVals[x] <= -(cubeHalfExtent))
		{
			//todo
		}
	}
	if (pobj.Position().x >= cubeCentre.x + (cubeHalfExtent) || pobj.Position().x <= cubeCentre.x - (cubeHalfExtent - 1))
	{
		impulse += vec3(2 * (pobj.Velocity().x * -coefficientOfRestitution), pobj.Velocity().y, pobj.Velocity().z);
		pobj.SetPosition(vec3(cubeCentre.x + (signage * cubeHalfExtent) - 1, pobj.Position().y, pobj.Position().z));
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

Particle InitParticle(const Mesh* particleMesh, const Shader* particleShader, vec4 Colour, vec3 Position, vec3 Scale, float mass, vec3 initVelocity)
{
	Particle newParticle;
	newParticle.SetMesh(particleMesh);
	newParticle.SetShader(particleShader);
	newParticle.SetColor(Colour);
	newParticle.SetPosition(Position);
	newParticle.SetScale(Scale);
	newParticle.SetMass(mass);
	newParticle.SetVelocity(initVelocity);
	return newParticle;
}

void Update_TimestepAlphaEval(Particle particle, int iter, vec3 loggedPos[], vec3 loggedVel[], float alpha)
{
	if (particle.IsFixed())
	{
		//particle.SetPosition(p_arr[iter]);
		//particle.SetVelocity(v_arr[iter]);
	}
	vec3 newState_Pos = particle.Position() * alpha + loggedPos[iter] * (1.0f - alpha);
	vec3 newState_Vel = particle.Velocity() * alpha + loggedVel[iter] * (1.0f - alpha);
	particle.SetPosition(newState_Pos);
	particle.SetVelocity(newState_Vel);
	loggedPos[iter] = particle.Position();
	loggedVel[iter] = particle.Velocity();
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
		particles[x] = InitParticle(meshDb.Get("cube"), defaultShader, vec4(0.2 * x,0,0,1), vec3(0, 5-x, 0), vec3(0.1), 1, vec3(0));
		//p_arr[x] = particles[x].Position();
		//v_arr[x] = particles[x].Velocity();
	}
	particles[0].SetFixed();
}

void PhysicsEngine::Task1Init()
{
}

void PhysicsEngine::Task1Update(float deltaTime, float totalTime)
{
	printf("new frame \n");

	vec3 acceleration = GRAVITY;
	for (int x = 0; x < prt_len; x++)
	{
		particles[x].ClearForcesImpulses();
		//p_arr[x] = particles[x].Position(), v_arr[x] = particles[x].Velocity();
		if (!particles[x].IsFixed())
		{
			Force::Gravity(particles[x]);
		}
		if (x > prt_len)
		{
			continue;
		}
		else
		{
			Force::Hooke(particles[x], particles[x + 1], 1.f, 10.f, 0.5f);
		}
	}
	for (int x = 0; x < prt_len; x++)
	{
		if (particles[x].IsFixed())
		{
			SymplecticEuler(particles[x], particles[x].Mass(), vec3(0), vec3(0), deltaTime);
		}
		else
		{
			SymplecticEuler(particles[x], particles[x].Mass(), particles[x].AccumulatedForce(), particles[x].AccumulatedImpulse(), deltaTime);
		}
	}
}


// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	double timeStep = 0.001;
	float alpha = 0;
	if (deltaTime > 0.25)
	{
		deltaTime = 0.25;
	}
	physAcca += deltaTime;
	while (physAcca >= timeStep)
	{
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// TODO: Handle collisions and calculate impulse
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		Task1Update(deltaTime, totalTime);
		totalTime += timeStep;
		physAcca -= timeStep;
	}
	alpha = physAcca / timeStep;
	/*for (int x = 0; x < prt_len; x++)
	{
		Update_TimestepAlphaEval(particles[x], x, p_arr, v_arr, alpha);
	}*/
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