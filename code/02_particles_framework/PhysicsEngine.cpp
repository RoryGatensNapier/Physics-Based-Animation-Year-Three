#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"

using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);
float t = 0;
float y = 0;


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
	vel = vel + (accel * dt) + impulse;
	pos = pos + (vel * dt);
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void RungeKutta4th_Velocity(float& posOnAxis, float mass, const float& accelOnAxis, const float& impulseOntoAxis, float dt)
{
	t += dt;
	float h2 = dt / 2;
	float h6 = dt / 6;
	float k1 = posOnAxis + (dt * accelOnAxis);
	float k2 = (posOnAxis + (dt * (k1 / 2))) + (accelOnAxis * t + dt / 2);
	float k3 = (posOnAxis + (dt * (k2 / 2))) + (accelOnAxis * t + dt / 2);
	float k4 = (posOnAxis + (dt * k3)) + (accelOnAxis * t + dt);
	y = y + 1 / 6 * (dt * (k1 + k2 + k3 + k4));

	posOnAxis = y;
}

void RungeKutta4th_Accel(float& velOnAxis, float mass, const float& accelOnAxis, const float& impulseOntoAxis, float dt)
{
	t += dt;
	float h2 = dt / 2;
	float h6 = dt / 6;
	float k1 = velOnAxis + (dt * accelOnAxis);
	float k2 = (velOnAxis + (dt * (k1 / 2))) + (accelOnAxis * t + dt / 2);
	float k3 = (velOnAxis + (dt * (k2 / 2))) + (accelOnAxis * t + dt / 2);
	float k4 = (velOnAxis + (dt * k3)) + (accelOnAxis * t + dt);
	y = y + 1 / 6 * (dt * (k1 + k2 + k3 + k4));

	velOnAxis = y;
}

vec3 CollisionImpulse(Particle& pobj, const glm::vec3& cubeCentre, float cubeHalfExtent, float coefficientOfRestitution = 0.9f)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate collision impulse
	vec3 impulse{ 0.0f };
	if (abs(pobj.Position().x) > cubeCentre.x + cubeHalfExtent)
	{
		impulse = vec3(pobj.Velocity().x * -coefficientOfRestitution, pobj.Velocity().y, pobj.Velocity().z);
		pobj.SetVelocity(vec3(0, pobj.Velocity().y, pobj.Velocity().z));
	}
	if (abs(pobj.Position().y) > cubeCentre.y + cubeHalfExtent)
	{
		impulse = vec3(pobj.Velocity().x, pobj.Velocity().y * -coefficientOfRestitution, pobj.Velocity().z);
		pobj.SetVelocity(vec3(pobj.Velocity().x, 0, pobj.Velocity().z));
	}
	if (abs(pobj.Position().z) > cubeCentre.z + cubeHalfExtent)
	{
		impulse = vec3(pobj.Velocity().x, pobj.Velocity().y, pobj.Velocity().z * -coefficientOfRestitution);
		pobj.SetVelocity(vec3(pobj.Velocity().x, pobj.Velocity().y, 0));
	}
	return impulse;
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

vec3 BlowDryerForce(const vec3& particlePosition, float cone_y_base, float cone_y_tip, float cone_r_base, float max_force = 100)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate blow dryer force
	vec3 force = {0,0,0};
	return force;
}

// This is called once
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");
	auto particleMesh = meshDb.Get("tetra");
	auto groundMesh = meshDb.Get("plane");

	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));
	auto mesh = meshDb.Get("cube");

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(10.0f));

	// Initialise particle
	particle.SetMesh(mesh);
	particle.SetShader(defaultShader);
	particle.SetColor(vec4(1, 0, 0, 1));
	particle.SetPosition(vec3(0, 5, 0));
	particle.SetScale(vec3(0.1f));
	particle.SetVelocity(vec3(0.f, 0.0f, 0.f));

	camera = Camera(vec3(0, 2.5, 10));

}

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Handle collisions and calculate impulse
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	auto impulse = CollisionImpulse(particle, glm::vec3(0.0f, 5.0f, 0.0f), 5.0f);// , 1.0f);
	// Calculate acceleration by accumulating all forces (here we just have gravity) and dividing by the mass
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement a simple integration scheme
	vec3 p = particle.Position(), v = particle.Velocity();
	vec3 acceleration = vec3(0.0f, -9.81f, 0.0f);
	SymplecticEuler(p,v, particle.Mass(), acceleration, impulse, deltaTime);
	particle.SetPosition(p);
	particle.SetVelocity(v);

	

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	particle.Draw(viewMatrix, projMatrix);
	ground.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
	case GLFW_KEY_1:
		printf("Key 1 was %s\n", pressed ? "pressed" : "released");
		break; // don't forget this at the end of every "case" statement!
	default:
		break;
	}
}