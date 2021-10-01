#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"

using namespace glm;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);
float t = 0;
float y = 0;

double physTime = 0.0;
double physDeltaTime = 0.01;
double currentTime = glfwGetTime();
double physAcca = 0.0;

vec3 p_verlet;
vec3 phys_log_Pos[5], phys_log_Vel[5], p_arr[5], v_arr[5] = { vec3(0) };


void ExplicitEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement
	vec3 newvel = vel + (dt * accel) + (impulse);// *dt);
	pos = pos + (vel * dt);
	vel = newvel;
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement
	vel = vel + (accel * dt) + (impulse);// *dt);
	pos = pos + (vel * dt);
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void VerletIntegration(vec3& currentPos, vec3& previousPos, vec3& vel, float mass, const vec3& currentAccel, const vec3& impulse, float dt)
{
	vec3 temp = currentPos;
	currentPos = (vec3(2) * currentPos) - previousPos + ((dt * dt) * currentAccel);
	previousPos = temp;
}

vec3 CollisionImpulse(Particle& pobj, const glm::vec3& cubeCentre, float cubeHalfExtent, float coefficientOfRestitution = 0.9f)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate collision impulse
	vec3 impulse{ 0.0f };
	float signs[] = { signbit(pobj.Position().x), signbit(pobj.Position().y), signbit(pobj.Position().z) };
	for (auto x : signs)
	{
		if (x == 0)
			x = 1;
	}
	if (pobj.Position().x >= cubeCentre.x + signs[0] * (cubeHalfExtent - 1))
	{
		impulse = vec3(pobj.Velocity().x * -coefficientOfRestitution, pobj.Velocity().y, pobj.Velocity().z);
		pobj.SetVelocity(vec3(0, pobj.Velocity().y, pobj.Velocity().z));
	}
	if (pobj.Position().y >= cubeCentre.y + signs[1] * (cubeHalfExtent - 1))
	{
		impulse = vec3(pobj.Velocity().x, pobj.Velocity().y * -coefficientOfRestitution, pobj.Velocity().z);
		pobj.SetVelocity(vec3(pobj.Velocity().x, 0, pobj.Velocity().z));
	}
	if (pobj.Position().z >= cubeCentre.z + signs[2] * (cubeHalfExtent - 1))
	{ 
		impulse = vec3(pobj.Velocity().x, pobj.Velocity().y, pobj.Velocity().z * -coefficientOfRestitution);
		pobj.SetVelocity(vec3(pobj.Velocity().x, pobj.Velocity().y, 0));
	}
	return impulse;
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

bool ConicalCalculation(vec3 particlePos, double base, double tip, double radius)
{
	//calculate trajectory (direction/maginitude(velocity)), compare with maximum intersect on vector trajectory with limit of volume of cone from formula.
	double coneHeight = base + tip;
	double coneRatio = coneHeight / radius;
	if (particlePos.y >= particlePos.x * coneRatio && particlePos.y >= particlePos.z * coneRatio && particlePos.y < coneHeight && particlePos.y > 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

vec3 BlowDryerForce(const vec3& particlePosition, float cone_y_base, float cone_y_tip, float cone_r_base, float max_force = 100)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate blow dryer force
	vec3 force = {0,5,0};
	float posArr[3] = { particlePosition.x, particlePosition.y, particlePosition.z };
	float linearMultiplier[sizeof(posArr)] = { 0 };
	for (int x = 0; x < sizeof(posArr); x++)
	{
		if (posArr[x] <= -1)
		{
			linearMultiplier[x] = 1 / posArr[x];
		}
		if (posArr[x] < 0 && posArr[x] > - 1)
		{
			linearMultiplier[x] = 1 * posArr[x];
		}
		if (posArr[x] == 0)
		{
			linearMultiplier[x] = 1;
		}
		else if (posArr[x] < 1 && posArr[x] > 0)
		{
			linearMultiplier[x] = 1 * posArr[x];
		}
		else if (posArr[x] > 1)
		{
			linearMultiplier[x] = 1 / posArr[x];
		}
	}
	if (ConicalCalculation(particlePosition, cone_y_base, cone_y_tip, cone_r_base))
	{
		force = vec3(force.x * linearMultiplier[0], force.y * linearMultiplier[1], force.z * linearMultiplier[2]);
		return force;
	}
	else
	{
		return vec3(0);
	}
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

	camera = Camera(vec3(0, 2.5, 10));

	for (int x = 1; x < 5; x++)
	{
		particles[x] = InitParticle(mesh, defaultShader, vec4(x/3, x/2, x/1, 1), vec3(-1.5 + x, 2, 0), vec3(0.1f), vec3(0));
		p_arr[x] = particles[x].Position();
		v_arr[x] = particles[x].Velocity();
	}

	// Initialise fan particle
	particles[0].SetMesh(mesh);
	particles[0].SetShader(defaultShader);
	particles[0].SetColor(vec4(1, 0, 0, 1));
	particles[0].SetPosition(vec3(0, 2, 0));
	particles[0].SetScale(vec3(0.1f));
	particles[0].SetVelocity(vec3(0.f, 0.0f, 0.f));

	p_verlet = particles[2].Position();
}

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
#if 0
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
#endif
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
		vec3 impulse[5];
		for (int x = 0; x < sizeof(particles) / sizeof(particles[0]); x++)
		{
			impulse[x] = CollisionImpulse(particles[x], glm::vec3(0.0f, 5.0f, 0.0f), 5.0f);// , 1.0f);
		}
		for (int x = 0; x < sizeof(particles) / sizeof(particles[0]); x++)
		{
			p_arr[x] = particles[x].Position(), v_arr[x] = particles[x].Velocity();
		}
		impulse[0] += BlowDryerForce(particles[0].Position(), 1, 5, 3);
		// Calculate acceleration by accumulating all forces (here we just have gravity) and dividing by the mass
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		// TODO: Implement a simple integration scheme
		vec3 acceleration = GRAVITY;
		SymplecticEuler(p_arr[0], v_arr[0], particles[0].Mass(), acceleration, impulse[0], physDeltaTime);
		ExplicitEuler(p_arr[1] , v_arr[1], particles[1].Mass(), acceleration, impulse[1], physDeltaTime);
		VerletIntegration(p_arr[2], p_verlet, v_arr[2], particles[2].Mass(), acceleration, impulse[2], physDeltaTime);
		SymplecticEuler(p_arr[3], v_arr[3], particles[3].Mass(), acceleration, impulse[3], physDeltaTime);
		for (int x = 0; x < sizeof(particles) / sizeof(particles[0]); x++)
		{
			particles[x].SetPosition(p_arr[x]);
			particles[x].SetVelocity(v_arr[x]);
		}
		physTime += physDeltaTime;
		physAcca -= physDeltaTime;
	}
	for (int x = 0; x < sizeof(particles)/sizeof(particles[0]); x++)
	{
		Update_TimestepAlphaEval(particles[x], p_arr[x], v_arr[x], physDeltaTime);
	}


	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	for (auto particle : particles)
	{
		particle.Draw(viewMatrix, projMatrix);
	}
	ground.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
	case GLFW_KEY_1:
		printf("Key 1 was %s\n", pressed ? "pressed" : "released");
		break; // don't forget this at the end of every "case" statement!
	case GLFW_KEY_I:
		if (pressed)
			physDeltaTime += 0.005;
		break;
	case GLFW_KEY_K:
		if (pressed)
			physDeltaTime -= 0.005;
		break;
	case GLFW_KEY_0:
		if (pressed)
			physDeltaTime = 0.01;
		break;
	default:
		break;
	}
}