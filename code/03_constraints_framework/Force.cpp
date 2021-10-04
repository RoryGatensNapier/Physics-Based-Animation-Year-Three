#include "PhysicsEngine.h"
#include "Force.h"
#include "PhysicsObject.h"

using namespace glm;

void Force::Gravity(Particle& p)
{
	auto force = vec3(0, -9.81, 0) * p.Mass();
	p.ApplyForce(force);
}

void Force::Drag(Particle& p)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Should apply the aerodynamic drag force
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void Force::Hooke(Particle& p1, Particle& p2, float restLength, float ks, float kd)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Should return the aerodynamic drag force
	float curLength = p2.Position().length() - p1.Position().length();
	auto forceSpring = -ks * (restLength - curLength);
	auto forceDamp_p1 = -kd * p1.Velocity();
	auto forceDamp_p2 = -kd * p2.Velocity();
	auto forceSD_p1 = forceSpring + forceDamp_p1;
	auto forceSD_p2 = forceSpring + forceDamp_p2;
	p1.ApplyForce(forceSD_p1);
	p2.ApplyForce(forceSD_p2);

}