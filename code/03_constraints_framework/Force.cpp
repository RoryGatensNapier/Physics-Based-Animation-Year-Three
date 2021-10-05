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
	float curLength = glm::distance(p2.Position(), p1.Position());
	auto unitVector = (p2.Position() - p1.Position()) / curLength;
	auto p1_vel1D = glm::dot(unitVector, p1.Velocity());
	auto p2_vel1D = glm::dot(unitVector, p2.Velocity());
	auto force_p1 = (-ks * (restLength - curLength)) - (kd * (p1.Velocity()));
	auto force_p2 = (-ks * (restLength - curLength)) - (kd * (p2.Velocity()));
	/*auto forceSpring = -ks * (restLength - curLength);
	auto forceDamp_p1 = -kd * p1.Velocity();
	auto forceDamp_p2 = -kd * p2.Velocity();
	auto forceSD_p1 = forceSpring + forceDamp_p1;
	auto forceSD_p2 = forceSpring + forceDamp_p2;
	auto finalForce_p1 = forceSD_p1 * p1_vel1D;
	auto finalForce_p2 = -forceSD_p2 * p2_vel1D;*/
	p1.ApplyForce(force_p1);
	p2.ApplyForce(-force_p2);

}