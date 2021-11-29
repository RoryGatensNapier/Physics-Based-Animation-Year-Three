#include "PhysicsObject.h"

#include <glm/glm.hpp>

#include "PhysicsEngine.h"
#include "Mesh.h"
#include "Shader.h"


void PhysicsBody::Draw(const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix) const
{
	m_shader->Use();
	//m_shader->SetUniform("model", ModelMatrix());
	//m_shader->SetUniform("view", viewMatrix);
	//m_shader->SetUniform("projection", projectionMatrix);
	m_shader->SetUniform("color", m_color);

	auto mvp = projectionMatrix * viewMatrix * ModelMatrix();
	m_shader->SetUniform("modelViewProjectionMatrix", mvp);
	m_shader->SetUniform("normalMatrix", transpose(inverse(viewMatrix * ModelMatrix())));
	m_mesh->DrawVertexArray();
}

glm::mat3 RigidBody::InverseInertia(glm::vec3 dimensions)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate the matrix
	glm::mat3 inertia = glm::mat3(0);
	inertia = glm::mat3({ (1.0 / 12.0) * Mass() * (pow(dimensions.y, 2) + pow(dimensions.z, 2)), 0, 0 }, { 0, (1.0 / 12.0) * Mass() * (pow(dimensions.x, 2) + pow(dimensions.z, 2)), 0 }, { 0, 0, (1.0 / 12.0) * Mass() * (pow(dimensions.x, 2) + pow(dimensions.y, 2)) });
	inertia = glm::inverse(inertia);
	auto ort_Transposed = glm::transpose(Orientation());
	auto inverse_inertia = (glm::mat3)Orientation() * inertia * (glm::mat3)ort_Transposed;
	return inverse_inertia;
}

glm::mat3 RigidBody::Inertia(glm::vec3 dimensions)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate the matrix
	glm::mat3 inertia = glm::mat3(0);
	inertia = glm::mat3({ (1.0 / 12.0) * Mass() * (pow(dimensions.y, 2) + pow(dimensions.z, 2)), 0, 0 }, { 0, (1.0 / 12.0) * Mass() * (pow(dimensions.x, 2) + pow(dimensions.z, 2)), 0 }, { 0, 0, (1.0 / 12.0) * Mass() * (pow(dimensions.x, 2) + pow(dimensions.y, 2)) });
	auto ort_Transposed = glm::transpose(Orientation());
	auto new_inertia = (glm::mat3)Orientation() * inertia * (glm::mat3)ort_Transposed;
	return new_inertia;
}

glm::mat3 RigidBody::InverseInertia_Sphere(float radius)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate the matrix
	glm::mat3 inertia = glm::mat3(0);
	inertia = glm::mat3({ (2.0 / 5) * Mass() * (pow(radius, 2)), 0, 0 }, { 0, (2.0 / 5) * Mass() * (pow(radius, 2)), 0 }, { 0, 0, (2.0 / 5) * Mass() * (pow(radius, 2)) });
	inertia = glm::inverse(inertia);
	auto ort_Transposed = glm::transpose(Orientation());
	auto inverse_inertia = (glm::mat3)Orientation() * inertia * (glm::mat3)ort_Transposed;
	return inverse_inertia;
}

glm::mat3 RigidBody::Inertia_Sphere(float radius)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate the matrix
	glm::mat3 inertia = glm::mat3(0);
	inertia = glm::mat3({ (2.0 / 5) * Mass() * (pow(radius, 2)), 0, 0 }, { 0, (2.0 / 5) * Mass() * (pow(radius, 2)), 0 }, { 0, 0, (2.0 / 5) * Mass() * (pow(radius, 2)) });
	auto ort_Transposed = glm::transpose(Orientation());
	auto inverse_inertia = (glm::mat3)Orientation() * inertia * (glm::mat3)ort_Transposed;
	return inverse_inertia;
}

