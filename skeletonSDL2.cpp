//DH2323 skeleton code, Lab2 (SDL2 version)
#include <iostream>
#include <glm/glm.hpp>
#include "SDL2auxiliary.h"
#include "TestModel.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/constants.hpp>

using namespace std;
using glm::vec3;
using glm::mat3;

// ----------------------------------------------------------------------------
// STRUCTURES
struct Intersection
{
	vec3 position;
	float distance;
	int triangleIndex;
};
// ----------------------------------------------------------------------------
// GLOBAL VARIABLES
const int SCREEN_WIDTH = 400;
const int SCREEN_HEIGHT = 400;
SDL2Aux *sdlAux;
int t;
std::vector<Triangle> triangles;
float focalLength = 2.0f;
vec3 cameraPos(0.0f, 0.0f, -2.0f);
float translationSpeed = 0.1f;
mat3 R;
float yaw = 0.0f;
float rotationSpeed = 0.5f;

vec3 lightPos(0.0f, -0.5f, -0.7f);
vec3 lightColor = 14.0f * vec3(1.0f, 1.0f, 1.0f);
vec3 indirectLight = 0.5f * vec3(1.0f, 1.0f, 1.0f);
// ----------------------------------------------------------------------------
// FUNCTIONS
void Update();
void Draw();

bool ClosestIntersection(
	vec3 start,
	vec3 dir,
	const std::vector<Triangle>& triangles,
	Intersection& closestIntersection
)
{
	float minDistance = std::numeric_limits<float>::max();
	bool intersectionFound = false;

	for (size_t i = 0; i < triangles.size(); ++i)
	{
		const Triangle& triangle = triangles[i];
		vec3 v0 = triangle.v0;
		vec3 v1 = triangle.v1;
		vec3 v2 = triangle.v2;
		vec3 e1 = v1 - v0;
		vec3 e2 = v2 - v0;
		vec3 b = start - v0;
		mat3 A(-dir, e1, e2);
		float determinant = glm::determinant(A);

		// If determinant is close to 0, the ray is parallel to the triangle
		if (std::abs(determinant) > 1e-6)
		{
			mat3 A_inv = glm::inverse(A);
			vec3 x = A_inv * b;
			float t = x.x;
			float u = x.y;
			float v = x.z;

			// Check if the intersection point is within the triangle and the ray's direction
			if (t > 0 && u >= 0 && v >= 0 && u + v <= 1)
			{
				// Calculate the intersection position
				glm::vec3 intersectionPosition = start + t * dir;
				float distance = glm::length(intersectionPosition - start);

				// Check if this intersection is the closest so far
				if (distance < minDistance)
				{
					minDistance = distance;
					closestIntersection.position = intersectionPosition;
					closestIntersection.distance = distance;
					closestIntersection.triangleIndex = static_cast<int>(i);
					intersectionFound = true;
				}
			}
		}
	}

	return intersectionFound;
}

vec3 DirectLight(const Intersection& i)
{
	glm::vec3 lightDir = glm::normalize(lightPos - i.position);
	glm::vec3 normal = triangles[i.triangleIndex].normal;
	float lightDistance = glm::length(lightPos - i.position);
	float r2 = lightDistance * lightDistance;
	float LambertTerm = glm::max(glm::dot(lightDir, normal), 0.0f);
	glm::vec3 directIllumination = (lightColor * LambertTerm) / (4.0f * glm::pi<float>() * r2);

	// Shadow ray
	Intersection shadowIntersection;
	glm::vec3 shadowRayStart = i.position + 0.001f * normal; // Add a small offset to avoid self-intersection
	if (ClosestIntersection(shadowRayStart, lightDir, triangles, shadowIntersection) &&
		shadowIntersection.distance < lightDistance)
	{
		return glm::vec3(0.0f); // In shadow, return black
	}

	return directIllumination;
}

int SDL_main(int argc, char* argv[])
{
	sdlAux = new SDL2Aux(SCREEN_WIDTH, SCREEN_HEIGHT);
	t = SDL_GetTicks();	// Set start value for timer.

	while (!sdlAux->quitEvent())
	{
		Update();
		Draw();
	}
	sdlAux->saveBMP("screenshot.bmp");
	return 0;
}

void Update()
{
	// Compute frame time:
	int t2 = SDL_GetTicks();
	float dt = float(t2-t);
	t = t2;
	cout << "Render time: " << dt << " ms." << endl;

	const Uint8* keystate = SDL_GetKeyboardState(nullptr);
	float lightSpeed = 0.1f;

	if (keystate[SDL_SCANCODE_W])
	{
		lightPos.z += lightSpeed;
	}
	if (keystate[SDL_SCANCODE_S])
	{
		lightPos.z -= lightSpeed;
	}
	if (keystate[SDL_SCANCODE_A])
	{
		lightPos.x -= lightSpeed;
	}
	if (keystate[SDL_SCANCODE_D])
	{
		lightPos.x += lightSpeed;
	}
	if (keystate[SDL_SCANCODE_Q])
	{
		lightPos.y += lightSpeed;
	}
	if (keystate[SDL_SCANCODE_E])
	{
		lightPos.y -= lightSpeed;
	}

	if (keystate[SDL_SCANCODE_UP])
	{
		cameraPos.z += translationSpeed;
	}
	if (keystate[SDL_SCANCODE_DOWN])
	{
		cameraPos.z -= translationSpeed;
	}
	if (keystate[SDL_SCANCODE_LEFT])
	{
		yaw += rotationSpeed;
	}
	if (keystate[SDL_SCANCODE_RIGHT])
	{
		yaw -= rotationSpeed;
	}

	// Create a 4x4 rotation matrix around the y-axis
	glm::mat4 rotationMatrix4 = glm::rotate(glm::mat4(1.0f), yaw, glm::vec3(0.0f, 1.0f, 0.0f));

	// Extract the 3x3 rotation matrix
	R = mat3(rotationMatrix4);
}

void Draw()
{
	sdlAux->clearPixels();
	LoadTestModel(triangles);

	Intersection closestIntersection;

	for (int y = 0; y < SCREEN_HEIGHT; ++y)
	{
		for (int x = 0; x < SCREEN_WIDTH; ++x)
		{
			float ndcX = (2.0f * x) / SCREEN_WIDTH - 1.0f;
			float ndcY = 1.0f - (2.0f * y) / SCREEN_HEIGHT;

			glm::vec3 rayDirectionCamera(ndcX, ndcY, 1.0f);
			glm::vec3 rayDirectionWorld = glm::normalize(R * rayDirectionCamera);
			glm::vec3 rayStart = cameraPos;

			if (ClosestIntersection(rayStart, rayDirectionWorld, triangles, closestIntersection))
			{
				glm::vec3 directIllumination = DirectLight(closestIntersection);
				glm::vec3 totalIllumination = directIllumination + indirectLight;
				glm::vec3 reflectedLight = triangles[closestIntersection.triangleIndex].color * totalIllumination;
				sdlAux->putPixel(x, SCREEN_HEIGHT - 1 - y, reflectedLight);
			}
			else
			{
				sdlAux->putPixel(x, SCREEN_HEIGHT - 1 - y, glm::vec3(0.0f));
			}
		}
	}
	sdlAux->render();
}