//DH2323 skeleton code, Lab2 (SDL2 version)
#include <iostream>
#include <glm/glm.hpp>
#include "SDL2auxiliary.h"
#include "TestModel.h"
#include <glm/gtc/matrix_transform.hpp>

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
// ----------------------------------------------------------------------------
// FUNCTIONS
void Update();
void Draw();
bool ClosestIntersection(
	vec3 start,
	vec3 dir,
	const std::vector<Triangle>& triangles,
	Intersection& closestIntersection
);

int SDL_main( int argc, char* argv[] )
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

	if (keystate[SDL_SCANCODE_UP])
	{
		cameraPos.z += translationSpeed;
	}
	if (keystate[SDL_SCANCODE_DOWN])
	{
		cameraPos.z -= translationSpeed; // Move camera backward
	}
	if (keystate[SDL_SCANCODE_LEFT])
	{
		yaw += rotationSpeed; // Increase yaw angle
	}
	if (keystate[SDL_SCANCODE_RIGHT])
	{
		yaw -= rotationSpeed; // Decrease yaw angle
	}

	// Create a 4x4 rotation matrix around the y-axis
	glm::mat4 rotationMatrix4 = glm::rotate(glm::mat4(1.0f), yaw, glm::vec3(0.0f, 1.0f, 0.0f));

	// Extract the 3x3 rotation matrix
	R = glm::mat3(rotationMatrix4);
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

			// Initial ray direction in camera space (forward)
			glm::vec3 rayDirectionCamera(ndcX, ndcY, 1.0f);

			// Rotate the ray direction from camera space to world space
			glm::vec3 rayDirectionWorld = glm::normalize(R * rayDirectionCamera);

			glm::vec3 rayStart = cameraPos;

			if (ClosestIntersection(rayStart, rayDirectionWorld, triangles, closestIntersection))
			{
				const Triangle& hitTriangle = triangles[closestIntersection.triangleIndex];
				sdlAux->putPixel(x, SCREEN_HEIGHT - 1 - y, hitTriangle.color);
			}
			else
			{
				sdlAux->putPixel(x, SCREEN_HEIGHT - 1 - y, glm::vec3(0.0f));
			}
		}
	}
	sdlAux->render();
}


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