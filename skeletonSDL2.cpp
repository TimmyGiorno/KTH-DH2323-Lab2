//DH2323 skeleton code, Lab2 (SDL2 version)
#include <iostream>
#include <glm/glm.hpp>
#include "SDL2auxiliary.h"
#include "TestModel.h"

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
const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
SDL2Aux *sdlAux;
int t;
std::vector<Triangle> triangles;
// ----------------------------------------------------------------------------
// FUNCTIONS
void Update();
void Draw();
bool ClosestIntersection();

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

void Update(void)
{
	// Compute frame time:
	int t2 = SDL_GetTicks();
	float dt = float(t2-t);
	t = t2;
	cout << "Render time: " << dt << " ms." << endl;
}

void Draw()
{
	sdlAux->clearPixels();

	LoadTestModel(triangles);

	for( int y=0; y<SCREEN_HEIGHT; ++y )
	{
		for( int x=0; x<SCREEN_WIDTH; ++x )
		{
			vec3 color( 1, 0.5, 0.5 );
			sdlAux->putPixel(x, y, color);
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