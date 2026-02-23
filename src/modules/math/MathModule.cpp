/**
 * Copyright (c) 2006-2025 LOVE Development Team
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 **/

// LOVE
#include "MathModule.h"
#include "common/Vector.h"
#include "common/int.h"
#include "common/StringMap.h"
#include "BezierCurve.h"
#include "Transform.h"

// STL
#include <cmath>
#include <list>
#include <iostream>
#include <string_view>
#include <string>

// C
#include <time.h>

#define PI (3.1415926536f)

using std::list;
using love::Vector2;

namespace
{

// check if an angle is oriented counter clockwise
inline bool is_oriented_ccw(const Vector2 &a, const Vector2 &b, const Vector2 &c)
{
	// return det(b-a, c-a) >= 0
	return ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) >= 0;
}

// check if a and b are on the same side of the line c->d
bool on_same_side(const Vector2 &a, const Vector2 &b, const Vector2 &c, const Vector2 &d)
{
	float px = d.x - c.x, py = d.y - c.y;
	// return det(p, a-c) * det(p, b-c) >= 0
	float l = px * (a.y - c.y) - py * (a.x - c.x);
	float m = px * (b.y - c.y) - py * (b.x - c.x);
	return l * m >= 0;
}

// checks is p is contained in the triangle abc
inline bool point_in_triangle(const Vector2 &p, const Vector2 &a, const Vector2 &b, const Vector2 &c)
{
	return on_same_side(p,a, b,c) && on_same_side(p,b, a,c) && on_same_side(p,c, a,b);
}

// checks if any vertex in `vertices' is in the triangle abc.
bool any_point_in_triangle(const std::list<const Vector2 *> &vertices, const Vector2 &a, const Vector2 &b, const Vector2 &c)
{
	for (const Vector2 *p : vertices)
	{
		if ((p != &a) && (p != &b) && (p != &c) && point_in_triangle(*p, a,b,c)) // oh god...
			return true;
	}

	return false;
}

inline bool is_ear(const Vector2 &a, const Vector2 &b, const Vector2 &c, const std::list<const Vector2 *> &vertices)
{
	return is_oriented_ccw(a,b,c) && !any_point_in_triangle(vertices, a,b,c);
}

} // anonymous namespace

namespace love
{
namespace math
{

std::vector<Triangle> triangulate(const std::vector<love::Vector2> &polygon)
{
	if (polygon.size() < 3)
		throw love::Exception("Not a polygon");
	else if (polygon.size() == 3)
		return std::vector<Triangle>(1, Triangle(polygon[0], polygon[1], polygon[2]));

	// collect list of connections and record leftmost item to check if the polygon
	// has the expected winding
	std::vector<size_t> next_idx(polygon.size()), prev_idx(polygon.size());
	size_t idx_lm = 0;
	for (size_t i = 0; i < polygon.size(); ++i)
	{
		const love::Vector2 &lm = polygon[idx_lm], &p = polygon[i];
		if (p.x < lm.x || (p.x == lm.x && p.y < lm.y))
			idx_lm = i;
		next_idx[i] = i+1;
		prev_idx[i] = i-1;
	}
	next_idx[next_idx.size()-1] = 0;
	prev_idx[0] = prev_idx.size()-1;

	// check if the polygon has the expected winding and reverse polygon if needed
	if (!is_oriented_ccw(polygon[prev_idx[idx_lm]], polygon[idx_lm], polygon[next_idx[idx_lm]]))
		next_idx.swap(prev_idx);

	// collect list of concave polygons
	std::list<const love::Vector2 *> concave_vertices;
	for (size_t i = 0; i < polygon.size(); ++i)
	{
		if (!is_oriented_ccw(polygon[prev_idx[i]], polygon[i], polygon[next_idx[i]]))
			concave_vertices.push_back(&polygon[i]);
	}

	// triangulation according to kong
	std::vector<Triangle> triangles;
	size_t n_vertices = polygon.size();
	size_t current = 1, skipped = 0, next, prev;
	while (n_vertices > 3)
	{
		next = next_idx[current];
		prev = prev_idx[current];
		const Vector2 &a = polygon[prev], &b = polygon[current], &c = polygon[next];
		if (is_ear(a,b,c, concave_vertices))
		{
			triangles.push_back(Triangle(a,b,c));
			next_idx[prev] = next;
			prev_idx[next] = prev;
			concave_vertices.remove(&b);
			--n_vertices;
			skipped = 0;
		}
		else if (++skipped > n_vertices)
		{
			throw love::Exception("Cannot triangulate polygon.");
		}
		current = next;
	}
	next = next_idx[current];
	prev = prev_idx[current];
	triangles.push_back(Triangle(polygon[prev], polygon[current], polygon[next]));

	return triangles;
}

bool isConvex(const std::vector<love::Vector2> &polygon)
{
	if (polygon.size() < 3)
		return false;

	// a polygon is convex if all corners turn in the same direction
	// turning direction can be determined using the cross-product of
	// the forward difference vectors
	size_t i = polygon.size() - 2, j = polygon.size() - 1, k = 0;
	Vector2 p(polygon[j] - polygon[i]);
	Vector2 q(polygon[k] - polygon[j]);
	float winding = Vector2::cross(p, q);

	while (k+1 < polygon.size())
	{
		i = j; j = k; k++;
		p = polygon[j] - polygon[i];
		q = polygon[k] - polygon[j];

		if (Vector2::cross(p, q) * winding < 0)
			return false;
	}
	return true;
}

/**
 * http://en.wikipedia.org/wiki/SRGB#The_reverse_transformation
 **/
float gammaToLinear(float c)
{
	if (c <= 0.04045f)
		return c / 12.92f;
	else
		return powf((c + 0.055f) / 1.055f, 2.4f);
}

/**
 * http://en.wikipedia.org/wiki/SRGB#The_forward_transformation_.28CIE_xyY_or_CIE_XYZ_to_sRGB.29
 **/
float linearToGamma(float c)
{
	if (c <= 0.0031308f)
		return c * 12.92f;
	else
		return 1.055f * powf(c, 1.0f / 2.4f) - 0.055f;
}

float dot(float ax, float ay, float bx, float by)
{
	return ax * bx + ay * by;
}

float dot(float amag, float bmag, float angle)
{
	return amag * bmag * cos(angle);
}

bool aabb(float ax, float ay, float aw, float ah, float bx, float by, float bw, float bh)
{
	return ax < bx + bw &&
		ax + aw > bx &&
		ay < by + bh &&
		ay + ah > by;
}

int modn(int x, int n, int offset)
{
	return ((x - offset) % n + n) % n + offset;
}

float fmodn(float x, float n, float offset)
{
	return std::fmod(std::fmod(x - offset, n) + n, n) + offset;
}

float lerp(float a, float b, float t)
{
	return a + t * (b-a);
}

inline constexpr auto hash_djb2a(const std::string_view stringView)
{
	unsigned long hash{5381};
	for (unsigned char c : stringView)
	{
		hash = ((hash << 5) + hash) ^ c;
	}
	return hash;
}
inline constexpr auto operator""_sh(const char *str, size_t len)
{
	return hash_djb2a(std::string_view{str, len});
}

float ease(float x, std::string style)
{
	x = clamp(x, 0.0, 1.0);

	auto easeOutBounce = [](float x)
	{
		const float n1 = 7.5625;
		const float d1 = 2.75;

		if (x < 1 / d1)
		{
			return  n1 * x * x;
		}
		else if (x < 2 / d1)
		{
			return n1 * (x -= 1.5 / d1) * x + 0.75f;
		}
		else if (x < 2.5 / d1)
		{
			return n1 * (x -= 2.25 / d1) * x + 0.9375f;
		}
		else
		{
			return n1 * (x -= 2.625 / d1) * x + 0.984375f;
		}
	};

	switch (hash_djb2a(style))
	{
		case "easeInSine"_sh:
			return 1 - cos((x * PI) / 2);
		case "easeOutSine"_sh:
			return sin((x * PI) / 2);
		case "easeInOutSine"_sh:
			return -(cos(PI * x) - 1) / 2;
		case "easeInQuad"_sh:
			return x * x;
		case "easeOutQuad"_sh:
			return 1 - (1 - x) * (1 - x);
		case "easeInOutQuad"_sh:
			return x < 0.5 ? (2 * x * x) : (1 - pow(-2 * x + 2, 2) / 2);
		case "easeInCubic"_sh:
			return x * x * x;
		case "easeOutCubic"_sh:
			return 1 - pow(1 - x, 3);
		case "easeInOutCubic"_sh:
			return x < 0.5 ? (4 * x * x * x * x) : (1 - pow(-2 * x + 2, 3) / 2);
		case "easeInQuart"_sh:
			return x * x * x * x;
		case "easeOutQuart"_sh:
			return 1 - pow(1 - x, 4);
		case "easeInOutQuart"_sh:
			return x < 0.5 ? (8 * x * x * x * x) : (1 - pow(-2 * x + 2, 4) / 2);
		case "easeInQuint"_sh:
			return x * x * x * x * x;
		case "easeOutQuint"_sh:
			return 1 - pow(1 - x, 5);
		case "easeInOutQuint"_sh:
			return x < 0.5 ? (16 * x * x * x * x * x) : (1 - pow(-2 * x + 2, 5) / 2);
		case "easeInExpo"_sh:
			return x == 0 ? 0 : pow(2, 10 * x - 10);
		case "easeOutExpo"_sh:
			return x == 1 ? 1 : 1 - pow(2, -10 * x);
		case "easeInOutExpo"_sh:
			return x == 0 ? 
			0 : x == 1 ? 
			1 : x < 0.5 ? 
			(pow(2, 20 * x - 10) / 2) : (2 - pow(2, -20 * x + 10) / 2);
		case "easeInCirc"_sh:
			return 1 - sqrt(1 - pow(x, 2));
		case "easeOutCirc"_sh:
			return sqrt(1 - pow(x - 1, 2));
		case "easeInOutCirc"_sh:
			return x < 0.5
			? (1 - sqrt(1 - pow(2 * x, 2)) / 2)
			: ((sqrt(1 - pow(-2 * x + 2, 2))) / 2);
		case "easeInBack"_sh:
		{
			const float c1 = 1.70158;
			const float c3 = c1 + 1;
			return c3 * x * x * x - c1 * x * x;
		}
		case "easeOutBack"_sh:
		{
			const float c1 = 1.70158;
			const float c3 = c1 + 1;
			return 1 + c3 * pow(x - 1, 3) + c1 * pow(x - 1, 2);
		}
		case "easeInOutBack"_sh:
		{
			const float c1 = 1.70158;
			const float c2 = c1 * 1.525;
			return x < 0.5
			? (pow(2 * x, 2) * ((c2 + 1) * 2 * x - c2)) / 2
			: (pow(2 * x - 2, 2) * ((c2 + 1) * (x * 2 - 2) + c2) + 2) / 2;
		}
		case "easeInElastic"_sh:
		{
			const float c4 = (2 * PI) / 3;
			return x == 0
			? 0 : 
			x == 1
			? 1 :
			-pow(2, 10 * x - 10) * sin((x * 10 - 10.75) * c4);
		}
		case "easeOutElastic"_sh:
		{
			const float c4 = (2 * PI) / 3;

			return x == 0
			? 0
			: x == 1
			? 1
			: pow(2, -10 * x) * sin((x * 10 - 0.75) * c4) + 1;
		}
		case "easeInOutElastic"_sh:
		{
			const float c5 = (2 * PI) / 4.5;
			return x == 0
			? 0
			: x == 1
			? 1
			: x < 0.5
			? -(pow(2, 20 * x - 10) * sin((20 * x - 11.125) * c5)) / 2
			: (pow(2, -20 * x + 10) * sin((20 * x - 11.125) * c5)) / 2 + 1;
		}
		case "easeInBounce"_sh:
		{
			return 1 - easeOutBounce(1 - x);
		};
		case "easeOutBounce"_sh:
		{
			return easeOutBounce(x);
		}
		case "easeInOutBounce"_sh:
		{
			return x < 0.5
			? (1 - easeOutBounce(1 - 2 * x)) / 2
			: (1 + easeOutBounce(2 * x - 1)) / 2;
		}
		default:
		return x;
	}
}

float clamp(float v, float lo, float hi)
{
	return (v < lo) ? lo : (v > hi) ? hi : v;
}

float fract(float x)
{
	return x - std::floor(x);
}

float smoothstep(float e0, float e1, float x)
{
	float t = clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
	return t * t * (3.0f - 2.0f * t);
}

float hash1(float n)
{
	return fract(std::sin(n) * 43758.5453123f);
}

love::Vector2 hash2(float n)
{
	return Vector2(
		fract(std::sin(n * 7.0f) * 43758.5453f),
		fract(std::sin(n * 13.0f) * 43758.5453f)
	);
}

float temperature(float uvx, float uvy, float t)
{
	constexpr float TEMP_COOL_TOP = 0.25f;
	constexpr float TEMP_HEAT_BOTTOM = 0.85f;
	constexpr float TEMP_WOBBLE = 0.12f;

	float temp = lerp(TEMP_COOL_TOP, TEMP_HEAT_BOTTOM, uvy);
	temp += TEMP_WOBBLE * (
		0.5f * std::sin(t * 0.15f + uvx * 2.3f + uvy * 3.1f) +
		0.5f * std::sin(t * 0.12f + uvx * 4.1f - uvy * 1.7f)
		);
	return clamp(temp, 0.0f, 1.0f);
}

float temperatureBlur(float uvx, float uvy, float t, int resx, int resy)
{
	constexpr float TEMP_RADIUS_FILTER_PX = 2.0f;
	float hX = TEMP_RADIUS_FILTER_PX / float(resx);
	float hY = TEMP_RADIUS_FILTER_PX / float(resy);

	float w1 = 1.0f, w2 = 0.5f, w3 = 0.25f;
	float W = w1 + 4.0f * w2 + 4.0f * w3;

	auto T = [&](float x, float y) {
		return temperature(fract(x), fract(y), t);
		};

	float tc = T(uvx, uvy);

	float ta = T(uvx + hX, uvy) + T(uvx - hX, uvy) +
		T(uvx, uvy + hY) + T(uvx, uvy - hY);

	float td = T(uvx + hX, uvy + hY) + T(uvx - hX, uvy - hY) +
		T(uvx - hX, uvy + hY) + T(uvx + hX, uvy - hY);

	return (w1 * tc + w2 * ta + w3 * td) / W;
}

void convection(float x, float y, float t, float &outDx, float &outDy)
{
	constexpr float FLOW_STRENGTH = 0.18f;
	constexpr float FLOW_SWIRL = 0.35f;
	constexpr float TAU = 6.28318530718f;

	float dx = x - 0.5f;
	float dy = y - 0.5f;

	// sqrt removed: use r^2 thresholds
	float r2 = dx * dx + dy * dy;
	float up = smoothstep(0.65f * 0.65f, 0.0f, r2);

	float phaseA = TAU * (x + y) + t * 0.2f;
	float phaseB = TAU * (x * 0.7f - y * 0.4f) + t * 0.1f;

	float ang = FLOW_SWIRL * (0.6f * std::sin(phaseA) + 0.4f * std::sin(phaseB));

	// mat2(1,-ang, ang,1) * vec2(0,-up) * FLOW_STRENGTH
	float vy = -up;
	outDx = (-ang * vy) * FLOW_STRENGTH;
	outDy = (vy)*FLOW_STRENGTH;
}

void blobCenter(int id, float t, float &outX, float &outY)
{
	float f = float(id);

	love::Vector2 base = hash2(f);
	float baseX = base.x;
	float baseY = base.y;

	float a = lerp(0.02f, 0.05f, hash1(f + 3.0f));
	float b = lerp(0.02f, 0.05f, hash1(f + 2.0f));

	float px = baseX + std::sin(t * 0.6f + f) * a;
	float py = baseY + (-t) * b;

	float cx, cy;
	convection(px + 0.001f * f, py + 0.001f * f, t, cx, cy);
	px += cx;
	py += cy;

	py += baseY;

	outX = fract(px);
	outY = fract(py);
}

Math::Math()
	: Module(M_MATH, "love.math")
{
	RandomGenerator::Seed seed;
	seed.b64 = (uint64) time(nullptr);

	rng.set(new RandomGenerator(), Acquire::NORETAIN);
	rng->setSeed(seed);
}

Math::~Math()
{
}

RandomGenerator *Math::newRandomGenerator()
{
	return new RandomGenerator();
}

BezierCurve *Math::newBezierCurve(const std::vector<Vector2> &points)
{
	return new BezierCurve(points);
}

Transform *Math::newTransform()
{
	return new Transform();
}

Transform *Math::newTransform(float x, float y, float a, float sx, float sy, float ox, float oy, float kx, float ky)
{
	return new Transform(x, y, a, sx, sy, ox, oy, kx, ky);
}

} // math
} // love
