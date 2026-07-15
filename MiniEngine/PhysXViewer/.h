#pragma once

#include "VectorMath.h"

#include <cstdint>

struct PrimitiveVertex {
	Math::Vector3 position;
	Math::Vector3 normal;
};

enum class PrimitiveType {
	Cube,
	Cylinder
};

struct RenderItem {
	PrimitiveType type=PrimitiveType::Cube;
	Math::Matrix4 worldMatrix=Math::Matrix4(Math::kIdentity);
	Math::Vector3 color=Math::Vector3(1.0f,1.0f,1.0f);
};