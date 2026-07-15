#pragma once

#include "Camera.h"
#include "CommandContext.h"
#include "PipelineState.h"
#include "RootSignature.h"
#include "VectorMath.h"

class PrimitiveRenderer {
public:
	PrimitiveRenderer();

	void Initialize();
	void Shutdown();

	void RenderCube(
		GraphicsContext& context,
		const Math::Camera& camera,
		const Math::Matrix4& worldMatrix,
		const Math::Vector4& color);

private:
	RootSignature m_RootSignature;
	GraphicsPSO m_PipelineState;
};