#include "pch.h"
#include "PrimitiveRenderer.h"

#include "BufferManager.h"
#include "Cube.h"
#include "GraphicsCommon.h"

#include "CompiledShaders/PrimitiveVS.h"
#include "CompiledShaders/PrimitivePS.h"

#include <cstddef>

using namespace Math;
using namespace Graphics;

namespace
{
    enum RootParameterIndex
    {
        kVertexConstants = 0,
        kPixelConstants,
        kRootParameterCount
    };

    // SetDynamicConstantBufferView()에 전달되는 주소는
    // 16바이트 정렬이어야 한다.
    struct alignas(16) VertexConstants
    {
        Matrix4 World;
        Matrix4 ViewProjection;
    };

    struct alignas(16) PixelConstants
    {
        Vector4 BaseColor;
    };
}

PrimitiveRenderer::PrimitiveRenderer()
    : m_PipelineState(L"Primitive Renderer PSO")
{
}

void PrimitiveRenderer::Initialize()
{
    //
    // 1. Root Signature
    //

    m_RootSignature.Reset(kRootParameterCount);

    // Vertex Shader의 b0
    m_RootSignature[kVertexConstants]
        .InitAsConstantBuffer(
            0,
            D3D12_SHADER_VISIBILITY_VERTEX);

    // Pixel Shader의 b0
    //
    // 같은 b0이지만 shader visibility가 서로 다르므로
    // 별개의 root parameter로 둘 수 있다.
    m_RootSignature[kPixelConstants]
        .InitAsConstantBuffer(
            0,
            D3D12_SHADER_VISIBILITY_PIXEL);

    m_RootSignature.Finalize(
        L"Primitive Renderer Root Signature",
        D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

    //
    // 2. Vertex Input Layout
    //

    using CubeVertex =
        Graphics::Shapes::Cube::CubeVertex;

    D3D12_INPUT_ELEMENT_DESC inputLayout[] =
    {
        {
            "POSITION",
            0,
            DXGI_FORMAT_R32G32B32_FLOAT,
            0,
            static_cast<UINT>(
                offsetof(CubeVertex, position)),
            D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA,
            0
        },
        {
            "NORMAL",
            0,
            DXGI_FORMAT_R32G32B32_FLOAT,
            0,
            static_cast<UINT>(
                offsetof(CubeVertex, normal)),
            D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA,
            0
        }
    };

    //
    // 3. Graphics Pipeline State Object
    //

    const DXGI_FORMAT colorFormat =
        g_SceneColorBuffer.GetFormat();

    const DXGI_FORMAT depthFormat =
        g_SceneDepthBuffer.GetFormat();

    m_PipelineState.SetRootSignature(
        m_RootSignature);

    m_PipelineState.SetRasterizerState(
        RasterizerDefault);

    m_PipelineState.SetBlendState(
        BlendDisable);

    m_PipelineState.SetDepthStencilState(
        DepthStateReadWrite);

    m_PipelineState.SetInputLayout(
        _countof(inputLayout),
        inputLayout);

    m_PipelineState.SetPrimitiveTopologyType(
        D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE);

    m_PipelineState.SetRenderTargetFormats(
        1,
        &colorFormat,
        depthFormat);

    m_PipelineState.SetVertexShader(
        g_pPrimitiveVS,
        sizeof(g_pPrimitiveVS));

    m_PipelineState.SetPixelShader(
        g_pPrimitivePS,
        sizeof(g_pPrimitivePS));

    m_PipelineState.Finalize();

    //
    // 4. MiniEngine Core에 있는 단위 Cube 생성
    //

    Graphics::Shapes::Cube::InitializeCubeBuffers();
}

void PrimitiveRenderer::Shutdown()
{
    Graphics::Shapes::Cube::DestroyCubeBuffers();
}

void PrimitiveRenderer::RenderCube(
    GraphicsContext& context,
    const Camera& camera,
    const Matrix4& worldMatrix,
    const Vector4& color)
{
    alignas(16) VertexConstants vertexConstants;
    vertexConstants.World = worldMatrix;
    vertexConstants.ViewProjection =
        camera.GetViewProjMatrix();

    alignas(16) PixelConstants pixelConstants;
    pixelConstants.BaseColor = color;

    context.SetRootSignature(
        m_RootSignature);

    context.SetPipelineState(
        m_PipelineState);

    context.SetPrimitiveTopology(
        D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    context.SetDynamicConstantBufferView(
        kVertexConstants,
        sizeof(vertexConstants),
        &vertexConstants);

    context.SetDynamicConstantBufferView(
        kPixelConstants,
        sizeof(pixelConstants),
        &pixelConstants);

    context.SetVertexBuffer(
        0,
        Graphics::Shapes::Cube::g_CubeVerts
        .VertexBufferView());

    context.Draw(
        Graphics::Shapes::Cube::g_NumVerts);
}