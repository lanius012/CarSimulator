#include "pch.h"

#include "GameCore.h"
#include "GraphicsCore.h"
#include "SystemTime.h"
#include "TextRenderer.h"
#include "GameInput.h"
#include "CommandContext.h"
#include "RootSignature.h"
#include "PipelineState.h"
#include "BufferManager.h"

#include "Camera.h"
#include "PrimitiveRenderer.h"

using namespace GameCore;
using namespace Graphics;
using namespace Math;

class PhysXViewer : public GameCore::IGameApp
{
public:
    PhysXViewer();

    virtual void Startup() override;
    virtual void Cleanup() override;
    virtual void Update(float deltaT) override;
    virtual void RenderScene() override;

private:
    Camera m_Camera;

    PrimitiveRenderer m_PrimitiveRenderer;

    Matrix4 m_CubeWorld;
};

CREATE_APPLICATION(PhysXViewer)

PhysXViewer::PhysXViewer()
    : m_CubeWorld(kIdentity)
{
}

void PhysXViewer::Startup()
{
    //
    // 1. 카메라 위치 설정
    //

    m_Camera.SetEyeAtUp(
        Vector3(4.0f, 3.0f, 6.0f),  // 카메라 위치
        Vector3(0.0f, 0.0f, 0.0f),  // 바라보는 위치
        Vector3(0.0f, 1.0f, 0.0f)); // Up 방향

    const float width =
        static_cast<float>(
            g_SceneColorBuffer.GetWidth());

    const float height =
        static_cast<float>(
            g_SceneColorBuffer.GetHeight());

    // MiniEngine Camera는 width / height가 아니라
    // height / width를 받는다.
    const float aspectHeightOverWidth =
        height / width;

    m_Camera.SetPerspectiveMatrix(
        XM_PIDIV4,             // 수직 FOV 45도
        aspectHeightOverWidth,
        0.1f,                  // near plane
        1000.0f);              // far plane

    m_Camera.Update();

    //
    // 2. Primitive Renderer 초기화
    //

    m_PrimitiveRenderer.Initialize();

    //
    // 3. Cube 월드 행렬
    //
    // 단위 행렬이므로:
    // 위치 = 원점
    // 회전 = 없음
    // 크기 = 1 x 1 x 1
    //

    m_CubeWorld = Matrix4(kIdentity);
}

void PhysXViewer::Cleanup()
{
    m_PrimitiveRenderer.Shutdown();
}

void PhysXViewer::Update(float /*deltaT*/)
{
    ScopedTimer timer(L"Update State");

    const float width =
        static_cast<float>(
            g_SceneColorBuffer.GetWidth());

    const float height =
        static_cast<float>(
            g_SceneColorBuffer.GetHeight());

    if (width > 0.0f && height > 0.0f)
    {
        // 창 크기가 바뀌어도 종횡비를 유지한다.
        m_Camera.SetAspectRatio(
            height / width);
    }

    // MiniEngine Camera는 상태를 변경한 뒤
    // 프레임마다 Update()해야 한다.
    m_Camera.Update();
}

void PhysXViewer::RenderScene()
{
    GraphicsContext& context =
        GraphicsContext::Begin(L"Scene Render");

    //
    // 1. Color/Depth 버퍼 상태 전환
    //

    context.TransitionResource(
        g_SceneColorBuffer,
        D3D12_RESOURCE_STATE_RENDER_TARGET);

    context.TransitionResource(
        g_SceneDepthBuffer,
        D3D12_RESOURCE_STATE_DEPTH_WRITE,
        true);

    //
    // 2. 이전 프레임 제거
    //

    context.ClearColor(
        g_SceneColorBuffer);

    context.ClearDepth(
        g_SceneDepthBuffer);

    //
    // 3. 렌더 타깃 설정
    //

    context.SetRenderTarget(
        g_SceneColorBuffer.GetRTV(),
        g_SceneDepthBuffer.GetDSV());

    context.SetViewportAndScissor(
        0,
        0,
        g_SceneColorBuffer.GetWidth(),
        g_SceneColorBuffer.GetHeight());

    //
    // 4. 단위 Cube 렌더링
    //

    m_PrimitiveRenderer.RenderCube(
        context,
        m_Camera,
        m_CubeWorld,
        Vector4(
            0.15f,
            0.60f,
            0.95f,
            1.0f));

    context.Finish();
}