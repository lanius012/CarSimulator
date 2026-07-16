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
#include "PhysicsSystem.h"

#include "PhysicsRenderBridge.h"
#include "PrimitiveTypes.h"

#include "VehicleInput.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <stdexcept>
#include <vector>

using namespace GameCore;
using namespace Graphics;
using namespace Math;

namespace
{
    namespace
    {
        VehicleInput ReadVehicleInput()
        {
            VehicleInput input;

            const bool steerLeft =
                GameInput::IsPressed(
                    GameInput::kKey_a);

            const bool steerRight =
                GameInput::IsPressed(
                    GameInput::kKey_d);

            const bool driveForward =
                GameInput::IsPressed(
                    GameInput::kKey_w);

            const bool driveReverse =
                GameInput::IsPressed(
                    GameInput::kKey_s);

            // bool은 정수 변환 시
            // false = 0
            // true  = 1
            //
            // A만 누름:
            // 0 - 1 = -1
            //
            // D만 누름:
            // 1 - 0 = +1
            //
            // 둘 다 누름:
            // 1 - 1 = 0
            input.steer =
                static_cast<int>(steerRight) -
                static_cast<int>(steerLeft);

            // W만 누름:
            // 1 - 0 = +1
            //
            // S만 누름:
            // 0 - 1 = -1
            //
            // 둘 다 누름:
            // 1 - 1 = 0
            input.drive =
                static_cast<int>(driveForward) -
                static_cast<int>(driveReverse);

            // R을 누른 첫 프레임에만 true
            input.reset =
                GameInput::IsFirstPressed(
                    GameInput::kKey_r);

            return input;
        }
    }
}

class PhysXViewer : public GameCore::IGameApp
{
public:
    PhysXViewer()=default;

    virtual void Startup() override;
    virtual void Cleanup() override;
    virtual void Update(float deltaT) override;
    virtual void RenderScene() override;

private:
    Camera m_Camera;

    PrimitiveRenderer m_PrimitiveRenderer;
    PhysicsSystem m_PhysicsSystem;
    PhysicsRenderBridge m_RenderBridge;

    std::vector<RenderItem> m_RenderItems;

    std::size_t m_GroundRenderItemIndex = 0;
    std::size_t m_ChassisRenderItemIndex = 0;

    std::array<
        std::size_t,
        PhysicsSystem::kWheelCount>
        m_WheelRenderItemIndices{};

    float m_PhysicsAccumulator = 0.0f;

    static constexpr float kPhysicsFixedDeltaTime =
        1.0f / 60.0f;
};

CREATE_APPLICATION(PhysXViewer)


void PhysXViewer::Startup()
{
    //
    // 1. 카메라 위치 설정
    //

    m_Camera.SetEyeAtUp(
        Vector3(8.0f, 6.0f, 12.0f),
        Vector3(0.0f, 2.0f, 0.0f),
        Vector3(0.0f, 1.0f, 0.0f));

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

    const bool physicsInitialized =
        m_PhysicsSystem.Initialize();

    if (!physicsInitialized)
    {
        throw std::runtime_error(
            "PhysicsSystem initialization failed.");
    }

    m_PhysicsAccumulator = 0.0f;

    m_RenderItems.clear();
    m_RenderBridge.Clear();

    // -------------------------------------------------------------
    // Ground
    // -------------------------------------------------------------

    m_GroundRenderItemIndex =
        m_RenderItems.size();

    m_RenderItems.emplace_back(
        PrimitiveType::Cube,
        Vector4(
            0.35f,
            0.38f,
            0.42f,
            1.0f));

    if (!m_RenderBridge.RegisterBox(
        m_PhysicsSystem.GetGroundActor(),
        m_PhysicsSystem.GetGroundShape(),
        m_GroundRenderItemIndex))
    {
        throw std::runtime_error(
            "Failed to register ground.");
    }

    // -------------------------------------------------------------
    // Chassis
    // -------------------------------------------------------------

    m_ChassisRenderItemIndex =
        m_RenderItems.size();

    m_RenderItems.emplace_back(
        PrimitiveType::Cube,
        Vector4(
            0.15f,
            0.35f,
            0.90f,
            1.0f));

    if (!m_RenderBridge.RegisterBox(
        m_PhysicsSystem.GetChassisActor(),
        m_PhysicsSystem.GetChassisShape(),
        m_ChassisRenderItemIndex))
    {
        throw std::runtime_error(
            "Failed to register chassis.");
    }

    // -------------------------------------------------------------
    // Wheels
    // -------------------------------------------------------------

    for (std::size_t i = 0;
        i < PhysicsSystem::kWheelCount;
        ++i)
    {
        m_WheelRenderItemIndices[i] =
            m_RenderItems.size();

        m_RenderItems.emplace_back(
            PrimitiveType::Cylinder,
            Vector4(
                0.08f,
                0.08f,
                0.08f,
                1.0f));

        const bool registered =
            m_RenderBridge.RegisterCylinder(
                m_PhysicsSystem.GetWheelActor(i),
                m_PhysicsSystem.GetWheelShape(i),
                m_WheelRenderItemIndices[i],
                m_PhysicsSystem.GetWheelRadius(),
                m_PhysicsSystem.GetWheelHalfWidth());

        if (!registered)
        {
            throw std::runtime_error(
                "Failed to register wheel.");
        }
    }

    // 최초 pose를 RenderItem에 반영한다.
    m_RenderBridge.Sync(
        m_RenderItems);
}

void PhysXViewer::Cleanup()
{
    //
    // Bridge는 PhysX actor/shape 포인터를 빌려 쓰므로
    // PhysX 객체를 해제하기 전에 먼저 연결을 제거한다.
    //

    m_RenderBridge.Clear();
    m_RenderItems.clear();

    m_PhysicsSystem.Shutdown();
    m_PrimitiveRenderer.Shutdown();
}
void PhysXViewer::Update(float deltaT)
{
    ScopedTimer timer(L"Update State");

    //
    // 1. 렌더 프레임에서 키 입력을 한 번 읽는다.
    //

    const VehicleInput vehicleInput =
        ReadVehicleInput();

    m_PhysicsSystem.SetVehicleInput(
        vehicleInput);

    //
    // 2. 렌더 deltaT 제한 및 누적
    //

    const float clampedDeltaTime =
        (std::min)(deltaT, 0.1f);

    m_PhysicsAccumulator +=
        clampedDeltaTime;

    //
    // 3. 고정 물리 스텝
    //

    while (m_PhysicsAccumulator >=
        kPhysicsFixedDeltaTime)
    {
        m_PhysicsSystem.Step(
            kPhysicsFixedDeltaTime);

        m_PhysicsAccumulator -=
            kPhysicsFixedDeltaTime;
    }

    //
    // 4. fetchResults 이후 pose 동기화
    //

    m_RenderBridge.Sync(
        m_RenderItems);

    //
    // 5. 카메라 갱신
    //

    const float width =
        static_cast<float>(
            g_SceneColorBuffer.GetWidth());

    const float height =
        static_cast<float>(
            g_SceneColorBuffer.GetHeight());

    if (width > 0.0f &&
        height > 0.0f)
    {
        m_Camera.SetAspectRatio(
            height / width);
    }

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

    //
    for (const RenderItem& renderItem :
        m_RenderItems)
    {
        m_PrimitiveRenderer.Render(
            context,
            m_Camera,
            renderItem);
    }

    context.Finish();
}