#include "pch.h"
#include "PrimitiveMeshGenerator.h"

#include <DirectXMath.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace
{
    PrimitiveVertex MakeVertex(
        float x,
        float y,
        float z,
        float u,
        float v,
        float normalX,
        float normalY,
        float normalZ,
        float tangentX,
        float tangentY,
        float tangentZ,
        float bitangentX,
        float bitangentY,
        float bitangentZ)
    {
        PrimitiveVertex vertex{};

        vertex.position =
        {
            x,
            y,
            z
        };

        vertex.texcoord =
        {
            u,
            v
        };

        vertex.normal =
        {
            normalX,
            normalY,
            normalZ
        };

        vertex.tangent =
        {
            tangentX,
            tangentY,
            tangentZ
        };

        vertex.bitangent =
        {
            bitangentX,
            bitangentY,
            bitangentZ
        };

        return vertex;
    }

    std::uint16_t ToIndex(std::size_t index)
    {
        return static_cast<std::uint16_t>(index);
    }
}

PrimitiveMeshData PrimitiveMeshGenerator::CreateCylinder(
    std::uint32_t segmentCount)
{
    if (segmentCount < 3)
    {
        throw std::invalid_argument(
            "Cylinder segmentCount must be at least 3.");
    }

    //
    // 현재 uint16_t index를 사용하므로 지나치게 큰 값은 막는다.
    //

    if (segmentCount > 16000)
    {
        throw std::invalid_argument(
            "Cylinder segmentCount is too large.");
    }

    PrimitiveMeshData mesh;

    //
    // 대략적인 크기를 미리 확보해서 vector 재할당을 줄인다.
    //

    mesh.vertices.reserve(
        static_cast<std::size_t>(
            segmentCount * 4 + 6));

    mesh.indices.reserve(
        static_cast<std::size_t>(
            segmentCount * 12));

    constexpr float leftX = -0.5f;
    constexpr float rightX = 0.5f;

    //
    // ---------------------------------------------------------
    // 1. 원기둥 옆면
    // ---------------------------------------------------------
    //
    // 각 segment 위치마다 왼쪽/오른쪽 정점을 하나씩 만든다.
    //
    // i == segmentCount인 정점은
    // i == 0과 같은 위치지만 UV 경계를 위해 중복한다.
    //

    const std::size_t sideVertexStart =
        mesh.vertices.size();

    for (std::uint32_t i = 0;
        i <= segmentCount;
        ++i)
    {
        const float ratio =
            static_cast<float>(i) /
            static_cast<float>(segmentCount);

        const float angle =
            DirectX::XM_2PI * ratio;

        const float y =
            std::cos(angle);

        const float z =
            std::sin(angle);

        //
        // 옆면 normal은 원기둥 중심축에서 바깥쪽을 향한다.
        //

        const float normalY = y;
        const float normalZ = z;

        //
        // 원주 방향 tangent
        //

        const float bitangentY = -z;
        const float bitangentZ = y;

        mesh.vertices.push_back(
            MakeVertex(
                leftX,
                y,
                z,
                0.0f,
                ratio,
                0.0f,
                normalY,
                normalZ,
                1.0f,
                0.0f,
                0.0f,
                0.0f,
                bitangentY,
                bitangentZ));

        mesh.vertices.push_back(
            MakeVertex(
                rightX,
                y,
                z,
                1.0f,
                ratio,
                0.0f,
                normalY,
                normalZ,
                1.0f,
                0.0f,
                0.0f,
                0.0f,
                bitangentY,
                bitangentZ));
    }

    for (std::uint32_t i = 0;
        i < segmentCount;
        ++i)
    {
        const std::uint16_t leftCurrent =
            ToIndex(sideVertexStart + i * 2);

        const std::uint16_t rightCurrent =
            ToIndex(sideVertexStart + i * 2 + 1);

        const std::uint16_t leftNext =
            ToIndex(sideVertexStart + (i + 1) * 2);

        const std::uint16_t rightNext =
            ToIndex(sideVertexStart + (i + 1) * 2 + 1);

        //
        // 첫 번째 삼각형
        //

        mesh.indices.push_back(leftCurrent);
        mesh.indices.push_back(leftNext);
        mesh.indices.push_back(rightCurrent);

        //
        // 두 번째 삼각형
        //

        mesh.indices.push_back(rightCurrent);
        mesh.indices.push_back(leftNext);
        mesh.indices.push_back(rightNext);
    }

    //
    // ---------------------------------------------------------
    // 2. 왼쪽 뚜껑
    // ---------------------------------------------------------
    //
    // 옆면 정점을 공유하지 않는다.
    //
    // 뚜껑 normal은 모두 (-1, 0, 0)이지만
    // 옆면 normal은 원주 방향이기 때문이다.
    //

    const std::uint16_t leftCenterIndex =
        ToIndex(mesh.vertices.size());

    mesh.vertices.push_back(
        MakeVertex(
            leftX,
            0.0f,
            0.0f,
            0.5f,
            0.5f,
            -1.0f,
            0.0f,
            0.0f,
            0.0f,
            1.0f,
            0.0f,
            0.0f,
            0.0f,
            1.0f));

    const std::size_t leftRimStart =
        mesh.vertices.size();

    for (std::uint32_t i = 0;
        i <= segmentCount;
        ++i)
    {
        const float ratio =
            static_cast<float>(i) /
            static_cast<float>(segmentCount);

        const float angle =
            DirectX::XM_2PI * ratio;

        const float y =
            std::cos(angle);

        const float z =
            std::sin(angle);

        mesh.vertices.push_back(
            MakeVertex(
                leftX,
                y,
                z,
                0.5f + y * 0.5f,
                0.5f + z * 0.5f,
                -1.0f,
                0.0f,
                0.0f,
                0.0f,
                1.0f,
                0.0f,
                0.0f,
                0.0f,
                1.0f));
    }

    for (std::uint32_t i = 0;
        i < segmentCount;
        ++i)
    {
        const std::uint16_t current =
            ToIndex(leftRimStart + i);

        const std::uint16_t next =
            ToIndex(leftRimStart + i + 1);

        //
        // 왼쪽 뚜껑의 바깥쪽 normal이 -X가 되도록
        // winding 순서를 반대로 둔다.
        //

        mesh.indices.push_back(leftCenterIndex);
        mesh.indices.push_back(next);
        mesh.indices.push_back(current);
    }

    //
    // ---------------------------------------------------------
    // 3. 오른쪽 뚜껑
    // ---------------------------------------------------------

    const std::uint16_t rightCenterIndex =
        ToIndex(mesh.vertices.size());

    mesh.vertices.push_back(
        MakeVertex(
            rightX,
            0.0f,
            0.0f,
            0.5f,
            0.5f,
            1.0f,
            0.0f,
            0.0f,
            0.0f,
            1.0f,
            0.0f,
            0.0f,
            0.0f,
            1.0f));

    const std::size_t rightRimStart =
        mesh.vertices.size();

    for (std::uint32_t i = 0;
        i <= segmentCount;
        ++i)
    {
        const float ratio =
            static_cast<float>(i) /
            static_cast<float>(segmentCount);

        const float angle =
            DirectX::XM_2PI * ratio;

        const float y =
            std::cos(angle);

        const float z =
            std::sin(angle);

        mesh.vertices.push_back(
            MakeVertex(
                rightX,
                y,
                z,
                0.5f + y * 0.5f,
                0.5f + z * 0.5f,
                1.0f,
                0.0f,
                0.0f,
                0.0f,
                1.0f,
                0.0f,
                0.0f,
                0.0f,
                1.0f));
    }

    for (std::uint32_t i = 0;
        i < segmentCount;
        ++i)
    {
        const std::uint16_t current =
            ToIndex(rightRimStart + i);

        const std::uint16_t next =
            ToIndex(rightRimStart + i + 1);

        mesh.indices.push_back(rightCenterIndex);
        mesh.indices.push_back(current);
        mesh.indices.push_back(next);
    }

    return mesh;
}