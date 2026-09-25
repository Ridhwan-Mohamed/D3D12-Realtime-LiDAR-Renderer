#pragma once

#include "GpuPoint.h"

#include <DirectXMath.h>

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

using PointHierarchyNodeIndex =
std::uint32_t;

inline constexpr PointHierarchyNodeIndex
InvalidPointHierarchyNode =
std::numeric_limits<
    PointHierarchyNodeIndex>::max();

struct PointRange
{
    std::uint32_t firstPoint = 0;
    std::uint32_t pointCount = 0;
};

struct PointHierarchyNode
{
    DirectX::XMFLOAT3 boundsMinimum{};
    DirectX::XMFLOAT3 boundsMaximum{};

    std::array<
        PointHierarchyNodeIndex,
        8>
        children{
            InvalidPointHierarchyNode,
            InvalidPointHierarchyNode,
            InvalidPointHierarchyNode,
            InvalidPointHierarchyNode,
            InvalidPointHierarchyNode,
            InvalidPointHierarchyNode,
            InvalidPointHierarchyNode,
            InvalidPointHierarchyNode
    };

    PointRange sourcePoints{};

    PointRange representativePoints{};

    float spacing = 0.0f;

    std::uint32_t depth = 0;

    [[nodiscard]]
    bool IsLeaf() const noexcept
    {
        for (const PointHierarchyNodeIndex child :
        children)
        {
            if (child
                != InvalidPointHierarchyNode)
            {
                return false;
            }
        }

        return true;
    }
};

struct PointCloudHierarchy
{
    std::vector<PointHierarchyNode> nodes;

    // Representative points from every hierarchy node
    // packed into one contiguous array.
    std::vector<GpuPoint> points;

    PointHierarchyNodeIndex rootIndex =
        InvalidPointHierarchyNode;

    std::uint64_t sourcePointCount = 0;

    std::uint32_t maximumDepth = 0;

    [[nodiscard]]
    bool Empty() const noexcept
    {
        return
            rootIndex == InvalidPointHierarchyNode;
    }
};