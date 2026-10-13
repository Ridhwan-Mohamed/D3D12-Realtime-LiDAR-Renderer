#pragma once

#include "GpuPoint.h"
#include "ClodPoint.h"

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

    // Existing discrete-LOD representation.
    //
    // The first sourcePointCount entries contain every
    // original point exactly once in leaf order.
    // Representative samples for internal nodes are
    // appended after them.
    std::vector<GpuPoint> points;

    // New continuous-LOD representation.
    //
    // Every original source point appears exactly once.
    // Points will be ordered from coarse levels to fine
    // levels when M11.1B populates this array.
    std::vector<ClodPoint> clodPoints;

    // One contiguous range for each CLOD level.
    //
    // clodLevelRanges[0] describes all level-zero points,
    // clodLevelRanges[1] describes level one, and so on.
    std::vector<PointRange> clodLevelRanges;

    // World-space spacing represented by level zero.
    //
    // A later level's approximate spacing is:
    //
    // clodRootSpacing / 2^level
    float clodRootSpacing = 0.0f;

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