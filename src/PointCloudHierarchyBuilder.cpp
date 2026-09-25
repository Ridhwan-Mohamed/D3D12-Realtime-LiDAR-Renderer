#include "PointCloudHierarchyBuilder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{
    struct BuildContext
    {
        std::span<const GpuPoint> sourcePoints;

        PointHierarchyBuildSettings settings;

        PointCloudHierarchy hierarchy;
    };

    bool BoundsAreValid(
        const DirectX::XMFLOAT3& minimum,
        const DirectX::XMFLOAT3& maximum)
    {
        return
            std::isfinite(minimum.x)
            && std::isfinite(minimum.y)
            && std::isfinite(minimum.z)
            && std::isfinite(maximum.x)
            && std::isfinite(maximum.y)
            && std::isfinite(maximum.z)
            && minimum.x <= maximum.x
            && minimum.y <= maximum.y
            && minimum.z <= maximum.z;
    }

    DirectX::XMFLOAT3 CalculateCenter(
        const DirectX::XMFLOAT3& minimum,
        const DirectX::XMFLOAT3& maximum)
    {
        return {
            minimum.x
                + (maximum.x - minimum.x) * 0.5f,

            minimum.y
                + (maximum.y - minimum.y) * 0.5f,

            minimum.z
                + (maximum.z - minimum.z) * 0.5f
        };
    }

    float CalculateMaximumExtent(
        const DirectX::XMFLOAT3& minimum,
        const DirectX::XMFLOAT3& maximum)
    {
        return std::max({
            maximum.x - minimum.x,
            maximum.y - minimum.y,
            maximum.z - minimum.z
            });
    }

    std::uint32_t FindOctant(
        const DirectX::XMFLOAT3& position,
        const DirectX::XMFLOAT3& center)
    {
        std::uint32_t octant = 0;

        if (position.x >= center.x)
        {
            octant |= 1;
        }

        if (position.y >= center.y)
        {
            octant |= 2;
        }

        if (position.z >= center.z)
        {
            octant |= 4;
        }

        return octant;
    }

    void CalculateChildBounds(
        const DirectX::XMFLOAT3& parentMinimum,
        const DirectX::XMFLOAT3& parentMaximum,
        const DirectX::XMFLOAT3& center,
        std::uint32_t octant,
        DirectX::XMFLOAT3& childMinimum,
        DirectX::XMFLOAT3& childMaximum)
    {
        childMinimum = parentMinimum;
        childMaximum = parentMaximum;

        if ((octant & 1) != 0)
        {
            childMinimum.x = center.x;
        }
        else
        {
            childMaximum.x = center.x;
        }

        if ((octant & 2) != 0)
        {
            childMinimum.y = center.y;
        }
        else
        {
            childMaximum.y = center.y;
        }

        if ((octant & 4) != 0)
        {
            childMinimum.z = center.z;
        }
        else
        {
            childMaximum.z = center.z;
        }
    }

    PointHierarchyNodeIndex AddNode(
        BuildContext& context,
        const DirectX::XMFLOAT3& boundsMinimum,
        const DirectX::XMFLOAT3& boundsMaximum,
        std::uint32_t depth)
    {
        if (context.hierarchy.nodes.size()
            >= InvalidPointHierarchyNode)
        {
            throw std::overflow_error(
                "Point hierarchy has too many nodes.");
        }

        const PointHierarchyNodeIndex nodeIndex =
            static_cast<PointHierarchyNodeIndex>(
                context.hierarchy.nodes.size());

        context.hierarchy.nodes.emplace_back();

        PointHierarchyNode& node =
            context.hierarchy.nodes[nodeIndex];

        node.boundsMinimum = boundsMinimum;
        node.boundsMaximum = boundsMaximum;
        node.depth = depth;

        context.hierarchy.maximumDepth =
            std::max(
                context.hierarchy.maximumDepth,
                depth);

        return nodeIndex;
    }

    void StoreLeafPoints(
        BuildContext& context,
        PointHierarchyNodeIndex nodeIndex,
        const std::vector<std::uint32_t>& pointIndices)
    {
        const std::size_t firstPoint =
            context.hierarchy.points.size();

        if (firstPoint
            > std::numeric_limits<std::uint32_t>::max()
            - pointIndices.size())
        {
            throw std::overflow_error(
                "Hierarchy point range exceeds "
                "the D3D12 vertex range.");
        }

        PointHierarchyNode& node =
            context.hierarchy.nodes[nodeIndex];

        node.representativePoints.firstPoint =
            static_cast<std::uint32_t>(
                firstPoint);

        node.representativePoints.pointCount =
            static_cast<std::uint32_t>(
                pointIndices.size());

        node.sourcePoints =
            node.representativePoints;

        for (const std::uint32_t pointIndex :
        pointIndices)
        {
            context.hierarchy.points.push_back(
                context.sourcePoints[pointIndex]);
        }
    }

    PointHierarchyNodeIndex BuildNode(
        BuildContext& context,
        std::vector<std::uint32_t> pointIndices,
        const DirectX::XMFLOAT3& boundsMinimum,
        const DirectX::XMFLOAT3& boundsMaximum,
        std::uint32_t depth)
    {
        const PointHierarchyNodeIndex nodeIndex =
            AddNode(
                context,
                boundsMinimum,
                boundsMaximum,
                depth);

        const bool reachedPointLimit =
            pointIndices.size()
            <= context.settings.maximumPointsPerLeaf;

        const bool reachedDepthLimit =
            depth >= context.settings.maximumDepth;

        const bool reachedSpatialLimit =
            CalculateMaximumExtent(
                boundsMinimum,
                boundsMaximum)
            <= context.settings.minimumNodeExtent;

        if (reachedPointLimit
            || reachedDepthLimit
            || reachedSpatialLimit)
        {
            StoreLeafPoints(
                context,
                nodeIndex,
                pointIndices);

            return nodeIndex;
        }

        const DirectX::XMFLOAT3 center =
            CalculateCenter(
                boundsMinimum,
                boundsMaximum);

        std::array<
            std::vector<std::uint32_t>,
            8>
            childPointIndices;

        std::array<std::size_t, 8>
            childPointCounts{};

        for (const std::uint32_t pointIndex :
        pointIndices)
        {
            const std::uint32_t octant =
                FindOctant(
                    context.sourcePoints[
                        pointIndex].position,
                        center);

            ++childPointCounts[octant];
        }

        for (std::uint32_t octant = 0;
            octant < 8;
            ++octant)
        {
            childPointIndices[octant].reserve(
                childPointCounts[octant]);
        }

        for (const std::uint32_t pointIndex :
        pointIndices)
        {
            const std::uint32_t octant =
                FindOctant(
                    context.sourcePoints[
                        pointIndex].position,
                        center);

            childPointIndices[octant].push_back(
                pointIndex);
        }

        // The child vectors now own the index lists.
        // Release the parent copy before recursing.
        std::vector<std::uint32_t>().swap(
            pointIndices);

        for (std::uint32_t octant = 0;
            octant < 8;
            ++octant)
        {
            if (childPointIndices[octant].empty())
            {
                continue;
            }

            DirectX::XMFLOAT3 childMinimum{};
            DirectX::XMFLOAT3 childMaximum{};

            CalculateChildBounds(
                boundsMinimum,
                boundsMaximum,
                center,
                octant,
                childMinimum,
                childMaximum);

            const PointHierarchyNodeIndex childIndex =
                BuildNode(
                    context,
                    std::move(
                        childPointIndices[octant]),
                    childMinimum,
                    childMaximum,
                    depth + 1);

            // Look the parent up again because recursion may
            // have caused the node vector to reallocate.
            context.hierarchy.nodes[
                nodeIndex].children[octant] =
                    childIndex;
        }

        PointRange sourceRange{};
        bool foundFirstChild = false;

        for (const PointHierarchyNodeIndex childIndex :
        context.hierarchy.nodes[nodeIndex].children)
        {
            if (childIndex
                == InvalidPointHierarchyNode)
            {
                continue;
            }

            const PointRange& childRange =
                context.hierarchy.nodes[
                    childIndex].sourcePoints;

            if (!foundFirstChild)
            {
                sourceRange.firstPoint =
                    childRange.firstPoint;

                foundFirstChild = true;
            }

            if (childRange.pointCount
            > std::numeric_limits<std::uint32_t>::max()
                - sourceRange.pointCount)
            {
                throw std::overflow_error(
                    "Hierarchy source range overflow.");
            }

            sourceRange.pointCount +=
                childRange.pointCount;
        }

        if (!foundFirstChild)
        {
            throw std::logic_error(
                "Internal hierarchy node has no children.");
        }

        context.hierarchy.nodes[
            nodeIndex].sourcePoints =
            sourceRange;

        return nodeIndex;
    }

    void ValidateHierarchy(
        const PointCloudHierarchy& hierarchy)
    {
        if (hierarchy.rootIndex
            >= hierarchy.nodes.size())
        {
            throw std::logic_error(
                "Hierarchy root index is invalid.");
        }

        std::uint64_t packedLeafPointCount = 0;

        for (const PointHierarchyNode& node :
            hierarchy.nodes)
        {
            for (const PointHierarchyNodeIndex child :
            node.children)
            {
                if (child != InvalidPointHierarchyNode
                    && child >= hierarchy.nodes.size())
                {
                    throw std::logic_error(
                        "Hierarchy child index is invalid.");
                }
            }

            const std::uint64_t sourceEnd =
                static_cast<std::uint64_t>(
                    node.sourcePoints.firstPoint)
                + node.sourcePoints.pointCount;

            if (sourceEnd
                > hierarchy.sourcePointCount)
            {
                throw std::logic_error(
                    "Node source range exceeds "
                    "the original point array.");
            }

            const std::uint64_t representativeEnd =
                static_cast<std::uint64_t>(
                    node.representativePoints.firstPoint)
                + node.representativePoints.pointCount;

            if (node.representativePoints.pointCount == 0
                || representativeEnd
                > hierarchy.points.size())
            {
                throw std::logic_error(
                    "Node representative range is invalid.");
            }

            if (!node.IsLeaf())
            {
                continue;
            }

            if (node.representativePoints.firstPoint
                != packedLeafPointCount)
            {
                throw std::logic_error(
                    "Leaf point ranges are not contiguous.");
            }

            packedLeafPointCount +=
                node.representativePoints.pointCount;

            if (packedLeafPointCount
            > hierarchy.points.size())
            {
                throw std::logic_error(
                    "Leaf point range exceeds "
                    "the packed point array.");
            }
        }

        if (packedLeafPointCount
            != hierarchy.sourcePointCount
            || hierarchy.points.size()
            < hierarchy.sourcePointCount)
        {
            throw std::logic_error(
                "Hierarchy lost or duplicated "
                "source points.");
        }
    }

    float EstimatePointSpacing(
        const PointHierarchyNode& node,
        std::uint32_t pointCount)
    {
        if (pointCount == 0)
        {
            return 0.0f;
        }

        const float width =
            node.boundsMaximum.x
            - node.boundsMinimum.x;

        const float depth =
            node.boundsMaximum.z
            - node.boundsMinimum.z;

        const float horizontalArea =
            std::max(width, 0.0f)
            * std::max(depth, 0.0f);

        if (horizontalArea > 0.0f)
        {
            return std::sqrt(
                horizontalArea
                / static_cast<float>(pointCount));
        }

        return
            CalculateMaximumExtent(
                node.boundsMinimum,
                node.boundsMaximum)
            / std::sqrt(
                static_cast<float>(pointCount));
    }

    void GenerateRepresentativePoints(
        PointCloudHierarchy& hierarchy,
        std::uint32_t maximumPointsPerNode)
    {
        // BuildNode packed every original point first.
        // New representative samples are appended afterward.
        for (PointHierarchyNode& node :
            hierarchy.nodes)
        {
            if (node.IsLeaf())
            {
                node.spacing =
                    EstimatePointSpacing(
                        node,
                        node.representativePoints.pointCount);

                continue;
            }

            const std::uint32_t sampleCount =
                std::min(
                    node.sourcePoints.pointCount,
                    maximumPointsPerNode);

            if (sampleCount == 0)
            {
                throw std::logic_error(
                    "Internal node has no source points.");
            }

            const std::size_t firstRepresentative =
                hierarchy.points.size();

            if (firstRepresentative
            > std::numeric_limits<std::uint32_t>::max()
                - sampleCount)
            {
                throw std::overflow_error(
                    "Representative point range "
                    "exceeds the vertex range.");
            }

            node.representativePoints.firstPoint =
                static_cast<std::uint32_t>(
                    firstRepresentative);

            node.representativePoints.pointCount =
                sampleCount;

            for (std::uint32_t sampleIndex = 0;
                sampleIndex < sampleCount;
                ++sampleIndex)
            {
                const std::uint64_t sourceOffset =
                    static_cast<std::uint64_t>(
                        sampleIndex)
                    * node.sourcePoints.pointCount
                    / sampleCount;

                const std::uint32_t sourceIndex =
                    node.sourcePoints.firstPoint
                    + static_cast<std::uint32_t>(
                        sourceOffset);

                // Copy before push_back because push_back can
                // reallocate the vector's memory.
                const GpuPoint representative =
                    hierarchy.points[sourceIndex];

                hierarchy.points.push_back(
                    representative);
            }

            node.spacing =
                EstimatePointSpacing(
                    node,
                    sampleCount);
        }
    }
}

PointCloudHierarchy BuildPointCloudHierarchy(
    std::span<const GpuPoint> sourcePoints,
    const DirectX::XMFLOAT3& boundsMinimum,
    const DirectX::XMFLOAT3& boundsMaximum,
    const PointHierarchyBuildSettings& settings)
{
    if (sourcePoints.empty())
    {
        return {};
    }

    if (sourcePoints.size()
        >= InvalidPointHierarchyNode)
    {
        throw std::overflow_error(
            "Source point count exceeds "
            "the supported vertex range.");
    }

    if (!BoundsAreValid(
        boundsMinimum,
        boundsMaximum))
    {
        throw std::invalid_argument(
            "Point hierarchy bounds are invalid.");
    }

    if (settings.maximumPointsPerLeaf == 0)
    {
        throw std::invalid_argument(
            "Maximum points per leaf must "
            "be greater than zero.");
    }

    if (!std::isfinite(
        settings.minimumNodeExtent)
        || settings.minimumNodeExtent < 0.0f)
    {
        throw std::invalid_argument(
            "Minimum node extent is invalid.");
    }

    if (settings.maximumRepresentativePointsPerNode == 0)
    {
        throw std::invalid_argument(
            "Maximum representative points "
            "must be greater than zero.");
    }

    BuildContext context{
        sourcePoints,
        settings
    };

    context.hierarchy.sourcePointCount =
        sourcePoints.size();

    context.hierarchy.points.reserve(
        sourcePoints.size());

    std::vector<std::uint32_t> rootPointIndices(
        sourcePoints.size());

    std::iota(
        rootPointIndices.begin(),
        rootPointIndices.end(),
        std::uint32_t{ 0 });

    context.hierarchy.rootIndex =
        BuildNode(
            context,
            std::move(rootPointIndices),
            boundsMinimum,
            boundsMaximum,
            0);

    GenerateRepresentativePoints(
        context.hierarchy,
        settings.maximumRepresentativePointsPerNode);

    ValidateHierarchy(
        context.hierarchy);

    return std::move(
        context.hierarchy);
}