#include "PointCloudHierarchyBuilder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>
#include <functional>
#include <unordered_map>

namespace
{
    struct BuildContext
    {
        std::span<const GpuPoint> sourcePoints;

        PointHierarchyBuildSettings settings;

        std::stop_token cancellationToken;

        PointCloudHierarchy hierarchy;
    };

    constexpr std::size_t
        CancellationCheckInterval = 4'096;

    void ThrowIfHierarchyBuildCancelled(
        const BuildContext& context)
    {
        if (context.cancellationToken.stop_requested())
        {
            throw std::runtime_error(
                "Point-cloud hierarchy build was cancelled.");
        }
    }

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

        for (std::size_t localIndex = 0;
            localIndex < pointIndices.size();
            ++localIndex)
        {
            if (localIndex
                % CancellationCheckInterval == 0)
            {
                ThrowIfHierarchyBuildCancelled(
                    context);
            }

            const std::uint32_t pointIndex =
                pointIndices[localIndex];

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

        ThrowIfHierarchyBuildCancelled(
            context);

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

        for (std::size_t position = 0;
            position < pointIndices.size();
            ++position)
        {
            if (position
                % CancellationCheckInterval == 0)
            {
                ThrowIfHierarchyBuildCancelled(
                    context);
            }

            const std::uint32_t pointIndex =
                pointIndices[position];

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

        for (std::size_t position = 0;
            position < pointIndices.size();
            ++position)
        {
            if (position
                % CancellationCheckInterval == 0)
            {
                ThrowIfHierarchyBuildCancelled(
                    context);
            }

            const std::uint32_t pointIndex =
                pointIndices[position];

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
        const BuildContext& context)
    {
        const PointCloudHierarchy& hierarchy =
            context.hierarchy;

        if (hierarchy.rootIndex
            >= hierarchy.nodes.size())
        {
            throw std::logic_error(
                "Hierarchy root index is invalid.");
        }

        std::uint64_t packedLeafPointCount = 0;

        for (std::size_t nodeIndex = 0;
            nodeIndex < hierarchy.nodes.size();
            ++nodeIndex)
        {
            ThrowIfHierarchyBuildCancelled(
                context);

            const PointHierarchyNode& node =
                hierarchy.nodes[nodeIndex];

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
        BuildContext& context)
    {
        PointCloudHierarchy& hierarchy =
            context.hierarchy;

        const std::uint32_t maximumPointsPerNode =
            context.settings.
            maximumRepresentativePointsPerNode;
        // BuildNode packed every original point first.
        // New representative samples are appended afterward.
        for (std::size_t nodeIndex = 0;
            nodeIndex < hierarchy.nodes.size();
            ++nodeIndex)
        {
            ThrowIfHierarchyBuildCancelled(
                context);

            PointHierarchyNode& node =
                hierarchy.nodes[nodeIndex];
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
                if (sampleIndex
                    % CancellationCheckInterval == 0)
                {
                    ThrowIfHierarchyBuildCancelled(
                        context);
                }

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

    struct ClodGridCellKey
    {
        std::int64_t x = 0;
        std::int64_t y = 0;
        std::int64_t z = 0;

        [[nodiscard]]
        bool operator==(
            const ClodGridCellKey&) const noexcept =
            default;
    };

    struct ClodGridCellKeyHasher
    {
        [[nodiscard]]
        std::size_t operator()(
            const ClodGridCellKey& key) const noexcept
        {
            std::size_t result = 0;

            const auto combine =
                [&result](std::int64_t value)
                {
                    const std::size_t valueHash =
                        std::hash<std::int64_t>{}(
                            value);

                    result ^=
                        valueHash
                        + 0x9e3779b9u
                        + (result << 6)
                        + (result >> 2);
                };

            combine(key.x);
            combine(key.y);
            combine(key.z);

            return result;
        }
    };

    class ClodMinimumDistanceGrid
    {
    public:
        ClodMinimumDistanceGrid(
            const DirectX::XMFLOAT3& boundsMinimum,
            float spacing)
            :
            boundsMinimum_(
                boundsMinimum),

            cellSize_(
                static_cast<double>(spacing)),

            spacingSquared_(
                static_cast<double>(spacing)
                * static_cast<double>(spacing))
        {
            if (!std::isfinite(spacing)
                || spacing <= 0.0f)
            {
                throw std::invalid_argument(
                    "CLOD node spacing must be positive.");
            }
        }

        [[nodiscard]]
        bool TryAccept(
            const DirectX::XMFLOAT3& position)
        {
            const ClodGridCellKey candidateCell =
                CalculateCell(position);

            for (std::int64_t zOffset = -1;
                zOffset <= 1;
                ++zOffset)
            {
                for (std::int64_t yOffset = -1;
                    yOffset <= 1;
                    ++yOffset)
                {
                    for (std::int64_t xOffset = -1;
                        xOffset <= 1;
                        ++xOffset)
                    {
                        const ClodGridCellKey
                            neighbourKey{
                                candidateCell.x
                                    + xOffset,

                                candidateCell.y
                                    + yOffset,

                                candidateCell.z
                                    + zOffset
                        };

                        const auto neighbour =
                            cells_.find(neighbourKey);

                        if (neighbour == cells_.end())
                        {
                            continue;
                        }

                        for (const DirectX::XMFLOAT3&
                            acceptedPoint :
                            neighbour->second)
                        {
                            const double x =
                                static_cast<double>(
                                    position.x)
                                - acceptedPoint.x;

                            const double y =
                                static_cast<double>(
                                    position.y)
                                - acceptedPoint.y;

                            const double z =
                                static_cast<double>(
                                    position.z)
                                - acceptedPoint.z;

                            const double distanceSquared =
                                x * x
                                + y * y
                                + z * z;

                            if (distanceSquared
                                < spacingSquared_)
                            {
                                return false;
                            }
                        }
                    }
                }
            }

            cells_[candidateCell].push_back(
                position);

            return true;
        }

    private:
        [[nodiscard]]
        ClodGridCellKey CalculateCell(
            const DirectX::XMFLOAT3& position)
            const noexcept
        {
            return {
                static_cast<std::int64_t>(
                    std::floor(
                        (static_cast<double>(position.x)
                            - boundsMinimum_.x)
                        / cellSize_)),

                static_cast<std::int64_t>(
                    std::floor(
                        (static_cast<double>(position.y)
                            - boundsMinimum_.y)
                        / cellSize_)),

                static_cast<std::int64_t>(
                    std::floor(
                        (static_cast<double>(position.z)
                            - boundsMinimum_.z)
                        / cellSize_))
            };
        }

        DirectX::XMFLOAT3 boundsMinimum_{};

        double cellSize_ = 0.0;
        double spacingSquared_ = 0.0;

        std::unordered_map<
            ClodGridCellKey,
            std::vector<DirectX::XMFLOAT3>,
            ClodGridCellKeyHasher>
            cells_;
    };

    struct ClodBuildBounds
    {
        DirectX::XMFLOAT3 minimum{};
        DirectX::XMFLOAT3 maximum{};
    };

    [[nodiscard]]
    ClodBuildBounds CalculateClodBuildBounds(
        const DirectX::XMFLOAT3& boundsMinimum,
        const DirectX::XMFLOAT3& boundsMaximum)
    {
        const float extent =
            CalculateMaximumExtent(
                boundsMinimum,
                boundsMaximum);

        if (!std::isfinite(extent)
            || extent <= 0.0f)
        {
            throw std::invalid_argument(
                "CLOD bounds must have positive extent.");
        }

        ClodBuildBounds result{};

        result.minimum =
            boundsMinimum;

        // This matches Potree's makeCubic():
        // preserve minimum and extend every axis to
        // the largest original extent.
        result.maximum = {
            boundsMinimum.x + extent,
            boundsMinimum.y + extent,
            boundsMinimum.z + extent
        };

        return result;
    }

    [[nodiscard]]
    float ResolveClodRootSpacing(
        const PointHierarchyBuildSettings& settings,
        const ClodBuildBounds& bounds)
    {
        if (settings.clodRootSpacing > 0.0f)
        {
            return settings.clodRootSpacing;
        }

        const float cubeExtent =
            bounds.maximum.x
            - bounds.minimum.x;

        const float diagonal =
            cubeExtent
            * std::sqrt(3.0f);

        const float rootSpacing =
            diagonal
            / settings.clodSpacingDiagonalFraction;

        if (!std::isfinite(rootSpacing)
            || rootSpacing <= 0.0f)
        {
            throw std::invalid_argument(
                "Calculated CLOD root spacing "
                "is invalid.");
        }

        return rootSpacing;
    }

    inline constexpr std::uint16_t
        UnassignedClodLevel =
        std::numeric_limits<std::uint16_t>::max();

    void AssignRemainingClodPoints(
        BuildContext& context,
        const std::vector<std::uint32_t>& pointIndices,
        std::uint32_t level,
        std::vector<std::uint16_t>& pointLevels)
    {
        if (level
        > std::numeric_limits<std::uint8_t>::max())
        {
            throw std::overflow_error(
                "CLOD level does not fit in one byte.");
        }

        for (std::size_t index = 0;
            index < pointIndices.size();
            ++index)
        {
            if (index
                % CancellationCheckInterval == 0)
            {
                ThrowIfHierarchyBuildCancelled(
                    context);
            }

            const std::uint32_t pointIndex =
                pointIndices[index];

            if (pointLevels[pointIndex]
                != UnassignedClodLevel)
            {
                throw std::logic_error(
                    "CLOD point was assigned twice.");
            }

            pointLevels[pointIndex] =
                static_cast<std::uint16_t>(
                    level);
        }
    }

    void AssignClodLevelsForNode(
        BuildContext& context,
        std::vector<std::uint32_t> pointIndices,
        const DirectX::XMFLOAT3& boundsMinimum,
        const DirectX::XMFLOAT3& boundsMaximum,
        std::uint32_t depth,
        float rootSpacing,
        std::vector<std::uint16_t>& pointLevels)
    {
        ThrowIfHierarchyBuildCancelled(
            context);

        if (pointIndices.empty())
        {
            return;
        }

        const bool remainsLeaf =
            pointIndices.size()
            < context.settings.clodStoreLimit;

        const bool reachedDepthLimit =
            depth
            >= context.settings.clodMaximumDepth;

        if (remainsLeaf || reachedDepthLimit)
        {
            AssignRemainingClodPoints(
                context,
                pointIndices,
                depth,
                pointLevels);

            return;
        }

        const float nodeSpacing =
            std::ldexp(
                rootSpacing,
                -static_cast<int>(depth));

        const DirectX::XMFLOAT3 center =
            CalculateCenter(
                boundsMinimum,
                boundsMaximum);

        std::array<
            std::vector<std::uint32_t>,
            8>
            rejectedChildPoints;

        // The grid only needs to live while this node
        // separates accepted and rejected points.
        {
            ClodMinimumDistanceGrid grid(
                boundsMinimum,
                nodeSpacing);

            for (std::size_t position = 0;
                position < pointIndices.size();
                ++position)
            {
                if (position
                    % CancellationCheckInterval == 0)
                {
                    ThrowIfHierarchyBuildCancelled(
                        context);
                }

                const std::uint32_t pointIndex =
                    pointIndices[position];

                if (pointLevels[pointIndex]
                    != UnassignedClodLevel)
                {
                    throw std::logic_error(
                        "CLOD point reached a node "
                        "after it was already assigned.");
                }

                const GpuPoint& sourcePoint =
                    context.sourcePoints[
                        pointIndex];

                if (grid.TryAccept(
                    sourcePoint.position))
                {
                    // Depth is stored only because the point
                    // passed this depth's spacing test.
                    pointLevels[pointIndex] =
                        static_cast<std::uint16_t>(
                            depth);

                    continue;
                }

                const std::uint32_t octant =
                    FindOctant(
                        sourcePoint.position,
                        center);

                rejectedChildPoints[
                    octant].push_back(
                        pointIndex);
            }
        }

        // The child lists now own every rejected index.
        // Release the parent list before recursion.
        std::vector<std::uint32_t>().swap(
            pointIndices);

        for (std::uint32_t octant = 0;
            octant < 8;
            ++octant)
        {
            if (rejectedChildPoints[
                octant].empty())
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

            AssignClodLevelsForNode(
                context,
                std::move(
                    rejectedChildPoints[octant]),
                childMinimum,
                childMaximum,
                depth + 1,
                rootSpacing,
                pointLevels);
        }
    }

    void GenerateClodRepresentation(
        BuildContext& context,
        const DirectX::XMFLOAT3& boundsMinimum,
        const DirectX::XMFLOAT3& boundsMaximum)
    {
        ThrowIfHierarchyBuildCancelled(
            context);

        const ClodBuildBounds clodBounds =
            CalculateClodBuildBounds(
                boundsMinimum,
                boundsMaximum);

        const float rootSpacing =
            ResolveClodRootSpacing(
                context.settings,
                clodBounds);

        std::vector<std::uint16_t> pointLevels(
            context.sourcePoints.size(),
            UnassignedClodLevel);

        std::vector<std::uint32_t>
            rootPointIndices(
                context.sourcePoints.size());

        std::iota(
            rootPointIndices.begin(),
            rootPointIndices.end(),
            0u);

        AssignClodLevelsForNode(
            context,
            std::move(rootPointIndices),
            clodBounds.minimum,
            clodBounds.maximum,
            0,
            rootSpacing,
            pointLevels);

        std::uint16_t maximumLevel = 0;

        for (const std::uint16_t level :
        pointLevels)
        {
            if (level == UnassignedClodLevel)
            {
                throw std::logic_error(
                    "CLOD construction lost a point.");
            }

            maximumLevel =
                std::max(
                    maximumLevel,
                    level);
        }

        std::vector<std::uint32_t>
            levelPointCounts(
                static_cast<std::size_t>(
                    maximumLevel)
                + 1,
                0);

        for (const std::uint16_t level :
        pointLevels)
        {
            ++levelPointCounts[level];
        }

        PointCloudHierarchy& hierarchy =
            context.hierarchy;

        hierarchy.clodRootSpacing =
            rootSpacing;

        hierarchy.clodLevelRanges.resize(
            levelPointCounts.size());

        std::uint32_t firstPoint = 0;

        for (std::size_t level = 0;
            level < levelPointCounts.size();
            ++level)
        {
            hierarchy.clodLevelRanges[level] = {
                firstPoint,
                levelPointCounts[level]
            };

            firstPoint +=
                levelPointCounts[level];
        }

        if (firstPoint
            != context.sourcePoints.size())
        {
            throw std::logic_error(
                "CLOD level counts do not match "
                "the source point count.");
        }

        hierarchy.clodPoints.resize(
            context.sourcePoints.size());

        std::vector<std::uint32_t>
            levelWritePositions(
                hierarchy.clodLevelRanges.size());

        for (std::size_t level = 0;
            level < hierarchy.clodLevelRanges.size();
            ++level)
        {
            levelWritePositions[level] =
                hierarchy.clodLevelRanges[
                    level].firstPoint;
        }

        for (std::size_t sourceIndex = 0;
            sourceIndex < context.sourcePoints.size();
            ++sourceIndex)
        {
            if (sourceIndex
                % CancellationCheckInterval == 0)
            {
                ThrowIfHierarchyBuildCancelled(
                    context);
            }

            const std::uint8_t level =
                static_cast<std::uint8_t>(
                    pointLevels[sourceIndex]);

            const std::uint32_t destinationIndex =
                levelWritePositions[level]++;

            const GpuPoint& sourcePoint =
                context.sourcePoints[sourceIndex];

            hierarchy.clodPoints[
                destinationIndex] = {
                    sourcePoint.position,

                    PackClodColorAndLevel(
                        sourcePoint.color,
                        level)
            };
        }

        for (std::size_t level = 0;
            level < hierarchy.clodLevelRanges.size();
            ++level)
        {
            const PointRange& range =
                hierarchy.clodLevelRanges[level];

            const std::uint32_t expectedEnd =
                range.firstPoint
                + range.pointCount;

            if (levelWritePositions[level]
                != expectedEnd)
            {
                throw std::logic_error(
                    "CLOD level range was not "
                    "filled correctly.");
            }
        }

        if (hierarchy.clodPoints.size()
            != hierarchy.sourcePointCount)
        {
            throw std::logic_error(
                "CLOD representation lost or "
                "duplicated source points.");
        }
    }
}

PointCloudHierarchy
BuildDiscretePointCloudHierarchy(
    std::span<const GpuPoint> sourcePoints,
    const DirectX::XMFLOAT3& boundsMinimum,
    const DirectX::XMFLOAT3& boundsMaximum,
    const PointHierarchyBuildSettings& settings,
    std::stop_token cancellationToken)
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
        settings,
        cancellationToken
    };

    ThrowIfHierarchyBuildCancelled(
        context);

    context.hierarchy.sourcePointCount =
        sourcePoints.size();

    context.hierarchy.points.reserve(
        sourcePoints.size());

    std::vector<std::uint32_t>
        rootPointIndices;

    rootPointIndices.reserve(
        sourcePoints.size());

    for (std::size_t pointIndex = 0;
        pointIndex < sourcePoints.size();
        ++pointIndex)
    {
        if (pointIndex
            % CancellationCheckInterval == 0)
        {
            ThrowIfHierarchyBuildCancelled(
                context);
        }

        rootPointIndices.push_back(
            static_cast<std::uint32_t>(
                pointIndex));
    }

    context.hierarchy.rootIndex =
        BuildNode(
            context,
            std::move(rootPointIndices),
            boundsMinimum,
            boundsMaximum,
            0);

    GenerateRepresentativePoints(
        context);

    ValidateHierarchy(
        context);

    return std::move(
        context.hierarchy);
}

void BuildClodRepresentation(
    PointCloudHierarchy& hierarchy,
    std::span<const GpuPoint> sourcePoints,
    const DirectX::XMFLOAT3& boundsMinimum,
    const DirectX::XMFLOAT3& boundsMaximum,
    const PointHierarchyBuildSettings& settings,
    std::stop_token cancellationToken)
{
    if (sourcePoints.empty())
    {
        if (hierarchy.sourcePointCount != 0)
        {
            throw std::invalid_argument(
                "An existing hierarchy cannot receive "
                "an empty CLOD source array.");
        }

        hierarchy.clodPoints.clear();
        hierarchy.clodLevelRanges.clear();
        hierarchy.clodRootSpacing = 0.0f;

        return;
    }

    if (sourcePoints.size()
        != hierarchy.sourcePointCount)
    {
        throw std::invalid_argument(
            "CLOD source count does not match "
            "the discrete hierarchy.");
    }

    if (sourcePoints.size()
        >= InvalidPointHierarchyNode)
    {
        throw std::overflow_error(
            "CLOD source count exceeds the "
            "supported point range.");
    }

    if (!BoundsAreValid(
        boundsMinimum,
        boundsMaximum))
    {
        throw std::invalid_argument(
            "CLOD hierarchy bounds are invalid.");
    }

    if (settings.clodStoreLimit == 0)
    {
        throw std::invalid_argument(
            "CLOD store limit must be "
            "greater than zero.");
    }

    if (settings.clodMaximumDepth
        > std::numeric_limits<std::uint8_t>::max())
    {
        throw std::invalid_argument(
            "CLOD maximum depth must fit "
            "in one byte.");
    }

    if (!std::isfinite(
        settings.clodRootSpacing)
        || settings.clodRootSpacing < 0.0f)
    {
        throw std::invalid_argument(
            "CLOD root spacing is invalid.");
    }

    if (settings.clodRootSpacing == 0.0f
        && (!std::isfinite(
            settings.clodSpacingDiagonalFraction)
            || settings.clodSpacingDiagonalFraction
            <= 0.0f))
    {
        throw std::invalid_argument(
            "CLOD diagonal fraction must "
            "be positive.");
    }

    BuildContext context{
        sourcePoints,
        settings,
        cancellationToken
    };

    // Move the existing discrete hierarchy into the same
    // build context used by the internal CLOD routines.
    context.hierarchy =
        std::move(hierarchy);

    ThrowIfHierarchyBuildCancelled(
        context);

    GenerateClodRepresentation(
        context,
        boundsMinimum,
        boundsMaximum);

    hierarchy =
        std::move(context.hierarchy);
}

PointCloudHierarchy BuildPointCloudHierarchy(
    std::span<const GpuPoint> sourcePoints,
    const DirectX::XMFLOAT3& boundsMinimum,
    const DirectX::XMFLOAT3& boundsMaximum,
    const PointHierarchyBuildSettings& settings,
    std::stop_token cancellationToken)
{
    PointCloudHierarchy hierarchy =
        BuildDiscretePointCloudHierarchy(
            sourcePoints,
            boundsMinimum,
            boundsMaximum,
            settings,
            cancellationToken);

    if (hierarchy.Empty())
    {
        return hierarchy;
    }

    BuildClodRepresentation(
        hierarchy,
        sourcePoints,
        boundsMinimum,
        boundsMaximum,
        settings,
        cancellationToken);

    return hierarchy;
}