#include "Application.h"

#include "Win32Helpers.h"
#include "LasPointCloudLoader.h"
#include "PointCloudHierarchyBuilder.h"

#include <cstdlib>
#include <exception>
#include <string>
#include <Windowsx.h>
#include <algorithm>
#include <chrono>
#include <utility>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <DirectXCollision.h>
#include <queue>

namespace
{
    constexpr wchar_t WindowClassName[] =
        L"PointCloudRendererWindowClass";

    constexpr wchar_t WindowTitle[] =
        L"D3D12 Point Cloud Renderer";

    float DistanceOutsideInterval(
        float value,
        float minimum,
        float maximum)
    {
        if (value < minimum)
        {
            return minimum - value;
        }

        if (value > maximum)
        {
            return value - maximum;
        }

        return 0.0f;
    }

    float DistanceSquaredToBounds(
        const DirectX::XMFLOAT3& position,
        const DirectX::XMFLOAT3& minimum,
        const DirectX::XMFLOAT3& maximum)
    {
        const float distanceX =
            DistanceOutsideInterval(
                position.x,
                minimum.x,
                maximum.x);

        const float distanceY =
            DistanceOutsideInterval(
                position.y,
                minimum.y,
                maximum.y);

        const float distanceZ =
            DistanceOutsideInterval(
                position.z,
                minimum.z,
                maximum.z);

        return
            distanceX * distanceX
            + distanceY * distanceY
            + distanceZ * distanceZ;
    }

    struct PointRefinementCandidate
    {
        std::size_t tileStateIndex = 0;

        PointHierarchyNodeIndex nodeIndex =
            InvalidPointHierarchyNode;

        float projectedSpacingPixels = 0.0f;
    };

    struct PointRefinementCandidateLess
    {
        bool operator()(
            const PointRefinementCandidate& left,
            const PointRefinementCandidate& right) const
        {
            if (left.projectedSpacingPixels
                != right.projectedSpacingPixels)
            {
                // priority_queue places the candidate with the
                // greatest projected error at the top.
                return
                    left.projectedSpacingPixels
                    < right.projectedSpacingPixels;
            }

            if (left.tileStateIndex
                != right.tileStateIndex)
            {
                return
                    left.tileStateIndex
                    > right.tileStateIndex;
            }

            return left.nodeIndex > right.nodeIndex;
        }
    };

    bool BoundsIntersectFrustum(
        const DirectX::BoundingFrustum& frustum,
        const DirectX::XMFLOAT3& minimum,
        const DirectX::XMFLOAT3& maximum)
    {
        DirectX::BoundingBox box{};

        box.Center = {
            minimum.x
                + (maximum.x - minimum.x) * 0.5f,

            minimum.y
                + (maximum.y - minimum.y) * 0.5f,

            minimum.z
                + (maximum.z - minimum.z) * 0.5f
        };

        box.Extents = {
            std::max(
                (maximum.x - minimum.x) * 0.5f,
                0.0f),

            std::max(
                (maximum.y - minimum.y) * 0.5f,
                0.0f),

            std::max(
                (maximum.z - minimum.z) * 0.5f,
                0.0f)
        };

        return frustum.Intersects(box);
    }
}

Application::Application(
    HINSTANCE instance,
    std::vector<std::filesystem::path>
    pointCloudPaths)
    : instance_(instance),
    pointCloudPaths_(
        std::move(pointCloudPaths))
{
}

int Application::Run(int showCommand)
{
    RegisterWindowClass();
    CreateApplicationWindow();

    graphics_.Initialize(
        window_,
        clientWidth_,
        clientHeight_);

    windowTitlePrefix_ =
        std::wstring(WindowTitle)
        + L" | "
        + graphics_.GetAdapterName();

    if (!pointCloudPaths_.empty())
    {
        pointCloudDataset_ =
            InspectPointCloudDataset(
                pointCloudPaths_);

        const PointCloudDatasetInfo& dataset =
            *pointCloudDataset_;

        tileResidencyStates_.clear();

        tileResidencyStates_.reserve(
            dataset.tileMetadata.size());

        for (std::size_t index = 0;
            index < dataset.tileMetadata.size();
            ++index)
        {
            const PointCloudMetadata& metadata =
                dataset.tileMetadata[index];

            if (metadata.pointCount
            > std::numeric_limits<std::uint64_t>::max()
                / sizeof(GpuPoint))
            {
                throw std::overflow_error(
                    "Tile GPU memory estimate overflow.");
            }

            TileResidencyState state{};

            state.metadataIndex = index;

            state.requiredGpuBytes =
                metadata.pointCount
                * sizeof(GpuPoint);

            const Bounds3d& sourceBounds =
                metadata.sourceBounds;

            const Double3& origin =
                dataset.localOrigin;

            state.localBoundsMinimum = {
                static_cast<float>(
                    sourceBounds.minimum.x
                    - origin.x),

                static_cast<float>(
                    sourceBounds.minimum.z
                    - origin.z),

                static_cast<float>(
                    sourceBounds.minimum.y
                    - origin.y)
            };

            state.localBoundsMaximum = {
                static_cast<float>(
                    sourceBounds.maximum.x
                    - origin.x),

                static_cast<float>(
                    sourceBounds.maximum.z
                    - origin.z),

                static_cast<float>(
                    sourceBounds.maximum.y
                    - origin.y)
            };

            tileResidencyStates_.push_back(
                state);
        }

        tilePriorityOrder_.resize(
            tileResidencyStates_.size());

        std::iota(
            tilePriorityOrder_.begin(),
            tilePriorityOrder_.end(),
            std::size_t{ 0 });

        windowTitlePrefix_ +=
            L" | "
            + std::to_wstring(
                dataset.tileMetadata.size())
            + L" tile";

        if (dataset.tileMetadata.size() != 1)
        {
            windowTitlePrefix_ += L"s";
        }

        graphics_.ClearPointCloudTiles();

        OutputDebugStringW(
            L"\n========== Point Cloud Dataset ==========\n");

        OutputDebugStringW(
            (
                L"Tiles: "
                + std::to_wstring(
                    dataset.tileMetadata.size())
                + L"\nTotal points: "
                + std::to_wstring(
                    dataset.totalPointCount)
                + L"\n"
                ).c_str());

        OutputDebugStringW(
            L"=========================================\n\n");

        graphics_.SetElevationRange(
            dataset.localBoundsMinimum.y,
            dataset.localBoundsMaximum.y);

        FramePointCloud();
    }

    SetWindowTextW(
        window_,
        windowTitlePrefix_.c_str());

    ShowWindow(window_, showCommand);
    UpdateWindow(window_);

    MSG message{};

    auto previousTime =
        std::chrono::steady_clock::now();

    while (message.message != WM_QUIT)
    {
        if (PeekMessageW(
            &message,
            nullptr,
            0,
            0,
            PM_REMOVE))
        {
            TranslateMessage(&message);
            DispatchMessageW(&message);
        }
        else if (!minimized_)
        {
            const auto currentTime =
                std::chrono::steady_clock::now();

            const float deltaSeconds =
                std::min(
                    std::chrono::duration<float>(
                        currentTime - previousTime).count(),
                    0.1f);

            previousTime = currentTime;

            UpdateCamera(deltaSeconds);

            UpdateTilePriorities();
            UpdateTileResidency(deltaSeconds);
            UpdatePointDetailSelection();

            const auto renderStart =
                std::chrono::steady_clock::now();

            graphics_.Render(camera_);

            const auto renderEnd =
                std::chrono::steady_clock::now();

            const double frameSeconds =
                std::chrono::duration<double>(
                    renderEnd - renderStart).count();

            UpdateBenchmark(frameSeconds);

            UpdateFrameStatistics(
                frameSeconds);
        }
        else
        {
            WaitMessage();
        }
    }

    graphics_.Shutdown();

    return static_cast<int>(message.wParam);
}

void Application::FramePointCloud()
{
    if (!pointCloudDataset_)
    {
        return;
    }

    const DirectX::XMFLOAT3& minimum =
        pointCloudDataset_->
        localBoundsMinimum;

    const DirectX::XMFLOAT3& maximum =
        pointCloudDataset_->
        localBoundsMaximum;

    camera_.FrameBounds(
        minimum,
        maximum);

    const float width =
        maximum.x - minimum.x;

    const float height =
        maximum.y - minimum.y;

    const float depth =
        maximum.z - minimum.z;

    const float diagonal =
        std::sqrt(
            width * width
            + height * height
            + depth * depth);

    cameraMovementSpeed_ =
        std::max(
            diagonal * 0.1f,
            1.0f);
}

void Application::UpdateTilePriorities()
{
    if (!pointCloudDataset_
        || tileResidencyStates_.empty())
    {
        wantedTileCount_ = 0;
        wantedPointBytes_ = 0;
        return;
    }

    const DirectX::XMFLOAT3& cameraPosition =
        camera_.GetPosition();

    for (TileResidencyState& state :
        tileResidencyStates_)
    {
        state.cameraDistanceSquared =
            DistanceSquaredToBounds(
                cameraPosition,
                state.localBoundsMinimum,
                state.localBoundsMaximum);

        state.wanted = false;
    }

    std::sort(
        tilePriorityOrder_.begin(),
        tilePriorityOrder_.end(),
        [this](
            std::size_t leftIndex,
            std::size_t rightIndex)
        {
            constexpr float residentBiasSquared = 0.64f;

            const TileResidencyState& left =
                tileResidencyStates_[leftIndex];

            const TileResidencyState& right =
                tileResidencyStates_[rightIndex];

            const float leftPriority =
                left.cameraDistanceSquared
                * (left.IsResident()
                    ? residentBiasSquared
                    : 1.0f);

            const float rightPriority =
                right.cameraDistanceSquared
                * (right.IsResident()
                    ? residentBiasSquared
                    : 1.0f);

            if (leftPriority != rightPriority)
            {
                return leftPriority < rightPriority;
            }

            // Deterministic tie-breaker.
            return
                left.metadataIndex
                < right.metadataIndex;
        });

    wantedTileCount_ = 0;
    wantedPointBytes_ = 0;

    for (const std::size_t stateIndex :
    tilePriorityOrder_)
    {
        TileResidencyState& state =
            tileResidencyStates_[
                stateIndex];

        const bool fitsBudget =
            state.requiredGpuBytes
            <= PointMemoryBudgetBytes
            && wantedPointBytes_
            <= PointMemoryBudgetBytes
            - state.requiredGpuBytes;

        if (!fitsBudget)
        {
            continue;
        }

        state.wanted = true;

        wantedPointBytes_ +=
            state.requiredGpuBytes;

        ++wantedTileCount_;
    }
}

void Application::UpdateTileResidency(
    float deltaSeconds)
{
    if (!pointCloudDataset_)
    {
        return;
    }

    residencyUpdateAccumulator_ +=
        static_cast<double>(deltaSeconds);

    if (residencyUpdateAccumulator_
        < ResidencyUpdateIntervalSeconds)
    {
        return;
    }

    residencyUpdateAccumulator_ = 0.0;

    const PointCloudDatasetInfo& dataset =
        *pointCloudDataset_;

    // First remove active tiles that are no longer wanted.
    for (TileResidencyState& state :
        tileResidencyStates_)
    {
        if (!state.IsResident()
            || state.wanted)
        {
            continue;
        }

        const PointCloudTileId tileId =
            *state.gpuTileId;

        const bool removed =
            graphics_.RemovePointCloudTile(
                tileId);

        if (!removed)
        {
            throw std::logic_error(
                "Application residency state "
                "does not match the renderer.");
        }

        state.gpuTileId.reset();
        state.hierarchy = PointCloudHierarchy{};
        state.selectedNodeFlags.clear();
        state.refinedNodeFlags.clear();
        state.nextRefinedNodeFlags.clear();
        state.selectedDrawRanges.clear();
    }

    // Load at most one wanted tile per update.
    for (const std::size_t stateIndex :
    tilePriorityOrder_)
    {
        TileResidencyState& state =
            tileResidencyStates_[
                stateIndex];

        if (!state.wanted
            || state.IsResident())
        {
            continue;
        }

        const std::uint64_t allocatedBytes =
            graphics_.
            GetPointBufferSizeBytes();

        const bool memoryAvailable =
            state.requiredGpuBytes
            <= PointMemoryBudgetBytes
            && allocatedBytes
            <= PointMemoryBudgetBytes
            - state.requiredGpuBytes;

        if (!memoryAvailable)
        {
            continue;
        }

        const PointCloudMetadata& metadata =
            dataset.tileMetadata[
                state.metadataIndex];

        PointCloudData tile =
            LasPointCloudLoader::Load(
                metadata,
                dataset.localOrigin);

        PointCloudHierarchy hierarchy =
            BuildPointCloudHierarchy(
                tile.points,
                tile.localBoundsMinimum,
                tile.localBoundsMaximum);

        const std::uint64_t actualBytes =
            static_cast<std::uint64_t>(
                hierarchy.points.size())
            * sizeof(GpuPoint);

        const std::uint64_t sourceBytes =
            metadata.pointCount
            * sizeof(GpuPoint);

        if (actualBytes < sourceBytes)
        {
            throw std::runtime_error(
                "Hierarchy contains fewer points "
                "than the source tile.");
        }

        state.requiredGpuBytes =
            actualBytes;

        const std::uint64_t currentAllocatedBytes =
            graphics_.GetPointBufferSizeBytes();

        const bool hierarchyFitsBudget =
            actualBytes <= PointMemoryBudgetBytes
            && currentAllocatedBytes
            <= PointMemoryBudgetBytes
            - actualBytes;

        if (!hierarchyFitsBudget)
        {
            OutputDebugStringW(
                L"Hierarchy does not currently fit "
                L"the point-memory budget.\n");

            continue;
        }

        const std::wstring message =
            L"Loading resident tile: "
            + metadata.sourcePath.filename().wstring()
            + L"\n";

        OutputDebugStringW(
            message.c_str());

        state.gpuTileId =
            graphics_.AddPointCloudTile(
                hierarchy.points,
                static_cast<std::uint32_t>(
                    hierarchy.sourcePointCount));

        const std::size_t hierarchyNodeCount =
            hierarchy.nodes.size();

        // The GPU now owns its copy of the packed points.
        // Keep the lightweight node metadata on the CPU.
        hierarchy.points =
            std::vector<GpuPoint>{};

        state.hierarchy =
            std::move(hierarchy);

        OutputDebugStringW(
            (
                L"Hierarchy nodes: "
                + std::to_wstring(
                    hierarchyNodeCount)
                + L"\n"
                ).c_str());

        // Only one decode/upload per update.
        ValidateTileResidency();
        return;
    }

    ValidateTileResidency();
}

void Application::ValidateTileResidency() const
{
    const std::uint64_t allocatedBytes =
        graphics_.GetPointBufferSizeBytes();

    if (allocatedBytes > PointMemoryBudgetBytes)
    {
        throw std::logic_error(
            "Point-buffer memory budget exceeded.");
    }

    std::size_t applicationResidentCount = 0;

    for (const TileResidencyState& state :
        tileResidencyStates_)
    {
        if (state.IsResident())
        {
            ++applicationResidentCount;
        }
    }

    if (applicationResidentCount
        != graphics_.GetPointCloudTileCount())
    {
        throw std::logic_error(
            "Application and renderer resident "
            "tile counts disagree.");
    }
}

void Application::UpdatePointDetailSelection()
{
    if (clientHeight_ == 0)
    {
        return;
    }

    selectedDrawRangeCount_ = 0;
    frustumCulledNodeCount_ = 0;

    if (fullDetailMode_)
    {
        for (TileResidencyState& state :
            tileResidencyStates_)
        {
            if (!state.IsResident()
                || state.hierarchy.Empty())
            {
                continue;
            }

            state.selectedDrawRanges.clear();

            state.selectedDrawRanges.push_back({
                0,
                static_cast<std::uint32_t>(
                    state.hierarchy.sourcePointCount)
                });

            const bool updated =
                graphics_.SetPointCloudTileDrawRanges(
                    *state.gpuTileId,
                    state.selectedDrawRanges);

            if (!updated)
            {
                throw std::logic_error(
                    "Full-detail tile does not "
                    "exist in the renderer.");
            }

            ++selectedDrawRangeCount_;

            // Restart hysteresis cleanly when returning to LOD.
            std::fill(
                state.refinedNodeFlags.begin(),
                state.refinedNodeFlags.end(),
                std::uint8_t{ 0 });

            std::fill(
                state.nextRefinedNodeFlags.begin(),
                state.nextRefinedNodeFlags.end(),
                std::uint8_t{ 0 });
        }

        return;
    }

    DirectX::XMFLOAT4X4 projection{};

    DirectX::XMStoreFloat4x4(
        &projection,
        camera_.GetProjectionMatrix());

    const float projectionScale =
        static_cast<float>(clientHeight_)
        * 0.5f
        * projection._22;

    const DirectX::XMFLOAT3& cameraPosition =
        camera_.GetPosition();

    DirectX::BoundingFrustum viewFrustum{};

    DirectX::BoundingFrustum::CreateFromMatrix(
        viewFrustum,
        camera_.GetProjectionMatrix());

    const DirectX::XMMATRIX inverseView =
        DirectX::XMMatrixInverse(
            nullptr,
            camera_.GetViewMatrix());

    DirectX::BoundingFrustum worldFrustum{};

    viewFrustum.Transform(
        worldFrustum,
        inverseView);

    using RefinementQueue =
        std::priority_queue<
        PointRefinementCandidate,
        std::vector<PointRefinementCandidate>,
        PointRefinementCandidateLess>;

    RefinementQueue refinementQueue;

    std::uint64_t selectedPointCount = 0;

    // Calculates the visual error of one node.
    const auto calculateProjectedSpacing =
        [&cameraPosition, projectionScale](
            const PointHierarchyNode& node)
        {
            const float distanceSquared =
                DistanceSquaredToBounds(
                    cameraPosition,
                    node.boundsMinimum,
                    node.boundsMaximum);

            const float distance =
                std::max(
                    std::sqrt(distanceSquared),
                    0.01f);

            return
                node.spacing
                * projectionScale
                / distance;
        };

    // Adds a selected node to the refinement queue when
    // its projected error exceeds its current threshold.
    const auto queueRefinement =
        [this,
        &refinementQueue,
        &calculateProjectedSpacing](
            std::size_t tileStateIndex,
            PointHierarchyNodeIndex nodeIndex)
        {
            const TileResidencyState& state =
                tileResidencyStates_[
                    tileStateIndex];

            const PointHierarchyNode& node =
                state.hierarchy.nodes[nodeIndex];

            if (node.IsLeaf())
            {
                return;
            }

            const bool wasRefinedLastFrame =
                state.refinedNodeFlags[nodeIndex]
                != 0;

            const float threshold =
                wasRefinedLastFrame
                ? CoarsenPointSpacingPixels
                : RefinePointSpacingPixels;

            const float projectedSpacing =
                calculateProjectedSpacing(node);

            if (projectedSpacing > threshold)
            {
                refinementQueue.push({
                    tileStateIndex,
                    nodeIndex,
                    projectedSpacing
                    });
            }
        };

    // Begin with the coarsest valid representation:
    // one root node per resident tile.
    for (std::size_t tileStateIndex = 0;
        tileStateIndex
        < tileResidencyStates_.size();
        ++tileStateIndex)
    {
        TileResidencyState& state =
            tileResidencyStates_[
                tileStateIndex];

        if (!state.IsResident()
            || state.hierarchy.Empty())
        {
            continue;
        }

        const std::size_t nodeCount =
            state.hierarchy.nodes.size();

        if (state.refinedNodeFlags.size()
            != nodeCount)
        {
            state.refinedNodeFlags.assign(
                nodeCount,
                std::uint8_t{ 0 });
        }

        state.selectedNodeFlags.assign(
            nodeCount,
            std::uint8_t{ 0 });

        state.nextRefinedNodeFlags.assign(
            nodeCount,
            std::uint8_t{ 0 });

        state.selectedDrawRanges.clear();

        const PointHierarchyNodeIndex rootIndex =
            state.hierarchy.rootIndex;

        const PointHierarchyNode& root =
            state.hierarchy.nodes[rootIndex];

        if (!BoundsIntersectFrustum(
            worldFrustum,
            root.boundsMinimum,
            root.boundsMaximum))
        {
            ++frustumCulledNodeCount_;
            continue;
        }

        state.selectedNodeFlags[rootIndex] = 1;

        const std::uint64_t rootPointCount =
            state.hierarchy.nodes[
                rootIndex].
            representativePoints.pointCount;

        if (rootPointCount
            > std::numeric_limits<std::uint64_t>::max()
            - selectedPointCount)
        {
            throw std::overflow_error(
                "Selected root point count overflow.");
        }

        selectedPointCount +=
            rootPointCount;

        queueRefinement(
            tileStateIndex,
            rootIndex);
    }

    // Spend the point budget on the nodes with the largest
    // visible error first.
    while (!refinementQueue.empty())
    {
        const PointRefinementCandidate candidate =
            refinementQueue.top();

        refinementQueue.pop();

        TileResidencyState& state =
            tileResidencyStates_[
                candidate.tileStateIndex];

        if (state.selectedNodeFlags[
            candidate.nodeIndex] == 0)
        {
            continue;
        }

        const PointHierarchyNode& parent =
            state.hierarchy.nodes[
                candidate.nodeIndex];

        std::array<
            PointHierarchyNodeIndex,
            8>
            visibleChildren{};

        std::uint32_t visibleChildCount = 0;
        std::uint64_t childPointCount = 0;

        for (const PointHierarchyNodeIndex childIndex :
        parent.children)
        {
            if (childIndex
                == InvalidPointHierarchyNode)
            {
                continue;
            }

            const PointHierarchyNode& child =
                state.hierarchy.nodes[childIndex];

            if (!BoundsIntersectFrustum(
                worldFrustum,
                child.boundsMinimum,
                child.boundsMaximum))
            {
                ++frustumCulledNodeCount_;
                continue;
            }

            visibleChildren[
                visibleChildCount++] =
                childIndex;

                const std::uint64_t childPoints =
                    child.representativePoints.pointCount;

                if (childPoints
                > std::numeric_limits<std::uint64_t>::max()
                    - childPointCount)
                {
                    throw std::overflow_error(
                        "Child point count overflow.");
                }

                childPointCount += childPoints;
        }

        if (visibleChildCount == 0)
        {
            // Keep the intersecting parent as a conservative
            // representation near the frustum boundary.
            continue;
        }

        const std::uint64_t parentPointCount =
            parent.representativePoints.pointCount;

        if (selectedPointCount
            < parentPointCount)
        {
            throw std::logic_error(
                "Selected point accounting is invalid.");
        }

        const std::uint64_t countWithoutParent =
            selectedPointCount
            - parentPointCount;

        const bool childrenFitBudget =
            childPointCount <= MaximumDrawnPoints
            && countWithoutParent
            <= MaximumDrawnPoints
            - childPointCount;

        if (!childrenFitBudget)
        {
            continue;
        }

        selectedPointCount =
            countWithoutParent
            + childPointCount;

        state.selectedNodeFlags[
            candidate.nodeIndex] = 0;

        state.nextRefinedNodeFlags[
            candidate.nodeIndex] = 1;

        for (std::uint32_t childOffset = 0;
            childOffset < visibleChildCount;
            ++childOffset)
        {
            const PointHierarchyNodeIndex childIndex =
                visibleChildren[childOffset];

            state.selectedNodeFlags[
                childIndex] = 1;

            queueRefinement(
                candidate.tileStateIndex,
                childIndex);
        }
    }

    // Convert the selected node flags into GPU draw ranges.
    for (TileResidencyState& state :
        tileResidencyStates_)
    {
        if (!state.IsResident()
            || state.hierarchy.Empty())
        {
            continue;
        }

        for (std::size_t nodeIndex = 0;
            nodeIndex < state.hierarchy.nodes.size();
            ++nodeIndex)
        {
            if (state.selectedNodeFlags[
                nodeIndex] == 0)
            {
                continue;
            }

            const PointRange& range =
                state.hierarchy.nodes[
                    nodeIndex].
                representativePoints;

            state.selectedDrawRanges.push_back({
                range.firstPoint,
                range.pointCount
                });

            ++selectedDrawRangeCount_;

        }

        const bool updated =
            graphics_.SetPointCloudTileDrawRanges(
                *state.gpuTileId,
                state.selectedDrawRanges);

        if (!updated)
        {
            throw std::logic_error(
                "Selected hierarchy tile does not "
                "exist in the renderer.");
        }

        state.refinedNodeFlags.swap(
            state.nextRefinedNodeFlags);
    }
}

void Application::UpdateFrameStatistics(
    double frameSeconds)
{
    frameTimeAccumulator_ +=
        frameSeconds;

    ++measuredFrameCount_;

    cpuRenderTimeAccumulator_ +=
        graphics_.GetCpuRenderMilliseconds();

    const double latestGpuMilliseconds =
        graphics_.GetGpuFrameMilliseconds();

    if (latestGpuMilliseconds > 0.0)
    {
        gpuRenderTimeAccumulator_ +=
            latestGpuMilliseconds;

        ++gpuMeasuredFrameCount_;
    }

    constexpr double updateInterval =
        0.5;

    if (frameTimeAccumulator_
        < updateInterval)
    {
        return;
    }

    const double averageFrameSeconds =
        frameTimeAccumulator_
        / static_cast<double>(
            measuredFrameCount_);

    const double averageCpuMilliseconds =
        cpuRenderTimeAccumulator_
        / static_cast<double>(
            measuredFrameCount_);

    const double averageGpuMilliseconds =
        gpuMeasuredFrameCount_ > 0
        ? gpuRenderTimeAccumulator_
        / static_cast<double>(
            gpuMeasuredFrameCount_)
        : 0.0;

    const double averageFrameMilliseconds =
        averageFrameSeconds * 1000.0;

    const double framesPerSecond =
        averageFrameSeconds > 0.0
        ? 1.0 / averageFrameSeconds
        : 0.0;

    const wchar_t* colorModeName =
        L"Classification";

    if (graphics_.GetPointColorMode()
        == PointColorMode::Elevation)
    {
        colorModeName =
            L"Elevation";
    }

    const std::uint64_t loadedPointCount =
        pointCloudDataset_
        ? pointCloudDataset_->
        totalPointCount
        : 0;

    const std::uint64_t gpuPointBytes =
        graphics_.GetPointBufferSizeBytes();

    constexpr double bytesPerMebibyte =
        1024.0 * 1024.0;

    const double gpuPointMebibytes =
        static_cast<double>(gpuPointBytes)
        / bytesPerMebibyte;

    constexpr double pointsPerMillion =
        1'000'000.0;

    const double availablePointMillions =
        static_cast<double>(loadedPointCount)
        / pointsPerMillion;

    const double residentPointMillions =
        static_cast<double>(
            graphics_.GetPointCount())
        / pointsPerMillion;

    std::size_t residentTileCount = 0;

    for (const TileResidencyState& state :
        tileResidencyStates_)
    {
        if (state.IsResident())
        {
            ++residentTileCount;
        }
    }

    const double pointBudgetMebibytes =
        static_cast<double>(
            PointMemoryBudgetBytes)
        / bytesPerMebibyte;

    const wchar_t* detailModeName =
        fullDetailMode_
        ? L"Full"
        : L"LOD";

    const double drawBudgetMillions =
        static_cast<double>(
            MaximumDrawnPoints)
        / pointsPerMillion;

    std::wostringstream title;

    if (!benchmarkStatusText_.empty())
    {
        title
            << benchmarkStatusText_
            << L" | ";
    }

    title
        << WindowTitle
        << L" | "
        << clientWidth_
        << L"x"
        << clientHeight_
        << L" | "
        << colorModeName
        << L" | Tiles "
        << residentTileCount
        << L"/"
        << tileResidencyStates_.size()
        << L" wanted "
        << wantedTileCount_
        << std::fixed
        << std::setprecision(2)
        << L" | "
        << detailModeName
        << L" | Drawn "
        << residentPointMillions
        << L"/"
        << availablePointMillions
        << L"M"
        << L" | VRAM "
        << std::setprecision(1)
        << gpuPointMebibytes
        << L"/"
        << pointBudgetMebibytes
        << L" MiB"
        << L" | "
        << framesPerSecond
        << L" FPS"
        << std::setprecision(2)
        << L" | CPU "
        << averageCpuMilliseconds
        << L" ms"
        << L" | GPU "
        << averageGpuMilliseconds
        << L" ms"
        << L" | Frame "
        << averageFrameMilliseconds
        << L" ms";

    if (!fullDetailMode_)
    {
        title
            << L" cap "
            << drawBudgetMillions
            << L"M"
            << L" | Ranges "
            << selectedDrawRangeCount_
            << L" | Culled "
            << frustumCulledNodeCount_;
    }
    else
    {
        title
            << L" | Ranges "
            << selectedDrawRangeCount_;
    }

    const std::size_t retiringTileCount =
        graphics_.GetRetiredPointCloudTileCount();

    if (retiringTileCount > 0)
    {
        title
            << L" | Retiring "
            << retiringTileCount;
    }

    SetWindowTextW(
        window_,
        title.str().c_str());

    frameTimeAccumulator_ = 0.0;
    cpuRenderTimeAccumulator_ = 0.0;
    gpuRenderTimeAccumulator_ = 0.0;

    measuredFrameCount_ = 0;
    gpuMeasuredFrameCount_ = 0;
}

void Application::UpdateCamera(
    float deltaSeconds)
{
    const bool benchmarkKeyIsDown =
        (GetAsyncKeyState('B') & 0x8000)
        != 0;

    if (benchmarkKeyIsDown
        && !benchmarkKeyWasDown_)
    {
        StartBenchmark();
    }

    benchmarkKeyWasDown_ =
        benchmarkKeyIsDown;

    const bool detailModeKeyIsDown =
        (GetAsyncKeyState('L') & 0x8000)
        != 0;

    if (benchmarkPhase_ == BenchmarkPhase::Idle
        && detailModeKeyIsDown
        && !detailModeKeyWasDown_)
    {
        fullDetailMode_ =
            !fullDetailMode_;
    }

    detailModeKeyWasDown_ =
        detailModeKeyIsDown;

    if (benchmarkPhase_
        != BenchmarkPhase::Idle)
    {
        return;
    }

    if (GetAsyncKeyState('1') & 0x8000)
    {
        graphics_.SetPointColorMode(
            PointColorMode::Classification);
    }

    if (GetAsyncKeyState('2') & 0x8000)
    {
        graphics_.SetPointColorMode(
            PointColorMode::Elevation);
    }

    const bool resetViewKeyIsDown =
        (GetAsyncKeyState('R') & 0x8000)
        != 0;

    if (resetViewKeyIsDown
        && !resetViewKeyWasDown_)
    {
        FramePointCloud();
    }

    resetViewKeyWasDown_ =
        resetViewKeyIsDown;

    float speed =
        cameraMovementSpeed_;

    const bool movingFast =
        (GetAsyncKeyState(VK_SHIFT) & 0x8000)
        != 0;

    const bool movingPrecisely =
        (GetAsyncKeyState(VK_CONTROL) & 0x8000)
        != 0;

    if (movingFast)
    {
        speed *= 4.0f;
    }

    if (movingPrecisely)
    {
        speed *= 0.2f;
    }

    const float distance =
        speed * deltaSeconds;

    float right = 0.0f;
    float up = 0.0f;
    float forward = 0.0f;

    if (GetAsyncKeyState('W') & 0x8000)
        forward += distance;

    if (GetAsyncKeyState('S') & 0x8000)
        forward -= distance;

    if (GetAsyncKeyState('D') & 0x8000)
        right += distance;

    if (GetAsyncKeyState('A') & 0x8000)
        right -= distance;

    if (GetAsyncKeyState('E') & 0x8000)
        up += distance;

    if (GetAsyncKeyState('Q') & 0x8000)
        up -= distance;

    camera_.MoveLocal(
        right,
        up,
        forward);

    constexpr float rotationSpeed =
        1.5f;

    float yawDelta = 0.0f;
    float pitchDelta = 0.0f;

    if (GetAsyncKeyState(VK_LEFT) & 0x8000)
        yawDelta -= rotationSpeed * deltaSeconds;

    if (GetAsyncKeyState(VK_RIGHT) & 0x8000)
        yawDelta += rotationSpeed * deltaSeconds;

    if (GetAsyncKeyState(VK_UP) & 0x8000)
        pitchDelta += rotationSpeed * deltaSeconds;

    if (GetAsyncKeyState(VK_DOWN) & 0x8000)
        pitchDelta -= rotationSpeed * deltaSeconds;

    camera_.Rotate(
        yawDelta,
        pitchDelta);
}

void Application::UpdateBenchmark(
    double frameSeconds)
{
    constexpr std::uint32_t warmupFrames =
        120;

    constexpr std::uint32_t measuredFrames =
        300;

    if (benchmarkPhase_
        == BenchmarkPhase::Idle)
    {
        return;
    }

    if (benchmarkPhase_
        == BenchmarkPhase::WarmingUp)
    {
        ++benchmarkFrameCount_;

        if (benchmarkFrameCount_
            >= warmupFrames)
        {
            benchmarkPhase_ =
                BenchmarkPhase::Measuring;

            benchmarkFrameCount_ = 0;

            benchmarkFrameTimeTotal_ = 0.0;
            benchmarkCpuTimeTotal_ = 0.0;
            benchmarkGpuTimeTotal_ = 0.0;

            benchmarkStatusText_ =
                L"Benchmark: measuring";
        }

        return;
    }

    benchmarkFrameTimeTotal_ +=
        frameSeconds * 1000.0;

    benchmarkCpuTimeTotal_ +=
        graphics_.GetCpuRenderMilliseconds();

    benchmarkGpuTimeTotal_ +=
        graphics_.GetGpuFrameMilliseconds();

    ++benchmarkFrameCount_;

    if (benchmarkFrameCount_
        < measuredFrames)
    {
        return;
    }

    const double averageFrameMilliseconds =
        benchmarkFrameTimeTotal_
        / measuredFrames;

    const double averageCpuMilliseconds =
        benchmarkCpuTimeTotal_
        / measuredFrames;

    const double averageGpuMilliseconds =
        benchmarkGpuTimeTotal_
        / measuredFrames;

    const double averageFramesPerSecond =
        averageFrameMilliseconds > 0.0
        ? 1000.0 / averageFrameMilliseconds
        : 0.0;

    std::wostringstream summary;

    summary
        << std::fixed
        << std::setprecision(2)
        << L"Baseline: Frame "
        << averageFrameMilliseconds
        << L" ms, CPU "
        << averageCpuMilliseconds
        << L" ms, GPU "
        << averageGpuMilliseconds
        << L" ms, "
        << std::setprecision(1)
        << averageFramesPerSecond
        << L" FPS";

    benchmarkStatusText_ =
        summary.str();

    std::wostringstream report;

    report
        << L"\n========== Baseline Result ==========\n"
        << L"Resolution: "
        << clientWidth_
        << L"x"
        << clientHeight_
        << L"\nLoaded points: "
        << pointCloudDataset_->totalPointCount
        << L"\nAvailable tiles: "
        << tileResidencyStates_.size()
        << L"\nResident tiles: "
        << graphics_.GetPointCloudTileCount()
        << L"\nPoint-memory budget: "
        << static_cast<double>(
            PointMemoryBudgetBytes)
        / (1024.0 * 1024.0)
        << L" MiB"
        << L"\nDrawn points: "
        << graphics_.GetPointCount()
        << L"\nGPU point memory: "
        << std::fixed
        << std::setprecision(2)
        << static_cast<double>(
            graphics_.GetPointBufferSizeBytes())
        / (1024.0 * 1024.0)
        << L" MiB"
        << L"\nAverage frame: "
        << averageFrameMilliseconds
        << L" ms"
        << L"\nAverage CPU: "
        << averageCpuMilliseconds
        << L" ms"
        << L"\nAverage GPU: "
        << averageGpuMilliseconds
        << L" ms"
        << L"\nAverage FPS: "
        << std::setprecision(1)
        << averageFramesPerSecond
        << L"\n=====================================\n";

    OutputDebugStringW(
        report.str().c_str());

    MessageBoxW(
        window_,
        report.str().c_str(),
        L"Point Cloud Baseline Result",
        MB_OK | MB_ICONINFORMATION);

    benchmarkPhase_ =
        BenchmarkPhase::Idle;
}

void Application::StartBenchmark()
{
    if (!pointCloudDataset_)
    {
        return;
    }

    FramePointCloud();

    benchmarkPhase_ =
        BenchmarkPhase::WarmingUp;

    benchmarkFrameCount_ = 0;

    benchmarkFrameTimeTotal_ = 0.0;
    benchmarkCpuTimeTotal_ = 0.0;
    benchmarkGpuTimeTotal_ = 0.0;

    benchmarkStatusText_ =
        L"Benchmark: warming up";
}

void Application::RegisterWindowClass()
{
    WNDCLASSEXW windowClass{};

    windowClass.cbSize = sizeof(windowClass);
    windowClass.style =
        CS_HREDRAW | CS_VREDRAW;

    windowClass.lpfnWndProc =
        WindowProcedure;

    windowClass.hInstance = instance_;

    windowClass.hCursor =
        LoadCursorW(nullptr, IDC_ARROW);

    windowClass.lpszClassName =
        WindowClassName;

    if (RegisterClassExW(&windowClass) == 0)
    {
        ThrowIfFailed(
            HRESULT_FROM_WIN32(GetLastError()),
            "Register window class");
    }
}

void Application::CreateApplicationWindow()
{
    RECT windowBounds{
        0,
        0,
        static_cast<LONG>(clientWidth_),
        static_cast<LONG>(clientHeight_)
    };

    if (!AdjustWindowRect(
        &windowBounds,
        WS_OVERLAPPEDWINDOW,
        FALSE))
    {
        ThrowIfFailed(
            HRESULT_FROM_WIN32(GetLastError()),
            "Adjust window rectangle");
    }

    window_ = CreateWindowExW(
        0,
        WindowClassName,
        WindowTitle,
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        windowBounds.right - windowBounds.left,
        windowBounds.bottom - windowBounds.top,
        nullptr,
        nullptr,
        instance_,
        this);

    if (window_ == nullptr)
    {
        ThrowIfFailed(
            HRESULT_FROM_WIN32(GetLastError()),
            "Create application window");
    }
}

LRESULT CALLBACK Application::WindowProcedure(
    HWND window,
    UINT message,
    WPARAM wParam,
    LPARAM lParam)
{
    Application* application = nullptr;

    if (message == WM_NCCREATE)
    {
        const auto* createInformation =
            reinterpret_cast<CREATESTRUCTW*>(
                lParam);

        application =
            static_cast<Application*>(
                createInformation->
                lpCreateParams);

        SetWindowLongPtrW(
            window,
            GWLP_USERDATA,
            reinterpret_cast<LONG_PTR>(
                application));
    }
    else
    {
        application =
            reinterpret_cast<Application*>(
                GetWindowLongPtrW(
                    window,
                    GWLP_USERDATA));
    }

    if (application == nullptr)
    {
        return DefWindowProcW(
            window,
            message,
            wParam,
            lParam);
    }

    try
    {
        return application->HandleMessage(
            window,
            message,
            wParam,
            lParam);
    }
    catch (const std::exception& error)
    {
        const std::string errorMessage =
            std::string(
                "Application message handling failed:\n\n")
            + error.what();

        MessageBoxA(
            window,
            errorMessage.c_str(),
            "Application Error",
            MB_OK | MB_ICONERROR);

        PostQuitMessage(EXIT_FAILURE);
        return 0;
    }
}

LRESULT Application::HandleMessage(
    HWND window,
    UINT message,
    WPARAM wParam,
    LPARAM lParam)
{
    switch (message)
    {
    case WM_SIZE:
    {
        clientWidth_ =
            static_cast<std::uint32_t>(
                LOWORD(lParam));

        clientHeight_ =
            static_cast<std::uint32_t>(
                HIWORD(lParam));

        minimized_ =
            (wParam == SIZE_MINIMIZED);

        if (!minimized_
            && clientWidth_ > 0
            && clientHeight_ > 0)
        {
            camera_.SetAspectRatio(
                static_cast<float>(clientWidth_)
                / static_cast<float>(clientHeight_));

            graphics_.Resize(
                clientWidth_,
                clientHeight_);
        }

        return 0;
    }
    case WM_RBUTTONDOWN:
        rotatingCamera_ = true;

        previousMousePosition_.x =
            GET_X_LPARAM(lParam);

        previousMousePosition_.y =
            GET_Y_LPARAM(lParam);

        SetCapture(window);
        return 0;

    case WM_RBUTTONUP:
        rotatingCamera_ = false;

        if (GetCapture() == window)
        {
            ReleaseCapture();
        }

        return 0;

    case WM_MOUSEMOVE:
        if (rotatingCamera_)
        {
            POINT currentPosition{
                GET_X_LPARAM(lParam),
                GET_Y_LPARAM(lParam)
            };

            const LONG deltaX =
                currentPosition.x
                - previousMousePosition_.x;

            const LONG deltaY =
                currentPosition.y
                - previousMousePosition_.y;

            constexpr float sensitivity =
                0.0025f;

            camera_.Rotate(
                static_cast<float>(deltaX)
                * sensitivity,
                -static_cast<float>(deltaY)
                * sensitivity);

            previousMousePosition_ =
                currentPosition;
        }

        return 0;

    case WM_KILLFOCUS:
        rotatingCamera_ = false;

        if (GetCapture() == window)
        {
            ReleaseCapture();
        }

        return 0;
    case WM_ERASEBKGND:
        return 1;

    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;

    default:
        return DefWindowProcW(
            window,
            message,
            wParam,
            lParam);
    }
}
