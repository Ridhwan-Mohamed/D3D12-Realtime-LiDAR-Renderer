#include "Application.h"

#include "Win32Helpers.h"

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
#include <stop_token>
#include <fstream>

namespace
{
    constexpr wchar_t WindowClassName[] =
        L"PointCloudRendererWindowClass";

    constexpr wchar_t WindowTitle[] =
        L"D3D12 Point Cloud Renderer";

    double CalculatePercentile(
        const std::vector<double>& sortedSamples,
        double percentile)
    {
        if (sortedSamples.empty())
        {
            return 0.0;
        }

        const double samplePosition =
            percentile
            * static_cast<double>(
                sortedSamples.size() - 1);

        const std::size_t lowerIndex =
            static_cast<std::size_t>(
                std::floor(samplePosition));

        const std::size_t upperIndex =
            std::min(
                lowerIndex + 1,
                sortedSamples.size() - 1);

        const double interpolation =
            samplePosition
            - static_cast<double>(lowerIndex);

        const double lowerValue =
            sortedSamples[lowerIndex];

        const double upperValue =
            sortedSamples[upperIndex];

        return lowerValue
            + (upperValue - lowerValue)
            * interpolation;
    }

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
        clientHeight_,
        static_cast<std::uint32_t>(
            MaximumDrawnPoints));

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

            constexpr std::uint64_t
                MinimumGpuBytesPerSourcePoint =
                sizeof(GpuPoint)
                + sizeof(ClodPoint);

            if (metadata.pointCount
            > std::numeric_limits<std::uint64_t>::max()
                / MinimumGpuBytesPerSourcePoint)
            {
                throw std::overflow_error(
                    "Tile GPU memory estimate overflow.");
            }

            TileResidencyState state{};

            state.metadataIndex = index;

            // This is a lower-bound estimate:
            //
            // - one original discrete point
            // - one original CLOD point
            //
            // Internal discrete representative copies are not known
            // until the CPU hierarchy has been constructed.
            state.requiredGpuBytes =
                metadata.pointCount
                * MinimumGpuBytesPerSourcePoint;

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

        tileLoadWorker_.Start();
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

            const double frameSeconds =
                std::chrono::duration<double>(
                    currentTime - previousTime).count();

            const float deltaSeconds =
                std::min(
                    static_cast<float>(frameSeconds),
                    0.1f);

            previousTime = currentTime;

            UpdateCamera(deltaSeconds);

            UpdateTilePriorities();
            UpdateTileResidency(deltaSeconds);

            // Keep the discrete fallback current while CLOD is
            // preparing, but stop paying for CPU hierarchy
            // traversal once a completed CLOD result is active.
            if (requestedPointRenderMode_
                    == PointRenderMode::Discrete
                || activePointRenderMode_
                    == PointRenderMode::Discrete)
            {
                UpdatePointDetailSelection();
            }

            graphics_.Render(
                camera_,
                requestedPointRenderMode_
                == PointRenderMode::Continuous);

            activePointRenderMode_ =
                graphics_.WasClodDrawnLastFrame()
                ? PointRenderMode::Continuous
                : PointRenderMode::Discrete;

            UpdateBenchmark(frameSeconds);

            UpdateFrameStatistics(
                frameSeconds);
        }
        else
        {
            WaitMessage();
        }
    }

    tileLoadWorker_.Stop();

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

bool Application::CanTransitionTileState(
    TileLifecycleState currentState,
    TileLifecycleState nextState) noexcept
{
    switch (currentState)
    {
    case TileLifecycleState::Unloaded:
        return
            nextState == TileLifecycleState::Queued
            || nextState == TileLifecycleState::Loading;

    case TileLifecycleState::Queued:
        return
            nextState == TileLifecycleState::Loading
            || nextState == TileLifecycleState::Unloaded;

    case TileLifecycleState::Loading:
        return
            nextState == TileLifecycleState::ReadyForUpload
            || nextState == TileLifecycleState::Unloaded;

    case TileLifecycleState::ReadyForUpload:
        return
            nextState == TileLifecycleState::Uploading
            || nextState == TileLifecycleState::Unloaded;

    case TileLifecycleState::Uploading:
        return
            nextState == TileLifecycleState::Resident
            || nextState == TileLifecycleState::Retiring;

    case TileLifecycleState::Resident:
        return
            nextState == TileLifecycleState::Retiring;

    case TileLifecycleState::Retiring:
        return
            nextState == TileLifecycleState::Unloaded;
    }

    return false;
}

void Application::TransitionTileState(
    TileResidencyState& state,
    TileLifecycleState nextState)
{
    if (!CanTransitionTileState(
        state.lifecycleState,
        nextState))
    {
        throw std::logic_error(
            std::string(
                "Invalid tile lifecycle transition: ")
            + GetTileLifecycleStateName(
                state.lifecycleState)
            + " -> "
            + GetTileLifecycleStateName(
                nextState));
    }

    state.lifecycleState =
        nextState;

    ValidateTileLifecycleState(state);
}

const char*
Application::GetTileLifecycleStateName(
    TileLifecycleState state) noexcept
{
    switch (state)
    {
    case TileLifecycleState::Unloaded:
        return "Unloaded";

    case TileLifecycleState::Queued:
        return "Queued";

    case TileLifecycleState::Loading:
        return "Loading";

    case TileLifecycleState::ReadyForUpload:
        return "ReadyForUpload";

    case TileLifecycleState::Uploading:
        return "Uploading";

    case TileLifecycleState::Resident:
        return "Resident";

    case TileLifecycleState::Retiring:
        return "Retiring";
    }

    return "Unknown";
}

void Application::ValidateTileLifecycleState(
    const TileResidencyState& state)
{
    const bool hasGpuTile =
        state.gpuTileId.has_value();

    const bool hasHierarchy =
        !state.hierarchy.Empty();

    switch (state.lifecycleState)
    {
    case TileLifecycleState::Unloaded:
    case TileLifecycleState::Queued:
    case TileLifecycleState::Loading:
    case TileLifecycleState::ReadyForUpload:
        if (hasGpuTile || hasHierarchy)
        {
            throw std::logic_error(
                "Nonresident tile unexpectedly owns "
                "resident rendering resources.");
        }

        break;

    case TileLifecycleState::Uploading:
        if (!hasGpuTile || !hasHierarchy)
        {
            throw std::logic_error(
                "Uploading tile is missing its pending "
                "GPU tile or hierarchy.");
        }
        break;
    case TileLifecycleState::Resident:
    case TileLifecycleState::Retiring:
        if (!hasGpuTile || !hasHierarchy)
        {
            throw std::logic_error(
                "Resident tile is missing its GPU tile "
                "or hierarchy.");
        }

        break;
    }
}

std::uint64_t Application::BeginTileRequest(
    TileResidencyState& state)
{
    if (state.lifecycleState
        != TileLifecycleState::Unloaded)
    {
        throw std::logic_error(
            "Tile request began while tile was not unloaded.");
    }

    if (state.requestGeneration
        == std::numeric_limits<std::uint64_t>::max())
    {
        throw std::overflow_error(
            "Tile request generation overflow.");
    }

    state.loadStopSource =
        std::stop_source{};

    ++state.requestGeneration;

    return state.requestGeneration;
}

void Application::InvalidateTileRequest(
    TileResidencyState& state)
{
    if (state.lifecycleState
        != TileLifecycleState::Queued
        && state.lifecycleState
        != TileLifecycleState::Loading)
    {
        throw std::logic_error(
            "Only queued or loading tile requests "
            "can be invalidated.");
    }

    if (state.requestGeneration
        == std::numeric_limits<std::uint64_t>::max())
    {
        throw std::overflow_error(
            "Tile request generation overflow.");
    }

    ++state.requestGeneration;

    state.loadStopSource.request_stop();

    TransitionTileState(
        state,
        TileLifecycleState::Unloaded);
}

bool Application::IsCurrentTileRequest(
    const TileResidencyState& state,
    std::uint64_t requestGeneration) noexcept
{
    return
        state.requestGeneration
        == requestGeneration;
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
            constexpr float activeBiasSquared =
                0.64f;

            const TileResidencyState& left =
                tileResidencyStates_[leftIndex];

            const TileResidencyState& right =
                tileResidencyStates_[rightIndex];

            const bool leftIsActive =
                left.lifecycleState
                != TileLifecycleState::Unloaded;

            const bool rightIsActive =
                right.lifecycleState
                != TileLifecycleState::Unloaded;

            const float leftPriority =
                left.cameraDistanceSquared
                * (leftIsActive
                    ? activeBiasSquared
                    : 1.0f);

            const float rightPriority =
                right.cameraDistanceSquared
                * (rightIsActive
                    ? activeBiasSquared
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

void Application::UpdateTileStabilityTimers(
    float deltaSeconds)
{
    for (TileResidencyState& state :
        tileResidencyStates_)
    {
        if (state.wanted)
        {
            state.unwantedSeconds = 0.0f;
            continue;
        }

        state.unwantedSeconds =
            std::min(
                state.unwantedSeconds
                + deltaSeconds,
                TileUnwantedGraceSeconds);
    }
}

void Application::ProcessTileLoadStartedEvents()
{
    while (true)
    {
        std::optional<TileLoadStarted> started =
            tileLoadWorker_.TryTakeStarted();

        if (!started)
        {
            return;
        }

        if (started->tileStateIndex
            >= tileResidencyStates_.size())
        {
            throw std::logic_error(
                "Tile worker returned an invalid "
                "tile-state index.");
        }

        TileResidencyState& state =
            tileResidencyStates_[
                started->tileStateIndex];

        if (!IsCurrentTileRequest(
            state,
            started->requestGeneration))
        {
            // A future cancellation may make an event stale.
            continue;
        }

        if (state.lifecycleState
            != TileLifecycleState::Queued)
        {
            throw std::logic_error(
                "Current tile request started while "
                "the tile was not queued.");
        }

        TransitionTileState(
            state,
            TileLifecycleState::Loading);
    }
}

void Application::CancelUnwantedCpuTileRequests()
{
    for (std::size_t stateIndex = 0;
        stateIndex < tileResidencyStates_.size();
        ++stateIndex)
    {
        TileResidencyState& state =
            tileResidencyStates_[stateIndex];

        if (state.wanted
            || state.unwantedSeconds
            < TileUnwantedGraceSeconds)
        {
            continue;
        }

        if (state.lifecycleState
            == TileLifecycleState::Queued)
        {
            const bool removedFromQueue =
                tileLoadWorker_.TryCancelPending(
                    stateIndex,
                    state.requestGeneration);

            if (removedFromQueue)
            {
                OutputDebugStringW(
                    L"Cancelled queued tile load.\n");
            }

            InvalidateTileRequest(state);
        }
        else if (state.lifecycleState
            == TileLifecycleState::Loading)
        {
            // LASzip loading is not cooperatively
            // interruptible yet. Incrementing the generation
            // ensures its eventual result is discarded.
            InvalidateTileRequest(state);

            OutputDebugStringW(
                L"Invalidated active tile load.\n");
        }
    }
}

void Application::CancelUnwantedPointUploads()
{
    for (TileResidencyState& state :
        tileResidencyStates_)
    {
        if (state.wanted
            || state.unwantedSeconds
            < TileUnwantedGraceSeconds
            || state.lifecycleState
            != TileLifecycleState::Uploading)
        {
            continue;
        }

        if (!state.gpuTileId)
        {
            throw std::logic_error(
                "Uploading tile has no GPU tile ID.");
        }

        const PointCloudTileId tileId =
            *state.gpuTileId;

        CancelClodWork(
            static_cast<std::size_t>(
                &state - tileResidencyStates_.data()),
            state);

        TransitionTileState(
            state,
            TileLifecycleState::Retiring);

        const bool abandoned =
            graphics_.AbandonPointCloudTileUpload(
                tileId);

        if (!abandoned)
        {
            throw std::logic_error(
                "Application tried to abandon an "
                "unknown point upload.");
        }

        state.gpuTileId.reset();

        state.hierarchy =
            PointCloudHierarchy{};

        state.selectedNodeFlags.clear();
        state.refinedNodeFlags.clear();
        state.nextRefinedNodeFlags.clear();
        state.selectedDrawRanges.clear();

        TransitionTileState(
            state,
            TileLifecycleState::Unloaded);

        OutputDebugStringW(
            L"Abandoned unwanted GPU tile upload.\n");
    }
}

void Application::ProcessOneCompletedTileLoad()
{
    if (graphics_.GetPendingPointCloudUploadCount()
        >= MaximumConcurrentPointUploads)
    {
        return;
    }

    std::optional<TileLoadResult> completed =
        tileLoadWorker_.TryTakeCompleted();

    if (!completed)
    {
        return;
    }

    if (completed->tileStateIndex
        >= tileResidencyStates_.size())
    {
        throw std::logic_error(
            "Tile worker completed an invalid "
            "tile-state index.");
    }

    TileResidencyState& state =
        tileResidencyStates_[
            completed->tileStateIndex];

    if (!IsCurrentTileRequest(
        state,
        completed->requestGeneration))
    {
        // Destroying the result releases its CPU hierarchy.
        return;
    }

    if (state.lifecycleState
        != TileLifecycleState::Loading)
    {
        throw std::logic_error(
            "Current tile request completed while "
            "the tile was not loading.");
    }

    if (!completed->Succeeded())
    {
        TransitionTileState(
            state,
            TileLifecycleState::Unloaded);

        std::rethrow_exception(
            completed->failure);
    }

    PointCloudHierarchy& hierarchy =
        completed->hierarchy;

    const PointCloudDatasetInfo& dataset =
        *pointCloudDataset_;

    const PointCloudMetadata& metadata =
        dataset.tileMetadata[
            state.metadataIndex];

    if (hierarchy.sourcePointCount
        != metadata.pointCount)
    {
        throw std::runtime_error(
            "Hierarchy source count does not match "
            "the tile metadata.");
    }

    if (hierarchy.points.size()
        > std::numeric_limits<std::uint64_t>::max()
        / sizeof(GpuPoint))
    {
        throw std::overflow_error(
            "Discrete hierarchy byte size overflow.");
    }

    const std::uint64_t discreteBytes =
        static_cast<std::uint64_t>(
            hierarchy.points.size())
        * sizeof(GpuPoint);

    const std::uint64_t expectedClodBytes =
        hierarchy.sourcePointCount
        * sizeof(ClodPoint);

    if (expectedClodBytes
        > std::numeric_limits<std::uint64_t>::max()
        - discreteBytes)
    {
        throw std::overflow_error(
            "Combined hierarchy byte size overflow.");
    }

    const std::uint64_t actualBytes =
        discreteBytes
        + expectedClodBytes;

    const std::uint64_t sourceDiscreteBytes =
        metadata.pointCount
        * sizeof(GpuPoint);

    if (discreteBytes < sourceDiscreteBytes)
    {
        throw std::runtime_error(
            "Discrete hierarchy contains fewer points "
            "than the source tile.");
    }

    state.requiredGpuBytes =
        actualBytes;

    if (!state.wanted)
    {
        TransitionTileState(
            state,
            TileLifecycleState::Unloaded);

        return;
    }

    const std::uint64_t allocatedBytes =
        graphics_.GetPointBufferSizeBytes();

    const bool hierarchyFitsBudget =
        actualBytes <= PointMemoryBudgetBytes
        && allocatedBytes
        <= PointMemoryBudgetBytes
        - actualBytes;

    if (!hierarchyFitsBudget)
    {
        OutputDebugStringW(
            L"Prepared hierarchy does not currently "
            L"fit the point-memory budget.\n");

        TransitionTileState(
            state,
            TileLifecycleState::Unloaded);

        return;
    }

    const std::wstring message =
        L"Uploading discrete tile: "
        + metadata.sourcePath.filename().wstring()
        + L"\n";

    OutputDebugStringW(
        message.c_str());

    const std::size_t hierarchyNodeCount =
        hierarchy.nodes.size();

    TransitionTileState(
        state,
        TileLifecycleState::ReadyForUpload);

    state.gpuTileId =
        graphics_.BeginPointCloudTileUpload(
            hierarchy.points,
            {},
            static_cast<std::uint32_t>(
                hierarchy.sourcePointCount));

    // The GPU upload has copied the discrete bytes into its
    // persistent upload resource. Keep the CPU array briefly
    // so ownership can move into the CLOD build job.
    state.hierarchy =
        std::move(hierarchy);

    state.clodLifecycleState =
        ClodLifecycleState::ReadyToBuild;

    TransitionTileState(
        state,
        TileLifecycleState::Uploading);

    OutputDebugStringW(
        (
            L"Hierarchy nodes: "
            + std::to_wstring(
                hierarchyNodeCount)
            + L"\n"
            ).c_str());
}

void Application::TryStartClodBuilds()
{
    for (std::size_t stateIndex = 0;
        stateIndex < tileResidencyStates_.size();
        ++stateIndex)
    {
        TileResidencyState& state =
            tileResidencyStates_[stateIndex];

        if (state.clodLifecycleState
            != ClodLifecycleState::ReadyToBuild)
        {
            continue;
        }

        ClodBuildRequest request{};

        request.tileStateIndex = stateIndex;
        request.requestGeneration =
            state.requestGeneration;
        request.discretePoints =
            std::move(state.hierarchy.points);
        request.sourcePointCount =
            state.hierarchy.sourcePointCount;
        request.boundsMinimum =
            state.localBoundsMinimum;
        request.boundsMaximum =
            state.localBoundsMaximum;
        request.cancellationSource =
            state.loadStopSource;

        const bool queued =
            tileLoadWorker_.TryEnqueueClodBuild(
                request);

        if (!queued)
        {
            // The reference-based enqueue leaves ownership in
            // request when the shared worker queue is full.
            state.hierarchy.points =
                std::move(request.discretePoints);
            return;
        }

        state.clodLifecycleState =
            ClodLifecycleState::Building;

        OutputDebugStringW(
            L"Queued asynchronous CLOD build.\n");
    }
}

void Application::ProcessCompletedClodBuilds()
{
    while (true)
    {
        std::optional<ClodBuildResult> completed =
            tileLoadWorker_.TryTakeCompletedClodBuild();

        if (!completed)
        {
            return;
        }

        if (completed->tileStateIndex
            >= tileResidencyStates_.size())
        {
            throw std::logic_error(
                "CLOD worker completed an invalid tile index.");
        }

        TileResidencyState& state =
            tileResidencyStates_[
                completed->tileStateIndex];

        if (!IsCurrentTileRequest(
            state,
            completed->requestGeneration))
        {
            continue;
        }

        if (state.clodLifecycleState
            != ClodLifecycleState::Building)
        {
            throw std::logic_error(
                "Current CLOD result arrived while its tile "
                "was not building CLOD.");
        }

        // The discrete bytes are already in the renderer's
        // upload resource. Their temporary CPU copy can go.
        completed->discretePoints.clear();

        if (!completed->Succeeded())
        {
            state.clodLifecycleState =
                ClodLifecycleState::NotRequested;

            // A failed optional CLOD build must not take down
            // the already-working discrete renderer.
            OutputDebugStringW(
                L"CLOD preparation failed; keeping the "
                L"discrete tile.\n");
            continue;
        }

        if (completed->sourcePointCount
            != state.hierarchy.sourcePointCount
            || completed->clodPoints.size()
                != completed->sourcePointCount)
        {
            throw std::logic_error(
                "Completed CLOD data does not match its "
                "discrete tile.");
        }

        std::uint64_t rangedPointCount = 0;

        for (const PointRange& range :
            completed->clodLevelRanges)
        {
            rangedPointCount += range.pointCount;
        }

        if (rangedPointCount
            != completed->sourcePointCount)
        {
            throw std::logic_error(
                "CLOD level ranges do not reconstruct "
                "the source point count.");
        }

        state.hierarchy.clodPoints =
            std::move(completed->clodPoints);
        state.hierarchy.clodLevelRanges =
            std::move(completed->clodLevelRanges);
        state.hierarchy.clodRootSpacing =
            completed->clodRootSpacing;
        state.clodLifecycleState =
            ClodLifecycleState::ReadyForUpload;

        OutputDebugStringW(
            L"CLOD CPU representation is ready.\n");
    }
}

void Application::TryStartClodUploads()
{
    if (graphics_.GetPendingPointCloudUploadCount()
        >= MaximumConcurrentPointUploads)
    {
        return;
    }

    for (TileResidencyState& state :
        tileResidencyStates_)
    {
        if (state.clodLifecycleState
                != ClodLifecycleState::ReadyForUpload
            || !state.IsResident()
            || !state.gpuTileId)
        {
            continue;
        }

        const bool started =
            graphics_.BeginPointCloudTileClodUpload(
                *state.gpuTileId,
                state.hierarchy.clodPoints,
                state.hierarchy.clodRootSpacing);

        if (!started)
        {
            throw std::logic_error(
                "Could not begin CLOD attachment upload.");
        }

        // BeginPointCloudTileClodUpload copied the bytes to a
        // persistent upload resource owned by the renderer.
        state.hierarchy.clodPoints.clear();
        state.clodLifecycleState =
            ClodLifecycleState::Uploading;

        OutputDebugStringW(
            L"Uploading CLOD attachment.\n");
        return;
    }
}

void Application::ProcessCompletedClodUploads()
{
    for (TileResidencyState& state :
        tileResidencyStates_)
    {
        if (state.clodLifecycleState
                != ClodLifecycleState::Uploading
            || !state.gpuTileId)
        {
            continue;
        }

        if (!graphics_.IsPointCloudTileClodUploadComplete(
            *state.gpuTileId))
        {
            continue;
        }

        if (!graphics_.FinalizePointCloudTileClodUpload(
            *state.gpuTileId))
        {
            throw std::logic_error(
                "Completed CLOD attachment could not be "
                "finalized.");
        }

        state.clodLifecycleState =
            ClodLifecycleState::Resident;

        OutputDebugStringW(
            L"CLOD attachment became resident.\n");
    }
}

void Application::CancelClodWork(
    std::size_t stateIndex,
    TileResidencyState& state)
{
    if (state.clodLifecycleState
        == ClodLifecycleState::NotRequested)
    {
        return;
    }

    state.loadStopSource.request_stop();

    if (state.clodLifecycleState
        == ClodLifecycleState::Building)
    {
        const bool removed =
            tileLoadWorker_.TryCancelPendingClodBuild(
                stateIndex,
                state.requestGeneration);

        (void)removed;
    }
    else if (state.clodLifecycleState
        == ClodLifecycleState::Uploading)
    {
        if (state.gpuTileId)
        {
            const bool abandoned =
                graphics_.AbandonPointCloudTileClodUpload(
                    *state.gpuTileId);

            (void)abandoned;
        }
    }

    state.hierarchy.points.clear();
    state.hierarchy.clodPoints.clear();
    state.hierarchy.clodLevelRanges.clear();
    state.hierarchy.clodRootSpacing = 0.0f;
    state.clodLifecycleState =
        ClodLifecycleState::NotRequested;

    if (state.requestGeneration
        == std::numeric_limits<std::uint64_t>::max())
    {
        throw std::overflow_error(
            "Tile request generation overflow.");
    }

    ++state.requestGeneration;
}

void Application::ProcessCompletedPointUploads()
{
    for (TileResidencyState& state :
        tileResidencyStates_)
    {
        if (state.lifecycleState
            != TileLifecycleState::Uploading)
        {
            continue;
        }

        if (!state.gpuTileId)
        {
            throw std::logic_error(
                "Uploading tile has no GPU tile ID.");
        }

        const PointCloudTileId tileId =
            *state.gpuTileId;

        if (!graphics_.
            IsPointCloudTileUploadComplete(
                tileId))
        {
            continue;
        }

        const bool finalized =
            graphics_.FinalizePointCloudTileUpload(
                tileId);

        if (!finalized)
        {
            throw std::logic_error(
                "Completed point upload could not "
                "be finalized.");
        }

        TransitionTileState(
            state,
            TileLifecycleState::Resident);

        const PointCloudMetadata& metadata =
            pointCloudDataset_->tileMetadata[
                state.metadataIndex];

        const std::wstring message =
            L"Point tile became resident: "
            + metadata.sourcePath.filename().wstring()
            + L"\n";

        OutputDebugStringW(
            message.c_str());
    }
}

bool Application::TryQueueNextTileLoad()
{
    std::size_t activeCpuRequestCount = 0;

    for (const TileResidencyState& state :
        tileResidencyStates_)
    {
        if (state.lifecycleState
            == TileLifecycleState::Queued
            || state.lifecycleState
            == TileLifecycleState::Loading
            || state.lifecycleState
            == TileLifecycleState::ReadyForUpload)
        {
            ++activeCpuRequestCount;
        }
    }

    if (activeCpuRequestCount
        >= MaximumConcurrentCpuTileRequests)
    {
        return false;
    }

    const PointCloudDatasetInfo& dataset =
        *pointCloudDataset_;

    for (const std::size_t stateIndex :
    tilePriorityOrder_)
    {
        TileResidencyState& state =
            tileResidencyStates_[
                stateIndex];

        if (!state.wanted
            || state.lifecycleState
            != TileLifecycleState::Unloaded)
        {
            continue;
        }

        const std::uint64_t allocatedBytes =
            graphics_.GetPointBufferSizeBytes();

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

        const std::uint64_t requestGeneration =
            BeginTileRequest(state);

        const PointCloudMetadata& metadata =
            dataset.tileMetadata[
                state.metadataIndex];

        TileLoadRequest request{};

        request.tileStateIndex =
            stateIndex;

        request.requestGeneration =
            requestGeneration;

        request.metadata =
            metadata;

        request.localOrigin =
            dataset.localOrigin;

        request.cancellationSource =
            state.loadStopSource;

        TransitionTileState(
            state,
            TileLifecycleState::Queued);

        const bool queued =
            tileLoadWorker_.TryEnqueue(
                std::move(request));

        if (!queued)
        {
            TransitionTileState(
                state,
                TileLifecycleState::Unloaded);

            return false;
        }

        return true;
    }

    return false;
}

void Application::UpdateTileResidency(
    float deltaSeconds)
{
    if (!pointCloudDataset_)
    {
        return;
    }

    // Process lightweight CPU and GPU completion
    // notifications every frame.
    ProcessTileLoadStartedEvents();
    ProcessCompletedClodBuilds();
    UpdateTileStabilityTimers(deltaSeconds);
    CancelUnwantedCpuTileRequests();
    CancelUnwantedPointUploads();
    ProcessCompletedPointUploads();
    ProcessCompletedClodUploads();

    residencyUpdateAccumulator_ +=
        static_cast<double>(deltaSeconds);

    const bool performResidencyUpdate =
        residencyUpdateAccumulator_
        >= ResidencyUpdateIntervalSeconds;

    if (performResidencyUpdate)
    {
        residencyUpdateAccumulator_ = 0.0;
    }

    // Check retirement every frame. A tile may become unpinned
    // immediately after the final chunk of a reduction is
    // submitted; checking only every 0.25 seconds could let the
    // next reduction capture the unwanted tile again first.
    for (TileResidencyState& state :
        tileResidencyStates_)
    {
        if (!state.IsResident()
            || state.wanted
            || state.unwantedSeconds
                < TileUnwantedGraceSeconds)
        {
            continue;
        }

        const PointCloudTileId tileId =
            *state.gpuTileId;

        if (graphics_.
            IsPointCloudTileReferencedByActiveClodReduction(
                tileId))
        {
            continue;
        }

        CancelClodWork(
            static_cast<std::size_t>(
                &state - tileResidencyStates_.data()),
            state);

        TransitionTileState(
            state,
            TileLifecycleState::Retiring);

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
        state.hierarchy =
            PointCloudHierarchy{};

        state.selectedNodeFlags.clear();
        state.refinedNodeFlags.clear();
        state.nextRefinedNodeFlags.clear();
        state.selectedDrawRanges.clear();

        TransitionTileState(
            state,
            TileLifecycleState::Unloaded);
    }

    // Accept CPU work every frame instead of waiting for
    // the 0.25-second residency interval.
    ProcessOneCompletedTileLoad();
    TryStartClodBuilds();
    TryStartClodUploads();

    if (performResidencyUpdate)
    {
        const bool queued =
            TryQueueNextTileLoad();

        (void)queued;
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
        ValidateTileLifecycleState(state);

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
        L"Source Color";

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

    const std::uint64_t displayedPointCount =
        activePointRenderMode_
        == PointRenderMode::Continuous
        ? graphics_.GetClodSelectedPointCount()
        : graphics_.GetPointCount();

    const double residentPointMillions =
        static_cast<double>(
            displayedPointCount)
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

    const wchar_t* pointRenderModeName =
        L"Discrete";

    if (requestedPointRenderMode_
        == PointRenderMode::Continuous)
    {
        pointRenderModeName =
            activePointRenderMode_
            == PointRenderMode::Continuous
            ? L"CLOD"
            : L"CLOD unavailable - Discrete active";
    }

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
        << L" | Render "
        << pointRenderModeName
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

    if (requestedPointRenderMode_
        == PointRenderMode::Continuous)
    {
        title
            << std::fixed
            << std::setprecision(3)
            << L" factor "
            << graphics_.GetClodFactor()
            << std::setprecision(0)
            << L" scan "
            << graphics_.
                GetClodReductionSourceBudgetPerFrame()
                / 1000
            << L"K";

        const std::uint64_t reductionTotal =
            graphics_.GetClodReductionTotalPointCount();

        if (reductionTotal != 0)
        {
            const double reductionPercent =
                100.0
                * static_cast<double>(
                    graphics_.
                    GetClodReductionProcessedPointCount())
                / static_cast<double>(reductionTotal);

            title
                << std::setprecision(0)
                << L" build "
                << reductionPercent
                << L"%";
        }

        const std::uint32_t overflowCount =
            graphics_.GetClodOverflowPointCount();

        if (overflowCount != 0)
        {
            title
                << L" overflow "
                << overflowCount;
        }
    }

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

    const bool pointRenderModeKeyIsDown =
        (GetAsyncKeyState('C') & 0x8000)
        != 0;

    if (benchmarkPhase_ == BenchmarkPhase::Idle
        && pointRenderModeKeyIsDown
        && !pointRenderModeKeyWasDown_)
    {
        if (requestedPointRenderMode_
            == PointRenderMode::Discrete)
        {
            requestedPointRenderMode_ =
                PointRenderMode::Continuous;
        }
        else
        {
            requestedPointRenderMode_ =
                PointRenderMode::Discrete;

            activePointRenderMode_ =
                PointRenderMode::Discrete;
        }
    }

    const bool denserClodKeyIsDown =
        (GetAsyncKeyState(VK_OEM_4) & 0x8000)
        != 0;

    const bool sparserClodKeyIsDown =
        (GetAsyncKeyState(VK_OEM_6) & 0x8000)
        != 0;

    if (benchmarkPhase_ == BenchmarkPhase::Idle)
    {
        constexpr float factorStep = 1.25f;
        constexpr float minimumFactor = 0.125f;
        constexpr float maximumFactor = 16.0f;

        if (denserClodKeyIsDown
            && !denserClodKeyWasDown_)
        {
            graphics_.SetClodFactor(
                std::max(
                    graphics_.GetClodFactor()
                    / factorStep,
                    minimumFactor));
        }

        if (sparserClodKeyIsDown
            && !sparserClodKeyWasDown_)
        {
            graphics_.SetClodFactor(
                std::min(
                    graphics_.GetClodFactor()
                    * factorStep,
                    maximumFactor));
        }
    }

    denserClodKeyWasDown_ =
        denserClodKeyIsDown;

    sparserClodKeyWasDown_ =
        sparserClodKeyIsDown;

    pointRenderModeKeyWasDown_ =
        pointRenderModeKeyIsDown;

    if (benchmarkPhase_
        != BenchmarkPhase::Idle)
    {
        return;
    }

    // Leave the rest of UpdateCamera exactly as it is.
    if (GetAsyncKeyState('1') & 0x8000)
    {
        graphics_.SetPointColorMode(
            PointColorMode::SourceColor);
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

            benchmarkDrawnPointTotal_ = 0;
            benchmarkDrawRangeTotal_ = 0;

            benchmarkFrameTimeSamples_.clear();

            benchmarkStatusText_ =
                L"Benchmark: measuring";
        }

        return;
    }

    const double frameMilliseconds =
        frameSeconds * 1000.0;

    const double cpuMilliseconds =
        graphics_.GetCpuRenderMilliseconds();

    const double gpuMilliseconds =
        graphics_.GetGpuFrameMilliseconds();

    benchmarkFrameTimeTotal_ +=
        frameMilliseconds;

    benchmarkCpuTimeTotal_ +=
        cpuMilliseconds;

    benchmarkGpuTimeTotal_ +=
        gpuMilliseconds;

    benchmarkFrameTimeSamples_.push_back(
        frameMilliseconds);

    const bool benchmarkingClod =
        benchmarkPointRenderMode_
        == PointRenderMode::Continuous;

    const std::uint64_t drawnPointCount =
        benchmarkingClod
        ? graphics_.GetClodSelectedPointCount()
        : graphics_.GetPointCount();

    benchmarkDrawnPointTotal_ +=
        drawnPointCount;

    // CLOD submits one indirect draw. The discrete path submits
    // one draw for every selected node range.
    benchmarkDrawRangeTotal_ +=
        benchmarkingClod
        ? (drawnPointCount == 0 ? 0u : 1u)
        : selectedDrawRangeCount_;

    benchmarkOverflowPointCount_ =
        std::max(
            benchmarkOverflowPointCount_,
            graphics_.GetClodOverflowPointCount());

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

    const double averageDrawnPoints =
        static_cast<double>(
            benchmarkDrawnPointTotal_)
        / measuredFrames;

    const double averageDrawRanges =
        static_cast<double>(
            benchmarkDrawRangeTotal_)
        / measuredFrames;

    const double averageFramesPerSecond =
        averageFrameMilliseconds > 0.0
        ? 1000.0 / averageFrameMilliseconds
        : 0.0;

    std::vector<double> sortedFrameTimes =
        benchmarkFrameTimeSamples_;

    std::sort(
        sortedFrameTimes.begin(),
        sortedFrameTimes.end());

    const double maximumFrameMilliseconds =
        sortedFrameTimes.empty()
        ? 0.0
        : sortedFrameTimes.back();

    const double frameP95Milliseconds =
        CalculatePercentile(
            sortedFrameTimes,
            0.95);

    const double frameP99Milliseconds =
        CalculatePercentile(
            sortedFrameTimes,
            0.99);

    const wchar_t* benchmarkPointRenderModeName =
        benchmarkPointRenderMode_
        == PointRenderMode::Continuous
        ? L"CLOD"
        : L"Discrete";

    const wchar_t* benchmarkDetailModeName =
        benchmarkFullDetailMode_
        ? L"Full"
        : L"LOD";

    std::wostringstream summary;

    summary
        << std::fixed
        << std::setprecision(2)
        << L"Benchmark ("
        << benchmarkPointRenderModeName
        << L"): average "
        << averageFrameMilliseconds
        << L" ms, p95 "
        << frameP95Milliseconds
        << L" ms, p99 "
        << frameP99Milliseconds
        << L" ms";

    benchmarkStatusText_ =
        summary.str();

    std::wostringstream report;

    report
        << L"\n========== Benchmark Result ==========\n"
        << L"Render mode: "
        << benchmarkPointRenderModeName
        << L"\nDetail mode: "
        << benchmarkDetailModeName
        << L"\nResolution: "
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
        << L"\nDraw-point budget: "
        << MaximumDrawnPoints
        << L"\nGPU point memory: "
        << std::fixed
        << std::setprecision(2)
        << static_cast<double>(
            graphics_.GetPointBufferSizeBytes())
        / (1024.0 * 1024.0)
        << L" MiB"
        << L"\nCLOD reduction buffers: "
        << static_cast<double>(
            graphics_.GetClodReductionBufferSizeBytes())
        / (1024.0 * 1024.0)
        << L" MiB"
        << L"\nCLOD factor: "
        << graphics_.GetClodFactor()
        << L"\nMaximum CLOD overflow: "
        << benchmarkOverflowPointCount_
        << L"\nAverage drawn points: "
        << std::setprecision(0)
        << averageDrawnPoints
        << L"\nAverage draw ranges: "
        << averageDrawRanges
        << std::setprecision(2)
        << L"\nAverage frame: "
        << averageFrameMilliseconds
        << L" ms"
        << L"\nMaximum frame: "
        << maximumFrameMilliseconds
        << L" ms"
        << L"\nFrame p95: "
        << frameP95Milliseconds
        << L" ms"
        << L"\nFrame p99: "
        << frameP99Milliseconds
        << L" ms"
        << L"\nAverage CPU render: "
        << averageCpuMilliseconds
        << L" ms"
        << L"\nAverage GPU render: "
        << averageGpuMilliseconds
        << L" ms"
        << L"\nAverage FPS: "
        << std::setprecision(1)
        << averageFramesPerSecond
        << L"\n======================================\n";

    OutputDebugStringW(
        report.str().c_str());

    // Append every run so discrete and CLOD measurements can be
    // captured separately and compared after closing the app.
    std::wofstream reportFile(
        std::filesystem::path(
            L"PointCloudRenderer_Benchmark.txt"),
        std::ios::app);

    if (reportFile)
    {
        reportFile << report.str();
    }

    MessageBoxW(
        window_,
        report.str().c_str(),
        L"Point Cloud Benchmark Result",
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

    if (requestedPointRenderMode_
        != activePointRenderMode_)
    {
        MessageBoxW(
            window_,
            L"The requested renderer is not ready.\n"
            L"Press C to return to Discrete mode "
            L"before starting the benchmark.",
            L"Benchmark Cannot Start",
            MB_OK | MB_ICONINFORMATION);

        return;
    }

    benchmarkPointRenderMode_ =
        activePointRenderMode_;

    benchmarkFullDetailMode_ =
        fullDetailMode_;

    FramePointCloud();

    benchmarkPhase_ =
        BenchmarkPhase::WarmingUp;

    benchmarkFrameCount_ = 0;

    benchmarkFrameTimeTotal_ = 0.0;
    benchmarkCpuTimeTotal_ = 0.0;
    benchmarkGpuTimeTotal_ = 0.0;

    benchmarkDrawnPointTotal_ = 0;
    benchmarkDrawRangeTotal_ = 0;
    benchmarkOverflowPointCount_ = 0;

    benchmarkFrameTimeSamples_.clear();
    benchmarkFrameTimeSamples_.reserve(300);

    const wchar_t* benchmarkModeName =
        benchmarkPointRenderMode_
        == PointRenderMode::Continuous
        ? L"CLOD"
        : L"Discrete";

    benchmarkStatusText_ =
        std::wstring(L"Benchmark: warming up (")
        + benchmarkModeName
        + L")";
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
