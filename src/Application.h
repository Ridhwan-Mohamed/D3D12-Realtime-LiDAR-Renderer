#pragma once

#include "D3D12Context.h"
#include "Camera.h"
#include "PointCloudDataset.h"
#include "PointCloudHierarchy.h"
#include "TileLoadWorker.h"

#include <Windows.h>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>
#include <queue>
#include <stop_token>


class Application
{
    enum class BenchmarkPhase
    {
        Idle,
        WarmingUp,
        Measuring
    };

    enum class PointRenderMode : std::uint8_t
    {
        Discrete,
        Continuous
    };

    void StartBenchmark();

    void UpdateBenchmark(
        double frameSeconds);

public:
    Application(
        HINSTANCE instance,
        std::vector<std::filesystem::path>
        pointCloudPaths);

    int Run(int showCommand);
    void UpdateCamera(float deltaSeconds);

private:
    enum class TileLifecycleState : std::uint8_t
    {
        Unloaded,
        Queued,
        Loading,
        ReadyForUpload,
        Uploading,
        Resident,
        Retiring
    };

    enum class ClodLifecycleState : std::uint8_t
    {
        NotRequested,
        ReadyToBuild,
        Building,
        ReadyForUpload,
        Uploading,
        Resident
    };

    struct TileResidencyState
    {
        std::size_t metadataIndex = 0;

        std::uint64_t requiredGpuBytes = 0;

        std::optional<PointCloudTileId>
            gpuTileId;

        DirectX::XMFLOAT3 localBoundsMinimum{};
        DirectX::XMFLOAT3 localBoundsMaximum{};

        float cameraDistanceSquared = 0.0f;

        bool wanted = false;

        float unwantedSeconds = 0.0f;

        TileLifecycleState lifecycleState =
            TileLifecycleState::Unloaded;

        std::uint64_t requestGeneration = 0;

        std::stop_source loadStopSource;

        // CLOD preparation proceeds independently after the
        // discrete tile has begun uploading.
        ClodLifecycleState clodLifecycleState =
            ClodLifecycleState::NotRequested;

        [[nodiscard]]
        bool IsResident() const noexcept
        {
            return lifecycleState == TileLifecycleState::Resident;
        }

        PointCloudHierarchy hierarchy;

        std::vector<std::uint8_t>
            selectedNodeFlags;

        std::vector<std::uint8_t>
            refinedNodeFlags;

        std::vector<std::uint8_t>
            nextRefinedNodeFlags;

        std::vector<PointCloudDrawRange>
            selectedDrawRanges;
    };

    std::vector<TileResidencyState>
        tileResidencyStates_;

    [[nodiscard]]
    static bool CanTransitionTileState(
        TileLifecycleState currentState,
        TileLifecycleState nextState) noexcept;

    static void TransitionTileState(
        TileResidencyState& state,
        TileLifecycleState nextState);

    [[nodiscard]]
    static const char* GetTileLifecycleStateName(
        TileLifecycleState state) noexcept;

    static void ValidateTileLifecycleState(
        const TileResidencyState& state);

    [[nodiscard]]
    static std::uint64_t BeginTileRequest(
        TileResidencyState& state);

    static void InvalidateTileRequest(
        TileResidencyState& state);

    [[nodiscard]]
    static bool IsCurrentTileRequest(
        const TileResidencyState& state,
        std::uint64_t requestGeneration) noexcept;

    static constexpr std::uint64_t
        PointMemoryBudgetBytes =
        3ull * 1024ull * 1024ull * 1024ull;

    static LRESULT CALLBACK WindowProcedure(
        HWND window,
        UINT message,
        WPARAM wParam,
        LPARAM lParam);

    LRESULT HandleMessage(
        HWND window,
        UINT message,
        WPARAM wParam,
        LPARAM lParam);

    void RegisterWindowClass();
    void CreateApplicationWindow();

    HINSTANCE instance_ = nullptr;
    HWND window_ = nullptr;

    D3D12Context graphics_;

    TileLoadWorker tileLoadWorker_{
        2
    };

    Camera camera_{
    1600.0f / 900.0f
    };

    bool rotatingCamera_ = false;
    POINT previousMousePosition_{};

    std::uint32_t clientWidth_ = 1600;
    std::uint32_t clientHeight_ = 900;

    bool minimized_ = false;

    void FramePointCloud();
    float cameraMovementSpeed_ = 5.0f;
    bool resetViewKeyWasDown_ = false;

    void UpdateFrameStatistics(
        double frameSeconds);
    void UpdateTileStabilityTimers(
        float deltaSeconds);
    std::wstring windowTitlePrefix_;
    double frameTimeAccumulator_ = 0.0;
    std::uint32_t measuredFrameCount_ = 0;
    double cpuRenderTimeAccumulator_ = 0.0;
    double gpuRenderTimeAccumulator_ = 0.0;

    std::uint32_t gpuMeasuredFrameCount_ = 0;

    BenchmarkPhase benchmarkPhase_ =
        BenchmarkPhase::Idle;

    bool benchmarkKeyWasDown_ = false;

    PointRenderMode benchmarkPointRenderMode_ =
        PointRenderMode::Discrete;

    std::uint32_t benchmarkFrameCount_ = 0;

    double benchmarkFrameTimeTotal_ = 0.0;
    double benchmarkCpuTimeTotal_ = 0.0;
    double benchmarkGpuTimeTotal_ = 0.0;

    std::vector<double>
        benchmarkFrameTimeSamples_;

    std::uint64_t benchmarkDrawnPointTotal_ = 0;
    std::uint64_t benchmarkDrawRangeTotal_ = 0;

    std::uint32_t benchmarkOverflowPointCount_ = 0;

    bool benchmarkFullDetailMode_ = false;

    std::wstring benchmarkStatusText_;

    std::vector<std::filesystem::path> pointCloudPaths_;
    std::optional<PointCloudDatasetInfo> pointCloudDataset_;


    // Tiling and LOD loading.
    void UpdateTilePriorities();
    void UpdateTileResidency(float deltaSeconds);
    void ProcessTileLoadStartedEvents();
    void ProcessOneCompletedTileLoad();
    void TryStartClodBuilds();
    void ProcessCompletedClodBuilds();
    void TryStartClodUploads();
    void ProcessCompletedClodUploads();
    void CancelClodWork(
        std::size_t stateIndex,
        TileResidencyState& state);
    void ProcessCompletedPointUploads();
    void CancelUnwantedCpuTileRequests();
    void CancelUnwantedPointUploads();
    [[nodiscard]]
    bool TryQueueNextTileLoad();
    void ValidateTileResidency() const;
    void UpdatePointDetailSelection();
    std::vector<std::size_t> tilePriorityOrder_;
    std::size_t wantedTileCount_ = 0;
    std::uint64_t wantedPointBytes_ = 0;
    double residencyUpdateAccumulator_ = 1.0;
    static constexpr float TileUnwantedGraceSeconds = 0.75f;
    static constexpr std::size_t MaximumConcurrentCpuTileRequests = 1;
    static constexpr std::size_t MaximumConcurrentPointUploads = 1;
    static constexpr float RefinePointSpacingPixels = 2.2f;
    static constexpr float CoarsenPointSpacingPixels = 1.8f;
    static constexpr std::uint64_t MaximumDrawnPoints = 3'500'000;
    PointRenderMode requestedPointRenderMode_ =
        PointRenderMode::Discrete;

    PointRenderMode activePointRenderMode_ =
        PointRenderMode::Discrete;

    bool pointRenderModeKeyWasDown_ = false;
    bool denserClodKeyWasDown_ = false;
    bool sparserClodKeyWasDown_ = false;

    bool fullDetailMode_ = false;
    bool detailModeKeyWasDown_ = false;

    std::size_t selectedDrawRangeCount_ = 0;
    std::size_t frustumCulledNodeCount_ = 0;

    static constexpr double
        ResidencyUpdateIntervalSeconds = 0.25;
};
