#pragma once

#include "D3D12Context.h"
#include "Camera.h"
#include "PointCloudDataset.h"
#include "PointCloudHierarchy.h"

#include <Windows.h>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>
#include <queue>

class Application
{
    enum class BenchmarkPhase
    {
        Idle,
        WarmingUp,
        Measuring
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

        [[nodiscard]]
        bool IsResident() const noexcept
        {
            return gpuTileId.has_value();
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
    std::wstring windowTitlePrefix_;
    double frameTimeAccumulator_ = 0.0;
    std::uint32_t measuredFrameCount_ = 0;
    double cpuRenderTimeAccumulator_ = 0.0;
    double gpuRenderTimeAccumulator_ = 0.0;

    std::uint32_t gpuMeasuredFrameCount_ = 0;

    BenchmarkPhase benchmarkPhase_ =
        BenchmarkPhase::Idle;

    bool benchmarkKeyWasDown_ = false;

    std::uint32_t benchmarkFrameCount_ = 0;

    double benchmarkFrameTimeTotal_ = 0.0;
    double benchmarkCpuTimeTotal_ = 0.0;
    double benchmarkGpuTimeTotal_ = 0.0;

    std::wstring benchmarkStatusText_;

    std::vector<std::filesystem::path> pointCloudPaths_;
    std::optional<PointCloudDatasetInfo> pointCloudDataset_;


    // Tiling and LOD loading.
    void UpdateTilePriorities();
    void UpdateTileResidency(float deltaSeconds);
    void ValidateTileResidency() const;
    void UpdatePointDetailSelection();
    std::vector<std::size_t> tilePriorityOrder_;
    std::size_t wantedTileCount_ = 0;
    std::uint64_t wantedPointBytes_ = 0;
    double residencyUpdateAccumulator_ = 1.0;
    static constexpr float RefinePointSpacingPixels = 2.2f;
    static constexpr float CoarsenPointSpacingPixels = 1.8f;
    static constexpr std::uint64_t MaximumDrawnPoints = 2'000'000;
    bool fullDetailMode_ = false;
    bool detailModeKeyWasDown_ = false;
    std::size_t selectedDrawRangeCount_ = 0;
    std::size_t frustumCulledNodeCount_ = 0;

    static constexpr double
        ResidencyUpdateIntervalSeconds = 0.25;
};
