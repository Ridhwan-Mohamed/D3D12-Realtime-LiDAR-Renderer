#pragma once

#include <d3d12.h>
#include <dxgi1_6.h>
#include <wrl/client.h>

#include <Windows.h>
#include <cstdint>
#include <string>
#include <array>
#include <span>
#include <vector>

#include "GpuPoint.h"

class Camera;

using PointCloudTileId = std::uint64_t;

enum class PointColorMode : std::uint32_t
{
    Classification = 0,
    Elevation = 1
};

struct PointCloudDrawRange
{
    std::uint32_t firstPoint = 0;
    std::uint32_t pointCount = 0;
};

class D3D12Context
{
public:
    void Initialize(
        HWND window,
        std::uint32_t width,
        std::uint32_t height);

    ~D3D12Context();

    void Render(const Camera& camera);
    void ClearPointCloudTiles();

    [[nodiscard]]
    PointCloudTileId AddPointCloudTile(
        std::span<const GpuPoint> points,
        std::uint32_t initialDrawPointCount);

    [[nodiscard]]
    bool SetPointCloudTileDrawRanges(
        PointCloudTileId tileId,
        std::span<
        const PointCloudDrawRange>
        drawRanges);

    [[nodiscard]]
    bool RemovePointCloudTile(
        PointCloudTileId tileId);

    [[nodiscard]]
    std::size_t
        GetRetiredPointCloudTileCount() const noexcept
    {
        return retiredPointTiles_.size();
    }

    void Resize(
        std::uint32_t width,
        std::uint32_t height);
    void Shutdown() noexcept;

    [[nodiscard]]
    const std::wstring& GetAdapterName() const;
    static constexpr std::uint32_t FrameCount = 3;

    [[nodiscard]]
    std::uint64_t GetPointCount() const noexcept
    {
        return pointCount_;
    }

    [[nodiscard]]
    std::size_t GetPointCloudTileCount() const noexcept
    {
        return pointTiles_.size();
    }

    void SetPointColorMode(
        PointColorMode mode) noexcept;

    [[nodiscard]]
    PointColorMode GetPointColorMode() const noexcept;

    void SetElevationRange(
        float minimumElevation,
        float maximumElevation);

    [[nodiscard]]
    double GetGpuFrameMilliseconds() const noexcept
    {
        return lastGpuFrameMilliseconds_;
    }

    [[nodiscard]]
    double GetCpuRenderMilliseconds() const noexcept
    {
        return lastCpuRenderMilliseconds_;
    }

    [[nodiscard]]
    std::uint64_t GetPointBufferSizeBytes() const noexcept
    {
        return pointBufferSizeBytes_;
    }

private:
    struct FrameResource
    {
        Microsoft::WRL::ComPtr<
            ID3D12CommandAllocator>
            commandAllocator;

        Microsoft::WRL::ComPtr<ID3D12Resource>
            cameraBuffer;

        std::uint8_t* mappedCameraData = nullptr;

        std::uint64_t fenceValue = 0;

        Microsoft::WRL::ComPtr<ID3D12Resource>
            gpuTimestampReadback;

        bool gpuTimingPending = false;
    };

    struct GpuPointTile
    {
        Microsoft::WRL::ComPtr<ID3D12Resource>
            pointBuffer;

        D3D12_VERTEX_BUFFER_VIEW
            pointBufferView{};

        std::vector<PointCloudDrawRange>
            drawRanges;

        std::uint32_t uploadedPointCount = 0;
        std::uint64_t selectedPointCount = 0;

        std::uint64_t sizeBytes = 0;
        PointCloudTileId id = 0;
    };

    struct RetiredPointTile
    {
        PointCloudTileId id = 0;

        Microsoft::WRL::ComPtr<ID3D12Resource>
            pointBuffer;

        std::uint64_t sizeBytes = 0;

        std::uint64_t releaseFenceValue = 0;
    };

    void CollectRetiredPointTiles();

    std::vector<RetiredPointTile>
        retiredPointTiles_;

    void EnableDebugLayer();
    void CreateFactory();
    void SelectAdapter();
    void CreateDevice();
    void ConfigureValidation();

    void CreateCommandQueue();
    void CreateSwapChain(
        HWND window,
        std::uint32_t width,
        std::uint32_t height);
    void CreateRenderTargets();
    void CreateGraphicsPipeline();
    void CreateCommandObjects();
    void CreateFence();
    void CreateBackBufferViews();
    void CreateDepthResources(
        std::uint32_t width,
        std::uint32_t height);
    void UpdateViewportAndScissor(
        std::uint32_t width,
        std::uint32_t height);
    Microsoft::WRL::ComPtr<ID3D12Resource> CreateBuffer(
            std::uint64_t size,
            D3D12_HEAP_TYPE heapType,
            D3D12_RESOURCE_STATES initialState
    );

    void WaitForFrame(FrameResource& frame);
    void SignalFrame(FrameResource& frame);
    void WaitForGpu();
    void WaitForFenceValue(std::uint64_t fenceValue);

    Microsoft::WRL::ComPtr<IDXGIFactory7> factory_;
    Microsoft::WRL::ComPtr<IDXGIAdapter4> adapter_;
    Microsoft::WRL::ComPtr<ID3D12Device> device_;
    Microsoft::WRL::ComPtr<ID3D12CommandQueue> commandQueue_;
    Microsoft::WRL::ComPtr<IDXGISwapChain4> swapChain_;
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> rtvHeap_;
    std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, FrameCount> backBuffers_;
    std::array<FrameResource, FrameCount> frameResources_;
    Microsoft::WRL::ComPtr<ID3D12GraphicsCommandList> commandList_;
    Microsoft::WRL::ComPtr<ID3D12Fence> fence_;
    Microsoft::WRL::ComPtr<ID3D12RootSignature> rootSignature_;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> pipelineState_;
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> dsvHeap_;
    Microsoft::WRL::ComPtr<ID3D12Resource> depthBuffer_;

    std::vector<GpuPointTile> pointTiles_;
    std::uint64_t pointCount_ = 0;

    D3D12_VIEWPORT viewport_{};
    D3D12_RECT scissorRectangle_{};

    HANDLE fenceEvent_ = nullptr;

    std::uint32_t rtvDescriptorSize_ = 0;
    std::uint64_t nextFenceValue_ = 1;
    std::uint32_t frameIndex_ = 0;
    std::uint32_t width_ = 0;
    std::uint32_t height_ = 0;
    bool initialized_ = false;

    std::wstring adapterName_;
    bool debugLayerEnabled_ = false;

    PointColorMode pointColorMode_ =
        PointColorMode::Elevation;

    float minimumElevation_ = 0.0f;
    float maximumElevation_ = 1.0f;


    //TIMING STUFF
    void CreateGpuTimingResources();
    void ReadGpuTimingResult(FrameResource& frame);
    Microsoft::WRL::ComPtr<ID3D12QueryHeap> gpuTimestampQueryHeap_;
    std::uint64_t gpuTimestampFrequency_ = 0;
    double lastGpuFrameMilliseconds_ = 0.0;
    double lastCpuRenderMilliseconds_ = 0.0;
    std::uint64_t pointBufferSizeBytes_ = 0;

    PointCloudTileId nextPointCloudTileId_ = 1;
};
