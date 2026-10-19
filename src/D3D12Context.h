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
#include <limits>
#include <optional>

#include "GpuPoint.h"
#include "ClodPoint.h"

class Camera;

using PointCloudTileId = std::uint64_t;

enum class PointColorMode : std::uint32_t
{
    SourceColor = 0,
    Elevation = 1
};

struct PointCloudDrawRange
{
    std::uint32_t firstPoint = 0;
    std::uint32_t pointCount = 0;
};

struct LodDebugBox
{
    DirectX::XMFLOAT3 minimum{};
    DirectX::XMFLOAT3 maximum{};
    std::uint32_t color = 0;
};

class D3D12Context
{
public:
    void Initialize(
        HWND window,
        std::uint32_t width,
        std::uint32_t height,
        std::uint32_t clodOutputCapacity,
        std::uint32_t clodTargetPointCount);

    ~D3D12Context();

    void Render(const Camera& camera, bool runClodReduction);
    void ClearPointCloudTiles();

    [[nodiscard]]
    PointCloudTileId AddPointCloudTile(
        std::span<const GpuPoint> points,
        std::uint32_t initialDrawPointCount);

    [[nodiscard]]
    PointCloudTileId BeginPointCloudTileUpload(
        std::span<const GpuPoint> discretePoints,
        std::span<const ClodPoint> clodPoints,
        std::uint32_t initialDrawPointCount);

    [[nodiscard]]
    bool IsPointCloudTileUploadComplete(
        PointCloudTileId tileId) const;

    [[nodiscard]]
    bool FinalizePointCloudTileUpload(
        PointCloudTileId tileId);

    [[nodiscard]]
    bool AbandonPointCloudTileUpload(
        PointCloudTileId tileId);

    // Adds the continuous-LOD source to an already
    // resident discrete tile without replacing that tile.
    [[nodiscard]]
    bool BeginPointCloudTileClodUpload(
        PointCloudTileId tileId,
        std::span<const ClodPoint> clodPoints,
        float clodRootSpacing);

    [[nodiscard]]
    bool IsPointCloudTileClodUploadComplete(
        PointCloudTileId tileId) const;

    [[nodiscard]]
    bool FinalizePointCloudTileClodUpload(
        PointCloudTileId tileId);

    [[nodiscard]]
    bool AbandonPointCloudTileClodUpload(
        PointCloudTileId tileId);

    [[nodiscard]]
    std::size_t
    GetPendingPointCloudUploadCount() const noexcept
    {
        return
            pendingPointTileUploads_.size()
            + pendingClodTileUploads_.size();
    }

    [[nodiscard]]
    bool SetPointCloudTileDrawRanges(
        PointCloudTileId tileId,
        std::span<
        const PointCloudDrawRange>
        drawRanges);

    [[nodiscard]]
    bool SetPointCloudTileClodDebugRange(
        PointCloudTileId tileId,
        PointCloudDrawRange drawRange);

    void SetClodLevelDebugEnabled(
        bool enabled) noexcept;

    void SetLodDebugBoxes(
        std::span<const LodDebugBox> boxes);

    [[nodiscard]]
    bool RemovePointCloudTile(
        PointCloudTileId tileId);

    [[nodiscard]]
    bool IsPointCloudTileReferencedByActiveClodReduction(
        PointCloudTileId tileId) const noexcept;

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

    [[nodiscard]]
    std::uint64_t
        GetClodReductionBufferSizeBytes() const noexcept;

    [[nodiscard]]
    std::uint64_t
        GetClodReductionProcessedPointCount() const noexcept
    {
        return clodReductionCycle_.processedSourcePointCount;
    }

    [[nodiscard]]
    std::uint64_t
        GetClodReductionTotalPointCount() const noexcept
    {
        return clodReductionCycle_.totalSourcePointCount;
    }

    [[nodiscard]]
    bool WasClodDrawnLastFrame() const noexcept
    {
        return clodDrawnLastFrame_;
    }

    void SetClodFactor(float factor);

    [[nodiscard]]
    float GetClodFactor() const noexcept
    {
        return clodFactor_;
    }

    [[nodiscard]]
    std::uint64_t
        GetClodReductionSourceBudgetPerFrame() const noexcept
    {
        return clodReductionSourcePointBudgetPerFrame_;
    }

    [[nodiscard]]
    std::uint32_t
        GetClodSelectedPointCount() const noexcept;

    [[nodiscard]]
    std::uint32_t
        GetClodOverflowPointCount() const noexcept;

private:
    static constexpr std::uint32_t
        ClodReductionSlotCount = 2;

    static constexpr std::uint32_t
        ClodDescriptorsPerReductionSlot = 2;

    // The real resident tile count will be much lower
    // because of the GPU memory budget.
    //
    // Reserving descriptor capacity is inexpensive compared
    // with reserving actual GPU point memory.
    static constexpr std::uint32_t
        MaximumClodTileDescriptors = 4'096;

    static constexpr std::uint32_t
        ClodFixedDescriptorCount =
        ClodReductionSlotCount
        * ClodDescriptorsPerReductionSlot;

    static constexpr std::uint32_t
        ClodTileDescriptorStart =
        ClodFixedDescriptorCount;

    static constexpr std::uint32_t
        ClodDescriptorCount =
        ClodFixedDescriptorCount
        + MaximumClodTileDescriptors;

    static constexpr std::uint32_t
        InvalidClodDescriptorIndex =
        std::numeric_limits<
        std::uint32_t>::max();

    struct ClodReductionSlot
    {
        // Compact ClodPoint output written by the compute
        // shader.
        Microsoft::WRL::ComPtr<ID3D12Resource>
            pointBuffer;

        // Indirect D3D12_DRAW_ARGUMENTS followed by the overflow
        // counter. InstanceCount is the accepted point count.
        Microsoft::WRL::ComPtr<ID3D12Resource>
            counterBuffer;

        // CPU-readable copy of the two counters.
        Microsoft::WRL::ComPtr<ID3D12Resource>
            counterReadback;

        std::uint32_t outputUavDescriptorIndex = 0;
        std::uint32_t counterUavDescriptorIndex = 0;

        std::uint32_t capacity = 0;

        std::uint32_t lastAcceptedPointCount = 0;
        std::uint32_t lastOverflowPointCount = 0;

        // CLOD factor used to create this result. If the
        // output overflowed, it provides the baseline for the
        // following cycle's automatic quality reduction.
        float resultClodFactor = 1.0f;

        // Frozen view used to build this completed result.
        DirectX::XMFLOAT3 cameraPosition{};
        DirectX::XMFLOAT4X4 viewProjection{};

        std::uint64_t sourceGeneration = 0;

        D3D12_RESOURCE_STATES pointBufferState =
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS;

        D3D12_RESOURCE_STATES counterBufferState =
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS;

        // Fence for the command list that most recently wrote
        // this slot. Zero means the slot is available.
        std::uint64_t completionFenceValue = 0;

        // Number of points that the fixed-stride test should
        // encounter before applying the output capacity.
        std::uint64_t expectedCandidateCount = 0;

        // Becomes true once a completed and validated result
        // exists in this slot.
        bool hasValidResult = false;
    };

    struct ClodReductionSource
    {
        // Stable identity used to find the tile again on a
        // later frame. We cannot store a GpuPointTile pointer
        // because pointTiles_ may move its elements.
        PointCloudTileId tileId = 0;

        // Number of points this reduction expects to scan from
        // this tile.
        std::uint32_t pointCount = 0;
    };

    struct ClodReductionCycle
    {
        // Slot receiving the gradually accumulated output.
        //
        // No value means that a reduction is not currently
        // being constructed.
        std::optional<std::uint32_t> outputSlotIndex;

        // Snapshot of the tiles included when this cycle began.
        std::vector<ClodReductionSource> sources;

        // Current entry inside sources.
        std::size_t sourceIndex = 0;

        // Next point to process inside sources[sourceIndex].
        std::uint32_t sourceFirstPoint = 0;

        // Useful for progress reporting and validation.
        std::uint64_t totalSourcePointCount = 0;
        std::uint64_t processedSourcePointCount = 0;

        // Changes whenever a tile CLOD source is attached or
        // removed. A completed result is drawable only while it
        // still describes the current source set.
        std::uint64_t sourceGeneration = 0;

        // Freeze the view for one complete reduction so every
        // chunk is evaluated using the same camera.
        DirectX::XMFLOAT3 cameraPosition{};
        DirectX::XMFLOAT4X4 viewProjection{};

        float clodFactor = 1.0f;

        float frustumExpansion = 2.0f;
        float peripheralDensityFalloff = 0.5f;
        float minimumPeripheralDensity = 0.3f;

        [[nodiscard]]
        bool IsActive() const noexcept
        {
            return outputSlotIndex.has_value();
        }

        void Reset() noexcept
        {
            outputSlotIndex.reset();

            // clear() retains the vector allocation so starting
            // the following cycle usually requires no new memory.
            sources.clear();

            sourceIndex = 0;
            sourceFirstPoint = 0;

            totalSourcePointCount = 0;
            processedSourcePointCount = 0;
            sourceGeneration = 0;

            cameraPosition = {};
            viewProjection = {};

            clodFactor = 1.0f;

            frustumExpansion = 2.0f;
            peripheralDensityFalloff = 0.5f;
            minimumPeripheralDensity = 0.3f;
        }
    };

    std::uint32_t nextClodReductionSlotIndex_ = 0;

    // The most recent completed reduction being displayed.
    //
    // Compute must never overwrite this slot.
    std::optional<std::uint32_t>
        activeClodReductionSlotIndex_;

    // Reduction currently being assembled across several frames.
    //
    // Its output slot must not be displayed until the complete
    // source snapshot has been processed.
    ClodReductionCycle clodReductionCycle_;

    bool clodDrawnLastFrame_ = false;

    float clodFactor_ = 1.0f;
    std::uint32_t clodTargetPointCount_ = 0;

    std::uint64_t clodSourceGeneration_ = 0;

    struct FrameResource
    {
        Microsoft::WRL::ComPtr<
            ID3D12CommandAllocator>
            commandAllocator;

        Microsoft::WRL::ComPtr<ID3D12Resource>
            cameraBuffer;

        std::uint8_t* mappedCameraData = nullptr;

        Microsoft::WRL::ComPtr<ID3D12Resource>
            lodDebugLineBuffer;

        std::uint8_t* mappedLodDebugLineData = nullptr;

        std::uint64_t lodDebugLineCapacityBytes = 0;

        std::uint64_t fenceValue = 0;

        Microsoft::WRL::ComPtr<ID3D12Resource>
            gpuTimestampReadback;

        bool gpuTimingPending = false;
    };

    struct GpuPointTile
    {
        // Existing discrete hierarchy buffer.
        //
        // The graphics pipeline reads this as a vertex
        // buffer using pointBufferView.
        Microsoft::WRL::ComPtr<ID3D12Resource>
            pointBuffer;

        // New flat additive CLOD representation.
        //
        // M11.3 will expose this to the compute shader
        // through a Shader Resource View.
        Microsoft::WRL::ComPtr<ID3D12Resource>
            clodSourceBuffer;

        // The copy queue leaves both default buffers in
        // COPY_DEST. The direct queue performs their final
        // transitions before using them.
        bool needsCopyToVertexTransition = false;
        bool needsCopyToClodSourceTransition = false;

        D3D12_VERTEX_BUFFER_VIEW
            pointBufferView{};

        std::vector<PointCloudDrawRange>
            drawRanges;

        PointCloudDrawRange clodDebugDrawRange{};

        // Number of GpuPoint records in pointBuffer.
        std::uint32_t uploadedPointCount = 0;

        // Number of ClodPoint records in clodSourceBuffer.
        std::uint32_t uploadedClodPointCount = 0;

        // World-space spacing represented by CLOD level zero.
        float clodRootSpacing = 0.0f;

        // Shader-visible SRV descriptor for
        // clodSourceBuffer.
        //
        // Allocated when the copy upload is finalized.
        std::uint32_t clodSrvDescriptorIndex =
            InvalidClodDescriptorIndex;

        // Number of points selected by the current
        // discrete traversal.
        std::uint64_t selectedPointCount = 0;

        // Individual sizes are useful for diagnostics
        // and later descriptor construction.
        std::uint64_t discreteSizeBytes = 0;
        std::uint64_t clodSizeBytes = 0;

        // Combined default-heap allocation for this tile:
        //
        // discreteSizeBytes + clodSizeBytes
        //
        // Existing residency accounting can continue using
        // one total value per logical tile.
        std::uint64_t sizeBytes = 0;

        PointCloudTileId id = 0;
    };

    struct PendingPointTileUpload
    {
        GpuPointTile tile;

        Microsoft::WRL::ComPtr<ID3D12Resource>
            uploadBuffer;

        Microsoft::WRL::ComPtr<
            ID3D12CommandAllocator>
            commandAllocator;

        Microsoft::WRL::ComPtr<
            ID3D12GraphicsCommandList>
            commandList;

        bool abandoned = false;

        std::uint64_t copyFenceValue = 0;
    };

    struct PendingClodTileUpload
    {
        PointCloudTileId tileId = 0;

        Microsoft::WRL::ComPtr<ID3D12Resource>
            clodSourceBuffer;

        Microsoft::WRL::ComPtr<ID3D12Resource>
            uploadBuffer;

        Microsoft::WRL::ComPtr<
            ID3D12CommandAllocator>
            commandAllocator;

        Microsoft::WRL::ComPtr<
            ID3D12GraphicsCommandList>
            commandList;

        float rootSpacing = 0.0f;
        std::uint32_t pointCount = 0;
        std::uint64_t sizeBytes = 0;
        std::uint64_t copyFenceValue = 0;
        bool abandoned = false;
    };

    struct RetiredPointTile
    {
        PointCloudTileId id = 0;

        // Keep the discrete buffer alive until the
        // retirement fence completes.
        Microsoft::WRL::ComPtr<ID3D12Resource>
            pointBuffer;

        // Keep the CLOD source buffer alive for the same
        // reason. Later compute dispatches may reference it.
        Microsoft::WRL::ComPtr<ID3D12Resource>
            clodSourceBuffer;

        // Combined discrete and CLOD allocation.
        std::uint64_t sizeBytes = 0;

        std::uint32_t clodSrvDescriptorIndex =
            InvalidClodDescriptorIndex;

        std::uint64_t releaseFenceValue = 0;
    };

    void CollectAbandonedPointUploads();
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
    void CreateCopyQueue();
    void CreateCopyFence();
    void CreateBackBufferViews();
    void CreateDepthResources(
        std::uint32_t width,
        std::uint32_t height);
    void UpdateViewportAndScissor(
        std::uint32_t width,
        std::uint32_t height);
    Microsoft::WRL::ComPtr<ID3D12Resource>
        CreateBuffer(
            std::uint64_t size,
            D3D12_HEAP_TYPE heapType,
            D3D12_RESOURCE_STATES initialState,
            D3D12_RESOURCE_FLAGS flags =
            D3D12_RESOURCE_FLAG_NONE);

    void CreateClodReductionPipeline();
    void CreateClodReductionResources(std::uint32_t outputCapacity);
    [[nodiscard]]
    D3D12_CPU_DESCRIPTOR_HANDLE
        GetClodCpuDescriptorHandle(std::uint32_t descriptorIndex) const noexcept;
    [[nodiscard]]
    D3D12_GPU_DESCRIPTOR_HANDLE
        GetClodGpuDescriptorHandle(std::uint32_t descriptorIndex) const noexcept;

    //clod tile heap code
    [[nodiscard]]
    std::uint32_t AllocateClodTileDescriptor();

    void ReleaseClodTileDescriptor(
        std::uint32_t descriptorIndex);

    void ResetClodTileDescriptorAllocator();

    void CreateClodTileSourceView(
        GpuPointTile& tile);

    [[nodiscard]]
    std::optional<std::uint32_t>
        RecordClodReduction(
            const Camera& camera);

    void CollectCompletedClodReductions();

    void WaitForFrame(FrameResource& frame);
    void SignalFrame(FrameResource& frame);
    void WaitForGpu();
    void WaitForFenceValue(std::uint64_t fenceValue);
    void WaitForCopyFenceValue(std::uint64_t fenceValue);
    void WaitForPendingPointUploads();
    void RecordPointTileUploadTransitions();

    Microsoft::WRL::ComPtr<IDXGIFactory7> factory_;
    Microsoft::WRL::ComPtr<IDXGIAdapter4> adapter_;
    Microsoft::WRL::ComPtr<ID3D12Device> device_;
    Microsoft::WRL::ComPtr<ID3D12CommandQueue> commandQueue_;
    Microsoft::WRL::ComPtr<ID3D12CommandQueue> copyQueue_;
    Microsoft::WRL::ComPtr<IDXGISwapChain4> swapChain_;
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> rtvHeap_;
    std::array<Microsoft::WRL::ComPtr<ID3D12Resource>, FrameCount> backBuffers_;
    std::array<FrameResource, FrameCount> frameResources_;
    Microsoft::WRL::ComPtr<ID3D12GraphicsCommandList> commandList_;
    Microsoft::WRL::ComPtr<ID3D12Fence> fence_;
    Microsoft::WRL::ComPtr<ID3D12Fence> copyFence_;
    Microsoft::WRL::ComPtr<ID3D12RootSignature> rootSignature_;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> pipelineState_;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> clodDrawPipelineState_;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> debugLinePipelineState_;
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> dsvHeap_;
    Microsoft::WRL::ComPtr<ID3D12Resource> depthBuffer_;

    //Clod pipeline stuff
    Microsoft::WRL::ComPtr<ID3D12RootSignature>
        clodReductionRootSignature_;
    Microsoft::WRL::ComPtr<ID3D12PipelineState>
        clodReductionPipelineState_;
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap>
        clodDescriptorHeap_;
    Microsoft::WRL::ComPtr<ID3D12CommandSignature>
        clodDrawCommandSignature_;
    Microsoft::WRL::ComPtr<ID3D12Resource>
        clodQuadVertexBuffer_;
    D3D12_VERTEX_BUFFER_VIEW
        clodQuadVertexBufferView_{};
    // Descriptor indices from ClodTileDescriptorStart
    // onward that are currently available for tile SRVs.
    std::vector<std::uint32_t>
        freeClodTileDescriptorIndices_;

    std::uint32_t clodDescriptorSize_ = 0;

    std::array<ClodReductionSlot, ClodReductionSlotCount>
        clodReductionSlots_;

    // Maximum number of source points examined by the CLOD
    // compute shader during one frame.
    //
    // This limits compute work, not the number of points that
    // survive into the reduced output.
    std::uint64_t
        clodReductionSourcePointBudgetPerFrame_ =
        1'000'000;

    // Persistent indirect-argument template used to reset the
    // accepted instance count and overflow count while retaining
    // six vertices per point quad.
    Microsoft::WRL::ComPtr<ID3D12Resource>
        clodCounterResetUpload_;

    std::vector<GpuPointTile> pointTiles_;
    std::vector<PendingPointTileUpload> pendingPointTileUploads_;
    std::vector<PendingClodTileUpload> pendingClodTileUploads_;
    std::uint64_t pointCount_ = 0;

    bool clodLevelDebugEnabled_ = false;
    std::uint64_t clodDebugSelectedPointCount_ = 0;

    std::vector<GpuPoint> lodDebugLineVertices_;

    D3D12_VIEWPORT viewport_{};
    D3D12_RECT scissorRectangle_{};

    HANDLE fenceEvent_ = nullptr;
    std::uint64_t nextFenceValue_ = 1;
    //Copy queue and fence
    HANDLE copyFenceEvent_ = nullptr;
    std::uint64_t nextCopyFenceValue_ = 1;

    std::uint32_t rtvDescriptorSize_ = 0;
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
