#include "D3D12Context.h"

#include "Win32Helpers.h"

#include <stdexcept>
#include <dxgidebug.h>
#include <iterator>
#include <climits>
#include <cstring>
#include <limits>
#include <cmath>
#include <chrono>
#include <utility>
#include <algorithm>

#include "Camera.h"
#include "ReferenceVS.h"
#include "ReferencePS.h"

namespace
{
    struct CameraConstants
    {
        DirectX::XMFLOAT4X4 viewProjection;

        float minimumElevation;
        float maximumElevation;

        std::uint32_t pointColorMode;
        float padding;
    };

    static_assert(
        sizeof(CameraConstants) == 80,
        "CameraConstants must match the HLSL layout.");

    constexpr std::uint64_t
        ConstantBufferAlignment = 256;

    constexpr std::uint64_t
        AlignConstantBufferSize(std::uint64_t size)
    {
        return
            (size + ConstantBufferAlignment - 1)
            & ~(ConstantBufferAlignment - 1);
    }
}

using Microsoft::WRL::ComPtr;

void D3D12Context::Initialize(
    HWND window,
    std::uint32_t width,
    std::uint32_t height)
{
    EnableDebugLayer();
    CreateFactory();
    SelectAdapter();
    CreateDevice();
    ConfigureValidation();
    CreateCommandQueue();
    CreateSwapChain(window, width, height);
    CreateRenderTargets();
    CreateDepthResources(width, height);
    UpdateViewportAndScissor(width, height);
    CreateGraphicsPipeline();
    CreateCommandObjects();
    CreateGpuTimingResources();
    CreateFence();

    width_ = width;
    height_ = height;
    initialized_ = true;
}

void D3D12Context::EnableDebugLayer()
{
#if defined(_DEBUG)

    ComPtr<ID3D12Debug> debugController;

    ThrowIfFailed(
        D3D12GetDebugInterface(
            IID_PPV_ARGS(&debugController)),
        "D3D12GetDebugInterface");

    debugController->EnableDebugLayer();
    debugLayerEnabled_ = true;

#endif
}

void D3D12Context::CreateFactory()
{
    UINT factoryFlags = 0;

    if (debugLayerEnabled_)
    {
        factoryFlags |= DXGI_CREATE_FACTORY_DEBUG;
    }

    ThrowIfFailed(
        CreateDXGIFactory2(
            factoryFlags,
            IID_PPV_ARGS(&factory_)),
        "CreateDXGIFactory2");
}

void D3D12Context::SelectAdapter()
{
    for (UINT adapterIndex = 0; ; ++adapterIndex)
    {
        ComPtr<IDXGIAdapter1> candidate;

        const HRESULT result =
            factory_->EnumAdapterByGpuPreference(
                adapterIndex,
                DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE,
                IID_PPV_ARGS(&candidate));

        if (result == DXGI_ERROR_NOT_FOUND)
        {
            break;
        }

        ThrowIfFailed(
            result,
            "EnumAdapterByGpuPreference");

        DXGI_ADAPTER_DESC1 description{};

        ThrowIfFailed(
            candidate->GetDesc1(&description),
            "IDXGIAdapter1::GetDesc1");

        if ((description.Flags &
            DXGI_ADAPTER_FLAG_SOFTWARE) != 0)
        {
            continue;
        }

        const HRESULT supportResult =
            D3D12CreateDevice(
                candidate.Get(),
                D3D_FEATURE_LEVEL_12_0,
                __uuidof(ID3D12Device),
                nullptr);

        if (FAILED(supportResult))
        {
            continue;
        }

        ThrowIfFailed(
            candidate.As(&adapter_),
            "Query IDXGIAdapter4");

        adapterName_ = description.Description;
        return;
    }

    throw std::runtime_error(
        "No hardware adapter supports D3D feature level 12_0.");
}

void D3D12Context::CreateDevice()
{
    ThrowIfFailed(
        D3D12CreateDevice(
            adapter_.Get(),
            D3D_FEATURE_LEVEL_12_0,
            IID_PPV_ARGS(&device_)),
        "D3D12CreateDevice");

    device_->SetName(
        L"Point Cloud Renderer Device");
}

void D3D12Context::ConfigureValidation()
{
#if defined(_DEBUG)

    ComPtr<ID3D12InfoQueue> infoQueue;

    if (SUCCEEDED(device_.As(&infoQueue)))
    {
        infoQueue->SetBreakOnSeverity(
            D3D12_MESSAGE_SEVERITY_CORRUPTION,
            TRUE);

        infoQueue->SetBreakOnSeverity(
            D3D12_MESSAGE_SEVERITY_ERROR,
            TRUE);
    }

#endif
}

const std::wstring&
D3D12Context::GetAdapterName() const
{
    return adapterName_;
}

void D3D12Context::CreateCommandQueue()
{
    D3D12_COMMAND_QUEUE_DESC description{};

    description.Type =
        D3D12_COMMAND_LIST_TYPE_DIRECT;

    description.Priority =
        D3D12_COMMAND_QUEUE_PRIORITY_NORMAL;

    description.Flags =
        D3D12_COMMAND_QUEUE_FLAG_NONE;

    description.NodeMask = 0;

    ThrowIfFailed(
        device_->CreateCommandQueue(
            &description,
            IID_PPV_ARGS(&commandQueue_)),
        "ID3D12Device::CreateCommandQueue");

    commandQueue_->SetName(
        L"Main Direct Command Queue");
}

void D3D12Context::CreateSwapChain(
    HWND window,
    std::uint32_t width,
    std::uint32_t height)
{
    DXGI_SWAP_CHAIN_DESC1 description{};

    description.Width = width;
    description.Height = height;
    description.Format =
        DXGI_FORMAT_R8G8B8A8_UNORM;

    description.Stereo = FALSE;

    description.SampleDesc.Count = 1;
    description.SampleDesc.Quality = 0;

    description.BufferUsage =
        DXGI_USAGE_RENDER_TARGET_OUTPUT;

    description.BufferCount = FrameCount;

    description.Scaling =
        DXGI_SCALING_STRETCH;

    description.SwapEffect =
        DXGI_SWAP_EFFECT_FLIP_DISCARD;

    description.AlphaMode =
        DXGI_ALPHA_MODE_UNSPECIFIED;

    description.Flags = 0;

    Microsoft::WRL::ComPtr<IDXGISwapChain1>
        initialSwapChain;

    ThrowIfFailed(
        factory_->CreateSwapChainForHwnd(
            commandQueue_.Get(),
            window,
            &description,
            nullptr,
            nullptr,
            &initialSwapChain),
        "IDXGIFactory::CreateSwapChainForHwnd");

    ThrowIfFailed(
        initialSwapChain.As(&swapChain_),
        "Query IDXGISwapChain4");

    ThrowIfFailed(
        factory_->MakeWindowAssociation(
            window,
            DXGI_MWA_NO_ALT_ENTER),
        "IDXGIFactory::MakeWindowAssociation");

    frameIndex_ =
        swapChain_->GetCurrentBackBufferIndex();
}

void D3D12Context::CreateRenderTargets()
{
    D3D12_DESCRIPTOR_HEAP_DESC heapDescription{};

    heapDescription.Type =
        D3D12_DESCRIPTOR_HEAP_TYPE_RTV;

    heapDescription.NumDescriptors =
        FrameCount;

    heapDescription.Flags =
        D3D12_DESCRIPTOR_HEAP_FLAG_NONE;

    heapDescription.NodeMask = 0;

    ThrowIfFailed(
        device_->CreateDescriptorHeap(
            &heapDescription,
            IID_PPV_ARGS(&rtvHeap_)),
        "Create RTV descriptor heap");

    rtvHeap_->SetName(
        L"Swap Chain RTV Heap");

    rtvDescriptorSize_ =
        device_->GetDescriptorHandleIncrementSize(
            D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

    CreateBackBufferViews();
}

void D3D12Context::CreateGraphicsPipeline()
{
    D3D12_ROOT_PARAMETER cameraParameter{};

    cameraParameter.ParameterType =
        D3D12_ROOT_PARAMETER_TYPE_CBV;

    cameraParameter.Descriptor.ShaderRegister = 0;
    cameraParameter.Descriptor.RegisterSpace = 0;

    cameraParameter.ShaderVisibility =
        D3D12_SHADER_VISIBILITY_VERTEX;

    D3D12_ROOT_SIGNATURE_DESC rootDescription{};

    rootDescription.NumParameters = 1;
    rootDescription.pParameters =
        &cameraParameter;

    rootDescription.NumStaticSamplers = 0;
    rootDescription.pStaticSamplers = nullptr;

    rootDescription.Flags =
        D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT
        | D3D12_ROOT_SIGNATURE_FLAG_DENY_HULL_SHADER_ROOT_ACCESS
        | D3D12_ROOT_SIGNATURE_FLAG_DENY_DOMAIN_SHADER_ROOT_ACCESS
        | D3D12_ROOT_SIGNATURE_FLAG_DENY_GEOMETRY_SHADER_ROOT_ACCESS
        | D3D12_ROOT_SIGNATURE_FLAG_DENY_PIXEL_SHADER_ROOT_ACCESS;

    Microsoft::WRL::ComPtr<ID3DBlob>
        serializedRootSignature;

    Microsoft::WRL::ComPtr<ID3DBlob>
        rootSignatureErrors;

    const HRESULT serializationResult =
        D3D12SerializeRootSignature(
            &rootDescription,
            D3D_ROOT_SIGNATURE_VERSION_1,
            &serializedRootSignature,
            &rootSignatureErrors);

    if (rootSignatureErrors)
    {
        const char* errorText =
            static_cast<const char*>(
                rootSignatureErrors->
                GetBufferPointer());

        OutputDebugStringA(errorText);
    }

    ThrowIfFailed(
        serializationResult,
        "Serialize root signature");

    ThrowIfFailed(
        device_->CreateRootSignature(
            0,
            serializedRootSignature->
            GetBufferPointer(),
            serializedRootSignature->
            GetBufferSize(),
            IID_PPV_ARGS(&rootSignature_)),
        "Create root signature");

    rootSignature_->SetName(
        L"Reference Geometry Root Signature");

    constexpr D3D12_INPUT_ELEMENT_DESC
        inputElements[]{
        {
            "POSITION",
            0,
            DXGI_FORMAT_R32G32B32_FLOAT,
            0,
            0,
            D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA,
            0
        },
        {
            "COLOR",
            0,
            DXGI_FORMAT_R8G8B8A8_UNORM,
            0,
            12,
            D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA,
            0
        }
    };

    D3D12_BLEND_DESC blendDescription{};

    blendDescription.AlphaToCoverageEnable =
        FALSE;

    blendDescription.IndependentBlendEnable =
        FALSE;

    D3D12_RENDER_TARGET_BLEND_DESC&
        renderTargetBlend =
        blendDescription.RenderTarget[0];

    renderTargetBlend.BlendEnable = FALSE;
    renderTargetBlend.LogicOpEnable = FALSE;

    renderTargetBlend.SrcBlend =
        D3D12_BLEND_ONE;

    renderTargetBlend.DestBlend =
        D3D12_BLEND_ZERO;

    renderTargetBlend.BlendOp =
        D3D12_BLEND_OP_ADD;

    renderTargetBlend.SrcBlendAlpha =
        D3D12_BLEND_ONE;

    renderTargetBlend.DestBlendAlpha =
        D3D12_BLEND_ZERO;

    renderTargetBlend.BlendOpAlpha =
        D3D12_BLEND_OP_ADD;

    renderTargetBlend.LogicOp =
        D3D12_LOGIC_OP_NOOP;

    renderTargetBlend.RenderTargetWriteMask =
        D3D12_COLOR_WRITE_ENABLE_ALL;

    D3D12_RASTERIZER_DESC rasterizerDescription{};

    rasterizerDescription.FillMode =
        D3D12_FILL_MODE_SOLID;

    rasterizerDescription.CullMode =
        D3D12_CULL_MODE_NONE;

    rasterizerDescription.FrontCounterClockwise =
        FALSE;

    rasterizerDescription.DepthBias =
        D3D12_DEFAULT_DEPTH_BIAS;

    rasterizerDescription.DepthBiasClamp =
        D3D12_DEFAULT_DEPTH_BIAS_CLAMP;

    rasterizerDescription.SlopeScaledDepthBias =
        D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS;

    rasterizerDescription.DepthClipEnable =
        TRUE;

    rasterizerDescription.MultisampleEnable =
        FALSE;

    rasterizerDescription.AntialiasedLineEnable =
        FALSE;

    rasterizerDescription.ForcedSampleCount = 0;

    rasterizerDescription.ConservativeRaster =
        D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF;

    D3D12_DEPTH_STENCIL_DESC
        depthStencilDescription{};

    depthStencilDescription.DepthEnable = TRUE;

    depthStencilDescription.DepthWriteMask =
        D3D12_DEPTH_WRITE_MASK_ALL;

    depthStencilDescription.DepthFunc =
        D3D12_COMPARISON_FUNC_LESS;

    depthStencilDescription.StencilEnable =
        FALSE;

    depthStencilDescription.StencilReadMask =
        D3D12_DEFAULT_STENCIL_READ_MASK;

    depthStencilDescription.StencilWriteMask =
        D3D12_DEFAULT_STENCIL_WRITE_MASK;

    D3D12_GRAPHICS_PIPELINE_STATE_DESC
        pipelineDescription{};

    pipelineDescription.pRootSignature =
        rootSignature_.Get();

    pipelineDescription.VS = {
        g_referenceVertexShader,
        sizeof(g_referenceVertexShader)
    };

    pipelineDescription.PS = {
        g_referencePixelShader,
        sizeof(g_referencePixelShader)
    };

    pipelineDescription.BlendState =
        blendDescription;

    pipelineDescription.SampleMask =
        UINT_MAX;

    pipelineDescription.RasterizerState =
        rasterizerDescription;

    pipelineDescription.DepthStencilState =
        depthStencilDescription;

    pipelineDescription.InputLayout = {
        inputElements,
        static_cast<UINT>(
            std::size(inputElements))
    };

    pipelineDescription.IBStripCutValue =
        D3D12_INDEX_BUFFER_STRIP_CUT_VALUE_DISABLED;

    pipelineDescription.PrimitiveTopologyType =
        D3D12_PRIMITIVE_TOPOLOGY_TYPE_POINT;

    pipelineDescription.NumRenderTargets = 1;

    pipelineDescription.RTVFormats[0] =
        DXGI_FORMAT_R8G8B8A8_UNORM;

    pipelineDescription.DSVFormat =
        DXGI_FORMAT_D32_FLOAT;

    pipelineDescription.SampleDesc.Count = 1;
    pipelineDescription.SampleDesc.Quality = 0;

    pipelineDescription.NodeMask = 0;

    pipelineDescription.CachedPSO = {
        nullptr,
        0
    };

    pipelineDescription.Flags =
        D3D12_PIPELINE_STATE_FLAG_NONE;

    ThrowIfFailed(
        device_->CreateGraphicsPipelineState(
            &pipelineDescription,
            IID_PPV_ARGS(&pipelineState_)),
        "Create reference graphics pipeline");

    pipelineState_->SetName(
        L"Reference Geometry Pipeline");
}

void D3D12Context::CreateBackBufferViews()
{
    D3D12_CPU_DESCRIPTOR_HANDLE rtvHandle =
        rtvHeap_->
        GetCPUDescriptorHandleForHeapStart();

    for (std::uint32_t index = 0;
        index < FrameCount;
        ++index)
    {
        ThrowIfFailed(
            swapChain_->GetBuffer(
                index,
                IID_PPV_ARGS(
                    &backBuffers_[index])),
            "Get swap-chain back buffer");

        const std::wstring bufferName =
            L"Back Buffer "
            + std::to_wstring(index);

        backBuffers_[index]->SetName(
            bufferName.c_str());

        device_->CreateRenderTargetView(
            backBuffers_[index].Get(),
            nullptr,
            rtvHandle);

        rtvHandle.ptr += rtvDescriptorSize_;
    }
}

void D3D12Context::UpdateViewportAndScissor(
    std::uint32_t width,
    std::uint32_t height)
{
    viewport_.TopLeftX = 0.0f;
    viewport_.TopLeftY = 0.0f;

    viewport_.Width =
        static_cast<float>(width);

    viewport_.Height =
        static_cast<float>(height);

    viewport_.MinDepth = 0.0f;
    viewport_.MaxDepth = 1.0f;

    scissorRectangle_.left = 0;
    scissorRectangle_.top = 0;

    scissorRectangle_.right =
        static_cast<LONG>(width);

    scissorRectangle_.bottom =
        static_cast<LONG>(height);
}

Microsoft::WRL::ComPtr<ID3D12Resource>
D3D12Context::CreateBuffer(
    std::uint64_t size,
    D3D12_HEAP_TYPE heapType,
    D3D12_RESOURCE_STATES initialState)
{
    D3D12_HEAP_PROPERTIES heapProperties{};

    heapProperties.Type = heapType;
    heapProperties.CreationNodeMask = 1;
    heapProperties.VisibleNodeMask = 1;

    D3D12_RESOURCE_DESC description{};

    description.Dimension =
        D3D12_RESOURCE_DIMENSION_BUFFER;

    description.Width = size;
    description.Height = 1;
    description.DepthOrArraySize = 1;
    description.MipLevels = 1;
    description.Format = DXGI_FORMAT_UNKNOWN;
    description.SampleDesc.Count = 1;

    description.Layout =
        D3D12_TEXTURE_LAYOUT_ROW_MAJOR;

    Microsoft::WRL::ComPtr<ID3D12Resource>
        resource;

    ThrowIfFailed(
        device_->CreateCommittedResource(
            &heapProperties,
            D3D12_HEAP_FLAG_NONE,
            &description,
            initialState,
            nullptr,
            IID_PPV_ARGS(&resource)),
        "Create buffer");

    return resource;
}

void D3D12Context::CreateDepthResources(
    std::uint32_t width,
    std::uint32_t height)
{
    if (!dsvHeap_)
    {
        D3D12_DESCRIPTOR_HEAP_DESC
            heapDescription{};

        heapDescription.Type =
            D3D12_DESCRIPTOR_HEAP_TYPE_DSV;

        heapDescription.NumDescriptors = 1;

        heapDescription.Flags =
            D3D12_DESCRIPTOR_HEAP_FLAG_NONE;

        ThrowIfFailed(
            device_->CreateDescriptorHeap(
                &heapDescription,
                IID_PPV_ARGS(&dsvHeap_)),
            "Create DSV descriptor heap");

        dsvHeap_->SetName(
            L"Main DSV Heap");
    }

    D3D12_HEAP_PROPERTIES heapProperties{};

    heapProperties.Type =
        D3D12_HEAP_TYPE_DEFAULT;

    heapProperties.CPUPageProperty =
        D3D12_CPU_PAGE_PROPERTY_UNKNOWN;

    heapProperties.MemoryPoolPreference =
        D3D12_MEMORY_POOL_UNKNOWN;

    heapProperties.CreationNodeMask = 1;
    heapProperties.VisibleNodeMask = 1;

    D3D12_RESOURCE_DESC resourceDescription{};

    resourceDescription.Dimension =
        D3D12_RESOURCE_DIMENSION_TEXTURE2D;

    resourceDescription.Alignment = 0;
    resourceDescription.Width = width;
    resourceDescription.Height = height;
    resourceDescription.DepthOrArraySize = 1;
    resourceDescription.MipLevels = 1;

    resourceDescription.Format =
        DXGI_FORMAT_D32_FLOAT;

    resourceDescription.SampleDesc.Count = 1;
    resourceDescription.SampleDesc.Quality = 0;

    resourceDescription.Layout =
        D3D12_TEXTURE_LAYOUT_UNKNOWN;

    resourceDescription.Flags =
        D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;

    D3D12_CLEAR_VALUE clearValue{};

    clearValue.Format =
        DXGI_FORMAT_D32_FLOAT;

    clearValue.DepthStencil.Depth = 1.0f;
    clearValue.DepthStencil.Stencil = 0;

    ThrowIfFailed(
        device_->CreateCommittedResource(
            &heapProperties,
            D3D12_HEAP_FLAG_NONE,
            &resourceDescription,
            D3D12_RESOURCE_STATE_DEPTH_WRITE,
            &clearValue,
            IID_PPV_ARGS(&depthBuffer_)),
        "Create depth buffer");

    depthBuffer_->SetName(
        L"Main Depth Buffer");

    D3D12_DEPTH_STENCIL_VIEW_DESC
        viewDescription{};

    viewDescription.Format =
        DXGI_FORMAT_D32_FLOAT;

    viewDescription.ViewDimension =
        D3D12_DSV_DIMENSION_TEXTURE2D;

    viewDescription.Flags =
        D3D12_DSV_FLAG_NONE;

    viewDescription.Texture2D.MipSlice = 0;

    device_->CreateDepthStencilView(
        depthBuffer_.Get(),
        &viewDescription,
        dsvHeap_->
        GetCPUDescriptorHandleForHeapStart());
}

void D3D12Context::CreateCommandObjects()
{
    for (std::uint32_t index = 0;
        index < FrameCount;
        ++index)
    {
        FrameResource& frame =
            frameResources_[index];

        ThrowIfFailed(
            device_->CreateCommandAllocator(
                D3D12_COMMAND_LIST_TYPE_DIRECT,
                IID_PPV_ARGS(
                    &frame.commandAllocator)),
            "Create frame command allocator");

        const std::wstring allocatorName =
            L"Frame "
            + std::to_wstring(index)
            + L" Command Allocator";

        frame.commandAllocator->SetName(
            allocatorName.c_str());

        constexpr std::uint64_t cameraBufferSize =
            AlignConstantBufferSize(
                sizeof(CameraConstants));

        frame.cameraBuffer = CreateBuffer(
            cameraBufferSize,
            D3D12_HEAP_TYPE_UPLOAD,
            D3D12_RESOURCE_STATE_GENERIC_READ);

        const std::wstring cameraName =
            L"Frame "
            + std::to_wstring(index)
            + L" Camera Buffer";

        frame.cameraBuffer->SetName(
            cameraName.c_str());

        void* mappedData = nullptr;

        D3D12_RANGE noCpuReads{
            0,
            0
        };

        ThrowIfFailed(
            frame.cameraBuffer->Map(
                0,
                &noCpuReads,
                &mappedData),
            "Map camera buffer");

        frame.mappedCameraData =
            static_cast<std::uint8_t*>(
                mappedData);
    }

    ThrowIfFailed(
        device_->CreateCommandList(
            0,
            D3D12_COMMAND_LIST_TYPE_DIRECT,
            frameResources_[0]
            .commandAllocator.Get(),
            nullptr,
            IID_PPV_ARGS(&commandList_)),
        "Create command list");

    commandList_->SetName(
        L"Main Graphics Command List");

    ThrowIfFailed(
        commandList_->Close(),
        "Close initial command list");
}

void D3D12Context::CreateGpuTimingResources()
{
    D3D12_QUERY_HEAP_DESC queryDescription{};

    queryDescription.Type =
        D3D12_QUERY_HEAP_TYPE_TIMESTAMP;

    queryDescription.Count =
        FrameCount * 2;

    queryDescription.NodeMask = 0;

    ThrowIfFailed(
        device_->CreateQueryHeap(
            &queryDescription,
            IID_PPV_ARGS(
                &gpuTimestampQueryHeap_)),
        "Create GPU timestamp query heap");

    ThrowIfFailed(
        commandQueue_->GetTimestampFrequency(
            &gpuTimestampFrequency_),
        "Get GPU timestamp frequency");

    for (std::uint32_t index = 0;
        index < FrameCount;
        ++index)
    {
        FrameResource& frame =
            frameResources_[index];

        frame.gpuTimestampReadback =
            CreateBuffer(
                sizeof(std::uint64_t) * 2,
                D3D12_HEAP_TYPE_READBACK,
                D3D12_RESOURCE_STATE_COPY_DEST);

        const std::wstring resourceName =
            L"GPU Timestamp Readback "
            + std::to_wstring(index);

        frame.gpuTimestampReadback->SetName(
            resourceName.c_str());
    }
}

void D3D12Context::CreateFence()
{
    ThrowIfFailed(
        device_->CreateFence(
            0,
            D3D12_FENCE_FLAG_NONE,
            IID_PPV_ARGS(&fence_)),
        "Create fence");

    fence_->SetName(
        L"Main Command Queue Fence");

    fenceEvent_ = CreateEventW(
        nullptr,
        FALSE,
        FALSE,
        nullptr);

    if (fenceEvent_ == nullptr)
    {
        ThrowIfFailed(
            HRESULT_FROM_WIN32(GetLastError()),
            "Create fence event");
    }
}

void D3D12Context::ReadGpuTimingResult(
    FrameResource& frame)
{
    if (!frame.gpuTimingPending)
    {
        return;
    }

    std::array<std::uint64_t, 2>
        timestamps{};

    void* mappedData = nullptr;

    D3D12_RANGE readRange{
        0,
        sizeof(timestamps)
    };

    ThrowIfFailed(
        frame.gpuTimestampReadback->Map(
            0,
            &readRange,
            &mappedData),
        "Map GPU timestamp readback");

    std::memcpy(
        timestamps.data(),
        mappedData,
        sizeof(timestamps));

    D3D12_RANGE noCpuWrites{
        0,
        0
    };

    frame.gpuTimestampReadback->Unmap(
        0,
        &noCpuWrites);

    if (timestamps[1] >= timestamps[0]
        && gpuTimestampFrequency_ != 0)
    {
        const std::uint64_t elapsedTicks =
            timestamps[1] - timestamps[0];

        lastGpuFrameMilliseconds_ =
            static_cast<double>(elapsedTicks)
            * 1000.0
            / static_cast<double>(
                gpuTimestampFrequency_);
    }

    frame.gpuTimingPending = false;
}

void D3D12Context::WaitForFrame(
    FrameResource& frame)
{
    if (frame.fenceValue == 0)
    {
        return;
    }

    WaitForFenceValue(frame.fenceValue);

    frame.fenceValue = 0;
}

void D3D12Context::SignalFrame(
    FrameResource& frame)
{
    const std::uint64_t fenceValue =
        nextFenceValue_++;

    ThrowIfFailed(
        commandQueue_->Signal(
            fence_.Get(),
            fenceValue),
        "Signal frame fence");

    frame.fenceValue = fenceValue;
}

void D3D12Context::WaitForGpu()
{
    if (!commandQueue_
        || !fence_
        || fenceEvent_ == nullptr)
    {
        return;
    }

    const std::uint64_t fenceValue =
        nextFenceValue_++;

    ThrowIfFailed(
        commandQueue_->Signal(
            fence_.Get(),
            fenceValue),
        "Signal command queue fence");

    WaitForFenceValue(fenceValue);

    for (FrameResource& frame : frameResources_)
    {
        frame.fenceValue = 0;
    }
}

void D3D12Context::WaitForFenceValue(
    std::uint64_t fenceValue)
{
    if (fence_->GetCompletedValue() >= fenceValue)
    {
        return;
    }

    ThrowIfFailed(
        fence_->SetEventOnCompletion(
            fenceValue,
            fenceEvent_),
        "Set fence completion event");

    const DWORD waitResult =
        WaitForSingleObject(
            fenceEvent_,
            INFINITE);

    if (waitResult == WAIT_FAILED)
    {
        ThrowIfFailed(
            HRESULT_FROM_WIN32(GetLastError()),
            "Wait for fence event");
    }
}

void D3D12Context::Render(const Camera& camera)
{
    if (!initialized_
        || width_ == 0
        || height_ == 0)
    {
        return;
    }

    CollectRetiredPointTiles();

    FrameResource& currentFrame =
        frameResources_[frameIndex_];

    WaitForFrame(currentFrame);

    ReadGpuTimingResult(currentFrame);

    const auto cpuRenderStart =
        std::chrono::steady_clock::now();

    CameraConstants constants{};

    DirectX::XMStoreFloat4x4(
        &constants.viewProjection,
        DirectX::XMMatrixTranspose(
            camera.GetViewProjectionMatrix()));

    constants.minimumElevation =
        minimumElevation_;

    constants.maximumElevation =
        maximumElevation_;

    constants.pointColorMode =
        static_cast<std::uint32_t>(
            pointColorMode_);

    constants.padding = 0.0f;

    std::memcpy(
        currentFrame.mappedCameraData,
        &constants,
        sizeof(constants));

    ThrowIfFailed(
        currentFrame.commandAllocator->Reset(),
        "Reset frame command allocator");

    ThrowIfFailed(
        commandList_->Reset(
            currentFrame.commandAllocator.Get(),
            pipelineState_.Get()),
        "Reset command list");

    const std::uint32_t timestampStartIndex =
        frameIndex_ * 2;

    const std::uint32_t timestampEndIndex =
        timestampStartIndex + 1;

    commandList_->EndQuery(
        gpuTimestampQueryHeap_.Get(),
        D3D12_QUERY_TYPE_TIMESTAMP,
        timestampStartIndex);

    commandList_->RSSetViewports(
        1,
        &viewport_);

    commandList_->RSSetScissorRects(
        1,
        &scissorRectangle_);

    D3D12_RESOURCE_BARRIER toRenderTarget{};

    toRenderTarget.Type =
        D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;

    toRenderTarget.Transition.pResource =
        backBuffers_[frameIndex_].Get();

    toRenderTarget.Transition.Subresource =
        D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;

    toRenderTarget.Transition.StateBefore =
        D3D12_RESOURCE_STATE_PRESENT;

    toRenderTarget.Transition.StateAfter =
        D3D12_RESOURCE_STATE_RENDER_TARGET;

    commandList_->ResourceBarrier(
        1,
        &toRenderTarget);

    D3D12_CPU_DESCRIPTOR_HANDLE rtvHandle =
        rtvHeap_->GetCPUDescriptorHandleForHeapStart();

    rtvHandle.ptr +=
        static_cast<SIZE_T>(frameIndex_)
        * rtvDescriptorSize_;

    const D3D12_CPU_DESCRIPTOR_HANDLE
        dsvHandle = dsvHeap_->GetCPUDescriptorHandleForHeapStart();

    constexpr std::array<float, 4> clearColor{
        0.025f,
        0.055f,
        0.075f,
        1.0f
    };

    commandList_->OMSetRenderTargets(
        1,
        &rtvHandle,
        FALSE,
        &dsvHandle);

    commandList_->ClearRenderTargetView(
        rtvHandle,
        clearColor.data(),
        0,
        nullptr);

    commandList_->ClearDepthStencilView(
        dsvHandle,
        D3D12_CLEAR_FLAG_DEPTH,
        1.0f,
        0,
        0,
        nullptr);

    commandList_->SetGraphicsRootSignature(
        rootSignature_.Get());

    commandList_->SetGraphicsRootConstantBufferView(
        0,
        currentFrame.cameraBuffer->
        GetGPUVirtualAddress());

    if (!pointTiles_.empty())
    {
        commandList_->IASetPrimitiveTopology(
            D3D_PRIMITIVE_TOPOLOGY_POINTLIST);

        for (const GpuPointTile& tile :
            pointTiles_)
        {
            if (!tile.pointBuffer
                || tile.drawRanges.empty())
            {
                continue;
            }

            commandList_->IASetVertexBuffers(
                0,
                1,
                &tile.pointBufferView);

            for (const PointCloudDrawRange& range :
                tile.drawRanges)
            {
                if (range.pointCount == 0)
                {
                    continue;
                }

                commandList_->DrawInstanced(
                    range.pointCount,
                    1,
                    range.firstPoint,
                    0);
            }
        }
    }

    D3D12_RESOURCE_BARRIER toPresent =
        toRenderTarget;

    toPresent.Transition.StateBefore =
        D3D12_RESOURCE_STATE_RENDER_TARGET;

    toPresent.Transition.StateAfter =
        D3D12_RESOURCE_STATE_PRESENT;

    commandList_->ResourceBarrier(
        1,
        &toPresent);

    commandList_->EndQuery(
        gpuTimestampQueryHeap_.Get(),
        D3D12_QUERY_TYPE_TIMESTAMP,
        timestampEndIndex);

    commandList_->ResolveQueryData(
        gpuTimestampQueryHeap_.Get(),
        D3D12_QUERY_TYPE_TIMESTAMP,
        timestampStartIndex,
        2,
        currentFrame.gpuTimestampReadback.Get(),
        0);

    ThrowIfFailed(
        commandList_->Close(),
        "Close command list");

    ID3D12CommandList* commandLists[]{
        commandList_.Get()
    };

    commandQueue_->ExecuteCommandLists(
        1,
        commandLists);

    const auto cpuRenderEnd =
        std::chrono::steady_clock::now();

    lastCpuRenderMilliseconds_ =
        std::chrono::duration<
        double,
        std::milli>(
            cpuRenderEnd
            - cpuRenderStart).count();

    currentFrame.gpuTimingPending = true;

    ThrowIfFailed(
        swapChain_->Present(1, 0),
        "Present swap-chain buffer");

    SignalFrame(currentFrame);

    frameIndex_ =
        swapChain_->GetCurrentBackBufferIndex();
}

PointCloudTileId D3D12Context::AddPointCloudTile(
    std::span<const GpuPoint> points,
    std::uint32_t initialDrawPointCount)
{
    if (!initialized_)
    {
        throw std::logic_error(
            "D3D12 must be initialized before loading points.");
    }

    WaitForGpu();

    if (points.empty())
    {
        throw std::invalid_argument(
            "Cannot create an empty GPU point tile.");
    }

    if (initialDrawPointCount == 0
        || initialDrawPointCount > points.size())
    {
        throw std::invalid_argument(
            "Initial point draw count is invalid.");
    }

    const std::uint64_t dataSize =
        points.size_bytes();

    if (dataSize
        > std::numeric_limits<UINT>::max())
    {
        throw std::runtime_error(
            "Point buffer exceeds the D3D12 vertex-view size limit.");
    }

    GpuPointTile tile{};

    tile.id =
        nextPointCloudTileId_++;

    tile.pointBuffer =
        CreateBuffer(
            dataSize,
            D3D12_HEAP_TYPE_DEFAULT,
            D3D12_RESOURCE_STATE_COPY_DEST);

    const std::wstring tileName =
        L"Point Cloud Tile "
        + std::to_wstring(
            pointTiles_.size());

    tile.pointBuffer->SetName(
        tileName.c_str());

    Microsoft::WRL::ComPtr<ID3D12Resource>
        uploadBuffer =
        CreateBuffer(
            dataSize,
            D3D12_HEAP_TYPE_UPLOAD,
            D3D12_RESOURCE_STATE_GENERIC_READ);

    uploadBuffer->SetName(
        L"Point Cloud Upload Buffer");

    void* mappedData = nullptr;

    D3D12_RANGE noCpuReads{
        0,
        0
    };

    ThrowIfFailed(
        uploadBuffer->Map(
            0,
            &noCpuReads,
            &mappedData),
        "Map point upload buffer");

    std::memcpy(
        mappedData,
        points.data(),
        dataSize);

    D3D12_RANGE writtenRange{
        0,
        static_cast<SIZE_T>(dataSize)
    };

    uploadBuffer->Unmap(
        0,
        &writtenRange);

    FrameResource& uploadFrame =
        frameResources_[0];

    ThrowIfFailed(
        uploadFrame.commandAllocator->Reset(),
        "Reset point upload allocator");

    ThrowIfFailed(
        commandList_->Reset(
            uploadFrame.commandAllocator.Get(),
            nullptr),
        "Reset point upload command list");

    commandList_->CopyBufferRegion(
        tile.pointBuffer.Get(),
        0,
        uploadBuffer.Get(),
        0,
        dataSize);

    D3D12_RESOURCE_BARRIER barrier{};

    barrier.Type =
        D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;

    barrier.Transition.pResource =
        tile.pointBuffer.Get();

    barrier.Transition.Subresource =
        D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;

    barrier.Transition.StateBefore =
        D3D12_RESOURCE_STATE_COPY_DEST;

    barrier.Transition.StateAfter =
        D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER;

    commandList_->ResourceBarrier(
        1,
        &barrier);

    ThrowIfFailed(
        commandList_->Close(),
        "Close point upload command list");

    ID3D12CommandList* lists[]{
        commandList_.Get()
    };

    commandQueue_->ExecuteCommandLists(
        1,
        lists);

    WaitForGpu();

    tile.pointBufferView.BufferLocation =
        tile.pointBuffer->
        GetGPUVirtualAddress();

    tile.pointBufferView.SizeInBytes =
        static_cast<UINT>(dataSize);

    tile.pointBufferView.StrideInBytes =
        sizeof(GpuPoint);

    tile.uploadedPointCount =
        static_cast<std::uint32_t>(
            points.size());

    tile.drawRanges.push_back({
        0,
        initialDrawPointCount
        });

    tile.selectedPointCount =
        initialDrawPointCount;

    tile.sizeBytes = dataSize;

    if (tile.selectedPointCount
    > std::numeric_limits<std::uint64_t>::max()
        - pointCount_)
    {
        throw std::overflow_error(
            "Resident point-count overflow.");
    }

    if (tile.sizeBytes
    > std::numeric_limits<std::uint64_t>::max()
        - pointBufferSizeBytes_)
    {
        throw std::overflow_error(
            "Resident point-memory overflow.");
    }

    pointCount_ +=
        tile.selectedPointCount;

    pointBufferSizeBytes_ +=
        tile.sizeBytes;

    const PointCloudTileId tileId =
        tile.id;

    pointTiles_.push_back(
        std::move(tile));

    return tileId;
}

bool D3D12Context::SetPointCloudTileDrawRanges(
    PointCloudTileId tileId,
    std::span<
    const PointCloudDrawRange>
    drawRanges)
{
    if (!initialized_)
    {
        throw std::logic_error(
            "D3D12 must be initialized before "
            "updating point draw ranges.");
    }

    const auto tileIterator =
        std::find_if(
            pointTiles_.begin(),
            pointTiles_.end(),
            [tileId](
                const GpuPointTile& tile)
            {
                return tile.id == tileId;
            });

    if (tileIterator == pointTiles_.end())
    {
        return false;
    }

    std::uint64_t selectedPointCount = 0;

    for (const PointCloudDrawRange& range :
        drawRanges)
    {
        if (range.pointCount == 0)
        {
            throw std::invalid_argument(
                "Point draw range cannot be empty.");
        }

        const std::uint64_t rangeEnd =
            static_cast<std::uint64_t>(
                range.firstPoint)
            + range.pointCount;

        if (rangeEnd
            > tileIterator->uploadedPointCount)
        {
            throw std::out_of_range(
                "Point draw range exceeds "
                "the uploaded point buffer.");
        }

        if (range.pointCount
            > std::numeric_limits<std::uint64_t>::max()
            - selectedPointCount)
        {
            throw std::overflow_error(
                "Selected point count overflow.");
        }

        selectedPointCount +=
            range.pointCount;
    }

    // Allocate before changing the active counters.
    std::vector<PointCloudDrawRange>
        newDrawRanges(
            drawRanges.begin(),
            drawRanges.end());

    const std::uint64_t remainingPointCount =
        pointCount_
        - tileIterator->selectedPointCount;

    if (selectedPointCount
        > std::numeric_limits<std::uint64_t>::max()
        - remainingPointCount)
    {
        throw std::overflow_error(
            "Global selected point count overflow.");
    }

    pointCount_ =
        remainingPointCount
        + selectedPointCount;

    tileIterator->drawRanges =
        std::move(newDrawRanges);

    tileIterator->selectedPointCount =
        selectedPointCount;

    return true;
}

bool D3D12Context::RemovePointCloudTile(
    PointCloudTileId tileId)
{
    if (!initialized_)
    {
        throw std::logic_error(
            "D3D12 must be initialized before "
            "removing point-cloud tiles.");
    }

    if (tileId == 0)
    {
        return false;
    }

    const auto tileIterator =
        std::find_if(
            pointTiles_.begin(),
            pointTiles_.end(),
            [tileId](
                const GpuPointTile& tile)
            {
                return tile.id == tileId;
            });

    if (tileIterator
        == pointTiles_.end())
    {
        return false;
    }

    // Allocate vector capacity before changing any state.
    // If allocation fails, the active tile remains untouched.
    retiredPointTiles_.reserve(
        retiredPointTiles_.size() + 1);

    const std::uint64_t releaseFenceValue =
        nextFenceValue_++;

    ThrowIfFailed(
        commandQueue_->Signal(
            fence_.Get(),
            releaseFenceValue),
        "Signal point-tile retirement fence");

    RetiredPointTile retiredTile{};

    retiredTile.id =
        tileIterator->id;

    retiredTile.pointBuffer =
        std::move(
            tileIterator->pointBuffer);

    retiredTile.sizeBytes =
        tileIterator->sizeBytes;

    retiredTile.releaseFenceValue =
        releaseFenceValue;

    pointCount_ -=
        tileIterator->selectedPointCount;

    retiredPointTiles_.push_back(
        std::move(retiredTile));

    pointTiles_.erase(
        tileIterator);

    return true;
}

void D3D12Context::CollectRetiredPointTiles()
{
    if (retiredPointTiles_.empty()
        || !fence_)
    {
        return;
    }

    const std::uint64_t completedFenceValue =
        fence_->GetCompletedValue();

    const auto firstRemainingTile =
        std::remove_if(
            retiredPointTiles_.begin(),
            retiredPointTiles_.end(),
            [this, completedFenceValue](
                const RetiredPointTile& tile)
            {
                if (tile.releaseFenceValue
                    > completedFenceValue)
                {
                    return false;
                }

                pointBufferSizeBytes_ -=
                    tile.sizeBytes;

                return true;
            });

    retiredPointTiles_.erase(
        firstRemainingTile,
        retiredPointTiles_.end());
}

void D3D12Context::ClearPointCloudTiles()
{
    if (!initialized_)
    {
        throw std::logic_error(
            "D3D12 must be initialized before "
            "clearing point-cloud tiles.");
    }

    WaitForGpu();

    pointTiles_.clear();
    retiredPointTiles_.clear();

    pointCount_ = 0;
    pointBufferSizeBytes_ = 0;
}

void D3D12Context::Resize(
    std::uint32_t width,
    std::uint32_t height)
{
    if (!initialized_
        || width == 0
        || height == 0)
    {
        return;
    }

    if (width == width_
        && height == height_)
    {
        return;
    }

    WaitForGpu();

    for (auto& backBuffer : backBuffers_)
    {
        backBuffer.Reset();
    }

    depthBuffer_.Reset();

    ThrowIfFailed(
        swapChain_->ResizeBuffers(
            FrameCount,
            width,
            height,
            DXGI_FORMAT_R8G8B8A8_UNORM,
            0),
        "Resize swap-chain buffers");

    frameIndex_ =
        swapChain_->GetCurrentBackBufferIndex();

    width_ = width;
    height_ = height;

    CreateBackBufferViews();
    CreateDepthResources(width, height);
    UpdateViewportAndScissor(width, height);
}

void D3D12Context::Shutdown() noexcept
{
    if (!initialized_ && !device_)
    {
        return;
    }

    if (initialized_)
    {
        try
        {
            WaitForGpu();
        }
        catch (...)
        {
            // Destruction cannot safely propagate an exception.
        }
    }

    initialized_ = false;

    if (fenceEvent_ != nullptr)
    {
        CloseHandle(fenceEvent_);
        fenceEvent_ = nullptr;
    }

    for (auto& backBuffer : backBuffers_)
    {
        backBuffer.Reset();
    }

    depthBuffer_.Reset();

    pointTiles_.clear();
    retiredPointTiles_.clear();
    pointCount_ = 0;
    pointBufferSizeBytes_ = 0;

    commandList_.Reset();

    for (FrameResource& frame :
        frameResources_)
    {
        if (frame.cameraBuffer
            && frame.mappedCameraData)
        {
            frame.cameraBuffer->Unmap(
                0,
                nullptr);

            frame.mappedCameraData = nullptr;
        }

        frame.gpuTimestampReadback.Reset();
        frame.gpuTimingPending = false;

        frame.cameraBuffer.Reset();
        frame.commandAllocator.Reset();
        frame.fenceValue = 0;
    }

    gpuTimestampQueryHeap_.Reset();
    gpuTimestampFrequency_ = 0;
    lastGpuFrameMilliseconds_ = 0.0;

    pointBufferSizeBytes_ = 0;
    lastCpuRenderMilliseconds_ = 0.0;

    pipelineState_.Reset();
    rootSignature_.Reset();

    fence_.Reset();
    dsvHeap_.Reset();
    rtvHeap_.Reset();
    swapChain_.Reset();
    commandQueue_.Reset();
    device_.Reset();
    adapter_.Reset();
    factory_.Reset();

#if defined(_DEBUG)

    Microsoft::WRL::ComPtr<IDXGIDebug1>
        dxgiDebug;

    if (SUCCEEDED(
        DXGIGetDebugInterface1(
            0,
            IID_PPV_ARGS(&dxgiDebug))))
    {
        const auto reportFlags =
            static_cast<DXGI_DEBUG_RLO_FLAGS>(
                DXGI_DEBUG_RLO_SUMMARY
                | DXGI_DEBUG_RLO_IGNORE_INTERNAL);

        dxgiDebug->ReportLiveObjects(
            DXGI_DEBUG_ALL,
            reportFlags);
    }

#endif
}

void D3D12Context::SetPointColorMode(
    PointColorMode mode) noexcept
{
    pointColorMode_ = mode;
}

PointColorMode
D3D12Context::GetPointColorMode() const noexcept
{
    return pointColorMode_;
}

void D3D12Context::SetElevationRange(
    float minimumElevation,
    float maximumElevation)
{
    if (!std::isfinite(minimumElevation)
        || !std::isfinite(maximumElevation)
        || minimumElevation > maximumElevation)
    {
        throw std::invalid_argument(
            "Point-cloud elevation range is invalid.");
    }

    minimumElevation_ =
        minimumElevation;

    maximumElevation_ =
        maximumElevation;
}

D3D12Context::~D3D12Context()
{
    Shutdown();
}
