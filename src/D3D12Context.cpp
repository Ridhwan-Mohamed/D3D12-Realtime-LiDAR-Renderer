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
#include <cstddef>

#include "Camera.h"
#include "ReferenceVS.h"
#include "ReferencePS.h"
#include "ClodReduceCS.h"
#include "ClodDrawVS.h"
#include "ClodDrawPS.h"
#include "ReducedClodPoint.h"

namespace
{
    struct CameraConstants
    {
        DirectX::XMFLOAT4X4 viewProjection;

        float minimumElevation = 0.0f;
        float maximumElevation = 0.0f;

        std::uint32_t pointColorMode = 0;

        // Same density factor used by CLOD reduction.
        float clodFactor = 1.0f;

        DirectX::XMFLOAT2 viewportSize{};

        float clodPointSizePixels = 1.0f;
        float clodMinimumPointSizePixels = 1.0f;

        DirectX::XMFLOAT3 cameraPosition{};

        float clodBlendRangeFactor = 0.8f;

        float clodPeripheralDensityFalloff = 0.5f;
        float clodMinimumPeripheralDensity = 0.3f;
        DirectX::XMFLOAT2 clodPeripheralPadding{};
    };

    static_assert(
        sizeof(CameraConstants) == 128,
        "CameraConstants must match the HLSL layout.");

    struct ClodReductionConstants
    {
        // Range within the currently bound tile.
        std::uint32_t sourceFirstPoint = 0;
        std::uint32_t sourcePointCount = 0;

        std::uint32_t outputCapacity = 0;
        float rootSpacing = 0.0f;

        float clodFactor = 1.0f;
        DirectX::XMFLOAT3 cameraPosition{};

        float frustumExpansion = 2.0f;
        float peripheralDensityFalloff = 0.5f;
        float minimumPeripheralDensity = 0.3f;
        float padding = 0.0f;

        DirectX::XMFLOAT4X4 viewProjection{};
    };

    static_assert(
        sizeof(ClodReductionConstants) == 112,
        "ClodReductionConstants must match the HLSL layout.");

    static_assert(
        sizeof(ClodReductionConstants)
        / sizeof(std::uint32_t)
        == 28,
        "CLOD root-signature constant count changed.");

    struct ClodReductionControl
    {
        D3D12_DRAW_ARGUMENTS drawArguments{
            6, // Six vertices form one quad.
            0, // Compute increments one instance per accepted point.
            0, // StartVertexLocation
            0  // StartInstanceLocation
        };

        std::uint32_t overflowPointCount = 0;
    };

    static_assert(
        offsetof(
            ClodReductionControl,
            drawArguments)
        == 0);

    static_assert(
        sizeof(ClodReductionControl)
        == sizeof(D3D12_DRAW_ARGUMENTS)
        + sizeof(std::uint32_t));

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
    std::uint32_t height,
    std::uint32_t clodOutputCapacity)
{
    EnableDebugLayer();
    CreateFactory();
    SelectAdapter();
    CreateDevice();
    ConfigureValidation();
    CreateCommandQueue();
    CreateCopyQueue();
    CreateSwapChain(window, width, height);
    CreateRenderTargets();
    CreateDepthResources(width, height);
    UpdateViewportAndScissor(width, height);
    CreateGraphicsPipeline();
    CreateClodReductionPipeline();
    CreateClodReductionResources(clodOutputCapacity);
    CreateCommandObjects();
    CreateGpuTimingResources();
    CreateFence();
    CreateCopyFence();

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

void D3D12Context::CreateCopyQueue()
{
    D3D12_COMMAND_QUEUE_DESC description{};

    description.Type =
        D3D12_COMMAND_LIST_TYPE_COPY;

    description.Priority =
        D3D12_COMMAND_QUEUE_PRIORITY_NORMAL;

    description.Flags =
        D3D12_COMMAND_QUEUE_FLAG_NONE;

    description.NodeMask = 0;

    ThrowIfFailed(
        device_->CreateCommandQueue(
            &description,
            IID_PPV_ARGS(&copyQueue_)),
        "Create copy command queue");

    copyQueue_->SetName(
        L"Point Cloud Copy Queue");
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

    constexpr D3D12_INPUT_ELEMENT_DESC
        clodInputElements[]{
        {
            "CORNER",
            0,
            DXGI_FORMAT_R32G32_FLOAT,
            0,
            0,
            D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA,
            0
        },
        {
            "POSITION",
            0,
            DXGI_FORMAT_R32G32B32_FLOAT,
            1,
            0,
            D3D12_INPUT_CLASSIFICATION_PER_INSTANCE_DATA,
            1
        },
        {
            "COLOR",
            0,
            DXGI_FORMAT_R8G8B8A8_UNORM,
            1,
            12,
            D3D12_INPUT_CLASSIFICATION_PER_INSTANCE_DATA,
            1
        },
        {
            "POINT_SPACING",
            0,
            DXGI_FORMAT_R32_FLOAT,
            1,
            16,
            D3D12_INPUT_CLASSIFICATION_PER_INSTANCE_DATA,
            1
        }
    };

    D3D12_GRAPHICS_PIPELINE_STATE_DESC
        clodPipelineDescription =
        pipelineDescription;

    clodPipelineDescription.VS = {
        g_clodDrawVertexShader,
        sizeof(g_clodDrawVertexShader)
    };

    clodPipelineDescription.PS = {
        g_clodDrawPixelShader,
        sizeof(g_clodDrawPixelShader)
    };

    clodPipelineDescription.InputLayout = {
        clodInputElements,
        static_cast<UINT>(
            std::size(clodInputElements))
    };

    clodPipelineDescription.PrimitiveTopologyType =
        D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;

    ThrowIfFailed(
        device_->CreateGraphicsPipelineState(
            &clodPipelineDescription,
            IID_PPV_ARGS(
                &clodDrawPipelineState_)),
        "Create CLOD splat graphics pipeline");

    clodDrawPipelineState_->SetName(
        L"CLOD Splat Graphics Pipeline");

    constexpr std::array<
        DirectX::XMFLOAT2,
        6>
        quadCorners{
            DirectX::XMFLOAT2{ -0.5f, -0.5f },
            DirectX::XMFLOAT2{ -0.5f,  0.5f },
            DirectX::XMFLOAT2{  0.5f,  0.5f },

            DirectX::XMFLOAT2{ -0.5f, -0.5f },
            DirectX::XMFLOAT2{  0.5f,  0.5f },
            DirectX::XMFLOAT2{  0.5f, -0.5f }
    };

    constexpr std::uint64_t quadBufferSize =
        sizeof(quadCorners);

    clodQuadVertexBuffer_ =
        CreateBuffer(
            quadBufferSize,
            D3D12_HEAP_TYPE_UPLOAD,
            D3D12_RESOURCE_STATE_GENERIC_READ);

    clodQuadVertexBuffer_->SetName(
        L"CLOD Shared Quad Vertex Buffer");

    void* mappedQuadData = nullptr;

    D3D12_RANGE noCpuReads{
        0,
        0
    };

    ThrowIfFailed(
        clodQuadVertexBuffer_->Map(
            0,
            &noCpuReads,
            &mappedQuadData),
        "Map CLOD quad vertex buffer");

    std::memcpy(
        mappedQuadData,
        quadCorners.data(),
        sizeof(quadCorners));

    D3D12_RANGE writtenQuadRange{
        0,
        sizeof(quadCorners)
    };

    clodQuadVertexBuffer_->Unmap(
        0,
        &writtenQuadRange);

    clodQuadVertexBufferView_.BufferLocation =
        clodQuadVertexBuffer_->
        GetGPUVirtualAddress();

    clodQuadVertexBufferView_.SizeInBytes =
        static_cast<UINT>(
            sizeof(quadCorners));

    clodQuadVertexBufferView_.StrideInBytes =
        sizeof(DirectX::XMFLOAT2);
}

void D3D12Context::
CreateClodReductionPipeline()
{
    // Root parameter 0:
    //
    // One SRV descriptor at t0 for the current
    // resident tile's CLOD source buffer.
    D3D12_DESCRIPTOR_RANGE sourceRange{};

    sourceRange.RangeType =
        D3D12_DESCRIPTOR_RANGE_TYPE_SRV;

    sourceRange.NumDescriptors = 1;
    sourceRange.BaseShaderRegister = 0;
    sourceRange.RegisterSpace = 0;

    sourceRange.OffsetInDescriptorsFromTableStart =
        D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

    // Root parameter 1:
    //
    // Two contiguous UAV descriptors:
    //
    // u0 = reduced point output
    // u1 = accepted/overflow counters
    D3D12_DESCRIPTOR_RANGE outputRanges{};

    outputRanges.RangeType =
        D3D12_DESCRIPTOR_RANGE_TYPE_UAV;

    outputRanges.NumDescriptors = 2;
    outputRanges.BaseShaderRegister = 0;
    outputRanges.RegisterSpace = 0;

    outputRanges.OffsetInDescriptorsFromTableStart =
        D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;

    std::array<D3D12_ROOT_PARAMETER, 3>
        rootParameters{};

    rootParameters[0].ParameterType =
        D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;

    rootParameters[0].
        DescriptorTable.NumDescriptorRanges = 1;

    rootParameters[0].
        DescriptorTable.pDescriptorRanges =
        &sourceRange;

    rootParameters[0].ShaderVisibility =
        D3D12_SHADER_VISIBILITY_ALL;

    rootParameters[1].ParameterType =
        D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;

    rootParameters[1].
        DescriptorTable.NumDescriptorRanges = 1;

    rootParameters[1].
        DescriptorTable.pDescriptorRanges =
        &outputRanges;

    rootParameters[1].ShaderVisibility =
        D3D12_SHADER_VISIBILITY_ALL;

    // Root parameter 2:
    //
    // Four uint values mapped directly onto b0.
    //
    // This avoids allocating a constant-buffer resource
    // for a tiny per-dispatch structure.
    rootParameters[2].ParameterType =
        D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;

    rootParameters[2].
        Constants.ShaderRegister = 0;

    rootParameters[2].
        Constants.RegisterSpace = 0;

    rootParameters[2].
        Constants.Num32BitValues =
        sizeof(ClodReductionConstants)
        / sizeof(std::uint32_t);

    rootParameters[2].ShaderVisibility =
        D3D12_SHADER_VISIBILITY_ALL;

    D3D12_ROOT_SIGNATURE_DESC
        rootDescription{};

    rootDescription.NumParameters =
        static_cast<UINT>(
            rootParameters.size());

    rootDescription.pParameters =
        rootParameters.data();

    rootDescription.NumStaticSamplers = 0;
    rootDescription.pStaticSamplers = nullptr;

    rootDescription.Flags =
        D3D12_ROOT_SIGNATURE_FLAG_NONE;

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
        "Serialize CLOD reduction root signature");

    ThrowIfFailed(
        device_->CreateRootSignature(
            0,
            serializedRootSignature->
            GetBufferPointer(),
            serializedRootSignature->
            GetBufferSize(),
            IID_PPV_ARGS(
                &clodReductionRootSignature_)),
        "Create CLOD reduction root signature");

    clodReductionRootSignature_->SetName(
        L"CLOD Reduction Root Signature");

    D3D12_COMPUTE_PIPELINE_STATE_DESC
        pipelineDescription{};

    pipelineDescription.pRootSignature =
        clodReductionRootSignature_.Get();

    pipelineDescription.CS = {
        g_clodReduceComputeShader,
        sizeof(g_clodReduceComputeShader)
    };

    pipelineDescription.NodeMask = 0;
    pipelineDescription.Flags =
        D3D12_PIPELINE_STATE_FLAG_NONE;

    ThrowIfFailed(
        device_->CreateComputePipelineState(
            &pipelineDescription,
            IID_PPV_ARGS(
                &clodReductionPipelineState_)),
        "Create CLOD reduction pipeline state");

    clodReductionPipelineState_->SetName(
        L"CLOD Reduction Pipeline State");

    D3D12_INDIRECT_ARGUMENT_DESC
        indirectArgument{};

    indirectArgument.Type =
        D3D12_INDIRECT_ARGUMENT_TYPE_DRAW;

    D3D12_COMMAND_SIGNATURE_DESC
        commandSignatureDescription{};

    commandSignatureDescription.ByteStride =
        sizeof(D3D12_DRAW_ARGUMENTS);

    commandSignatureDescription.NumArgumentDescs = 1;

    commandSignatureDescription.pArgumentDescs =
        &indirectArgument;

    commandSignatureDescription.NodeMask = 0;

    ThrowIfFailed(
        device_->CreateCommandSignature(
            &commandSignatureDescription,
            nullptr,
            IID_PPV_ARGS(
                &clodDrawCommandSignature_)),
        "Create CLOD draw command signature");

    clodDrawCommandSignature_->SetName(
        L"CLOD Draw Command Signature");
}

D3D12_CPU_DESCRIPTOR_HANDLE
D3D12Context::GetClodCpuDescriptorHandle(
    std::uint32_t descriptorIndex) const noexcept
{
    D3D12_CPU_DESCRIPTOR_HANDLE handle =
        clodDescriptorHeap_->
        GetCPUDescriptorHandleForHeapStart();

    handle.ptr +=
        static_cast<SIZE_T>(
            descriptorIndex)
        * clodDescriptorSize_;

    return handle;
}

D3D12_GPU_DESCRIPTOR_HANDLE
D3D12Context::GetClodGpuDescriptorHandle(
    std::uint32_t descriptorIndex) const noexcept
{
    D3D12_GPU_DESCRIPTOR_HANDLE handle =
        clodDescriptorHeap_->
        GetGPUDescriptorHandleForHeapStart();

    handle.ptr +=
        static_cast<UINT64>(
            descriptorIndex)
        * clodDescriptorSize_;

    return handle;
}

void D3D12Context::
ResetClodTileDescriptorAllocator()
{
    freeClodTileDescriptorIndices_.clear();

    freeClodTileDescriptorIndices_.reserve(
        MaximumClodTileDescriptors);

    // Push in reverse so pop_back() returns the lowest
    // available descriptor first.
    for (std::uint32_t descriptorIndex =
        ClodDescriptorCount;
        descriptorIndex
            > ClodTileDescriptorStart;
        --descriptorIndex)
    {
        freeClodTileDescriptorIndices_.push_back(
            descriptorIndex - 1);
    }
}

std::uint32_t
D3D12Context::AllocateClodTileDescriptor()
{
    if (freeClodTileDescriptorIndices_.empty())
    {
        throw std::runtime_error(
            "CLOD tile descriptor heap exhausted.");
    }

    const std::uint32_t descriptorIndex =
        freeClodTileDescriptorIndices_.back();

    freeClodTileDescriptorIndices_.pop_back();

    return descriptorIndex;
}

void D3D12Context::ReleaseClodTileDescriptor(
    std::uint32_t descriptorIndex)
{
    if (descriptorIndex
        < ClodTileDescriptorStart
        || descriptorIndex
        >= ClodDescriptorCount)
    {
        throw std::logic_error(
            "Invalid CLOD tile descriptor index.");
    }

    const auto existing =
        std::find(
            freeClodTileDescriptorIndices_.begin(),
            freeClodTileDescriptorIndices_.end(),
            descriptorIndex);

    if (existing
        != freeClodTileDescriptorIndices_.end())
    {
        throw std::logic_error(
            "CLOD tile descriptor was released twice.");
    }

    freeClodTileDescriptorIndices_.push_back(
        descriptorIndex);
}

void D3D12Context::CreateClodTileSourceView(
    GpuPointTile& tile)
{
    if (!tile.clodSourceBuffer)
    {
        throw std::logic_error(
            "Cannot create a CLOD SRV without "
            "a source buffer.");
    }

    if (tile.uploadedClodPointCount == 0)
    {
        throw std::logic_error(
            "Cannot create a CLOD SRV for an "
            "empty source buffer.");
    }

    if (tile.clodSrvDescriptorIndex
        == InvalidClodDescriptorIndex)
    {
        throw std::logic_error(
            "CLOD tile has no allocated "
            "descriptor index.");
    }

    D3D12_SHADER_RESOURCE_VIEW_DESC
        sourceViewDescription{};

    // DXGI_FORMAT_UNKNOWN plus a nonzero structure
    // stride creates a structured-buffer SRV.
    sourceViewDescription.Format =
        DXGI_FORMAT_UNKNOWN;

    sourceViewDescription.ViewDimension =
        D3D12_SRV_DIMENSION_BUFFER;

    sourceViewDescription.Shader4ComponentMapping =
        D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;

    sourceViewDescription.Buffer.FirstElement = 0;

    sourceViewDescription.Buffer.NumElements =
        tile.uploadedClodPointCount;

    sourceViewDescription.Buffer.StructureByteStride =
        sizeof(ClodPoint);

    sourceViewDescription.Buffer.Flags =
        D3D12_BUFFER_SRV_FLAG_NONE;

    device_->CreateShaderResourceView(
        tile.clodSourceBuffer.Get(),
        &sourceViewDescription,
        GetClodCpuDescriptorHandle(
            tile.clodSrvDescriptorIndex));
}

void D3D12Context::
CreateClodReductionResources(
    std::uint32_t outputCapacity)
{
    if (outputCapacity == 0)
    {
        throw std::invalid_argument(
            "CLOD output capacity must be positive.");
    }

    if (outputCapacity
        > std::numeric_limits<std::uint64_t>::max()
        / sizeof(ReducedClodPoint))
    {
        throw std::overflow_error(
            "CLOD output-buffer size overflow.");
    }

    const std::uint64_t outputBufferSize =
        static_cast<std::uint64_t>(
            outputCapacity)
        * sizeof(ReducedClodPoint);

    constexpr std::uint64_t counterBufferSize =
        sizeof(ClodReductionControl);

    D3D12_DESCRIPTOR_HEAP_DESC
        heapDescription{};

    heapDescription.Type =
        D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;

    heapDescription.NumDescriptors =
        ClodDescriptorCount;

    heapDescription.Flags =
        D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;

    heapDescription.NodeMask = 0;

    ThrowIfFailed(
        device_->CreateDescriptorHeap(
            &heapDescription,
            IID_PPV_ARGS(
                &clodDescriptorHeap_)),
        "Create CLOD descriptor heap");

    clodDescriptorHeap_->SetName(
        L"CLOD Descriptor Heap");

    clodDescriptorSize_ =
        device_->
        GetDescriptorHandleIncrementSize(
            D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

    // Persistent zero-filled source used to reset either
    // slot's counter buffer through CopyBufferRegion.
    clodCounterResetUpload_ =
        CreateBuffer(
            counterBufferSize,
            D3D12_HEAP_TYPE_UPLOAD,
            D3D12_RESOURCE_STATE_GENERIC_READ);

    clodCounterResetUpload_->SetName(
        L"CLOD Counter Reset Upload");

    void* mappedResetData = nullptr;

    D3D12_RANGE noCpuReads{
        0,
        0
    };

    ThrowIfFailed(
        clodCounterResetUpload_->Map(
            0,
            &noCpuReads,
            &mappedResetData),
        "Map CLOD counter reset upload");

    const ClodReductionControl
        resetControl{};

    std::memcpy(
        mappedResetData,
        &resetControl,
        sizeof(resetControl));

    D3D12_RANGE writtenRange{
        0,
        sizeof(resetControl)
    };

    clodCounterResetUpload_->Unmap(
        0,
        &writtenRange);

    for (std::uint32_t slotIndex = 0;
        slotIndex < ClodReductionSlotCount;
        ++slotIndex)
    {
        ClodReductionSlot& slot =
            clodReductionSlots_[slotIndex];

        slot.capacity =
            outputCapacity;

        slot.outputUavDescriptorIndex =
            slotIndex
            * ClodDescriptorsPerReductionSlot;

        slot.counterUavDescriptorIndex =
            slot.outputUavDescriptorIndex + 1;

        slot.pointBuffer =
            CreateBuffer(
                outputBufferSize,
                D3D12_HEAP_TYPE_DEFAULT,
                D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);

        slot.counterBuffer =
            CreateBuffer(
                counterBufferSize,
                D3D12_HEAP_TYPE_DEFAULT,
                D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);

        slot.counterReadback =
            CreateBuffer(
                counterBufferSize,
                D3D12_HEAP_TYPE_READBACK,
                D3D12_RESOURCE_STATE_COPY_DEST);

        const std::wstring pointBufferName =
            L"CLOD Reduced Point Buffer "
            + std::to_wstring(slotIndex);

        const std::wstring counterBufferName =
            L"CLOD Reduction Counter Buffer "
            + std::to_wstring(slotIndex);

        const std::wstring readbackName =
            L"CLOD Counter Readback "
            + std::to_wstring(slotIndex);

        slot.pointBuffer->SetName(
            pointBufferName.c_str());

        slot.counterBuffer->SetName(
            counterBufferName.c_str());

        slot.counterReadback->SetName(
            readbackName.c_str());

        D3D12_UNORDERED_ACCESS_VIEW_DESC
            pointUavDescription{};

        pointUavDescription.Format =
            DXGI_FORMAT_UNKNOWN;

        pointUavDescription.ViewDimension =
            D3D12_UAV_DIMENSION_BUFFER;

        pointUavDescription.Buffer.FirstElement = 0;

        pointUavDescription.Buffer.NumElements =
            outputCapacity;

        pointUavDescription.Buffer.StructureByteStride =
            sizeof(ReducedClodPoint);

        pointUavDescription.Buffer.CounterOffsetInBytes =
            0;

        pointUavDescription.Buffer.Flags =
            D3D12_BUFFER_UAV_FLAG_NONE;

        device_->CreateUnorderedAccessView(
            slot.pointBuffer.Get(),
            nullptr,
            &pointUavDescription,
            GetClodCpuDescriptorHandle(
                slot.outputUavDescriptorIndex));

        D3D12_UNORDERED_ACCESS_VIEW_DESC
            counterUavDescription{};

        // RWByteAddressBuffer requires a raw UAV.
        counterUavDescription.Format =
            DXGI_FORMAT_R32_TYPELESS;

        counterUavDescription.ViewDimension =
            D3D12_UAV_DIMENSION_BUFFER;

        counterUavDescription.Buffer.FirstElement = 0;

        counterUavDescription.Buffer.NumElements =
            static_cast<UINT>(
                counterBufferSize
                / sizeof(std::uint32_t));

        counterUavDescription.Buffer.StructureByteStride =
            0;

        counterUavDescription.Buffer.CounterOffsetInBytes =
            0;

        counterUavDescription.Buffer.Flags =
            D3D12_BUFFER_UAV_FLAG_RAW;

        device_->CreateUnorderedAccessView(
            slot.counterBuffer.Get(),
            nullptr,
            &counterUavDescription,
            GetClodCpuDescriptorHandle(
                slot.counterUavDescriptorIndex));
    }

    ResetClodTileDescriptorAllocator();
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
    D3D12_RESOURCE_STATES initialState,
    D3D12_RESOURCE_FLAGS flags)
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

    description.Flags =
        flags;

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

void D3D12Context::CreateCopyFence()
{
    ThrowIfFailed(
        device_->CreateFence(
            0,
            D3D12_FENCE_FLAG_NONE,
            IID_PPV_ARGS(&copyFence_)),
        "Create copy fence");

    copyFence_->SetName(
        L"Point Cloud Copy Fence");

    copyFenceEvent_ = CreateEventW(
        nullptr,
        FALSE,
        FALSE,
        nullptr);

    if (copyFenceEvent_ == nullptr)
    {
        ThrowIfFailed(
            HRESULT_FROM_WIN32(GetLastError()),
            "Create copy fence event");
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

void D3D12Context::WaitForCopyFenceValue(
    std::uint64_t fenceValue)
{
    if (fenceValue == 0
        || copyFence_->GetCompletedValue()
        >= fenceValue)
    {
        return;
    }

    ThrowIfFailed(
        copyFence_->SetEventOnCompletion(
            fenceValue,
            copyFenceEvent_),
        "Set copy fence completion event");

    const DWORD waitResult =
        WaitForSingleObject(
            copyFenceEvent_,
            INFINITE);

    if (waitResult == WAIT_FAILED)
    {
        ThrowIfFailed(
            HRESULT_FROM_WIN32(GetLastError()),
            "Wait for copy fence event");
    }
}

void D3D12Context::WaitForPendingPointUploads()
{
    std::uint64_t latestFenceValue = 0;

    for (const PendingPointTileUpload& upload :
        pendingPointTileUploads_)
    {
        latestFenceValue = std::max(
            latestFenceValue,
            upload.copyFenceValue);
    }

    for (const PendingClodTileUpload& upload :
        pendingClodTileUploads_)
    {
        latestFenceValue = std::max(
            latestFenceValue,
            upload.copyFenceValue);
    }

    if (latestFenceValue != 0)
    {
        WaitForCopyFenceValue(
            latestFenceValue);
    }
}

void D3D12Context::
RecordPointTileUploadTransitions()
{
    for (GpuPointTile& tile :
        pointTiles_)
    {
        std::array<
            D3D12_RESOURCE_BARRIER,
            2>
            barriers{};

        UINT barrierCount = 0;

        if (tile.needsCopyToVertexTransition
            && tile.pointBuffer)
        {
            D3D12_RESOURCE_BARRIER& barrier =
                barriers[barrierCount++];

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

            tile.needsCopyToVertexTransition =
                false;
        }

        if (tile.needsCopyToClodSourceTransition
            && tile.clodSourceBuffer)
        {
            D3D12_RESOURCE_BARRIER& barrier =
                barriers[barrierCount++];

            barrier.Type =
                D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;

            barrier.Transition.pResource =
                tile.clodSourceBuffer.Get();

            barrier.Transition.Subresource =
                D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;

            barrier.Transition.StateBefore =
                D3D12_RESOURCE_STATE_COPY_DEST;

            barrier.Transition.StateAfter =
                D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;

            tile.needsCopyToClodSourceTransition =
                false;
        }

        if (barrierCount != 0)
        {
            commandList_->ResourceBarrier(
                barrierCount,
                barriers.data());
        }
    }
}

std::optional<std::uint32_t>
D3D12Context::RecordClodReduction(
    const Camera& camera)
{
    constexpr std::uint32_t threadsPerGroup = 256;

    static_assert(
        ClodReductionSourcePointBudgetPerFrame
        <= std::numeric_limits<std::uint32_t>::max());

    bool startedCycleThisFrame = false;

    if (!clodReductionCycle_.IsActive())
    {
        std::vector<ClodReductionSource> sources;
        std::uint64_t totalSourcePointCount = 0;

        for (const GpuPointTile& tile : pointTiles_)
        {
            if (!tile.clodSourceBuffer
                || tile.uploadedClodPointCount == 0
                || tile.clodSrvDescriptorIndex
                == InvalidClodDescriptorIndex)
            {
                continue;
            }

            sources.push_back({
                tile.id,
                tile.uploadedClodPointCount
                });

            totalSourcePointCount +=
                tile.uploadedClodPointCount;
        }

        if (sources.empty())
        {
            return std::nullopt;
        }

        std::optional<std::uint32_t> selectedSlotIndex;

        for (std::uint32_t offset = 0;
            offset < ClodReductionSlotCount;
            ++offset)
        {
            const std::uint32_t slotIndex =
                (
                    nextClodReductionSlotIndex_
                    + offset
                    )
                % ClodReductionSlotCount;

            // Never overwrite the complete reduction currently
            // being displayed.
            if (activeClodReductionSlotIndex_
                && slotIndex
                == *activeClodReductionSlotIndex_)
            {
                continue;
            }

            const ClodReductionSlot& candidate =
                clodReductionSlots_[slotIndex];

            // A nonzero fence means the GPU result has not been
            // collected yet.
            if (candidate.completionFenceValue == 0)
            {
                selectedSlotIndex = slotIndex;
                break;
            }
        }

        // No output slot is currently available. Continue drawing
        // the previous completed result without blocking the CPU.
        if (!selectedSlotIndex)
        {
            return std::nullopt;
        }

        clodReductionCycle_.Reset();

        clodReductionCycle_.outputSlotIndex =
            *selectedSlotIndex;

        clodReductionCycle_.sources =
            std::move(sources);

        clodReductionCycle_.totalSourcePointCount =
            totalSourcePointCount;

        clodReductionCycle_.sourceGeneration =
            clodSourceGeneration_;

        clodReductionCycle_.cameraPosition =
            camera.GetPosition();

        DirectX::XMStoreFloat4x4(
            &clodReductionCycle_.viewProjection,
            DirectX::XMMatrixTranspose(
                camera.GetViewProjectionMatrix()));

        clodReductionCycle_.clodFactor =
            clodFactor_;

        clodReductionCycle_.frustumExpansion =
            2.0f;

        clodReductionCycle_.peripheralDensityFalloff =
            0.5f;

        clodReductionCycle_.minimumPeripheralDensity =
            0.3f;

        ClodReductionSlot& newSlot =
            clodReductionSlots_[*selectedSlotIndex];

        newSlot.expectedCandidateCount =
            totalSourcePointCount;

        newSlot.hasValidResult = false;

        startedCycleThisFrame = true;
    }

    const std::uint32_t selectedSlotIndex =
        *clodReductionCycle_.outputSlotIndex;

    ClodReductionSlot& slot =
        clodReductionSlots_[selectedSlotIndex];

    auto transitionResource =
        [this](
            ID3D12Resource* resource,
            D3D12_RESOURCE_STATES& currentState,
            D3D12_RESOURCE_STATES newState)
        {
            if (currentState == newState)
            {
                return;
            }

            D3D12_RESOURCE_BARRIER barrier{};

            barrier.Type =
                D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;

            barrier.Transition.pResource = resource;

            barrier.Transition.Subresource =
                D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;

            barrier.Transition.StateBefore =
                currentState;

            barrier.Transition.StateAfter =
                newState;

            commandList_->ResourceBarrier(
                1,
                &barrier);

            currentState = newState;
        };

    // The output slot remains in UAV state while several frames
    // append points into it.
    transitionResource(
        slot.pointBuffer.Get(),
        slot.pointBufferState,
        D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

    if (startedCycleThisFrame)
    {
        // Counters are reset once at the beginning of the entire
        // multi-frame reduction, not at the beginning of every
        // frame.
        transitionResource(
            slot.counterBuffer.Get(),
            slot.counterBufferState,
            D3D12_RESOURCE_STATE_COPY_DEST);

        commandList_->CopyBufferRegion(
            slot.counterBuffer.Get(),
            0,
            clodCounterResetUpload_.Get(),
            0,
            sizeof(ClodReductionControl));
    }

    transitionResource(
        slot.counterBuffer.Get(),
        slot.counterBufferState,
        D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

    ID3D12DescriptorHeap* descriptorHeaps[]{
        clodDescriptorHeap_.Get()
    };

    commandList_->SetDescriptorHeaps(
        1,
        descriptorHeaps);

    commandList_->SetComputeRootSignature(
        clodReductionRootSignature_.Get());

    commandList_->SetPipelineState(
        clodReductionPipelineState_.Get());

    // Root parameter 1 begins at the slot's u0
    // descriptor. Its u1 counter descriptor is directly
    // beside it in the heap.
    commandList_->SetComputeRootDescriptorTable(
        1,
        GetClodGpuDescriptorHandle(
            slot.outputUavDescriptorIndex));

    std::uint64_t remainingFrameBudget =
        ClodReductionSourcePointBudgetPerFrame;

    while (remainingFrameBudget > 0
        && clodReductionCycle_.sourceIndex
        < clodReductionCycle_.sources.size())
    {
        const ClodReductionSource& source =
            clodReductionCycle_.sources[
                clodReductionCycle_.sourceIndex];

        const auto tileIterator =
            std::find_if(
                pointTiles_.begin(),
                pointTiles_.end(),
                [&source](const GpuPointTile& tile)
                {
                    return tile.id == source.tileId;
                });

        // For now, abandon this result if its source set changes.
        // M11.9 will give in-progress reductions stronger tile
        // lifetime guarantees.
        if (tileIterator == pointTiles_.end()
            || !tileIterator->clodSourceBuffer
            || tileIterator->clodSrvDescriptorIndex
            == InvalidClodDescriptorIndex
            || tileIterator->uploadedClodPointCount
            != source.pointCount)
        {
            slot.hasValidResult = false;
            clodReductionCycle_.Reset();

            return std::nullopt;
        }

        const GpuPointTile& tile =
            *tileIterator;

        if (clodReductionCycle_.sourceFirstPoint
            >= source.pointCount)
        {
            ++clodReductionCycle_.sourceIndex;
            clodReductionCycle_.sourceFirstPoint = 0;

            continue;
        }

        const std::uint32_t pointsRemainingInTile =
            source.pointCount
            - clodReductionCycle_.sourceFirstPoint;

        const std::uint32_t budgetThisDispatch =
            static_cast<std::uint32_t>(
                std::min<std::uint64_t>(
                    remainingFrameBudget,
                    std::numeric_limits<
                    std::uint32_t>::max()));

        const std::uint32_t pointsThisDispatch =
            std::min(
                pointsRemainingInTile,
                budgetThisDispatch);

        commandList_->SetComputeRootDescriptorTable(
            0,
            GetClodGpuDescriptorHandle(
                tile.clodSrvDescriptorIndex));

        ClodReductionConstants reductionConstants{};

        reductionConstants.sourceFirstPoint =
            clodReductionCycle_.sourceFirstPoint;

        reductionConstants.sourcePointCount =
            pointsThisDispatch;

        reductionConstants.outputCapacity =
            slot.capacity;

        reductionConstants.rootSpacing =
            tile.clodRootSpacing;

        reductionConstants.clodFactor =
            clodReductionCycle_.clodFactor;

        reductionConstants.cameraPosition =
            clodReductionCycle_.cameraPosition;

        reductionConstants.frustumExpansion =
            clodReductionCycle_.frustumExpansion;

        reductionConstants.viewProjection =
            clodReductionCycle_.viewProjection;

        reductionConstants.peripheralDensityFalloff =
            clodReductionCycle_.peripheralDensityFalloff;

        reductionConstants.minimumPeripheralDensity =
            clodReductionCycle_.minimumPeripheralDensity;

        commandList_->SetComputeRoot32BitConstants(
            2,
            sizeof(ClodReductionConstants)
            / sizeof(std::uint32_t),
            &reductionConstants,
            0);

        const std::uint32_t groupCount =
            (
                pointsThisDispatch
                + threadsPerGroup
                - 1
                )
            / threadsPerGroup;

        commandList_->Dispatch(
            groupCount,
            1,
            1);

        // Ensure this dispatch's appends are visible before another
        // chunk begins appending to the same buffers.
        D3D12_RESOURCE_BARRIER uavBarrier{};

        uavBarrier.Type =
            D3D12_RESOURCE_BARRIER_TYPE_UAV;

        uavBarrier.UAV.pResource = nullptr;

        commandList_->ResourceBarrier(
            1,
            &uavBarrier);

        clodReductionCycle_.sourceFirstPoint +=
            pointsThisDispatch;

        clodReductionCycle_.processedSourcePointCount +=
            pointsThisDispatch;

        remainingFrameBudget -=
            pointsThisDispatch;

        if (clodReductionCycle_.sourceFirstPoint
            == source.pointCount)
        {
            ++clodReductionCycle_.sourceIndex;
            clodReductionCycle_.sourceFirstPoint = 0;
        }
    }

    const bool reductionComplete =
        clodReductionCycle_.sourceIndex
        >= clodReductionCycle_.sources.size();

    if (!reductionComplete)
    {
        // The slot remains private to the active cycle. Its
        // incomplete contents must not be drawn or read back.
        return std::nullopt;
    }

    // Copy the two final counters into CPU-readable memory.
    transitionResource(
        slot.counterBuffer.Get(),
        slot.counterBufferState,
        D3D12_RESOURCE_STATE_COPY_SOURCE);

    commandList_->CopyBufferRegion(
        slot.counterReadback.Get(),
        0,
        slot.counterBuffer.Get(),
        0,
        sizeof(ClodReductionControl));

    // Keep the completed draw arguments ready for
    // ExecuteIndirect.
    //
    // The next reduction that reuses this slot will transition
    // the buffer from INDIRECT_ARGUMENT back through COPY_DEST
    // and UNORDERED_ACCESS.
    transitionResource(
        slot.counterBuffer.Get(),
        slot.counterBufferState,
        D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);

    slot.expectedCandidateCount =
        clodReductionCycle_.totalSourcePointCount;

    slot.resultClodFactor =
        clodReductionCycle_.clodFactor;

    slot.sourceGeneration =
        clodReductionCycle_.sourceGeneration;

    slot.hasValidResult = false;

    const std::uint32_t completedSlotIndex =
        selectedSlotIndex;

    clodReductionCycle_.Reset();

    nextClodReductionSlotIndex_ =
        (completedSlotIndex + 1)
        % ClodReductionSlotCount;

    return completedSlotIndex;
}

void D3D12Context::CollectCompletedClodReductions()
{
    const std::uint64_t completedFenceValue =
        fence_->GetCompletedValue();

    std::optional<std::uint32_t>
        newestCompletedSlot;

    std::uint64_t newestCompletedFence = 0;

    for (std::uint32_t slotIndex = 0;
        slotIndex < ClodReductionSlotCount;
        ++slotIndex)
    {
        ClodReductionSlot& slot =
            clodReductionSlots_[slotIndex];

        if (slot.completionFenceValue == 0
            || slot.completionFenceValue
        > completedFenceValue)
        {
            continue;
        }

        // Save this before clearing it below. It tells us
        // which completed result was submitted most recently.
        const std::uint64_t resultFenceValue =
            slot.completionFenceValue;

        void* mappedData = nullptr;

        const D3D12_RANGE readRange{
            0,
            sizeof(ClodReductionControl)
        };

        ThrowIfFailed(
            slot.counterReadback->Map(
                0,
                &readRange,
                &mappedData),
            "Map CLOD reduction counters");

        ClodReductionControl control{};

        std::memcpy(
            &control,
            mappedData,
            sizeof(control));

        const D3D12_RANGE noCpuWrites{
            0,
            0
        };

        slot.counterReadback->Unmap(
            0,
            &noCpuWrites);

        slot.lastAcceptedPointCount =
            control.drawArguments.
            InstanceCount;

        slot.lastOverflowPointCount =
            control.overflowPointCount;

        if (slot.lastOverflowPointCount != 0)
        {
            // An overflowing append buffer is spatially biased
            // toward whichever source points happened to be
            // processed first. Make the following reduction
            // coarser until its complete result fits.
            constexpr float overflowRecoveryStep = 1.25f;
            constexpr float maximumAutomaticFactor = 16.0f;

            const float recoveryFactor =
                std::min(
                    slot.resultClodFactor
                        * overflowRecoveryStep,
                    maximumAutomaticFactor);

            clodFactor_ =
                std::max(
                    clodFactor_,
                    recoveryFactor);
        }

        if (slot.lastAcceptedPointCount
        > slot.capacity
            || static_cast<std::uint64_t>(
                slot.lastAcceptedPointCount)
            + slot.lastOverflowPointCount
        > slot.expectedCandidateCount)
        {
            throw std::logic_error(
                "CLOD reduction produced invalid counters.");
        }

        slot.hasValidResult = true;
        slot.completionFenceValue = 0;

        if (slot.sourceGeneration
                == clodSourceGeneration_
            && (!newestCompletedSlot.has_value()
            || resultFenceValue
                > newestCompletedFence))
        {
            newestCompletedSlot = slotIndex;
            newestCompletedFence =
                resultFenceValue;
        }
    }

    if (newestCompletedSlot.has_value())
    {
        activeClodReductionSlotIndex_ =
            newestCompletedSlot;
    }
}

void D3D12Context::Render(
    const Camera& camera,
    bool runClodReduction)
{
    if (!initialized_
        || width_ == 0
        || height_ == 0)
    {
        return;
    }

    CollectAbandonedPointUploads();
    CollectRetiredPointTiles();

    FrameResource& currentFrame =
        frameResources_[frameIndex_];

    WaitForFrame(currentFrame);

    ReadGpuTimingResult(currentFrame);

    CollectCompletedClodReductions();

    clodDrawnLastFrame_ = false;

    if (!runClodReduction)
    {
        // Cancel only the unfinished build so its source snapshot
        // cannot pin tiles while Discrete mode is active. Keep the
        // most recent complete result for an immediate fixed-view
        // A/B switch back to CLOD.
        clodReductionCycle_.Reset();
    }

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

    constants.clodFactor =
        clodFactor_;

    constants.viewportSize = {
        static_cast<float>(width_),
        static_cast<float>(height_)
    };

    constants.clodPeripheralDensityFalloff =
        0.5f;

    constants.clodMinimumPeripheralDensity =
        0.3f;

    DirectX::XMFLOAT4X4 projection{};

    DirectX::XMStoreFloat4x4(
        &projection,
        camera.GetProjectionMatrix());

    // targetSpacing grows linearly with distance:
    //
    //     distance * factor / 1000
    //
    // Perspective projection divides by approximately that same
    // distance. The resulting screen-space diameter is therefore
    // approximately constant.
    constants.clodPointSizePixels =
        static_cast<float>(height_)
        * 0.5f
        * projection._22
        * clodFactor_
        / 1000.0f;

    constants.clodMinimumPointSizePixels =
        1.0f;

    constants.cameraPosition =
        camera.GetPosition();

    constants.clodBlendRangeFactor =
        0.8f;

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

    RecordPointTileUploadTransitions();

    std::optional<std::uint32_t>
        submittedClodReduction;

    if (runClodReduction)
    {
        submittedClodReduction =
            RecordClodReduction(camera);
    }

    // RecordClodReduction changes the active pipeline state
    // to the compute PSO. Restore the graphics PSO before
    // recording point drawing.
    commandList_->SetPipelineState(
        pipelineState_.Get());

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

    commandList_->IASetPrimitiveTopology(
        D3D_PRIMITIVE_TOPOLOGY_POINTLIST);

    const bool canDrawClod =
        runClodReduction
        && activeClodReductionSlotIndex_.has_value()
        && clodReductionSlots_[
            *activeClodReductionSlotIndex_].
        hasValidResult
        && clodReductionSlots_[
            *activeClodReductionSlotIndex_].
        sourceGeneration == clodSourceGeneration_;

    // The CPU knows that a complete reduction exists, but it
    // does not supply that reduction's point count.
    //
    // ExecuteIndirect reads InstanceCount directly from the
    // control buffer written by the compute shader.
    if (canDrawClod)
    {
        commandList_->SetPipelineState(
            clodDrawPipelineState_.Get());

        commandList_->IASetPrimitiveTopology(
            D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

        ClodReductionSlot& slot =
            clodReductionSlots_[
                *activeClodReductionSlotIndex_];

        // Compute left this buffer as a UAV. The input
        // assembler needs it in vertex-buffer state.
        if (slot.pointBufferState
            != D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER)
        {
            D3D12_RESOURCE_BARRIER barrier{};

            barrier.Type =
                D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;

            barrier.Transition.pResource =
                slot.pointBuffer.Get();

            barrier.Transition.Subresource =
                D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;

            barrier.Transition.StateBefore =
                slot.pointBufferState;

            barrier.Transition.StateAfter =
                D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER;

            commandList_->ResourceBarrier(
                1,
                &barrier);

            slot.pointBufferState =
                D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER;
        }

        D3D12_VERTEX_BUFFER_VIEW
            reducedPointBufferView{};

        reducedPointBufferView.BufferLocation =
            slot.pointBuffer->GetGPUVirtualAddress();

        reducedPointBufferView.SizeInBytes =
            slot.capacity
            * static_cast<std::uint32_t>(
                sizeof(ReducedClodPoint));

        reducedPointBufferView.StrideInBytes =
            sizeof(ReducedClodPoint);

        const std::array<
            D3D12_VERTEX_BUFFER_VIEW,
            2>
            clodVertexBufferViews{
                clodQuadVertexBufferView_,
                reducedPointBufferView
        };

        commandList_->IASetVertexBuffers(
            0,
            static_cast<UINT>(
                clodVertexBufferViews.size()),
            clodVertexBufferViews.data());

        commandList_->ExecuteIndirect(
            clodDrawCommandSignature_.Get(),

            // Execute one D3D12_DRAW_ARGUMENTS command.
            1,

            // GPU buffer containing the arguments.
            slot.counterBuffer.Get(),

            // drawArguments begins at byte zero inside
            // ClodReductionControl.
            offsetof(
                ClodReductionControl,
                drawArguments),

            // No count buffer is needed because we always execute
            // exactly one draw command.
            nullptr,
            0);

        clodDrawnLastFrame_ = true;
    }
    else if (!pointTiles_.empty())
    {
        commandList_->SetPipelineState(
            pipelineState_.Get());

        commandList_->IASetPrimitiveTopology(
            D3D_PRIMITIVE_TOPOLOGY_POINTLIST);

        // No completed CLOD result yet, or Discrete mode
        // was requested. Draw the existing hierarchy.
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

    if (submittedClodReduction.has_value())
    {
        ClodReductionSlot& slot =
            clodReductionSlots_[
                *submittedClodReduction];

        slot.completionFenceValue =
            currentFrame.fenceValue;
    }

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

PointCloudTileId
D3D12Context::BeginPointCloudTileUpload(
    std::span<const GpuPoint> discretePoints,
    std::span<const ClodPoint> clodPoints,
    std::uint32_t initialDrawPointCount)
{
    if (!initialized_)
    {
        throw std::logic_error(
            "D3D12 must be initialized before "
            "loading points.");
    }

    if (discretePoints.empty())
    {
        throw std::invalid_argument(
            "Cannot upload an empty discrete "
            "point hierarchy.");
    }

    if (initialDrawPointCount == 0
        || initialDrawPointCount
            > discretePoints.size())
    {
        throw std::invalid_argument(
            "Initial point draw count is invalid.");
    }

    // CLOD contains every original point exactly once.
    // The discrete array may be larger because it also
    // contains representative copies.
    if (!clodPoints.empty()
        && clodPoints.size()
            != initialDrawPointCount)
    {
        throw std::invalid_argument(
            "CLOD point count does not match "
            "the source point count.");
    }

    const std::uint64_t discreteDataSize =
        discretePoints.size_bytes();

    const std::uint64_t clodDataSize =
        clodPoints.size_bytes();

    // D3D12_VERTEX_BUFFER_VIEW::SizeInBytes is UINT.
    // This limit applies to the discrete vertex buffer.
    if (discreteDataSize
        > std::numeric_limits<UINT>::max())
    {
        throw std::runtime_error(
            "Discrete point buffer exceeds the "
            "D3D12 vertex-view size limit.");
    }

    if (clodDataSize
        > std::numeric_limits<std::uint64_t>::max()
        - discreteDataSize)
    {
        throw std::overflow_error(
            "Combined point-upload size overflow.");
    }

    const std::uint64_t totalDataSize =
        discreteDataSize
        + clodDataSize;

    if (totalDataSize
        > std::numeric_limits<std::uint64_t>::max()
        - pointBufferSizeBytes_)
    {
        throw std::overflow_error(
            "Point-memory counter overflow.");
    }

    PendingPointTileUpload upload{};

    GpuPointTile& tile =
        upload.tile;

    tile.id =
        nextPointCloudTileId_++;

    // The existing discrete representation remains a
    // separate default-heap resource because the input
    // assembler will consume it as a vertex buffer.
    tile.pointBuffer =
        CreateBuffer(
            discreteDataSize,
            D3D12_HEAP_TYPE_DEFAULT,
            D3D12_RESOURCE_STATE_COPY_DEST);

    const std::wstring discreteName =
        L"Discrete Point Tile "
        + std::to_wstring(tile.id);

    tile.pointBuffer->SetName(
        discreteName.c_str());

    // CLOD source points live in their own default-heap
    // resource. M11.3 will expose this resource through
    // a structured-buffer SRV.
    if (clodDataSize != 0)
    {
        tile.clodSourceBuffer =
            CreateBuffer(
                clodDataSize,
                D3D12_HEAP_TYPE_DEFAULT,
                D3D12_RESOURCE_STATE_COPY_DEST);

        const std::wstring clodName =
            L"CLOD Source Tile "
            + std::to_wstring(tile.id);

        tile.clodSourceBuffer->SetName(
            clodName.c_str());
    }

    // One temporary upload heap contains both CPU arrays.
    // The CLOD bytes begin directly after the discrete
    // bytes.
    upload.uploadBuffer =
        CreateBuffer(
            totalDataSize,
            D3D12_HEAP_TYPE_UPLOAD,
            D3D12_RESOURCE_STATE_GENERIC_READ);

    const std::wstring uploadName =
        L"Combined Point Upload Buffer "
        + std::to_wstring(tile.id);

    upload.uploadBuffer->SetName(
        uploadName.c_str());

    void* mappedData = nullptr;

    D3D12_RANGE noCpuReads{
        0,
        0
    };

    ThrowIfFailed(
        upload.uploadBuffer->Map(
            0,
            &noCpuReads,
            &mappedData),
        "Map combined point upload buffer");

    std::uint8_t* uploadBytes =
        static_cast<std::uint8_t*>(
            mappedData);

    // First region:
    //
    // upload offset 0
    //     → discrete GpuPoint data
    std::memcpy(
        uploadBytes,
        discretePoints.data(),
        static_cast<std::size_t>(
            discreteDataSize));

    // Second region:
    //
    // upload offset discreteDataSize
    //     → additive ClodPoint data
    if (clodDataSize != 0)
    {
        std::memcpy(
            uploadBytes + discreteDataSize,
            clodPoints.data(),
            static_cast<std::size_t>(
                clodDataSize));
    }

    D3D12_RANGE writtenRange{
        0,
        static_cast<SIZE_T>(
            totalDataSize)
    };

    upload.uploadBuffer->Unmap(
        0,
        &writtenRange);

    ThrowIfFailed(
        device_->CreateCommandAllocator(
            D3D12_COMMAND_LIST_TYPE_COPY,
            IID_PPV_ARGS(
                &upload.commandAllocator)),
        "Create point upload command allocator");

    ThrowIfFailed(
        device_->CreateCommandList(
            0,
            D3D12_COMMAND_LIST_TYPE_COPY,
            upload.commandAllocator.Get(),
            nullptr,
            IID_PPV_ARGS(
                &upload.commandList)),
        "Create point upload command list");

    // Copy the first upload region into the existing
    // discrete vertex resource.
    upload.commandList->CopyBufferRegion(
        tile.pointBuffer.Get(),
        0,
        upload.uploadBuffer.Get(),
        0,
        discreteDataSize);

    // Copy the second upload region into the new CLOD
    // compute-source resource.
    if (clodDataSize != 0)
    {
        upload.commandList->CopyBufferRegion(
            tile.clodSourceBuffer.Get(),
            0,
            upload.uploadBuffer.Get(),
            discreteDataSize,
            clodDataSize);
    }

    ThrowIfFailed(
        upload.commandList->Close(),
        "Close point upload command list");

    // Only the discrete buffer needs a vertex-buffer view.
    // The CLOD buffer will receive an SRV in M11.3.
    tile.pointBufferView.BufferLocation =
        tile.pointBuffer->
        GetGPUVirtualAddress();

    tile.pointBufferView.SizeInBytes =
        static_cast<UINT>(
            discreteDataSize);

    tile.pointBufferView.StrideInBytes =
        sizeof(GpuPoint);

    tile.uploadedPointCount =
        static_cast<std::uint32_t>(
            discretePoints.size());

    tile.uploadedClodPointCount =
        static_cast<std::uint32_t>(
            clodPoints.size());

    tile.drawRanges.push_back({
        0,
        initialDrawPointCount
        });

    tile.selectedPointCount =
        initialDrawPointCount;

    tile.discreteSizeBytes =
        discreteDataSize;

    tile.clodSizeBytes =
        clodDataSize;

    tile.sizeBytes =
        totalDataSize;

    // Both resources are currently COPY_DEST.
    // The direct queue will perform their final
    // transitions after this upload is finalized.
    tile.needsCopyToVertexTransition =
        true;

    tile.needsCopyToClodSourceTransition =
        clodDataSize != 0;

    const PointCloudTileId tileId =
        tile.id;

    // This object now owns:
    //
    // - both default-heap destination resources
    // - the shared upload resource
    // - the copy allocator
    // - the copy command list
    //
    // All of them remain alive until the fence finishes.
    pendingPointTileUploads_.push_back(
        std::move(upload));

    PendingPointTileUpload& submittedUpload =
        pendingPointTileUploads_.back();

    ID3D12CommandList* commandLists[]{
        submittedUpload.commandList.Get()
    };

    copyQueue_->ExecuteCommandLists(
        1,
        commandLists);

    const std::uint64_t fenceValue =
        nextCopyFenceValue_++;

    ThrowIfFailed(
        copyQueue_->Signal(
            copyFence_.Get(),
            fenceValue),
        "Signal point upload copy fence");

    submittedUpload.copyFenceValue =
        fenceValue;

    // Count default-heap memory immediately.
    //
    // The resources already occupy GPU memory even
    // though the copy is still running.
    pointBufferSizeBytes_ +=
        totalDataSize;

    return tileId;
}

bool D3D12Context::IsPointCloudTileUploadComplete(
    PointCloudTileId tileId) const
{
    if (tileId == 0
        || !copyFence_)
    {
        return false;
    }

    const auto uploadIterator =
        std::find_if(
            pendingPointTileUploads_.begin(),
            pendingPointTileUploads_.end(),
            [tileId](
                const PendingPointTileUpload& upload)
            {
                return upload.tile.id == tileId;
            });

    if (uploadIterator
        == pendingPointTileUploads_.end())
    {
        return false;
    }

    if (uploadIterator->abandoned
        || uploadIterator->copyFenceValue == 0)
    {
        return false;
    }

    return copyFence_->GetCompletedValue()
        >= uploadIterator->copyFenceValue;
}

bool D3D12Context::FinalizePointCloudTileUpload(
    PointCloudTileId tileId)
{
    if (!initialized_)
    {
        throw std::logic_error(
            "D3D12 must be initialized before "
            "finalizing point uploads.");
    }

    const auto uploadIterator =
        std::find_if(
            pendingPointTileUploads_.begin(),
            pendingPointTileUploads_.end(),
            [tileId](
                const PendingPointTileUpload& upload)
            {
                return upload.tile.id == tileId;
            });

    if (uploadIterator
        == pendingPointTileUploads_.end())
    {
        return false;
    }

    if (uploadIterator->abandoned
        || uploadIterator->copyFenceValue == 0
        || copyFence_->GetCompletedValue()
        < uploadIterator->copyFenceValue)
    {
        return false;
    }

    const GpuPointTile& completedTile =
        uploadIterator->tile;

    if (!completedTile.pointBuffer)
    {
        throw std::logic_error(
            "Completed tile upload is missing a "
            "GPU point resource.");
    }

    if (completedTile.uploadedPointCount == 0)
    {
        throw std::logic_error(
            "Completed tile upload has an empty "
            "point representation.");
    }

    // At upload time, selectedPointCount is initialized
    // to the original source count.
    if (completedTile.uploadedClodPointCount != 0
        && completedTile.uploadedClodPointCount
            != completedTile.selectedPointCount)
    {
        throw std::logic_error(
            "Uploaded CLOD count does not match "
            "the source-point count.");
    }

    if (completedTile.clodSizeBytes
        > std::numeric_limits<std::uint64_t>::max()
        - completedTile.discreteSizeBytes)
    {
        throw std::logic_error(
            "Completed tile byte count overflow.");
    }

    const std::uint64_t expectedTileBytes =
        completedTile.discreteSizeBytes
        + completedTile.clodSizeBytes;

    if (completedTile.sizeBytes
        != expectedTileBytes)
    {
        throw std::logic_error(
            "Completed tile GPU byte count is invalid.");
    }

    const std::uint64_t selectedPointCount =
        uploadIterator->tile.selectedPointCount;

    if (selectedPointCount
        > std::numeric_limits<std::uint64_t>::max()
        - pointCount_)
    {
        throw std::overflow_error(
            "Resident point-count overflow.");
    }

    // Allocate before moving the tile. If allocation fails,
    // the pending upload remains intact.
    pointTiles_.reserve(
        pointTiles_.size() + 1);

    GpuPointTile& completedGpuTile =
        uploadIterator->tile;

    if (completedGpuTile.clodSourceBuffer)
    {
        const std::uint32_t clodDescriptorIndex =
            AllocateClodTileDescriptor();

        completedGpuTile.clodSrvDescriptorIndex =
            clodDescriptorIndex;

        CreateClodTileSourceView(
            completedGpuTile);

        if (clodSourceGeneration_
            == std::numeric_limits<std::uint64_t>::max())
        {
            throw std::overflow_error(
                "CLOD source generation overflow.");
        }

        ++clodSourceGeneration_;
    }

    pointTiles_.push_back(
        std::move(uploadIterator->tile));

    pointCount_ +=
        selectedPointCount;

    pendingPointTileUploads_.erase(
        uploadIterator);

    return true;
}

bool D3D12Context::AbandonPointCloudTileUpload(
    PointCloudTileId tileId)
{
    if (!initialized_)
    {
        throw std::logic_error(
            "D3D12 must be initialized before "
            "abandoning point uploads.");
    }

    const auto uploadIterator =
        std::find_if(
            pendingPointTileUploads_.begin(),
            pendingPointTileUploads_.end(),
            [tileId](
                const PendingPointTileUpload& upload)
            {
                return upload.tile.id == tileId;
            });

    if (uploadIterator
        == pendingPointTileUploads_.end())
    {
        return false;
    }

    uploadIterator->abandoned = true;

    return true;
}

bool D3D12Context::BeginPointCloudTileClodUpload(
    PointCloudTileId tileId,
    std::span<const ClodPoint> clodPoints,
    float clodRootSpacing)
{
    if (!initialized_)
    {
        throw std::logic_error(
            "D3D12 must be initialized before "
            "uploading CLOD points.");
    }

    if (clodPoints.empty())
    {
        throw std::invalid_argument(
            "Cannot attach an empty CLOD hierarchy.");
    }
    if (!std::isfinite(clodRootSpacing)
        || clodRootSpacing <= 0.0f)
    {
        throw std::invalid_argument(
            "CLOD root spacing must be finite "
            "and greater than zero.");
    }

    const auto tileIterator =
        std::find_if(
            pointTiles_.begin(),
            pointTiles_.end(),
            [tileId](const GpuPointTile& tile)
            {
                return tile.id == tileId;
            });

    if (tileIterator == pointTiles_.end()
        || tileIterator->clodSourceBuffer)
    {
        return false;
    }

    const bool alreadyPending =
        std::any_of(
            pendingClodTileUploads_.begin(),
            pendingClodTileUploads_.end(),
            [tileId](const PendingClodTileUpload& upload)
            {
                return upload.tileId == tileId;
            });

    if (alreadyPending)
    {
        return false;
    }

    if (clodPoints.size()
        > std::numeric_limits<std::uint32_t>::max())
    {
        throw std::runtime_error(
            "CLOD point count exceeds the supported limit.");
    }

    PendingClodTileUpload upload{};

    upload.tileId = tileId;
    upload.pointCount =
        static_cast<std::uint32_t>(clodPoints.size());
    upload.sizeBytes = clodPoints.size_bytes();

    if (upload.sizeBytes
        > std::numeric_limits<std::uint64_t>::max()
            - pointBufferSizeBytes_)
    {
        throw std::overflow_error(
            "CLOD upload memory counter overflow.");
    }

    upload.clodSourceBuffer =
        CreateBuffer(
            upload.sizeBytes,
            D3D12_HEAP_TYPE_DEFAULT,
            D3D12_RESOURCE_STATE_COPY_DEST);

    upload.rootSpacing =
        clodRootSpacing;

    const std::wstring sourceName =
        L"CLOD Source Tile "
        + std::to_wstring(tileId);

    upload.clodSourceBuffer->SetName(
        sourceName.c_str());

    upload.uploadBuffer =
        CreateBuffer(
            upload.sizeBytes,
            D3D12_HEAP_TYPE_UPLOAD,
            D3D12_RESOURCE_STATE_GENERIC_READ);

    void* mappedData = nullptr;
    D3D12_RANGE noCpuReads{ 0, 0 };

    ThrowIfFailed(
        upload.uploadBuffer->Map(
            0,
            &noCpuReads,
            &mappedData),
        "Map CLOD attachment upload buffer");

    std::memcpy(
        mappedData,
        clodPoints.data(),
        static_cast<std::size_t>(upload.sizeBytes));

    D3D12_RANGE writtenRange{
        0,
        static_cast<SIZE_T>(upload.sizeBytes)
    };

    upload.uploadBuffer->Unmap(
        0,
        &writtenRange);

    ThrowIfFailed(
        device_->CreateCommandAllocator(
            D3D12_COMMAND_LIST_TYPE_COPY,
            IID_PPV_ARGS(&upload.commandAllocator)),
        "Create CLOD attachment allocator");

    ThrowIfFailed(
        device_->CreateCommandList(
            0,
            D3D12_COMMAND_LIST_TYPE_COPY,
            upload.commandAllocator.Get(),
            nullptr,
            IID_PPV_ARGS(&upload.commandList)),
        "Create CLOD attachment command list");

    upload.commandList->CopyBufferRegion(
        upload.clodSourceBuffer.Get(),
        0,
        upload.uploadBuffer.Get(),
        0,
        upload.sizeBytes);

    ThrowIfFailed(
        upload.commandList->Close(),
        "Close CLOD attachment command list");

    pendingClodTileUploads_.push_back(
        std::move(upload));

    PendingClodTileUpload& submitted =
        pendingClodTileUploads_.back();

    ID3D12CommandList* commandLists[]{
        submitted.commandList.Get()
    };

    copyQueue_->ExecuteCommandLists(
        1,
        commandLists);

    const std::uint64_t fenceValue =
        nextCopyFenceValue_++;

    ThrowIfFailed(
        copyQueue_->Signal(
            copyFence_.Get(),
            fenceValue),
        "Signal CLOD attachment copy fence");

    submitted.copyFenceValue = fenceValue;
    pointBufferSizeBytes_ += submitted.sizeBytes;

    return true;
}

bool D3D12Context::IsPointCloudTileClodUploadComplete(
    PointCloudTileId tileId) const
{
    const auto uploadIterator =
        std::find_if(
            pendingClodTileUploads_.begin(),
            pendingClodTileUploads_.end(),
            [tileId](const PendingClodTileUpload& upload)
            {
                return upload.tileId == tileId;
            });

    return uploadIterator
        != pendingClodTileUploads_.end()
        && !uploadIterator->abandoned
        && uploadIterator->copyFenceValue != 0
        && copyFence_->GetCompletedValue()
            >= uploadIterator->copyFenceValue;
}

bool D3D12Context::FinalizePointCloudTileClodUpload(
    PointCloudTileId tileId)
{
    const auto uploadIterator =
        std::find_if(
            pendingClodTileUploads_.begin(),
            pendingClodTileUploads_.end(),
            [tileId](const PendingClodTileUpload& upload)
            {
                return upload.tileId == tileId;
            });

    if (uploadIterator == pendingClodTileUploads_.end()
        || uploadIterator->abandoned
        || uploadIterator->copyFenceValue == 0
        || copyFence_->GetCompletedValue()
            < uploadIterator->copyFenceValue)
    {
        return false;
    }

    const auto tileIterator =
        std::find_if(
            pointTiles_.begin(),
            pointTiles_.end(),
            [tileId](const GpuPointTile& tile)
            {
                return tile.id == tileId;
            });

    if (tileIterator == pointTiles_.end()
        || tileIterator->clodSourceBuffer)
    {
        return false;
    }

    const std::uint32_t descriptorIndex =
        AllocateClodTileDescriptor();

    tileIterator->clodSourceBuffer =
        std::move(uploadIterator->clodSourceBuffer);
    tileIterator->uploadedClodPointCount =
        uploadIterator->pointCount;
    tileIterator->clodRootSpacing =
        uploadIterator->rootSpacing;
    tileIterator->clodSizeBytes =
        uploadIterator->sizeBytes;
    tileIterator->sizeBytes +=
        uploadIterator->sizeBytes;
    tileIterator->needsCopyToClodSourceTransition =
        true;
    tileIterator->clodSrvDescriptorIndex =
        descriptorIndex;

    CreateClodTileSourceView(*tileIterator);

    if (clodSourceGeneration_
        == std::numeric_limits<std::uint64_t>::max())
    {
        throw std::overflow_error(
            "CLOD source generation overflow.");
    }

    ++clodSourceGeneration_;

    pendingClodTileUploads_.erase(
        uploadIterator);

    return true;
}

bool D3D12Context::AbandonPointCloudTileClodUpload(
    PointCloudTileId tileId)
{
    const auto uploadIterator =
        std::find_if(
            pendingClodTileUploads_.begin(),
            pendingClodTileUploads_.end(),
            [tileId](const PendingClodTileUpload& upload)
            {
                return upload.tileId == tileId;
            });

    if (uploadIterator == pendingClodTileUploads_.end())
    {
        return false;
    }

    uploadIterator->abandoned = true;
    return true;
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

    // A late CLOD attachment may still be copying while the
    // already-visible discrete tile is being removed.
    const auto pendingClodIterator =
        std::find_if(
            pendingClodTileUploads_.begin(),
            pendingClodTileUploads_.end(),
            [tileId](const PendingClodTileUpload& upload)
            {
                return upload.tileId == tileId;
            });

    if (pendingClodIterator
        != pendingClodTileUploads_.end())
    {
        pendingClodIterator->abandoned = true;
    }

    const bool removesClodSource =
        tileIterator->clodSourceBuffer != nullptr;

    if (removesClodSource
        && clodSourceGeneration_
            == std::numeric_limits<std::uint64_t>::max())
    {
        throw std::overflow_error(
            "CLOD source generation overflow.");
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

    retiredTile.clodSourceBuffer =
        std::move(
            tileIterator->clodSourceBuffer);

    retiredTile.clodSrvDescriptorIndex =
        tileIterator->clodSrvDescriptorIndex;

    tileIterator->clodSrvDescriptorIndex =
        InvalidClodDescriptorIndex;

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

    if (removesClodSource)
    {
        ++clodSourceGeneration_;
    }

    return true;
}

bool D3D12Context::
IsPointCloudTileReferencedByActiveClodReduction(
    PointCloudTileId tileId) const noexcept
{
    if (tileId == 0
        || !clodReductionCycle_.IsActive())
    {
        return false;
    }

    return std::any_of(
        clodReductionCycle_.sources.begin(),
        clodReductionCycle_.sources.end(),
        [tileId](const ClodReductionSource& source)
        {
            return source.tileId == tileId;
        });
}

void D3D12Context::CollectAbandonedPointUploads()
{
    if ((pendingPointTileUploads_.empty()
        && pendingClodTileUploads_.empty())
        || !copyFence_)
    {
        return;
    }

    const std::uint64_t completedFenceValue =
        copyFence_->GetCompletedValue();

    const auto firstRemainingUpload =
        std::remove_if(
            pendingPointTileUploads_.begin(),
            pendingPointTileUploads_.end(),
            [this, completedFenceValue](
                const PendingPointTileUpload& upload)
            {
                if (!upload.abandoned
                    || upload.copyFenceValue == 0
                    || upload.copyFenceValue
                        > completedFenceValue)
                {
                    return false;
                }

                if (upload.tile.sizeBytes
                    > pointBufferSizeBytes_)
                {
                    throw std::logic_error(
                        "Abandoned point-upload memory "
                        "accounting underflow.");
                }

                pointBufferSizeBytes_ -=
                    upload.tile.sizeBytes;

                return true;
            });

    pendingPointTileUploads_.erase(
        firstRemainingUpload,
        pendingPointTileUploads_.end());

    const auto firstRemainingClodUpload =
        std::remove_if(
            pendingClodTileUploads_.begin(),
            pendingClodTileUploads_.end(),
            [this, completedFenceValue](
                const PendingClodTileUpload& upload)
            {
                if (!upload.abandoned
                    || upload.copyFenceValue == 0
                    || upload.copyFenceValue
                        > completedFenceValue)
                {
                    return false;
                }

                if (upload.sizeBytes
                    > pointBufferSizeBytes_)
                {
                    throw std::logic_error(
                        "Abandoned CLOD upload memory "
                        "accounting underflow.");
                }

                pointBufferSizeBytes_ -=
                    upload.sizeBytes;

                return true;
            });

    pendingClodTileUploads_.erase(
        firstRemainingClodUpload,
        pendingClodTileUploads_.end());
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

                if (tile.clodSrvDescriptorIndex
                    != InvalidClodDescriptorIndex)
                {
                    ReleaseClodTileDescriptor(
                        tile.clodSrvDescriptorIndex);
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

    WaitForPendingPointUploads();
    WaitForGpu();

    pointTiles_.clear();
    pendingPointTileUploads_.clear();
    pendingClodTileUploads_.clear();
    retiredPointTiles_.clear();
    ResetClodTileDescriptorAllocator();

    activeClodReductionSlotIndex_.reset();
    nextClodReductionSlotIndex_ = 0;
    clodDrawnLastFrame_ = false;

    for (ClodReductionSlot& slot :
        clodReductionSlots_)
    {
        slot.completionFenceValue = 0;
        slot.expectedCandidateCount = 0;
        slot.lastAcceptedPointCount = 0;
        slot.lastOverflowPointCount = 0;
        slot.hasValidResult = false;
    }

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
            WaitForPendingPointUploads();
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

    if (copyFenceEvent_ != nullptr)
    {
        CloseHandle(copyFenceEvent_);
        copyFenceEvent_ = nullptr;
    }

    for (auto& backBuffer : backBuffers_)
    {
        backBuffer.Reset();
    }

    depthBuffer_.Reset();

    pointTiles_.clear();
    pendingPointTileUploads_.clear();
    pendingClodTileUploads_.clear();
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
    copyFence_.Reset();
    dsvHeap_.Reset();
    rtvHeap_.Reset();
    swapChain_.Reset();
    copyQueue_.Reset();
    commandQueue_.Reset();

    for (ClodReductionSlot& slot :
        clodReductionSlots_)
    {
        slot.pointBuffer.Reset();
        slot.counterBuffer.Reset();
        slot.counterReadback.Reset();

        slot.capacity = 0;
        slot.lastAcceptedPointCount = 0;
        slot.lastOverflowPointCount = 0;

        slot.pointBufferState =
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS;

        slot.counterBufferState =
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    }

    clodCounterResetUpload_.Reset();

    freeClodTileDescriptorIndices_.clear();
    clodDescriptorHeap_.Reset();
    clodDescriptorSize_ = 0;

    clodReductionPipelineState_.Reset();
    clodReductionRootSignature_.Reset();
    clodDrawCommandSignature_.Reset();
    clodDrawPipelineState_.Reset();
    clodQuadVertexBuffer_.Reset();
    clodQuadVertexBufferView_ = {};

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

void D3D12Context::SetClodFactor(
    float factor)
{
    if (!std::isfinite(factor)
        || factor <= 0.0f)
    {
        throw std::invalid_argument(
            "CLOD factor must be finite and "
            "greater than zero.");
    }

    clodFactor_ = factor;
}

std::uint32_t
D3D12Context::
GetClodSelectedPointCount() const noexcept
{
    if (!activeClodReductionSlotIndex_)
    {
        return 0;
    }

    const ClodReductionSlot& slot =
        clodReductionSlots_[
            *activeClodReductionSlotIndex_];

    return slot.hasValidResult
        && slot.sourceGeneration
            == clodSourceGeneration_
        ? slot.lastAcceptedPointCount
        : 0;
}

std::uint32_t
D3D12Context::
GetClodOverflowPointCount() const noexcept
{
    if (!activeClodReductionSlotIndex_)
    {
        return 0;
    }

    const ClodReductionSlot& slot =
        clodReductionSlots_[
            *activeClodReductionSlotIndex_];

    return slot.hasValidResult
        && slot.sourceGeneration
            == clodSourceGeneration_
        ? slot.lastOverflowPointCount
        : 0;
}

std::uint64_t
D3D12Context::
GetClodReductionBufferSizeBytes() const noexcept
{
    std::uint64_t sizeBytes = 0;

    for (const ClodReductionSlot& slot :
        clodReductionSlots_)
    {
        sizeBytes +=
            static_cast<std::uint64_t>(slot.capacity)
            * sizeof(ReducedClodPoint);

        sizeBytes += sizeof(ClodReductionControl);
    }

    return sizeBytes;
}

D3D12Context::~D3D12Context()
{
    Shutdown();
}
