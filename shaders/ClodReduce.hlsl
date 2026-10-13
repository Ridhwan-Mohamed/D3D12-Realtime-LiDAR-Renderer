struct ClodPoint
{
    float3 position;
    uint colorAndLevel;
};

struct ReducedClodPoint
{
    float3 position;
    uint colorAndLevel;
    float pointSpacing;
};

StructuredBuffer<ClodPoint>
    SourcePoints : register(t0);

RWStructuredBuffer<ReducedClodPoint>
    ReducedPoints : register(u0);

// Byte layout matches D3D12_DRAW_ARGUMENTS:
//
// offset 0:
//     VertexCountPerInstance = 6
//
// offset 4:
//     InstanceCount / accepted point count
//
// offset 8:
//     StartVertexLocation = 0
//
// offset 12:
//     StartInstanceLocation = 0
//
// offset 16:
//     overflow count
RWByteAddressBuffer
    ReductionControl : register(u1);

cbuffer ReductionConstants : register(b0)
{
    uint sourceFirstPoint;
    uint sourcePointCount;

    uint outputCapacity;
    float rootSpacing;

    float clodFactor;
    float3 cameraPosition;

    float frustumExpansion;
    float peripheralDensityFalloff;
    float minimumPeripheralDensity;
    float reductionPadding;

    float4x4 viewProjection;
};

uint HashUint(uint value)
{
    value ^= value >> 16;
    value *= 0x7feb352du;
    value ^= value >> 15;
    value *= 0x846ca68bu;
    value ^= value >> 16;

    return value;
}

float StableRandom01(
    ClodPoint sourcePoint)
{
    uint seed =
        HashUint(asuint(sourcePoint.position.x))
        ^ HashUint(asuint(sourcePoint.position.y))
        ^ HashUint(asuint(sourcePoint.position.z));

    return float(
        HashUint(seed) & 0x00ffffffu)
        / 16777216.0f;
}

bool TryProjectToReductionView(
    float3 position,
    out float3 ndc)
{
    const float4 clipPosition =
        mul(
            float4(position, 1.0f),
            viewProjection);

    if (clipPosition.w <= 0.0f)
    {
        ndc = 0.0f;
        return false;
    }

    ndc =
        clipPosition.xyz
        * rcp(clipPosition.w);

    return
        abs(ndc.x) <= frustumExpansion
        && abs(ndc.y) <= frustumExpansion
        && ndc.z >= 0.0f
        && ndc.z <= 1.0f;
}

float CalculatePointSpacing(
    ClodPoint sourcePoint)
{
    const uint integerLevel =
        sourcePoint.colorAndLevel >> 24;

    const float randomizedLevel =
        float(integerLevel)
        + StableRandom01(sourcePoint);

    return
        rootSpacing
        * exp2(-randomizedLevel);
}

bool IsUsefulDetail(
    float3 position,
    float2 normalizedScreenPosition,
    float pointSpacing)
{
    const float distanceToCamera =
        length(
            position
            - cameraPosition);

    const float desktopTargetSpacing =
        distanceToCamera
        * clodFactor
        / 1000.0f;

    // Zero everywhere inside the visible screen.
    //
    // Just beyond an edge it begins increasing toward one at
    // the outer boundary of the expanded [-2, 2] region.
    const float distanceOutsideVisibleScreen =
    max(
        max(
            abs(normalizedScreenPosition.x),
            abs(normalizedScreenPosition.y))
        - 1.0f,
        0.0f);

    const float peripheralDensity =
    max(
        1.0f
            - peripheralDensityFalloff
            * distanceOutsideVisibleScreen,
        minimumPeripheralDensity);

    // Lower density means accepting only points representing
    // a larger spacing.
    const float targetSpacing =
        desktopTargetSpacing
        / peripheralDensity;

    return pointSpacing >= targetSpacing;
}

bool TryAllocateOutputIndex(
    out uint outputIndex)
{
    uint observedCount =
        ReductionControl.Load(4);

    while (observedCount < outputCapacity)
    {
        uint originalCount = 0;

        ReductionControl.InterlockedCompareExchange(
            4,
            observedCount,
            observedCount + 1,
            originalCount);

        if (originalCount == observedCount)
        {
            outputIndex = observedCount;
            return true;
        }

        observedCount = originalCount;
    }

    outputIndex = 0;
    return false;
}

[numthreads(256, 1, 1)]
void CSMain(
    uint3 dispatchThreadId
        : SV_DispatchThreadID)
{
    // Index inside this particular chunk.
    const uint chunkPointIndex =
        dispatchThreadId.x;

    // The final thread group may contain extra threads.
    if (chunkPointIndex >= sourcePointCount)
    {
        return;
    }

    // Convert the chunk-local index into an index inside
    // the complete source array belonging to this tile.
    const uint sourceIndex =
        sourceFirstPoint
        + chunkPointIndex;

    const ClodPoint sourcePoint =
        SourcePoints[sourceIndex];

    float3 normalizedDevicePosition;

    if (!TryProjectToReductionView(
    sourcePoint.position,
    normalizedDevicePosition))
    {
        return;
    }

    const float pointSpacing =
    CalculatePointSpacing(
        sourcePoint);

    if (!IsUsefulDetail(
        sourcePoint.position,
        normalizedDevicePosition.xy,
        pointSpacing))
    {
        return;
    }

    uint outputIndex = 0;

    if (!TryAllocateOutputIndex(
        outputIndex))
    {
        uint previousOverflowCount = 0;

        ReductionControl.InterlockedAdd(
            16,
            1,
            previousOverflowCount);

        return;
    }

    ReducedClodPoint reducedPoint;

    reducedPoint.position =
    sourcePoint.position;

    reducedPoint.colorAndLevel =
    sourcePoint.colorAndLevel;

    reducedPoint.pointSpacing =
    pointSpacing;

    ReducedPoints[outputIndex] =
    reducedPoint;
}
