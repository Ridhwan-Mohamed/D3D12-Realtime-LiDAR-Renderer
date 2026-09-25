cbuffer CameraConstants : register(b0)
{
    float4x4 viewProjection;

    float minimumElevation;
    float maximumElevation;
    uint pointColorMode;
    float padding;
};

float3 GetElevationColor(float normalizedElevation)
{
    const float elevation =
        saturate(normalizedElevation);

    // A terrain-style gradient gives flat areas, vegetation,
    // exposed slopes, and peaks visually distinct bands while
    // retaining smooth transitions between them.
    if (elevation < 0.12f)
    {
        return lerp(
            float3(0.04f, 0.12f, 0.32f),
            float3(0.00f, 0.48f, 0.62f),
            elevation / 0.12f);
    }

    if (elevation < 0.30f)
    {
        return lerp(
            float3(0.00f, 0.48f, 0.62f),
            float3(0.08f, 0.48f, 0.18f),
            (elevation - 0.12f) / 0.18f);
    }

    if (elevation < 0.52f)
    {
        return lerp(
            float3(0.08f, 0.48f, 0.18f),
            float3(0.62f, 0.78f, 0.24f),
            (elevation - 0.30f) / 0.22f);
    }

    if (elevation < 0.70f)
    {
        return lerp(
            float3(0.62f, 0.78f, 0.24f),
            float3(0.88f, 0.63f, 0.22f),
            (elevation - 0.52f) / 0.18f);
    }

    if (elevation < 0.86f)
    {
        return lerp(
            float3(0.88f, 0.63f, 0.22f),
            float3(0.48f, 0.30f, 0.27f),
            (elevation - 0.70f) / 0.16f);
    }

    return lerp(
        float3(0.48f, 0.30f, 0.27f),
        float3(0.96f, 0.98f, 1.00f),
        (elevation - 0.86f) / 0.14f);
}

struct VertexInput
{
    float3 position : POSITION;
    float4 color : COLOR;
};

struct VertexOutput
{
    float4 position : SV_POSITION;
    float4 color : COLOR;
};

VertexOutput VSMain(VertexInput input)
{
    VertexOutput output;

    output.position = mul(
    float4(input.position, 1.0f),
    viewProjection);

    if (pointColorMode == 1)
    {
        const float elevationRange =
        max(
            maximumElevation
                - minimumElevation,
            0.0001f);

        const float normalizedElevation =
        (input.position.y
            - minimumElevation)
        / elevationRange;

        output.color = float4(
        GetElevationColor(
            normalizedElevation),
        1.0f);
    }
    else
    {
        output.color = input.color;
    }

    return output;
}

float4 PSMain(VertexOutput input) : SV_TARGET
{
    return input.color;
}
