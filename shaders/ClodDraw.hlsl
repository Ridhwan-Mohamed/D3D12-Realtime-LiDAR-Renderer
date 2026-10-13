cbuffer CameraConstants : register(b0)
{
    float4x4 viewProjection;

    float minimumElevation;
    float maximumElevation;
    uint pointColorMode;
    float clodFactor;

    float2 viewportSize;
    float clodPointSizePixels;
    float clodMinimumPointSizePixels;

    float3 cameraPosition;
    float clodBlendRangeFactor;

    float clodPeripheralDensityFalloff;
    float clodMinimumPeripheralDensity;
    float2 clodPeripheralPadding;
};

struct VertexInput
{
    float2 corner : CORNER;

    float3 position : POSITION;
    float4 color : COLOR;
    float pointSpacing : POINT_SPACING;
};

struct VertexOutput
{
    float4 position : SV_POSITION;
    float4 color : COLOR;

    // Used by the pixel shader to make the square quad
    // appear circular.
    float2 corner : TEXCOORD0;
};

float3 GetElevationColor(
    float normalizedElevation)
{
    const float elevation =
        saturate(normalizedElevation);

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

VertexOutput VSMain(VertexInput input)
{
    VertexOutput output;

    float4 clipPosition =
        mul(
            float4(input.position, 1.0f),
            viewProjection);

    const float2 normalizedScreenPosition =
    clipPosition.xy
    / max(clipPosition.w, 0.000001f);

    const float distanceFromScreenCenter =
    length(normalizedScreenPosition);

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
            - clodPeripheralDensityFalloff
            * distanceOutsideVisibleScreen,
        clodMinimumPeripheralDensity);

    const float distanceToCamera =
    length(
        input.position
        - cameraPosition);

    const float targetSpacing =
    (
        distanceToCamera
        * clodFactor
        / 1000.0f
    )
    / peripheralDensity;

    // At the appearance boundary:
    //
    //     targetSpacing == pointSpacing
    //
    // At full size:
    //
    //     targetSpacing
    //         == clodBlendRangeFactor * pointSpacing
    const float blendSpacingWidth =
    max(
        input.pointSpacing
        * (1.0f - clodBlendRangeFactor),
        0.000001f);

    const float growth =
    saturate(
        (input.pointSpacing
            - targetSpacing)
        / blendSpacingWidth);

    const float fullPointSizePixels =
    max(
        clodPointSizePixels
            / peripheralDensity,
        clodMinimumPointSizePixels);

    const float pointSizePixels =
        fullPointSizePixels
        * growth;

    // input.corner ranges from -0.5 to +0.5.
    //
    // Multiplying by two converts pixels into normalized
    // device-coordinate width and height.
    const float2 ndcOffset =
        input.corner
        * pointSizePixels
        * 2.0f
        / max(viewportSize, float2(1.0f, 1.0f));

    // Multiplying by W converts an NDC offset back into
    // clip-space coordinates.
    clipPosition.xy +=
        ndcOffset * clipPosition.w;

    output.position =
        clipPosition;


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
        // CLOD stores its hierarchy level in the source
        // alpha byte. Do not use that value as render alpha.
        output.color = float4(
            input.color.rgb,
            1.0f);
    }

    output.corner =
        input.corner;

    return output;
}

float4 PSMain(
    VertexOutput input)
    : SV_TARGET
{
    // The quad corners are �0.5, so radius 0.5 gives a
    // circle contained by the quad.
    if (dot(input.corner, input.corner)
        > 0.25f)
    {
        discard;
    }

    return input.color;
}
