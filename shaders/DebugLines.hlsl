cbuffer CameraConstants : register(b0)
{
    float4x4 viewProjection;
};

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

    output.color = input.color;

    return output;
}

float4 PSMain(VertexOutput input) : SV_TARGET
{
    return input.color;
}
