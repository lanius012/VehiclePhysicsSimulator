cbuffer VSConstants:register(b0) {
    float4x4 World;
    float4x4 ViewProjection;
};

struct VSInput {
    float3 Position :POSITION;
    float3 Normal: NORMAL;
};

struct VSOutput {
    float4 Position : SV_POSITION;
    float3 Normal : NORMAL;
    float3 WorldPosition : TEXCOORD0;
    float3 LocalPosition : TEXCOORD1;
};

VSOutput main(VSInput input) {
    VSOutput output;

    float4 worldPosition = mul(World, float4(input.Position, 1.0f));

    output.Position = mul(ViewProjection, worldPosition);
    output.Normal = normalize(mul((float3x3)World, input.Normal));
    output.WorldPosition = worldPosition.xyz;

    output.LocalPosition = input.Position;

    return output;
}