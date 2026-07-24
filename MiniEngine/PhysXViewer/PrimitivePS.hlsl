cbuffer PSConstants:register(b0) {
    float4 BaseColor;
};

struct PSInput {
    float4 Position:SV_POSITION;
    float3 Normal:NORMAL;
    float3 WorldPosition:TEXCOORD0;
};

float3 ApplyGroundPattern(float3 baseColor, float3 worldPosition)
{
    const float gridScale = 1.0f;   // 1.0 = 1m 단위 느낌
    const float lineWidth = 0.03f;  // 격자선 두께

    float2 uv = worldPosition.xz * gridScale;

    // 체크무늬
    float2 cell = floor(uv);
    float checker = fmod(cell.x + cell.y, 2.0f);

    float3 checkerA = baseColor * 0.80f;
    float3 checkerB = baseColor * 1.08f;
    float3 color = lerp(checkerA, checkerB, checker);

    // 격자선
    float2 f = frac(uv);

    float lineX =
        max(
            1.0f - step(lineWidth, f.x),
            1.0f - step(lineWidth, 1.0f - f.x));

    float lineZ =
        max(
            1.0f - step(lineWidth, f.y),
            1.0f - step(lineWidth, 1.0f - f.y));

    float gridMask = saturate(max(lineX, lineZ));

    float3 gridColor = float3(0.08f, 0.08f, 0.08f);

    return lerp(color, gridColor, gridMask * 0.85f);
}

float4 main(PSInput input) :SV_TARGET
{
    float3 normal = normalize(input.Normal);

    float3 toLight = normalize(float3(-0.4f, 1.0f, -0.3f));

    float diffuse = saturate(dot(normal, toLight));

    float ambient = 0.20f;
    float lighting = ambient + diffuse * 0.80f;

    float3 surfaceColor = BaseColor.rgb;

    // alpha를 임시 플래그로 사용
    if (BaseColor.a > 1.5f)
    {
        surfaceColor =
            ApplyGroundPattern(
                BaseColor.rgb,
                input.WorldPosition);
    }

    return float4(surfaceColor * lighting, 1.0f);
}