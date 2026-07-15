cbuffer PSConstants:register(b0) {
	float4 BaseColor;
};

struct PSInput {
	float4 Position:SV_POSITION;
	float3 Normal:NORMAL;
};

float4 main(PSInput input) :SV_TARGET{
	float3 normal = normalize(input.Normal);

	float3 toLight = normalize(float3(-0.4f, 1.0f, -0.3f));

	float diffuse = saturate(dot(normal, toLight));

	float ambient = 0.20f;
	float lighting = ambient + diffuse * 0.80f;

	return float4(BaseColor.rgb * lighting, BaseColor.a);
}