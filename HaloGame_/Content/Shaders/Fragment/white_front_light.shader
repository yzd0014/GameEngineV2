/*
	This is the standard fragment shader

	A fragment shader is responsible for telling the GPU what color a specific fragment should be
*/

#include <Shaders/shaders.inc>

#if defined( EAE6320_PLATFORM_D3D )

// Constant Buffers
//=================

cbuffer g_constantBuffer_perFrame : register( b0 )
{
	float4x4 g_transform_worldToCamera;
	float4x4 g_transform_cameraToProjected;

	float g_elapsedSecondCount_systemTime;
	float g_elapsedSecondCount_simulationTime;
	// For float4 alignment
	float2 g_padding;
};


// Entry Point
//============

void main(
	in const float4 i_position : SV_POSITION,
	in const float3 i_normal : NORMAL,

	// Output
	//=======

	// Whatever color value is output from the fragment shader
	// will determine the color of the corresponding pixel on the screen
	out float4 o_color : SV_TARGET

	)
{
	float3 normal_world = normalize(i_normal);
	float3 lightDir = float3(0, -1, -1);
	lightDir = normalize(lightDir);
	float lambertTerm;
	float dotProduct;
	
	dotProduct = dot(normal_world, -lightDir);
	lambertTerm = max(dotProduct, 0.0);
	lambertTerm = 0.6 * lambertTerm;
	
	float4 materialColor = float4(1, 1, 1, 1);
	// Output solid white
	o_color = materialColor * (lambertTerm + 0.4);
}

#elif defined( EAE6320_PLATFORM_GL )

// Constant Buffers
//=================

layout( std140, binding = 0 ) uniform g_constantBuffer_perFrame
{
	mat4 g_transform_worldToCamera;
	mat4 g_transform_cameraToProjected;

	float g_elapsedSecondCount_systemTime;
	float g_elapsedSecondCount_simulationTime;
	// For vec4 alignment
	vec2 g_padding;
};

//Input 
layout ( location = 0 ) in vec3 i_normal;
// Output
//=======

// Whatever color value is output from the fragment shader
// will determine the color of the corresponding pixel on the screen
out vec4 o_color;

// Entry Point
//============

void main()
{
	vec3 normal_world = normalize(i_normal);
	vec3 lightDir = vec3(0, -1, -1);
	lightDir = normalize(lightDir);
	float lambertTerm;
	float dotProduct;
	
	dotProduct = dot(normal_world, -lightDir);
	lambertTerm = max(dotProduct, 0.0);
	lambertTerm = 0.6 * lambertTerm;
	
	vec4 materialColor = vec4(1, 1, 1, 1);
	
	// Output solid white
	o_color = materialColor * (lambertTerm + 0.4);
}

#endif
