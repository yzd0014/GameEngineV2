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
	float2 g_padding0;
	
	float3 g_lightSourceADir;
	float3 g_lightSourceBDir;
	float2 g_padding1;
};

cbuffer g_constantBuffer_perDrawCall : register( b2 )
{
  float4x4 g_transform_localToWorld;
  
  float3 g_color;
  float padding;
};

// Entry Point
//============

void main(
	in const float4 i_position : SV_POSITION,
	in const float3 i_normal : NORMAL,
	in bool isFrontFacing : SV_IsFrontFace,

	// Output
	//=======

	// Whatever color value is output from the fragment shader
	// will determine the color of the corresponding pixel on the screen
	out float4 o_color : SV_TARGET

	)
{
	float3 normal_world;
	if (isFrontFacing)
	{
		normal_world = normalize(i_normal);
	}
	else
	{
		normal_world = normalize(-i_normal);
	}
	
	float dotProduct = dot(normal_world, -g_lightSourceADir);
	float lambertTerm = max(dotProduct, 0.0);
	lambertTerm = 0.6 * lambertTerm;
	
	float3 temp_color = g_color * (lambertTerm + 0.4);
	o_color = float4(temp_color, 1);
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
	vec2 g_padding0;
	
	vec3 g_lightSourceADir;
	vec3 g_lightSourceBDir;
	vec2 g_padding1;
};

layout( std140, binding = 2 ) uniform g_constantBuffer_perDrawCall
{
  mat4 g_transform_localToWorld;
  
  vec3 g_color;
  float padding;
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
	vec3 normal_world;
	if (gl_FrontFacing)
	{
		normal_world = normalize(i_normal);
	}
	else
	{
		normal_world = normalize(-i_normal);
	}	
	
	float dotProduct = dot(normal_world, -g_lightSourceADir);
	float lambertTerm = max(dotProduct, 0.0);
	lambertTerm = 0.6 * lambertTerm;
	
	vec3 temp_color = g_color * (lambertTerm + 0.4);
	o_color = vec4(temp_color, 1);
}

#endif
