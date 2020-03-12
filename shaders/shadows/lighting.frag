#version 320 es
/* Copyright (c) 2019-2020, Arm Limited and Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 the "License";
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

precision highp float;

#define PCF_RADIUS 1

#ifdef HAS_BASE_COLOR_TEXTURE
layout(set = 0, binding = 0) uniform sampler2D base_color_texture;
#endif

layout(location = 0) in vec4 in_pos;
layout(location = 1) in vec2 in_uv;
layout(location = 2) in vec3 in_normal;

layout(location = 0) out vec4 o_color;

layout(set = 0, binding = 1) uniform GlobalUniform
{
	mat4 model;
	mat4 view_proj;
	vec3 camera_position;
}
global_uniform;

struct Light
{
	vec4 position;         // position.w represents type of light
	vec4 color;            // color.w represents light intensity
	vec4 direction;        // direction.w represents range
	vec2 info;             // (only used for spot lights) info.x represents light inner cone angle, info.y represents light outer cone angle
};

layout(set = 0, binding = 4) uniform LightsInfo
{
	uint  count;
	Light light[MAX_FORWARD_LIGHT_COUNT];
}
lights;

layout(set = 0, binding = 5) uniform sampler2D shadowmap_texture;

layout(set = 0, binding = 6) uniform ShadowUniform
{
	mat4 light_matrix;
}
shadow_uniform;

// Push constants come with a limitation in the size of data.
// The standard requires at least 128 bytes
layout(push_constant, std430) uniform PBRMaterialUniform
{
	vec4  base_color_factor;
	float metallic_factor;
	float roughness_factor;
}
pbr_material_uniform;

vec3 apply_directional_light(uint index, vec3 normal)
{
	vec3 world_to_light = -lights.light[index].direction.xyz;

	world_to_light = normalize(world_to_light);

	float ndotl = clamp(dot(normal, world_to_light), 0.0, 1.0);

	return ndotl * lights.light[index].color.w * lights.light[index].color.rgb;
}

vec3 apply_point_light(uint index, vec3 normal)
{
	vec3 world_to_light = lights.light[index].position.xyz - in_pos.xyz;

	float dist = length(world_to_light);

	float atten = 1.0 / (dist * dist);

	world_to_light = normalize(world_to_light);

	float ndotl = clamp(dot(normal, world_to_light), 0.0, 1.0);

	return ndotl * lights.light[index].color.w * atten * lights.light[index].color.rgb;
}

float calculate_shadow(vec2 offset)
{
    vec4 projected_coord = shadow_uniform.light_matrix * vec4(in_pos.xyz, 1.0);

    projected_coord /= projected_coord.w;

    projected_coord.xy = 0.5 * projected_coord.xy + 0.5;

    float closest_depth = texture(shadowmap_texture, projected_coord.xy + offset).r;

    return projected_coord.z < closest_depth ? 0.0 : 1.0;
}

float calculate_shadow_pcf()
{
    ivec2 size = textureSize(shadowmap_texture, 0).xy;

    float scale = 1.5;

    float dx = scale * 1.0 / float(size.x);

    float dy = scale * 1.0 / float(size.y);

    float sum = 0.0;

    float count = 0.0;

    for(int x = -PCF_RADIUS; x <= PCF_RADIUS; x++) {
        for(int y = -PCF_RADIUS; y <= PCF_RADIUS; y++) {
            sum += calculate_shadow(vec2(float(x) * dx, float(y) * dy));
            count += 1.0f;
        }
    }

    return sum / count;
}

void main(void)
{
	vec3 normal = normalize(in_normal);

	vec3 light_contribution = vec3(0.0);

	for (uint i = 0U; i < lights.count; i++)
	{
		if (lights.light[i].position.w == DIRECTIONAL_LIGHT)
		{
			light_contribution += apply_directional_light(i, normal);
		}
		if (lights.light[i].position.w == POINT_LIGHT)
		{
			light_contribution += apply_point_light(i, normal);
		}
        if(i == 0U) 
        {
            light_contribution *= calculate_shadow_pcf();
        }
	}

	vec4 base_color = vec4(1.0, 0.0, 0.0, 1.0);

#ifdef HAS_BASE_COLOR_TEXTURE
	base_color = texture(base_color_texture, in_uv);
#else
	base_color = pbr_material_uniform.base_color_factor;
#endif

	vec3 ambient_color = vec3(0.2) * base_color.xyz;

	o_color = vec4(ambient_color + light_contribution * base_color.xyz, base_color.w);
}