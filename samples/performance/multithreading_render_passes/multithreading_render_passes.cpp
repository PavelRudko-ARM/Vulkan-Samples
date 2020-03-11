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

#include "multithreading_render_passes.h"

#include "common/vk_common.h"
#include "gltf_loader.h"
#include "gui.h"
#include "platform/filesystem.h"
#include "platform/platform.h"
#include "scene_graph/components/orthographic_camera.h"
#include "scene_graph/components/perspective_camera.h"
#include "scene_graph/components/mesh.h"
#include "scene_graph/components/material.h"
#include "stats.h"

MultithreadingRenderPasses::MultithreadingRenderPasses()
{
}

bool MultithreadingRenderPasses::prepare(vkb::Platform &platform)
{
	if (!VulkanSample::prepare(platform))
	{
		return false;
	}

    shadow_render_targets.resize(get_render_context().get_render_frames().size());
    for (uint32_t i = 0; i < shadow_render_targets.size(); i++) 
    {
        shadow_render_targets[i] = create_shadow_render_target(1024);
    }

	// Load a scene from the assets folder
	load_scene("scenes/sponza/Sponza01.gltf");

    scene->clear_components<vkb::sg::Light>();
    auto &light = vkb::add_directional_light(*scene, glm::quat({ glm::radians(-75.0f), glm::radians(-45.0f), glm::radians(0.0f) }));
    auto &light_transform = light.get_node()->get_transform();
    light_transform.set_translation(light_transform.get_rotation() * glm::vec3(0, 0, 1));

    // Attach a camera component to the light node
    auto light_camera_ptr = std::make_unique<vkb::sg::OrthographicCamera>("light_camera");
    light_camera_ptr->set_left(-1000.0f);
    light_camera_ptr->set_right(1000.0f);
    light_camera_ptr->set_bottom(-1000.0f);
    light_camera_ptr->set_top(1000.0f);
    light_camera_ptr->set_near_plane(-3000.0f);
    light_camera_ptr->set_far_plane(3000.0f);
    light_camera_ptr->set_node(*light.get_node());
    light_camera = light_camera_ptr.get();
    light.get_node()->set_component(*light_camera_ptr);
    scene->add_component(std::move(light_camera_ptr));

	// Attach a move script to the camera component in the scene
	//auto &camera_node = vkb::add_free_camera(*scene, light.get_node()->get_name(), get_render_context().get_surface_extent());
    auto &camera_node = vkb::add_free_camera(*scene, "main_camera", get_render_context().get_surface_extent());
	camera            = &camera_node.get_component<vkb::sg::Camera>();

    shadow_render_pipeline = create_shadow_renderpass();
    lighting_render_pipeline = create_lighting_renderpass();

	// Add a GUI with the stats you want to monitor
	stats = std::make_unique<vkb::Stats>(std::set<vkb::StatIndex>{vkb::StatIndex::frame_times});
	gui   = std::make_unique<vkb::Gui>(*this, platform.get_window().get_dpi_factor());

	return true;
}

std::unique_ptr<vkb::RenderTarget> MultithreadingRenderPasses::create_shadow_render_target(uint32_t size)
{
    VkExtent3D extent{ size, size, 1 };

    vkb::core::Image depth_image{ *device,
                                 extent,
                                 vkb::get_suitable_depth_format(device->get_gpu().get_handle()),
                                 VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                                 VMA_MEMORY_USAGE_GPU_ONLY };

    std::vector<vkb::core::Image> images;

    images.push_back(std::move(depth_image));

    return std::make_unique<vkb::RenderTarget>(std::move(images));
}

std::unique_ptr<vkb::RenderPipeline> MultithreadingRenderPasses::create_shadow_renderpass()
{
    // Shadowmap subpass
    auto shadowmap_vs = vkb::ShaderSource{ "shadows/shadowmap.vert" };
    auto shadowmap_fs = vkb::ShaderSource{ "shadows/shadowmap.frag" };
    auto scene_subpass = std::make_unique<ShadowSubpass>(get_render_context(), std::move(shadowmap_vs), std::move(shadowmap_fs), *scene, *light_camera);

    // Shadowmap pipeline
    auto shadowmap_render_pipeline = std::make_unique<vkb::RenderPipeline>();
    shadowmap_render_pipeline->add_subpass(std::move(scene_subpass));

    return shadowmap_render_pipeline;
}

std::unique_ptr<vkb::RenderPipeline> MultithreadingRenderPasses::create_lighting_renderpass()
{
    // Lighting subpass
    auto lighting_vs = vkb::ShaderSource{ "shadows/lighting.vert" };
    auto lighting_fs = vkb::ShaderSource{ "shadows/lighting.frag" };
    auto scene_subpass = std::make_unique<ForwardShadowSubpass>(get_render_context(), std::move(lighting_vs), std::move(lighting_fs), *scene, *camera, *light_camera, shadow_render_targets);

    // Lighting pipeline
    auto lighting_render_pipeline = std::make_unique<vkb::RenderPipeline>();
    lighting_render_pipeline->add_subpass(std::move(scene_subpass));

    return lighting_render_pipeline;
}

void set_viewport_and_scissor(vkb::CommandBuffer &command_buffer, const VkExtent2D &extent)
{
    VkViewport viewport{};
    viewport.width = static_cast<float>(extent.width);
    viewport.height = static_cast<float>(extent.height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    command_buffer.set_viewport(0, { viewport });

    VkRect2D scissor{};
    scissor.extent = extent;
    command_buffer.set_scissor(0, { scissor });
}

void MultithreadingRenderPasses::draw_renderpass(vkb::CommandBuffer &command_buffer, vkb::RenderTarget &render_target)
{
    auto &extent = render_target.get_extent();
    auto &shadow_render_target = *shadow_render_targets[get_render_context().get_active_frame_index()];
    auto &shadowmap_extent = shadow_render_target.get_extent();
   
    set_viewport_and_scissor(command_buffer, shadowmap_extent);
    shadow_render_pipeline->draw(command_buffer, shadow_render_target);

    set_viewport_and_scissor(command_buffer, extent);
    lighting_render_pipeline->draw(command_buffer, render_target);

    if (gui)
    {
        gui->draw(command_buffer);
    }

    command_buffer.end_render_pass();
}

std::unique_ptr<vkb::VulkanSample> create_multithreading_render_passes()
{
	return std::make_unique<MultithreadingRenderPasses>();
}

MultithreadingRenderPasses::ForwardShadowSubpass::ForwardShadowSubpass(vkb::RenderContext &render_context,
    vkb::ShaderSource &&vertex_source,
    vkb::ShaderSource &&fragment_source,
    vkb::sg::Scene &scene,
    vkb::sg::Camera &camera,
    vkb::sg::Camera &light_camera,
    std::vector<std::unique_ptr<vkb::RenderTarget>> &shadow_render_targets) :
    light_camera{light_camera},
    shadow_render_targets{shadow_render_targets},
    vkb::ForwardSubpass{render_context, std::move(vertex_source), std::move(fragment_source), scene, camera}
{
}

void MultithreadingRenderPasses::ForwardShadowSubpass::prepare()
{
    ForwardSubpass::prepare();

    dynamic_resources = { "GlobalUniform", "ShadowUniform" };

    // Create a sampler for sampling the shadowmap during the lighting process
    VkSamplerCreateInfo shadowmap_sampler_create_info{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
    shadowmap_sampler_create_info.minFilter = VK_FILTER_LINEAR;
    shadowmap_sampler_create_info.magFilter = VK_FILTER_LINEAR;
    shadowmap_sampler_create_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    shadowmap_sampler_create_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    shadowmap_sampler_create_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    shadowmap_sampler_create_info.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
    shadowmap_sampler = std::make_unique<vkb::core::Sampler>(get_render_context().get_device(), shadowmap_sampler_create_info);
}

void MultithreadingRenderPasses::ForwardShadowSubpass::draw(vkb::CommandBuffer &command_buffer)
{
    ShadowUniform shadow_uniform;
    shadow_uniform.light_matrix = vkb::vulkan_style_projection(light_camera.get_projection()) * light_camera.get_view();

    auto &shadow_render_target = *shadow_render_targets[get_render_context().get_active_frame_index()];
    command_buffer.bind_image(shadow_render_target.get_views()[0], *shadowmap_sampler, 0, 5, 0);

    auto &render_frame = get_render_context().get_active_frame();
    vkb::BufferAllocation shadow_buffer = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(glm::mat4));
    shadow_buffer.update(shadow_uniform);
    command_buffer.bind_buffer(shadow_buffer.get_buffer(), shadow_buffer.get_offset(), shadow_buffer.get_size(), 0, 6, 0);

    ForwardSubpass::draw(command_buffer);
}

MultithreadingRenderPasses::ShadowSubpass::ShadowSubpass(vkb::RenderContext &render_context, 
    vkb::ShaderSource &&vertex_source, 
    vkb::ShaderSource &&fragment_source, 
    vkb::sg::Scene &scene, 
    vkb::sg::Camera &camera) :
    vkb::GeometrySubpass{render_context, std::move(vertex_source), std::move(fragment_source), scene, camera}
{
}

void MultithreadingRenderPasses::ShadowSubpass::draw_submesh(vkb::CommandBuffer &command_buffer, vkb::sg::SubMesh &sub_mesh, VkFrontFace front_face)
{
    auto &device = command_buffer.get_device();

    vkb::RasterizationState rasterization_state{};
    rasterization_state.front_face = front_face;
    rasterization_state.depth_bias_enable = VK_TRUE;

    if (sub_mesh.get_material()->double_sided)
    {
        rasterization_state.cull_mode = VK_CULL_MODE_NONE;
    }

    command_buffer.set_rasterization_state(rasterization_state);
    command_buffer.set_depth_bias(-1.4f, 0.0f, -1.7f);

    vkb::MultisampleState multisample_state{};
    multisample_state.rasterization_samples = sample_count;
    command_buffer.set_multisample_state(multisample_state);

    auto &vert_shader_module = device.get_resource_cache().request_shader_module(VK_SHADER_STAGE_VERTEX_BIT, get_vertex_shader(), sub_mesh.get_shader_variant());
    auto &frag_shader_module = device.get_resource_cache().request_shader_module(VK_SHADER_STAGE_FRAGMENT_BIT, get_fragment_shader(), sub_mesh.get_shader_variant());

    std::vector<vkb::ShaderModule *> shader_modules{ &vert_shader_module, &frag_shader_module };

    vert_shader_module.set_resource_mode(vkb::ShaderResourceMode::Dynamic, "GlobalUniform");

    auto &pipeline_layout = device.get_resource_cache().request_pipeline_layout(shader_modules);

    command_buffer.bind_pipeline_layout(pipeline_layout);

    auto &descriptor_set_layout = pipeline_layout.get_descriptor_set_layout(0);

    auto vertex_input_resources = pipeline_layout.get_resources(vkb::ShaderResourceType::Input, VK_SHADER_STAGE_VERTEX_BIT);

    vkb::VertexInputState vertex_input_state;

    for (auto &input_resource : vertex_input_resources)
    {
        vkb::sg::VertexAttribute attribute;

        if (!sub_mesh.get_attribute(input_resource.name, attribute))
        {
            continue;
        }

        VkVertexInputAttributeDescription vertex_attribute{};
        vertex_attribute.binding = input_resource.location;
        vertex_attribute.format = attribute.format;
        vertex_attribute.location = input_resource.location;
        vertex_attribute.offset = attribute.offset;

        vertex_input_state.attributes.push_back(vertex_attribute);

        VkVertexInputBindingDescription vertex_binding{};
        vertex_binding.binding = input_resource.location;
        vertex_binding.stride = attribute.stride;

        vertex_input_state.bindings.push_back(vertex_binding);
    }

    command_buffer.set_vertex_input_state(vertex_input_state);

    // Find submesh vertex buffers matching the shader input attribute names
    for (auto &input_resource : vertex_input_resources)
    {
        const auto &buffer_iter = sub_mesh.vertex_buffers.find(input_resource.name);

        if (buffer_iter != sub_mesh.vertex_buffers.end())
        {
            std::vector<std::reference_wrapper<const vkb::core::Buffer>> buffers;
            buffers.emplace_back(std::ref(buffer_iter->second));

            // Bind vertex buffers only for the attribute locations defined
            command_buffer.bind_vertex_buffers(input_resource.location, std::move(buffers), { 0 });
        }
    }

    draw_submesh_command(command_buffer, sub_mesh);
}
