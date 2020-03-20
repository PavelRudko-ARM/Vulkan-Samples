/* Copyright (c) 2020, Arm Limited and Contributors
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
#include "scene_graph/components/material.h"
#include "scene_graph/components/mesh.h"
#include "scene_graph/components/orthographic_camera.h"
#include "scene_graph/components/perspective_camera.h"
#include "stats.h"

MultithreadingRenderPasses::MultithreadingRenderPasses()
{
	auto &config = get_configuration();

	config.insert<vkb::BoolSetting>(0, gui_use_separate_command_buffers, false);
	config.insert<vkb::BoolSetting>(0, gui_use_multithreading, false);

	config.insert<vkb::BoolSetting>(1, gui_use_separate_command_buffers, true);
	config.insert<vkb::BoolSetting>(1, gui_use_multithreading, false);

	config.insert<vkb::BoolSetting>(2, gui_use_separate_command_buffers, true);
	config.insert<vkb::BoolSetting>(2, gui_use_multithreading, true);
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
		shadow_render_targets[i] = create_shadow_render_target(SHADOWMAP_RESOLUTION);
	}

	load_scene("scenes/bonza/Bonza4X.gltf");

	scene->clear_components<vkb::sg::Light>();
	auto &light           = vkb::add_directional_light(*scene, glm::quat({glm::radians(-30.0f), glm::radians(175.0f), glm::radians(0.0f)}));
	auto &light_transform = light.get_node()->get_transform();
	light_transform.set_translation(glm::vec3(-50, 0, 0));

	// Attach a camera component to the light node
	auto shadowmap_camera_ptr = std::make_unique<vkb::sg::OrthographicCamera>("shadowmap_camera", -100.0f, 100.0f, -100.0f, 100.0f, -139.0f, 120.0f);
	shadowmap_camera_ptr->set_node(*light.get_node());
	shadowmap_camera = shadowmap_camera_ptr.get();
	light.get_node()->set_component(*shadowmap_camera_ptr);
	scene->add_component(std::move(shadowmap_camera_ptr));

	// Attach a move script to the camera component in the scene
	auto &camera_node = vkb::add_free_camera(*scene, "main_camera", get_render_context().get_surface_extent());
	camera            = &camera_node.get_component<vkb::sg::Camera>();

	shadow_render_pipeline = create_shadow_renderpass();
	main_render_pipeline   = create_main_renderpass();

	// Add a GUI with the stats you want to monitor
	stats = std::make_unique<vkb::Stats>(std::set<vkb::StatIndex>{vkb::StatIndex::frame_times});
	gui   = std::make_unique<vkb::Gui>(*this, platform.get_window().get_dpi_factor());

	return true;
}

void MultithreadingRenderPasses::prepare_render_context()
{
	get_render_context().prepare(2);
}

std::unique_ptr<vkb::RenderTarget> MultithreadingRenderPasses::create_shadow_render_target(uint32_t size)
{
	VkExtent3D extent{size, size, 1};

	vkb::core::Image depth_image{*device,
	                             extent,
	                             vkb::get_suitable_depth_format(device->get_gpu().get_handle()),
	                             VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
	                             VMA_MEMORY_USAGE_GPU_ONLY};

	std::vector<vkb::core::Image> images;

	images.push_back(std::move(depth_image));

	return std::make_unique<vkb::RenderTarget>(std::move(images));
}

std::unique_ptr<vkb::RenderPipeline> MultithreadingRenderPasses::create_shadow_renderpass()
{
	// Shadowmap subpass
	auto shadowmap_vs  = vkb::ShaderSource{"shadows/shadowmap.vert"};
	auto shadowmap_fs  = vkb::ShaderSource{"shadows/shadowmap.frag"};
	auto scene_subpass = std::make_unique<ShadowSubpass>(get_render_context(), std::move(shadowmap_vs), std::move(shadowmap_fs), *scene, *shadowmap_camera);

	shadow_subpass = scene_subpass.get();

	// Shadowmap pipeline
	auto shadowmap_render_pipeline = std::make_unique<vkb::RenderPipeline>();
	shadowmap_render_pipeline->add_subpass(std::move(scene_subpass));

	return shadowmap_render_pipeline;
}

std::unique_ptr<vkb::RenderPipeline> MultithreadingRenderPasses::create_main_renderpass()
{
	// Main subpass
	auto main_vs       = vkb::ShaderSource{"shadows/main.vert"};
	auto main_fs       = vkb::ShaderSource{"shadows/main.frag"};
	auto scene_subpass = std::make_unique<MainSubpass>(get_render_context(), std::move(main_vs), std::move(main_fs), *scene, *camera, *shadowmap_camera, shadow_render_targets);

	// Main pipeline
	auto main_render_pipeline = std::make_unique<vkb::RenderPipeline>();
	main_render_pipeline->add_subpass(std::move(scene_subpass));

	return main_render_pipeline;
}

void MultithreadingRenderPasses::update(float delta_time)
{
	update_scene(delta_time);

	update_stats(delta_time);

	update_gui(delta_time);

	auto &main_command_buffer = render_context->begin();

	auto command_buffers = record_command_buffers(main_command_buffer);

	render_context->submit(command_buffers);
}

void MultithreadingRenderPasses::draw_gui()
{
	gui->show_options_window([this]() {
		ImGui::AlignTextToFramePadding();
		ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.4f);
		ImGui::Checkbox("Use separate command buffers", &gui_use_separate_command_buffers);
		if (gui_use_separate_command_buffers)
		{
			ImGui::Checkbox("Multithreading", &gui_use_multithreading);
		}
	});
}

void set_viewport_and_scissor(vkb::CommandBuffer &command_buffer, const VkExtent2D &extent)
{
	VkViewport viewport{};
	viewport.width    = static_cast<float>(extent.width);
	viewport.height   = static_cast<float>(extent.height);
	viewport.minDepth = 0.0f;
	viewport.maxDepth = 1.0f;
	command_buffer.set_viewport(0, {viewport});

	VkRect2D scissor{};
	scissor.extent = extent;
	command_buffer.set_scissor(0, {scissor});
}

std::vector<vkb::CommandBuffer *> MultithreadingRenderPasses::record_command_buffers(vkb::CommandBuffer &main_command_buffer)
{
	auto        reset_mode = vkb::CommandBuffer::ResetMode::ResetPool;
	const auto &queue      = device->get_queue_by_flags(VK_QUEUE_GRAPHICS_BIT, 0);

	std::vector<vkb::CommandBuffer *> command_buffers;

	//Resources are requested from pools for thread #1 in shadow pass if multithreading is used
	shadow_subpass->set_thread_index(gui_use_multithreading ? 1 : 0);

	if (gui_use_separate_command_buffers)
	{
		if (gui_use_multithreading)
		{
			if (thread_pool.size() < 2)
			{
				thread_pool.resize(2);
			}

			std::vector<std::future<void>> cmd_buf_futures;

			auto &shadow_command_buffer = render_context->get_active_frame().request_command_buffer(queue,
			                                                                                        reset_mode,
			                                                                                        VK_COMMAND_BUFFER_LEVEL_PRIMARY,
			                                                                                        1);

			// Recording shadow command buffer
			auto fut = thread_pool.push(
			    [this, &shadow_command_buffer](size_t thread_id) {
				    shadow_command_buffer.begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);
				    draw_shadow_pass(shadow_command_buffer);
				    shadow_command_buffer.end();
			    });
			cmd_buf_futures.push_back(std::move(fut));

			// Recording lighting command buffer
			fut = thread_pool.push(
			    [this, &main_command_buffer](size_t thread_id) {
				    main_command_buffer.begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);
				    draw_main_pass(main_command_buffer);
				    main_command_buffer.end();
			    });
			cmd_buf_futures.push_back(std::move(fut));

			command_buffers.push_back(&shadow_command_buffer);
			command_buffers.push_back(&main_command_buffer);

			for (auto &fut : cmd_buf_futures)
			{
				fut.get();
			}
		}
		else
		{
			// Recording shadow command buffer
			auto &shadow_command_buffer = render_context->get_active_frame().request_command_buffer(queue, reset_mode);
			shadow_command_buffer.begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);
			draw_shadow_pass(shadow_command_buffer);
			shadow_command_buffer.end();

			// Recording lighting command buffer
			main_command_buffer.begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);
			draw_main_pass(main_command_buffer);
			main_command_buffer.end();

			command_buffers.push_back(&shadow_command_buffer);
			command_buffers.push_back(&main_command_buffer);
		}
	}
	else
	{
		// Recording both renderpasses into single command buffer
		main_command_buffer.begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);
		draw_shadow_pass(main_command_buffer);
		draw_main_pass(main_command_buffer);
		main_command_buffer.end();

		command_buffers.push_back(&main_command_buffer);
	}

	return command_buffers;
}

void MultithreadingRenderPasses::draw_shadow_pass(vkb::CommandBuffer &command_buffer)
{
	auto &shadow_render_target = *shadow_render_targets[get_render_context().get_active_frame_index()];
	auto &shadowmap_extent     = shadow_render_target.get_extent();

	set_viewport_and_scissor(command_buffer, shadowmap_extent);
	shadow_render_pipeline->draw(command_buffer, shadow_render_target);
	command_buffer.end_render_pass();
}

void MultithreadingRenderPasses::draw_main_pass(vkb::CommandBuffer &command_buffer)
{
	auto &views = render_context->get_active_frame().get_render_target().get_views();

	auto swapchain_layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
	{
		vkb::ImageMemoryBarrier memory_barrier{};
		memory_barrier.old_layout      = VK_IMAGE_LAYOUT_UNDEFINED;
		memory_barrier.new_layout      = swapchain_layout;
		memory_barrier.src_access_mask = 0;
		memory_barrier.dst_access_mask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		memory_barrier.src_stage_mask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		memory_barrier.dst_stage_mask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;

		command_buffer.image_memory_barrier(views.at(swapchain_attachment_index), memory_barrier);
	}

	{
		vkb::ImageMemoryBarrier memory_barrier{};
		memory_barrier.old_layout      = VK_IMAGE_LAYOUT_UNDEFINED;
		memory_barrier.new_layout      = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
		memory_barrier.src_access_mask = 0;
		memory_barrier.dst_access_mask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
		memory_barrier.src_stage_mask  = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
		memory_barrier.dst_stage_mask  = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;

		command_buffer.image_memory_barrier(views.at(depth_attachment_index), memory_barrier);
	}

	{
		auto &shadowmap = shadow_render_targets[render_context->get_active_frame_index()]->get_views().at(shadowmap_attachment_index);

		vkb::ImageMemoryBarrier memory_barrier{};
		memory_barrier.old_layout      = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
		memory_barrier.new_layout      = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		memory_barrier.src_access_mask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
		memory_barrier.dst_access_mask = VK_ACCESS_SHADER_READ_BIT;
		memory_barrier.src_stage_mask  = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
		memory_barrier.dst_stage_mask  = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;

		command_buffer.image_memory_barrier(shadowmap, memory_barrier);
	}

	auto &render_target = render_context->get_active_frame().get_render_target();
	auto &extent        = render_target.get_extent();

	set_viewport_and_scissor(command_buffer, extent);
	main_render_pipeline->draw(command_buffer, render_target);

	if (gui)
	{
		gui->draw(command_buffer);
	}

	command_buffer.end_render_pass();
}

MultithreadingRenderPasses::MainSubpass::MainSubpass(vkb::RenderContext &                             render_context,
                                                     vkb::ShaderSource &&                             vertex_source,
                                                     vkb::ShaderSource &&                             fragment_source,
                                                     vkb::sg::Scene &                                 scene,
                                                     vkb::sg::Camera &                                camera,
                                                     vkb::sg::Camera &                                shadowmap_camera,
                                                     std::vector<std::unique_ptr<vkb::RenderTarget>> &shadow_render_targets) :
    shadowmap_camera{shadowmap_camera},
    shadow_render_targets{shadow_render_targets},
    vkb::ForwardSubpass{render_context, std::move(vertex_source), std::move(fragment_source), scene, camera}
{
}

void MultithreadingRenderPasses::MainSubpass::prepare()
{
	ForwardSubpass::prepare();

	dynamic_resources = {"GlobalUniform", "ShadowUniform"};

	// Create a sampler for sampling the shadowmap during the lighting process
	// Address mode and border color are used to put everything outside of the light camera frustum into shadow
	// Depth is closer to 1 for near objects and closer to 0 for distant objects
	// If we sample outside the shadowmap range [0,0]-[1,1], sampler clamps to border and returns 1 (opaque white)
	VkSamplerCreateInfo shadowmap_sampler_create_info{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
	shadowmap_sampler_create_info.minFilter     = VK_FILTER_LINEAR;
	shadowmap_sampler_create_info.magFilter     = VK_FILTER_LINEAR;
	shadowmap_sampler_create_info.addressModeU  = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
	shadowmap_sampler_create_info.addressModeV  = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
	shadowmap_sampler_create_info.addressModeW  = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
	shadowmap_sampler_create_info.borderColor   = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
	shadowmap_sampler_create_info.compareEnable = VK_TRUE;
	shadowmap_sampler_create_info.compareOp     = VK_COMPARE_OP_GREATER_OR_EQUAL;
	shadowmap_sampler                           = std::make_unique<vkb::core::Sampler>(get_render_context().get_device(), shadowmap_sampler_create_info);
}

void MultithreadingRenderPasses::MainSubpass::draw(vkb::CommandBuffer &command_buffer)
{
	ShadowUniform shadow_uniform;
	shadow_uniform.shadowmap_projection_matrix = vkb::vulkan_style_projection(shadowmap_camera.get_projection()) * shadowmap_camera.get_view();

	auto &shadow_render_target = *shadow_render_targets[get_render_context().get_active_frame_index()];
	command_buffer.bind_image(shadow_render_target.get_views().at(0), *shadowmap_sampler, 0, 5, 0);

	auto &                render_frame  = get_render_context().get_active_frame();
	vkb::BufferAllocation shadow_buffer = render_frame.allocate_buffer(VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, sizeof(glm::mat4));
	shadow_buffer.update(shadow_uniform);
	command_buffer.bind_buffer(shadow_buffer.get_buffer(), shadow_buffer.get_offset(), shadow_buffer.get_size(), 0, 6, 0);

	ForwardSubpass::draw(command_buffer);
}

MultithreadingRenderPasses::ShadowSubpass::ShadowSubpass(vkb::RenderContext &render_context,
                                                         vkb::ShaderSource &&vertex_source,
                                                         vkb::ShaderSource &&fragment_source,
                                                         vkb::sg::Scene &    scene,
                                                         vkb::sg::Camera &   camera) :
    vkb::GeometrySubpass{render_context, std::move(vertex_source), std::move(fragment_source), scene, camera}
{
}

void MultithreadingRenderPasses::ShadowSubpass::prepare_pipeline_state(vkb::CommandBuffer &command_buffer, VkFrontFace front_face, bool double_sided_material)
{
	//Enabling depth bias to get rid of self-shadowing artifacts
	//Depth bias leterally "pushes" slightly all the primitives further away from the camera taking their slope into account
	//It helps to avoid precision related problems while doing depth comparisons in the final pass
	vkb::RasterizationState rasterization_state{};
	rasterization_state.front_face        = front_face;
	rasterization_state.depth_bias_enable = VK_TRUE;

	if (double_sided_material)
	{
		rasterization_state.cull_mode = VK_CULL_MODE_NONE;
	}

	command_buffer.set_rasterization_state(rasterization_state);
	command_buffer.set_depth_bias(-1.4f, 0.0f, -1.7f);

	vkb::MultisampleState multisample_state{};
	multisample_state.rasterization_samples = sample_count;
	command_buffer.set_multisample_state(multisample_state);
}

vkb::PipelineLayout &MultithreadingRenderPasses::ShadowSubpass::prepare_pipeline_layout(vkb::CommandBuffer &command_buffer, const std::vector<vkb::ShaderModule *> &shader_modules)
{
	//Only vertex shader is needed in the shadow subpass
	auto vertex_shader_module = shader_modules.at(0);

	vertex_shader_module->set_resource_mode(vkb::ShaderResourceMode::Dynamic, "GlobalUniform");

	return command_buffer.get_device().get_resource_cache().request_pipeline_layout({vertex_shader_module});
}

void MultithreadingRenderPasses::ShadowSubpass::prepare_push_constants(vkb::CommandBuffer &command_buffer, vkb::sg::SubMesh &sub_mesh)
{
	//No push constants are used the in shadow pass
	return;
}

std::unique_ptr<vkb::VulkanSample> create_multithreading_render_passes()
{
	return std::make_unique<MultithreadingRenderPasses>();
}