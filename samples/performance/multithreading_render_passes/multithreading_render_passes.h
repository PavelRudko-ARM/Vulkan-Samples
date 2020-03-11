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

#pragma once

#include "rendering/render_pipeline.h"
#include "scene_graph/components/camera.h"
#include "rendering/subpasses/forward_subpass.h"
#include "vulkan_sample.h"

struct alignas(16) ShadowUniform
{
    glm::mat4 light_matrix;         // Projection matrix used to render shadowmap
};

class MultithreadingRenderPasses : public vkb::VulkanSample
{
  public:
	MultithreadingRenderPasses();

	virtual bool prepare(vkb::Platform &platform) override;

	virtual ~MultithreadingRenderPasses() = default;

    /**
     * @brief This subpass is responsible for rendering a shadowmap
     */
    class ShadowSubpass : public vkb::GeometrySubpass
    {
    public:
        ShadowSubpass(vkb::RenderContext &render_context,
            vkb::ShaderSource &&vertex_source,
            vkb::ShaderSource &&fragment_source,
            vkb::sg::Scene &scene,
            vkb::sg::Camera &camera);

        virtual void draw_submesh(vkb::CommandBuffer &command_buffer, vkb::sg::SubMesh &sub_mesh, VkFrontFace front_face = VK_FRONT_FACE_COUNTER_CLOCKWISE) override;
    };

    /**
     * @brief This subpass is responsible for rendering a Scene
     *		  It implements a custom draw function which passes shadowmap and light matrix
     */
    class ForwardShadowSubpass : public vkb::ForwardSubpass
    {
    public:
        ForwardShadowSubpass(vkb::RenderContext &render_context,
            vkb::ShaderSource &&vertex_source, 
            vkb::ShaderSource &&fragment_source,
            vkb::sg::Scene &scene, 
            vkb::sg::Camera &camera,
            vkb::sg::Camera &light_camera,
            std::vector<std::unique_ptr<vkb::RenderTarget>> &shadow_render_targets);

        virtual void prepare() override;

        virtual void draw(vkb::CommandBuffer &command_buffer) override;

    private:
        std::unique_ptr<vkb::core::Sampler> shadowmap_sampler{};

        vkb::sg::Camera &light_camera;

        std::vector<std::unique_ptr<vkb::RenderTarget>> &shadow_render_targets;
    };

  private:
      std::unique_ptr<vkb::RenderTarget> create_shadow_render_target(uint32_t size);

      /**
       * @return A shadow render pass which should run first
       */
      std::unique_ptr<vkb::RenderPipeline> create_shadow_renderpass();

      /**
       * @return A lighting render pass which should run second
       */
      std::unique_ptr<vkb::RenderPipeline> create_lighting_renderpass();

      void draw_renderpass(vkb::CommandBuffer &command_buffer, vkb::RenderTarget &render_target) override;

      std::vector<std::unique_ptr<vkb::RenderTarget>> shadow_render_targets;

      /// 1. Pipeline for shadowmap rendering
      std::unique_ptr<vkb::RenderPipeline> shadow_render_pipeline{};

      /// 2. Pipeline which uses shadowmap
      std::unique_ptr<vkb::RenderPipeline> lighting_render_pipeline{};

      vkb::sg::Camera *light_camera{};

      vkb::sg::Camera *camera{};
};

std::unique_ptr<vkb::VulkanSample> create_multithreading_render_passes();
