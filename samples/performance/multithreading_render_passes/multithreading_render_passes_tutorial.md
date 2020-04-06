<!--
- Copyright (c) 2020, Arm Limited and Contributors
-
- SPDX-License-Identifier: Apache-2.0
-
- Licensed under the Apache License, Version 2.0 the "License";
- you may not use this file except in compliance with the License.
- You may obtain a copy of the License at
-
-     http://www.apache.org/licenses/LICENSE-2.0
-
- Unless required by applicable law or agreed to in writing, software
- distributed under the License is distributed on an "AS IS" BASIS,
- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
- See the License for the specific language governing permissions and
- limitations under the License.
-
-->

# Multi-threaded recording with multiple render passes

## Overview

In some cases multiple stages of frame rendering can't be performed in a single render pass. This sample shows how multi-threading can help to boost performance when using multiple render passes to render a single frame. 

## Using multiple render passes

 This sample uses two render passes to implement a technique called shadowmapping. 
 The first render pass is used to render a shadowmap. It contains only depth values and represents the scene as viewed from the light position.

 The second one pass renders the actual scene from the camera point of view and uses the shadowmap from the previous pass. When light calculation is performed in fragment shader the depth value from shadow map is used to determine whether the fragment is ocludded (and therefore is in shadow) or not.

 The diagram below shows this two step process:

![Render Passes Diagram](images/render_passes_diagram.png)

 Note that there is a dependency because the second pass uses the output of the first. Since these are two separate render passes we cannot use a ``VkSubpassDependency`` for synchronization. Instead ``VkImageMemoryBarrier`` is used.

## The Multi-threading Render Passes Sample

Given two or more render passes we can record them separately in multiple threads. 

Note that in order to achieve a good improvement the workload must be similiar for all the passes so that all the threads receive equal amounts of work. In this sample the same scene is rendered once for each render pass but from different view points. 

One way to use multi-threading is to create a separate primary level command buffer for each render pass. In this case command buffers can be recorded independently and then submitted to the queue all at once using ``vkQueueSubmit``.

This however adds a little overhead. In order to improve it we can use secondary level command buffers instead. This requires to follow the following rules (according to the Vulkan Spec):
* Primary command buffer must be in the [pending or executable state](https://www.khronos.org/registry/vulkan/specs/1.2-extensions//man/html/vkCmdExecuteCommands.html) when ``vkCmdExecuteCommands`` is called
* Render pass can begin only in [primary](https://www.khronos.org/registry/vulkan/specs/1.2-extensions//man/html/vkCmdBeginRenderPass.html) command buffer

The way to achieve this is to record all the secondary command buffers in multiple threads, then start recording the primary command buffer. Inside of each render pass a secondary command buffer is specified using ``vkCmdExecuteCommands``.

When using any of these methods for multi-threading general recommendations should be taken into account (see [Multi-threaded-recording](https://github.com/KhronosGroup/Vulkan-Samples/blob/master/samples/performance/command_buffer_usage/command_buffer_usage_tutorial.md#Multi-threaded-recording)).

This sample compares different approaches of recording multiple render passes.

If "Use separate command buffers" checkbox is disabled all the commands are recorded into a single command buffer. Enabling it while using only a single thread shows no visible difference in frame rate.

When separate command buffers are used the multi-threading strategy can be chosen between "Primary buffers" and "Secondary buffers" to enable one of the approaches described above.

Below are screenshots of the sample running on a phone with a Mali G72 GPU:

![Single Thread](images/single_thread.png)

Using two threads gives a 10ms improvement:

![Primary Command Buffers](images/primary_buffers.png)

And using secondary command buffers helps to reduce frame time by 4%:

![Secondary Command Buffers](images/secondary_buffers.png)

## Further reading

[Command buffer usage and multi-threaded recording](../command_buffer_usage/command_buffer_usage_tutorial.md)

## Best practice summary

**Do**

* Use secondary level command buffers instead of submitting multiple primary command buffers per frame.

**Avoid**

* Avoid having a separate thread for each renderpass if their workloads are significantly different.

**Impact**

* The impact highly depends on the size of the scene and complexity of drawing commands recording.

**Debugging**

* Measure CPU time or overall time for each frame and compare results of using single and multiple threads.