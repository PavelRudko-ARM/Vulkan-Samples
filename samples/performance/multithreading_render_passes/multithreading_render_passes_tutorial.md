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

Ideally you render all stages of your frame in a single render pass. However, in some cases different stages can't be performed in the same render pass. This sample shows how multi-threading can help to boost performance when using multiple render passes to render a single frame. 

## Using multiple render passes

 This sample uses two render passes to implement a technique called shadowmapping. 

 The first render pass is used to render a shadowmap. It contains only depth values and represents the scene as viewed from the light position.

 The second pass renders the actual scene from the camera point of view and uses the shadowmap from the previous pass. When the light calculation is performed in the fragment shader, the depth value from the shadow map is used to determine whether the fragment is occluded from the light (and therefore is in shadow) or not.

 The diagram below shows this two step process:

![Render Passes Diagram](images/render_passes_diagram.png)

 Note that there is a dependency because the second pass uses the output of the first. Since these are two separate render passes we cannot use a ``VkSubpassDependency`` for synchronization. Instead ``VkImageMemoryBarrier`` is used.

## The Multi-threading Render Passes Sample

If we have two or more render passes we can record them separately in different threads. 

Note that the more similar is the workload for all the passes, the more performance improvement you can get by splitting the work between multiple threads. In this sample the same scene is rendered once in each render pass but from different viewpoints and with different complexity of commands recording (shadow pass requires less descriptor and resources setup for each frame).

The way to use multi-threading with multiple render passes is to create a separate primary level command buffer for each of them. In this case command buffers can be recorded independently and then submitted to the queue all at once using ``vkQueueSubmit``.

When using this method for multi-threading, general recommendations should still be taken into account (see [Multi-threaded-recording](https://github.com/KhronosGroup/Vulkan-Samples/blob/master/samples/performance/command_buffer_usage/command_buffer_usage_tutorial.md#Multi-threaded-recording)).

This sample shows the difference between recording both render passes into a single command buffer in one thread and using the method described above.

Below are screenshots of the sample running on a phone with a Mali G72 GPU:

![Single Thread](images/multithreading_off.png)

Using two threads gives a 10.5ms frame time improvement and CPU cycles show an increase in CPU utilization:

![Primary Command Buffers](images/multithreading_on.png)

[Android Profiler](https://developer.android.com/studio/profile/android-profiler) can be useful to see if the process of command buffers recording takes a significant part of frame time. If this is the case then frame time can be noticeably reduced by multi-threading this process.

![Android Profiler Capture](images/android_profiler.png)
_Android Profiler capture_

In this particular case application is CPU bound and multi-threading shows a good performance increase. In the table below you can see how much time was spent on the C++ function which does command buffer recording and which part of the total capture time it takes.

Mode | Commands recording time (ms) | Contribution
---|---|---
No multi-threading | 9.85 | 98.2 %
Multi-threading | 8.63 | 86 %

_Total capture duration is 10.03ms in both cases_

## Further reading

[Command buffer usage and multi-threaded recording](../command_buffer_usage/command_buffer_usage_tutorial.md)

## Best practice summary

**Do**

* Use multi-threading for command buffer recording if possible.

**Avoid**

* Avoid having a separate thread for each renderpass if their workloads are significantly different.

**Impact**

* You can get a significant impact on frametime for a large scene with complex drawing commands recording.

**Debugging**

* Measure CPU time or overall time for each frame and compare results of using single and multiple threads.