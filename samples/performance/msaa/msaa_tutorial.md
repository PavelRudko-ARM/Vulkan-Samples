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

# MSAA (Multisample anti-aliasing)

For most uses of multisampling it is possible to keep all of the data for the additional samples in the tile memory inside of the GPU, and resolve the value to a single pixel color as part of tile write-back. This means that the additional bandwidth of those additional samples never hits external memory, which makes it exceptionally efficient.
MSAA can be integrated fully with Vulkan render passes, allowing a multisampled resolve to be explicitly specified at the end of a subpass.

## Best practice summary

**Do**

* Use 4x MSAA if possible; it's not expensive and provides good image quality improvements.
* Use `loadOp = LOAD_OP_CLEAR` or `loadOp = LOAD_OP_DONT_CARE` for the multisampled image.
* Use `pResolveAttachments` in a subpass to automatically resolve a multisampled color buffer into a single-sampled color buffer.
* Use `storeOp = STORE_OP_DONT_CARE` for the multisampled image.
* Use `LAZILY_ALLOCATED` memory to back the allocated multisampled images; they do not need to be persisted into main memory and therefore do not need physical backing storage.

**Don't**

* Use `vkCmdResolveImage()`; this has a significant negative impact on bandwidth and performance.
* Use `storeOp = STORE_OP_STORE` for multisampled image attachments.
* Use `storeOp = LOAD_OP_LOAD` for multisampled image attachments.
* Use more than 4x MSAA without checking performance, as it is not full throughput.

**Impact**

* Failing to get an inline resolve can result in substantially higher memory bandwidth and reduced performance; manually writing and resolving a 4x MSAA 1080p surface at 60 FPS requires 3.9GB/s of memory bandwidth compared to just 500MB/s when using an inline resolve.
