# Forest lighting

The shared GPU renderer uses warm directional sunlight, cooler sky ambient
light, a restrained ground bounce, and light haze beyond 25 metres. Lighting
and haze mixing run in linear RGB, with approximate gamma 2.2 conversion at the
texture/output boundaries. The framebuffer remains ordinary RGB for JPEG and
RTSP readback; see [Khronos' colour-space description](https://wikis.khronos.org/opengl/Framebuffer).
This is a lightweight outdoor lighting model, without PBR material maps or
real-time global illumination.

One 4096×4096 depth map covers the static terrain, computed at renderer startup
from its bounds and the sun direction. All five cameras and the observer share
it. Four filtered comparison samples soften shadow edges, using
[depth comparison sampling](https://moderngl.readthedocs.io/en/latest/reference/sampler.html).
The bake uses original texture alpha at the same cutoff as rendering and
LiDAR. Transparent leaf gaps therefore pass sunlight rather than casting solid
rectangular shadows. A regression scene verifies sunlight passes through an
alpha gap and that an opaque texel at that location casts a shadow.

The robot and moving props receive forest shadows. They do not cast visual
shadows: baking them would leave silhouettes after movement, while dynamic
shadow passes would consume additional frame time. Their LiDAR occlusion is
unchanged and follows their current physical poses. Moving the sun or replacing
terrain requires restarting the GPU worker to rebuild the static map.

`tools/gpu_worker.py --shadow-size 0` disables shadowing for comparisons; its
default is 4096. The map adds approximately 64 MiB of GPU memory and does not
redraw shadow geometry per camera or per frame. Original camera resolution,
texture resolution, physics integration and LiDAR coverage remain unchanged.

## Live captures and verification

[Observer capture](../media/optimization/forest_lighting.jpg) and
[front RTSP camera](../media/optimization/cam_front_lighting.png) are actual
frames from the running forest scene. Fifteen texture/render regression tests
pass, including the new alpha-aware shadow test, texture filtering, depth
discard, buffered readback and robot inertial-frame alignment.

[Full fleet measurement](lighting-performance.json) records a two-minute run
with five 640×480 cameras, the 800×600 observer, physics and both 30 Hz LiDARs.

| Measurement | Result |
| --- | --- |
| Fresh frames per camera | 28.36 fps mean; 24.49 fps minimum three-second window |
| Both LiDARs | 29.62 Hz mean; 28.13 Hz minimum window |
| Duration | 120.75 seconds |
| GPU memory | 437 MiB, including the 64 MiB shadow map |

All measured camera windows exceeded 24 fps. The earlier forest run averaged
28.68 fps; these runs used the live, interactive observer, so they do not isolate
lighting cost from camera viewpoint and host scheduling. This meets the target
on the measured fleet, with limited margin for additional workloads.
