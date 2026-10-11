"""One lean EGL/OpenGL context shared by every simulated camera.

Texture alpha is discarded before depth is written. Static meshes are split
into spatial draw ranges and frustum culled; their GPU buffers/textures are
uploaded once. Linear-light sunlight and sky/ground ambient illumination add
depth without per-frame shadow passes. Physics and ray workers never create a
GPU context.
"""

import numpy as np
import pybullet as p
import moderngl
from rove_sim.core.engine import _ensure_egl_vendor
from rove_sim.world.mesh_assets import load_mesh
from rove_sim.robot.frames import base_link_pose

VERTEX = """#version 330
uniform mat4 clip;
uniform mat4 model;
uniform mat4 parts[128];
uniform bool multipart;
in float part_id;
in vec3 vertex_colour;
in vec3 position;
in vec2 uv;
out vec2 texcoord;
out vec3 world_position;
out vec3 surface_colour;
void main(){
 mat4 transform=multipart?parts[int(part_id)]:model;
 vec4 world=transform*vec4(position,1.0);
 world_position=world.xyz;texcoord=uv;gl_Position=clip*world;
 surface_colour=multipart?vertex_colour:vec3(1);
}
"""
FRAGMENT = """#version 330
uniform sampler2D albedo;
uniform sampler2D normal_map;
uniform sampler2D roughness_map;
uniform bool has_material;
uniform float cutoff;
uniform vec3 eye;
uniform mat4 sun_clip;
uniform sampler2DShadow forest_shadow;
uniform float shadow_bias;
uniform vec3 sun_direction;
uniform vec3 sun_colour;
uniform bool has_environment;
uniform vec3 sky_sh[9];
in vec2 texcoord;
in vec3 world_position;
in vec3 surface_colour;
out vec4 color;
void main(){
  // Derivatives must be evaluated while the entire fragment quad is active.
  vec3 normal=normalize(cross(dFdx(world_position),dFdy(world_position)));
  vec3 geometric_normal=normal;
  vec3 dpdx=dFdx(world_position), dpdy=dFdy(world_position);
  vec2 duvdx=dFdx(texcoord), duvdy=dFdy(texcoord);
  float determinant=duvdx.x*duvdy.y-duvdx.y*duvdy.x;
  float roughness=0.65;
  vec4 base=texture(albedo,texcoord);
  // Filter colour over the pixel footprint, but retain pointwise coverage for
  // the same leaf/gap predicate used by LiDAR at the original texture level.
  if(cutoff>0.0 && textureLod(albedo,texcoord,0.0).a<cutoff) discard;
  // RGB mipmaps contain alpha-weighted colour, not black atlas padding.
  base.rgb=base.rgb/max(base.a,1.0/255.0)*surface_colour;
  if(has_material && abs(determinant)>1e-10){
    vec3 tangent=normalize((dpdx*duvdy.y-dpdy*duvdx.y)/determinant);
    tangent=normalize(tangent-normal*dot(tangent,normal));
    vec3 bitangent=normalize((dpdy*duvdx.x-dpdx*duvdy.x)/determinant);
    bitangent=normalize(bitangent-normal*dot(bitangent,normal));
    vec3 mapped=texture(normal_map,texcoord).xyz*2.0-1.0;
    normal=normalize(mat3(tangent,bitangent,normal)*mapped);
    roughness=clamp(texture(roughness_map,texcoord).r,0.15,1.0);
  }
  vec3 sun=sun_direction;
  // Foliage cards are two-sided. Keep their back faces illuminated, while
  // opaque surfaces retain directional shading rather than abs(dot).
  float facing=dot(normal,sun);
  float diffuse=cutoff>0.0 ? abs(facing) : max(facing,0.0);
  float sky=clamp(normal.z*0.5+0.5,0.0,1.0);
  vec3 ambient=mix(vec3(0.16,0.13,0.10),vec3(0.34,0.42,0.55),sky);
  if(has_environment){
    float x=normal.x,y=normal.y,z=normal.z;
    ambient=max(vec3(0.0),sky_sh[0]*0.282095+
      sky_sh[1]*(0.488603*y)+sky_sh[2]*(0.488603*z)+sky_sh[3]*(0.488603*x)+
      sky_sh[4]*(1.092548*x*y)+sky_sh[5]*(1.092548*y*z)+
      sky_sh[6]*(0.315392*(3.0*z*z-1.0))+sky_sh[7]*(1.092548*x*z)+
      sky_sh[8]*(0.546274*(x*x-y*y)));
  }
  vec3 shadow_position=(sun_clip*vec4(world_position,1.0)).xyz*0.5+0.5;
  float sunlight=1.0;
  if(all(greaterThan(shadow_position,vec3(0.0))) &&
     all(lessThan(shadow_position,vec3(1.0)))){
    float bias=shadow_bias*(1.0+2.0*(1.0-abs(dot(geometric_normal,sun))));
    vec2 texel=1.0/vec2(textureSize(forest_shadow,0));
    sunlight=0.0;
    for(int x=-1;x<=1;x+=2) for(int y=-1;y<=1;y+=2)
      sunlight+=texture(forest_shadow,vec3(shadow_position.xy+vec2(x,y)*texel,
                                        shadow_position.z-bias))*0.25;
  }
  vec3 illumination=ambient+sun_colour*diffuse*sunlight;
  // Texture/readback bytes are sRGB; lighting and atmospheric mixing are
  // linear. Explicit conversion also works with our ordinary RGB targets.
  vec3 linear=pow(base.rgb,vec3(2.2))*illumination;
  // A bounded dielectric microfacet highlight adds material response without
  // screen-space reflections or a second scene render.
  vec3 v=normalize(eye-world_position), half_vector=normalize(v+sun);
  float nv=max(dot(normal,v),0.001), nl=max(dot(normal,sun),0.0);
  float nh=max(dot(normal,half_vector),0.0), vh=max(dot(v,half_vector),0.0);
  float a2=pow(roughness,4.0);
  float denom=nh*nh*(a2-1.0)+1.0;
  float distribution=a2/(3.14159265*denom*denom);
  float k=(roughness+1.0)*(roughness+1.0)*0.125;
  float visibility=(nv/(nv*(1.0-k)+k))*(nl/(nl*(1.0-k)+k));
  float fresnel=0.04+0.96*pow(1.0-vh,5.0);
  if(has_environment && cutoff==0.0) linear+=sun_colour*min(distribution*visibility*fresnel/(4.0*nv),2.0)*sunlight;
  float distance_to_eye=length(eye-world_position);
  float haze=1.0-exp(-max(distance_to_eye-25.0,0.0)*0.0035);
  linear=mix(linear,vec3(0.50,0.60,0.72),haze);
  if(has_environment) linear=linear/(vec3(1.0)+linear);
  color=vec4(pow(clamp(linear,0.0,1.0),vec3(1.0/2.2)),1.0);
}
"""

SHADOW_VERTEX = """#version 330
uniform mat4 sun_clip;
in vec3 position;
in vec2 uv;
out vec2 texcoord;
void main(){texcoord=uv;gl_Position=sun_clip*vec4(position,1.0);}
"""
SHADOW_FRAGMENT = """#version 330
uniform sampler2D albedo;
uniform float cutoff;
in vec2 texcoord;
void main(){
  if(cutoff>0.0 && textureLod(albedo,texcoord,0.0).a<cutoff) discard;
}
"""

SKY_VERTEX = """#version 330
out vec2 ndc;
void main(){
  vec2 p=vec2((gl_VertexID<<1)&2,gl_VertexID&2)*2.0-1.0;
  ndc=p;gl_Position=vec4(p,1.0,1.0);
}
"""
SKY_FRAGMENT = """#version 330
uniform mat4 inverse_clip;
uniform vec3 eye;
uniform sampler2D panorama;
uniform float rotation;
uniform float exposure;
in vec2 ndc;
out vec4 color;
void main(){
  vec4 p=inverse_clip*vec4(ndc,1.0,1.0);
  vec3 ray=normalize(p.xyz/p.w-eye);
  vec2 uv=vec2((atan(ray.y,ray.x)-rotation)/6.28318530718+0.5,
               1.0-acos(clamp(ray.z,-1.0,1.0))/3.14159265359);
  vec3 light=texture(panorama,uv).rgb*exposure;
  color=vec4(pow(light/(vec3(1.0)+light),vec3(1.0/2.2)),1.0);
}
"""


OVERLAY_VERTEX = """#version 330
uniform mat4 clip;
in vec3 position;
in vec3 tint;
out vec3 colour;
void main(){gl_Position=clip*vec4(position,1);gl_PointSize=2.5;colour=tint;}
"""
OVERLAY_FRAGMENT = """#version 330
in vec3 colour;
out vec4 color;
void main(){color=vec4(colour,1);}
"""


YUV_FRAGMENT = """#version 330
uniform sampler2D source;
uniform ivec2 size;
out float value;
vec3 pixel(ivec2 p){return texelFetch(source,ivec2(p.x,size.y-1-p.y),0).rgb;}
void main(){
  ivec2 p=ivec2(gl_FragCoord.xy);
  if(p.y<size.y){
    value=(16.0+219.0*dot(pixel(p),vec3(0.2126,0.7152,0.0722)))/255.0;
  }else{
    int half_width=size.x/2;
    int plane_rows=size.y/4;
    bool is_v=p.y-size.y>=plane_rows;
    int row=(p.y-size.y)%plane_rows;
    ivec2 q=ivec2(p.x%half_width,row*2+p.x/half_width)*2;
    vec3 rgb=(pixel(q)+pixel(q+ivec2(1,0))+pixel(q+ivec2(0,1))+pixel(q+ivec2(1,1)))*0.25;
    vec3 weights=is_v?vec3(0.5,-0.454153,-0.045847):vec3(-0.114572,-0.385428,0.5);
    value=(128.0+224.0*dot(rgb,weights))/255.0;
  }
}
"""


def compose(pos, orn):
    matrix = np.eye(4, dtype=np.float32)
    matrix[:3, :3] = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
    matrix[:3, 3] = pos
    return matrix


def planes(clip):
    return np.array(
        [
            clip[3] + clip[0],
            clip[3] - clip[0],
            clip[3] + clip[1],
            clip[3] - clip[1],
            clip[3] + clip[2],
            clip[3] - clip[2],
        ]
    )


def visible(plane, low, high):
    support = np.where(plane[:, None, :3] >= 0, high[None, :, :], low[None, :, :])
    return np.all(
        np.sum(support * plane[:, None, :3], axis=2) + plane[:, None, 3] >= 0, axis=0
    )


def equivalent_view(current, previous, corners, width, height, tolerance=0.35):
    """Reuse only within a bounded subpixel footprint of the cached viewpoint.

    Compare against the last rendered pose, never the preceding pose, so drift
    cannot accumulate. Only bounds in or entering the view contribute; moving cameras invalidate.
    """
    if previous is None:
        return False
    if np.array_equal(current, previous):
        return True
    if not len(corners):
        return False
    a, b = corners @ current.T, corners @ previous.T
    ahead = (a[:, 3] >= 0.05) & (b[:, 3] >= 0.05)
    # Corners projected outside both views cannot bound on-screen motion; their
    # near-zero denominators otherwise amplify harmless jitter enormously.
    in_a = np.all(np.abs(a[:, :2]) <= 1.1 * np.abs(a[:, 3:4]), axis=1)
    in_b = np.all(np.abs(b[:, :2]) <= 1.1 * np.abs(b[:, 3:4]), axis=1)
    ahead &= in_a | in_b
    if not ahead.any():
        return False
    a, b = a[ahead], b[ahead]
    shift = np.abs(a[:, :2] / a[:, 3:4] - b[:, :2] / b[:, 3:4])
    equivalent_view.last_shift = float(np.max(shift * [width / 2, height / 2]))
    return bool(equivalent_view.last_shift <= tolerance)


class GpuScene:
    def __init__(
        self, assets, robot=None, tile_metres=16, shadow_size=4096, environment=None
    ):
        _ensure_egl_vendor()
        try:
            self.context = moderngl.create_standalone_context(
                backend="egl", require=430
            )
        except Exception:
            self.context = moderngl.create_standalone_context(
                backend="egl", require=330
            )
        print("[gpu] " + self.context.info["GL_RENDERER"], flush=True)
        if "llvmpipe" in self.context.info["GL_RENDERER"].lower():
            raise RuntimeError(
                "GPU renderer selected software OpenGL; check EGL vendor configuration"
            )
        self.program = self.context.program(
            vertex_shader=VERTEX, fragment_shader=FRAGMENT
        )
        self.context.enable(moderngl.DEPTH_TEST)
        self.program["albedo"] = 0
        self.static = []
        self.indirect = {}
        self.materials = {}
        self.program["normal_map"] = 4
        self.program["roughness_map"] = 5
        self.program["has_material"] = False
        self.robot_draws = []
        self.robot_batch = None
        self.program["multipart"] = False
        self.sensor_draws = []
        self.object_draws = {}
        self.targets = {}
        self.static_targets = {}
        self.static_cache_hits = 0
        self.static_cache_misses = 0
        self.cache_diagnostics = {}
        self.readbacks = {}
        self.yuv_targets = {}
        self.yuv_program = self.context.program(
            vertex_shader=SKY_VERTEX, fragment_shader=YUV_FRAGMENT
        )
        self.yuv_program["source"] = 3
        self.yuv_vao = self.context.vertex_array(self.yuv_program, [])
        self.resources = []
        self.overlay_program = self.context.program(
            vertex_shader=OVERLAY_VERTEX, fragment_shader=OVERLAY_FRAGMENT
        )
        self.overlay_buffer = self.context.buffer(reserve=24)
        self.overlay_vao = self.context.vertex_array(
            self.overlay_program, [(self.overlay_buffer, "3f 3f", "position", "tint")]
        )
        self.resources.extend(
            [self.overlay_vao, self.overlay_buffer, self.overlay_program]
        )
        self.sky_program = None
        self.sun_direction = np.array([-0.55, -0.4, 0.85], dtype="f4")
        self.sun_direction /= np.linalg.norm(self.sun_direction)
        self.program["sun_direction"] = tuple(self.sun_direction)
        self.program["sun_colour"] = (0.85, 0.77, 0.63)
        self.program["has_environment"] = False
        if environment:
            self._load_environment(environment)
        self.robot = robot
        for mesh in assets:
            centre = mesh.vertices[mesh.faces].mean(axis=1)
            tile = np.floor(centre[:, :2] / tile_metres).astype(np.int32)
            _, group = np.unique(tile, axis=0, return_inverse=True)
            order = np.argsort(group, kind="stable")
            faces = mesh.faces[order]
            ids, first, count = np.unique(
                group[order], return_index=True, return_counts=True
            )
            draws = self._upload(mesh.vertices, faces, mesh.uv, mesh.rgba)
            if mesh.normal is not None or mesh.roughness is not None:
                textures = []
                for pixels, components in (
                    (
                        (
                            mesh.normal
                            if mesh.normal is not None
                            else np.array([[[128, 128, 255]]], np.uint8)
                        ),
                        3,
                    ),
                    (
                        (
                            mesh.roughness
                            if mesh.roughness is not None
                            else np.array([[220]], np.uint8)
                        ),
                        1,
                    ),
                ):
                    texture = self.context.texture(
                        (pixels.shape[1], pixels.shape[0]),
                        components,
                        np.flipud(pixels).tobytes(),
                        alignment=1,
                    )
                    texture.build_mipmaps()
                    texture.filter = (moderngl.LINEAR_MIPMAP_LINEAR, moderngl.LINEAR)
                    self.resources.append(texture)
                    textures.append(texture)
                self.materials[draws[0].glo] = textures
            points = mesh.vertices[faces]
            lows = np.minimum.reduceat(points.min(axis=1), first)
            highs = np.maximum.reduceat(points.max(axis=1), first)
            self.static.append((draws, first * 3, count * 3, lows, highs, mesh.cutoff))
            if self.context.version_code >= 430 and len(first) > 1:
                # ModernGL uses a 20-byte stride for both indexed and array
                # commands. Non-indexed fields are count, instances, first,
                # baseInstance, followed by one padding word.
                commands = np.zeros((len(first), 5), np.uint32)
                commands[:, 0] = count * 3
                commands[:, 1] = 1
                commands[:, 2] = first * 3
                buffer = self.context.buffer(reserve=commands.nbytes)
                self.indirect[draws[0].glo] = (commands, buffer)
                self.resources.append(buffer)
        self.bounds_low = (
            np.concatenate([s[3] for s in self.static])
            if self.static
            else np.empty((0, 3))
        )
        self.bounds_high = (
            np.concatenate([s[4] for s in self.static])
            if self.static
            else np.empty((0, 3))
        )
        self.cache_corners = np.array(
            [
                [x, y, z, 1]
                for low, high in zip(self.bounds_low, self.bounds_high)
                for x in (low[0], high[0])
                for y in (low[1], high[1])
                for z in (low[2], high[2])
            ],
            dtype="f4",
        ).reshape(-1, 4)
        if robot is not None:
            self._build_robot(robot)
            self._batch_robot()
        self._bake_shadows(shadow_size)

    def _load_environment(self, config):
        from .environment import read_hdr, environment_lighting

        rgb = read_hdr(config["hdri"])
        rotation = float(config.get("rotation_deg", 0))
        sh, sun, colour, exposure = environment_lighting(
            rgb, rotation, float(config.get("strength", 1))
        )
        self.sun_direction = sun
        self.program["sun_direction"] = tuple(sun)
        self.program["sun_colour"] = tuple(colour)
        self.program["sky_sh"].write(sh.tobytes())
        self.program["has_environment"] = True
        texture = self.context.texture(
            (rgb.shape[1], rgb.shape[0]),
            3,
            np.flipud(rgb).astype("f4").tobytes(),
            dtype="f4",
        )
        texture.filter = (moderngl.LINEAR, moderngl.LINEAR)
        texture.repeat_y = False
        self.sky_texture = texture
        self.sky_program = self.context.program(
            vertex_shader=SKY_VERTEX, fragment_shader=SKY_FRAGMENT
        )
        self.sky_program["panorama"] = 2
        self.sky_program["rotation"] = np.radians(rotation)
        self.sky_program["exposure"] = exposure
        self.sky_vao = self.context.vertex_array(self.sky_program, [])
        self.resources.extend([texture, self.sky_vao, self.sky_program])

    def _bake_shadows(self, size):
        """Bake static terrain once; every camera reuses the same alpha-aware map.

        Moving links receive forest shadows but are deliberately not baked as
        casters, which would leave stale silhouettes when the robot moves.
        """
        self.shadow = self.context.depth_texture((max(size, 1), max(size, 1)))
        self.shadow.compare_func = "<="
        self.shadow.filter = (moderngl.LINEAR, moderngl.LINEAR)
        self.shadow.repeat_x = self.shadow.repeat_y = False
        self.resources.append(self.shadow)
        self.program["forest_shadow"] = 1
        identity = np.eye(4, dtype=np.float32)
        self.program["sun_clip"].write(identity.tobytes())
        self.program["shadow_bias"] = 0.0
        target = self.context.framebuffer(depth_attachment=self.shadow)
        target.use()
        target.clear(depth=1)
        if self.static and size > 0:
            low = np.min([item[3].min(axis=0) for item in self.static], axis=0)
            high = np.max([item[4].max(axis=0) for item in self.static], axis=0)
            centre = (low + high) / 2
            sun = self.sun_direction
            radius = max(float(np.linalg.norm(high - low)), 1.0)
            view = (
                np.array(
                    p.computeViewMatrix(
                        (centre + sun * radius).tolist(), centre.tolist(), [0, 0, 1]
                    ),
                    np.float32,
                )
                .reshape(4, 4)
                .T
            )
            corners = np.array(
                [
                    [x, y, z, 1]
                    for x in (low[0], high[0])
                    for y in (low[1], high[1])
                    for z in (low[2], high[2])
                ]
            )
            light_points = (view @ corners.T).T[:, :3]
            lo, hi = light_points.min(axis=0) - 1, light_points.max(axis=0) + 1
            projection = np.eye(4, dtype=np.float32)
            projection[:3, :3] = np.diag(np.array([2, 2, -2]) / (hi - lo))
            projection[:3, 3] = -(hi + lo) / (hi - lo) * [1, 1, -1]
            clip = projection @ view
            self.program["sun_clip"].write(clip.T.tobytes())
            self.program["shadow_bias"] = float(0.025 / (hi[2] - lo[2]))
            program = self.context.program(
                vertex_shader=SHADOW_VERTEX, fragment_shader=SHADOW_FRAGMENT
            )
            program["sun_clip"].write(clip.T.tobytes())
            program["albedo"] = 0
            for (_, texture, count, buffer), _, _, _, _, cutoff in self.static:
                texture.use(0)
                program["cutoff"] = cutoff
                vao = self.context.vertex_array(
                    program, [(buffer, "3f 2f", "position", "uv")]
                )
                vao.render(moderngl.TRIANGLES, vertices=count)
                vao.release()
            program.release()
        target.release()

    def _upload(self, vertices, faces, uv, rgba):
        data = np.concatenate(
            [vertices[faces].reshape(-1, 3), uv[faces].reshape(-1, 2)], axis=1
        ).astype("f4")
        buffer = self.context.buffer(data.tobytes())
        vao = self.context.vertex_array(
            self.program, [(buffer, "3f 2f", "position", "uv")]
        )
        rgba = np.asarray(rgba, np.uint8).copy()
        rgba[..., :3] = np.rint(
            rgba[..., :3].astype("f4") * (rgba[..., 3:4].astype("f4") / 255)
        ).astype(np.uint8)
        texture = self.context.texture(
            (rgba.shape[1], rgba.shape[0]), 4, np.flipud(rgba).tobytes()
        )
        texture.build_mipmaps()
        texture.filter = (moderngl.LINEAR_MIPMAP_LINEAR, moderngl.LINEAR)
        texture.anisotropy = min(4.0, self.context.max_anisotropy)
        self.resources.extend([buffer, vao, texture])
        return vao, texture, len(data), buffer

    def _build_robot(self, robot):
        from tools.live import _COL

        for visual in p.getVisualShapeData(robot.body_id):
            link, kind, dims, path, lpos, lorn = visual[1:7]
            name = next((n for n, i in robot.link_index.items() if i == link), "")
            color = visual[7]
            for key, rgb in _COL:
                if key.lower() in name.lower():
                    color = (*rgb, 1)
                    break
            if kind == p.GEOM_MESH and path:
                path = path.decode() if isinstance(path, bytes) else path
                data = load_mesh(path)
                vertices = data["vertices"] * np.array(dims)
            else:
                from rove_sim.world.moving_rays import primitive

                mesh = primitive(kind, dims)
                if mesh is None or max(dims) < 1e-4:
                    continue
                vertices = np.asarray(mesh.vertices, np.float32)
                data = {
                    "faces": mesh.faces,
                    "uv": np.zeros((len(vertices), 2), np.float32),
                }
            rgba = (np.asarray(color) * 255).astype(np.uint8).reshape(1, 1, 4)
            draw = self._upload(vertices, data["faces"], data["uv"], rgba)
            self.robot_draws.append(
                [draw, link, compose(lpos, lorn), np.eye(4, dtype=np.float32)]
            )

    def _batch_robot(self):
        """One draw for articulated flat-colour links; upload transforms once.

        Static forest meshes retain their original independent material draws.
        """
        if not self.robot_draws or len(self.robot_draws) > 128:
            return
        blocks = []
        for number, (draw, _, _, _) in enumerate(self.robot_draws):
            vertices = np.frombuffer(draw[3].read(), 'f4').reshape(-1, 5)
            colour = np.frombuffer(draw[1].read(), np.uint8)[:3] / 255
            blocks.append(np.column_stack([vertices, np.tile(colour, (len(vertices), 1)), np.full(len(vertices), number)]))
        vertices = np.concatenate(blocks).astype('f4')
        buffer = self.context.buffer(vertices.tobytes())
        vao = self.context.vertex_array(self.program, [(buffer, '3f 2f 3f 1f', 'position', 'uv', 'vertex_colour', 'part_id')])
        white = self.context.texture((1, 1), 4, bytes([255,255,255,255]))
        self.resources.extend([buffer, vao, white])
        self.robot_batch = (vao, white)

    def update_robot(self):
        frames = {-1: compose(*base_link_pose(self.robot.body_id))}
        links = sorted({draw[1] for draw in self.robot_draws if draw[1] >= 0})
        if links:
            states = p.getLinkStates(
                self.robot.body_id, links, computeForwardKinematics=True
            )
            frames.update(
                {link: compose(st[4], st[5]) for link, st in zip(links, states)}
            )
        for draw in self.robot_draws:
            draw[3] = frames[draw[1]] @ draw[2]
        if self.robot_batch:
            models = np.tile(np.eye(4, dtype='f4'), (128, 1, 1))
            models[:len(self.robot_draws)] = [draw[3] for draw in self.robot_draws]
            self.program['parts'].write(models.transpose(0, 2, 1).copy().tobytes())

    def update_sensors(self, sensors):
        if not self.sensor_draws:
            import trimesh

            for sensor in sensors:
                if not hasattr(sensor, "detector_pose") or not hasattr(
                    sensor, "visual_extents"
                ):
                    continue
                mesh = trimesh.creation.box(extents=sensor.visual_extents)
                rgba = (
                    (np.asarray(sensor.visual_rgba) * 255)
                    .astype(np.uint8)
                    .reshape(1, 1, 4)
                )
                draw = self._upload(
                    np.asarray(mesh.vertices),
                    mesh.faces,
                    np.zeros((len(mesh.vertices), 2)),
                    rgba,
                )
                self.sensor_draws.append([draw, sensor, None])
        for item in self.sensor_draws:
            item[2] = compose(*item[1].detector_pose())

    def update_objects(self, world):
        import trimesh

        for obj in world.scene_objects:
            if obj.id in self.object_draws and self.object_draws[obj.id][2] != obj:
                self._drop_object(obj.id)
            if obj.id not in self.object_draws:
                if obj.shape == "sphere":
                    mesh = trimesh.creation.icosphere(
                        subdivisions=2, radius=obj.extents[0] / 2
                    )
                elif obj.shape == "cylinder":
                    mesh = trimesh.creation.cylinder(
                        radius=obj.extents[0] / 2, height=obj.extents[2], sections=24
                    )
                elif obj.shape == "mesh" and obj.mesh:
                    data = load_mesh(obj.mesh)
                    mesh = trimesh.Trimesh(
                        vertices=data["vertices"] * np.array(obj.extents),
                        faces=data["faces"],
                        process=False,
                    )
                else:
                    mesh = trimesh.creation.box(extents=obj.extents)
                rgba = (np.asarray(obj.rgba) * 255).astype(np.uint8).reshape(1, 1, 4)
                draw = self._upload(
                    np.asarray(mesh.vertices, np.float32),
                    mesh.faces,
                    np.zeros((len(mesh.vertices), 2)),
                    rgba,
                )
                self.object_draws[obj.id] = [draw, None, obj]
            self.object_draws[obj.id][1] = compose(
                *p.getBasePositionAndOrientation(world.objects[obj.id])
            )
        for oid in set(self.object_draws) - set(world.objects):
            self._drop_object(oid)

    def _drop_object(self, oid):
        draw, _, _ = self.object_draws.pop(oid)
        for resource in (draw[0], draw[1], draw[3]):
            self.resources.remove(resource)
            resource.release()

    def render(
        self,
        view,
        projection,
        width,
        height,
        stream=None,
        output="rgb",
        scene=True,
        points=None,
        lines=None,
        defer=False,
    ):
        if output not in ("rgb", "yuv420p"):
            raise ValueError("Unknown pixel format")
        if output == "yuv420p" and (width % 2 or height % 4):
            raise ValueError(
                "Packed YUV requires even width and height divisible by four"
            )
        key = (width, height, stream)
        if key not in self.targets:
            colour = self.context.texture((width, height), 3)
            depth = self.context.depth_renderbuffer((width, height))
            self.resources.extend([colour, depth])
            self.targets[key] = self.context.framebuffer([colour], depth)
        target = self.targets[key]
        target.use()
        self.shadow.use(1)
        target.clear(0.59, 0.69, 0.80, 1, depth=1)
        view = np.array(view, dtype=np.float32).reshape(4, 4).T
        projection = np.array(projection, dtype=np.float32).reshape(4, 4).T
        clip = projection @ view
        # View matrices are rigid transforms; no general matrix inversion.
        self.program["eye"] = tuple(-view[:3, :3].T @ view[:3, 3])
        self.program["clip"].write(clip.T.tobytes())
        self.program["multipart"] = False
        self.program["model"].write(np.eye(4, dtype=np.float32).tobytes())
        if scene:
            cache = self.static_targets.get(key)
            if cache is None:
                colour = self.context.texture((width, height), 3)
                depth = self.context.depth_renderbuffer((width, height))
                self.resources.extend([colour, depth])
                cache = [self.context.framebuffer([colour], depth), None]
                self.static_targets[key] = cache
            background, saved_clip = cache
            if not equivalent_view(clip, saved_clip, self.cache_corners, width, height):
                self.cache_diagnostics[str(stream)] = getattr(
                    equivalent_view, "last_shift", None
                )
                self.static_cache_misses += 1
                background.use()
                background.clear(0.59, 0.69, 0.80, 1, depth=1)
                frustum = planes(clip)
                tile_visibility = visible(frustum, self.bounds_low, self.bounds_high)
                offset = 0
                for (
                    (vao, texture, _, _),
                    first,
                    count,
                    low,
                    high,
                    cutoff,
                ) in self.static:
                    texture.use()
                    material = self.materials.get(vao.glo)
                    self.program["has_material"] = material is not None
                    if material:
                        material[0].use(4)
                        material[1].use(5)
                    self.program["cutoff"] = cutoff
                    selected = np.flatnonzero(
                        tile_visibility[offset : offset + len(first)]
                    )
                    offset += len(first)
                    if len(selected) > 1 and vao.glo in self.indirect:
                        commands, buffer = self.indirect[vao.glo]
                        buffer.orphan()
                        buffer.write(commands[selected].tobytes())
                        vao.render_indirect(
                            buffer, mode=moderngl.TRIANGLES, count=len(selected)
                        )
                    else:
                        for i in selected:
                            vao.render(
                                moderngl.TRIANGLES,
                                vertices=int(count[i]),
                                first=int(first[i]),
                            )
                if self.sky_program is not None:
                    # Draw at far depth after opaque/cutout geometry: hidden sky pixels
                    # are rejected before expensive panoramic sampling.
                    self.sky_program["inverse_clip"].write(
                        np.linalg.inv(clip).T.astype("f4").tobytes()
                    )
                    self.sky_program["eye"] = tuple(-view[:3, :3].T @ view[:3, 3])
                    self.sky_texture.use(2)
                    self.context.depth_func = "<="
                    self.sky_vao.render(moderngl.TRIANGLES, vertices=3)
                    self.context.depth_func = "<"
                cache[1] = clip.copy()
            else:
                self.static_cache_hits += 1
            self.context.copy_framebuffer(target, background)
            target.use()
            self.program["cutoff"] = 0
            self.program["has_material"] = False
            if self.robot_batch:
                self.program['multipart'] = True
                self.robot_batch[1].use()
                self.robot_batch[0].render(moderngl.TRIANGLES)
                self.program['multipart'] = False
            else:
                for draw, _, _, model in self.robot_draws:
                    self.program["model"].write(model.T.tobytes())
                    draw[1].use()
                    draw[0].render(moderngl.TRIANGLES)
            for draw, _, model in self.sensor_draws:
                self.program["model"].write(model.T.tobytes())
                draw[1].use()
                draw[0].render(moderngl.TRIANGLES)
            for draw, model, _ in self.object_draws.values():
                self.program["model"].write(model.T.tobytes())
                draw[1].use()
                draw[0].render(moderngl.TRIANGLES)
        else:
            target.clear(0.035, 0.045, 0.065, 1, depth=1)
        # Overlays are visible through the scene for inspecting scan footprints.
        # Camera feeds never receive these observer-only geometries.
        self.context.disable(moderngl.DEPTH_TEST)
        self.context.enable(moderngl.PROGRAM_POINT_SIZE)
        self.overlay_program["clip"].write(clip.T.tobytes())
        for data, mode in ((points, moderngl.POINTS), (lines, moderngl.LINES)):
            if data is not None and len(data):
                payload = np.ascontiguousarray(data, dtype="f4").tobytes()
                self.overlay_buffer.orphan(len(payload))
                self.overlay_buffer.write(payload)
                self.overlay_vao.render(mode, vertices=len(data))
        self.context.enable(moderngl.DEPTH_TEST)
        components = 3
        if output == "yuv420p":
            if key not in self.yuv_targets:
                texture = self.context.texture((width, height * 3 // 2), 1)
                self.yuv_targets[key] = self.context.framebuffer([texture])
                self.resources.append(texture)
            target.color_attachments[0].use(3)
            target = self.yuv_targets[key]
            target.use()
            self.context.disable(moderngl.DEPTH_TEST)
            self.yuv_program["size"] = (width, height)
            self.yuv_vao.render(moderngl.TRIANGLES, vertices=3)
            self.context.enable(moderngl.DEPTH_TEST)
            components = 1
        read_key = (*key, output)
        byte_count = width * height * (3 if components == 3 else 3 / 2)
        if stream is None:
            data = target.read(components=components, alignment=1)
        else:
            if read_key not in self.readbacks:
                self.readbacks[read_key] = (
                    [self.context.buffer(reserve=int(byte_count)) for _ in range(2)],
                    0,
                    False,
                )
            buffers, slot, ready = self.readbacks[read_key]
            target.read_into(buffers[slot], components=components, alignment=1)
            previous_buffer = buffers[1 - slot if ready else slot]
            self.readbacks[read_key] = (buffers, 1 - slot, True)
            if defer:

                def collect():
                    data = previous_buffer.read()
                    if output == "yuv420p":
                        return data
                    return (
                        np.frombuffer(data, np.uint8)
                        .reshape(height, width, 3)[::-1]
                        .copy()
                    )

                return collect
            data = previous_buffer.read()
        if output == "yuv420p":
            return data
        return np.frombuffer(data, np.uint8).reshape(height, width, 3)[::-1].copy()

    def close(self):
        for buffers, _, _ in self.readbacks.values():
            for buffer in buffers:
                buffer.release()
        self.yuv_vao.release()
        self.yuv_program.release()
        for framebuffer, _ in self.static_targets.values():
            framebuffer.release()
        for framebuffer in self.yuv_targets.values():
            framebuffer.release()
        for framebuffer in self.targets.values():
            framebuffer.release()
        for resource in self.resources:
            resource.release()
        self.program.release()
        self.context.release()
