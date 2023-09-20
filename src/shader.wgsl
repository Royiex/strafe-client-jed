struct Camera {
	// from camera to screen
	proj: mat4x4<f32>,
	// from screen to camera
	proj_inv: mat4x4<f32>,
	// from world to camera
	view: mat4x4<f32>,
	// camera position
	cam_pos: vec4<f32>,
};

//group 0 is the camera
@group(0)
@binding(0)
var<uniform> camera: Camera;

struct SkyOutput {
	@builtin(position) position: vec4<f32>,
	@location(0) sampledir: vec3<f32>,
};

@vertex
fn vs_sky(@builtin(vertex_index) vertex_index: u32) -> SkyOutput {
	// hacky way to draw a large triangle
	let tmp1 = i32(vertex_index) / 2;
	let tmp2 = i32(vertex_index) & 1;
	let pos = vec4<f32>(
		f32(tmp1) * 4.0 - 1.0,
		f32(tmp2) * 4.0 - 1.0,
		1.0,
		1.0
	);

	// transposition = inversion for this orthonormal matrix
	let inv_model_view = transpose(mat3x3<f32>(camera.view[0].xyz, camera.view[1].xyz, camera.view[2].xyz));
	let unprojected = camera.proj_inv * pos;

	var result: SkyOutput;
	result.sampledir = inv_model_view * unprojected.xyz;
	result.position = pos;
	return result;
}

const MAX_ENTITY_INSTANCES=1024;
//group 1 is the model
@group(1)
@binding(0)
var<uniform> entity_transforms: array<mat4x4<f32>,MAX_ENTITY_INSTANCES>;
//var<uniform> entity_texture_transforms: array<mat3x3<f32>,MAX_ENTITY_INSTANCES>;
//my fancy idea is to create a megatexture for each model that includes all the textures each intance will need
//the texture transform then maps the texture coordinates to the location of the specific texture
//how to do no texture?
@group(1)
@binding(1)
var model_texture: texture_2d<f32>;
@group(1)
@binding(2)
var model_sampler: sampler;

struct EntityOutputTexture {
	@builtin(position) position: vec4<f32>,
	@location(1) texture: vec2<f32>,
	@location(2) normal: vec3<f32>,
	@location(3) view: vec3<f32>,
};
@vertex
fn vs_entity_texture(
	@builtin(instance_index) instance: u32,
	@location(0) pos: vec3<f32>,
	@location(1) texture: vec2<f32>,
	@location(2) normal: vec3<f32>,
) -> EntityOutputTexture {
	var position: vec4<f32> = entity_transforms[instance] * vec4<f32>(pos, 1.0);
	var result: EntityOutputTexture;
	result.normal = (entity_transforms[instance] * vec4<f32>(normal, 0.0)).xyz;
	result.texture=texture;//(entity_texture_transforms[instance] * vec3<f32>(texture, 1.0)).xy;
	result.view = position.xyz - camera.cam_pos.xyz;
	result.position = camera.proj * camera.view * position;
	return result;
}

//group 2 is the skybox texture
@group(2)
@binding(0)
var cube_texture: texture_cube<f32>;
@group(2)
@binding(1)
var cube_sampler: sampler;

@fragment
fn fs_sky(vertex: SkyOutput) -> @location(0) vec4<f32> {
	return textureSample(cube_texture, model_sampler, vertex.sampledir);
}

@fragment
fn fs_entity_texture(vertex: EntityOutputTexture) -> @location(0) vec4<f32> {
	let incident = normalize(vertex.view);
	let normal = normalize(vertex.normal);
	let d = dot(normal, incident);
	let reflected = incident - 2.0 * d * normal;

	let fragment_color = textureSample(model_texture, model_sampler, vertex.texture).rgb;
	let reflected_color = textureSample(cube_texture, cube_sampler, reflected).rgb;
	return vec4<f32>(mix(vec3<f32>(0.1) + 0.5 * reflected_color,fragment_color,1.0-pow(1.0-abs(d),2.0)), 1.0);
}
