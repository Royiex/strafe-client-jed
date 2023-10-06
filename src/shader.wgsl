struct Camera {
	// from camera to screen
	proj: mat4x4<f32>,
	// from screen to camera
	proj_inv: mat4x4<f32>,
	// from world to camera
	view: mat4x4<f32>,
	// from camera to world
	view_inv: mat4x4<f32>,
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

	let inv_model_view = mat3x3<f32>(camera.view_inv[0].xyz, camera.view_inv[1].xyz, camera.view_inv[2].xyz);
	let unprojected = camera.proj_inv * pos;

	var result: SkyOutput;
	result.sampledir = inv_model_view * unprojected.xyz;
	result.position = pos;
	return result;
}

struct ModelInstance{
	transform:mat4x4<f32>,
	normal_transform:mat3x3<f32>,
	color:vec4<f32>,
}
//my fancy idea is to create a megatexture for each model that includes all the textures each intance will need
//the texture transform then maps the texture coordinates to the location of the specific texture
//group 1 is the model
const MAX_MODEL_INSTANCES=4096;
@group(2)
@binding(0)
var<uniform> model_instances: array<ModelInstance, MAX_MODEL_INSTANCES>;
@group(2)
@binding(1)
var model_texture: texture_2d<f32>;
@group(2)
@binding(2)
var model_sampler: sampler;

struct EntityOutputTexture {
	@builtin(position) position: vec4<f32>,
	@location(1) texture: vec2<f32>,
	@location(2) normal: vec3<f32>,
	@location(3) view: vec3<f32>,
	@location(4) color: vec4<f32>,
	@location(5) @interpolate(flat) model_color: vec4<f32>,
};
@vertex
fn vs_entity_texture(
	@builtin(instance_index) instance: u32,
	@location(0) pos: vec3<f32>,
	@location(1) texture: vec2<f32>,
	@location(2) normal: vec3<f32>,
	@location(3) color: vec4<f32>,
) -> EntityOutputTexture {
	var position: vec4<f32> = model_instances[instance].transform * vec4<f32>(pos, 1.0);
	var result: EntityOutputTexture;
	result.normal = model_instances[instance].normal_transform * normal;
	result.texture = texture;
	result.color = color;
	result.model_color = model_instances[instance].color;
	result.view = position.xyz - camera.view_inv[3].xyz;//col(3)
	result.position = camera.proj * camera.view * position;
	return result;
}

//group 2 is the skybox texture
@group(1)
@binding(0)
var cube_texture: texture_cube<f32>;
@group(1)
@binding(1)
var cube_sampler: sampler;

@fragment
fn fs_sky(vertex: SkyOutput) -> @location(0) vec4<f32> {
	return textureSample(cube_texture, cube_sampler, vertex.sampledir);
}

@fragment
fn fs_entity_texture(vertex: EntityOutputTexture) -> @location(0) vec4<f32> {
	let incident = normalize(vertex.view);
	let normal = normalize(vertex.normal);
	let d = dot(normal, incident);
	let reflected = incident - 2.0 * d * normal;

	let fragment_color = textureSample(model_texture, model_sampler, vertex.texture)*vertex.color;
	let reflected_color = textureSample(cube_texture, cube_sampler, reflected).rgb;
	return mix(vec4<f32>(vec3<f32>(0.05) + 0.2 * reflected_color,1.0),mix(vertex.model_color,vec4<f32>(fragment_color.rgb,1.0),fragment_color.a),1.0-pow(1.0-abs(d),2.0));
}
