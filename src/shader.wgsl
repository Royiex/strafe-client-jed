struct SkyOutput {
	@builtin(position) position: vec4<f32>,
	@location(0) sampledir: vec3<f32>,
};

struct Data {
	// from camera to screen
	proj: mat4x4<f32>,
	// from screen to camera
	proj_inv: mat4x4<f32>,
	// from world to camera
	view: mat4x4<f32>,
	// camera position
	cam_pos: vec4<f32>,
};
@group(0)
@binding(0)
var<uniform> r_data: Data;

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
	let inv_model_view = transpose(mat3x3<f32>(r_data.view[0].xyz, r_data.view[1].xyz, r_data.view[2].xyz));
	let unprojected = r_data.proj_inv * pos;

	var result: SkyOutput;
	result.sampledir = inv_model_view * unprojected.xyz;
	result.position = pos;
	return result;
}

struct EntityOutput {
	@builtin(position) position: vec4<f32>,
	@location(1) texture: vec2<f32>,
	@location(2) normal: vec3<f32>,
	@location(3) view: vec3<f32>,
};

@group(1)
@binding(0)
var<uniform> r_EntityTransform: mat4x4<f32>;

@vertex
fn vs_entity(
	@location(0) pos: vec3<f32>,
	@location(1) texture: vec2<f32>,
	@location(2) normal: vec3<f32>,
) -> EntityOutput {
	var position: vec4<f32> = r_EntityTransform * vec4<f32>(pos, 1.0);
	var result: EntityOutput;
	result.normal = (r_EntityTransform * vec4<f32>(normal, 0.0)).xyz;
	result.texture=texture;
	result.view = position.xyz - r_data.cam_pos.xyz;
	result.position = r_data.proj * r_data.view * position;
	return result;
}

@group(0)
@binding(1)
var r_texture: texture_cube<f32>;
@group(0)
@binding(2)
var r_sampler: sampler;

@fragment
fn fs_sky(vertex: SkyOutput) -> @location(0) vec4<f32> {
	return textureSample(r_texture, r_sampler, vertex.sampledir);
}

@fragment
fn fs_entity(vertex: EntityOutput) -> @location(0) vec4<f32> {
	let incident = normalize(vertex.view);
	let normal = normalize(vertex.normal);
	let d = dot(normal, incident);
	let reflected = incident - 2.0 * d * normal;

	let dir = vec3<f32>(-1.0)+2.0*vec3<f32>(vertex.texture.x,0.0,vertex.texture.y);
	let texture_color = textureSample(r_texture, r_sampler, dir).rgb;
	let reflected_color = textureSample(r_texture, r_sampler, reflected).rgb;
	return vec4<f32>(mix(vec3<f32>(0.1) + 0.5 * reflected_color,texture_color,1.0-pow(1.0-abs(d),2.0)), 1.0);
}
