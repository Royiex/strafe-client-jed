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

struct GroundOutput {
	@builtin(position) position: vec4<f32>,
	@location(4) pos: vec3<f32>,
};

@vertex
fn vs_ground(@builtin(vertex_index) vertex_index: u32) -> GroundOutput {
	// hacky way to draw two triangles that make a square
	let tmp1 = i32(vertex_index)/2-i32(vertex_index)/3;
	let tmp2 = i32(vertex_index)&1;
	let pos = vec3<f32>(
		f32(tmp1) * 2.0 - 1.0,
		0.0,
		f32(tmp2) * 2.0 - 1.0
	) * 160.0;

	var result: GroundOutput;
	result.pos = pos;
	result.position = r_data.proj * r_data.view * vec4<f32>(pos, 1.0);
	return result;
}

struct EntityOutput {
	@builtin(position) position: vec4<f32>,
	@location(1) texture: vec2<f32>,
	@location(2) normal: vec3<f32>,
	@location(3) view: vec3<f32>,
};

struct TransformData {
	transform: mat4x4<f32>,
	depth: mat4x4<f32>,
	use_depth: vec4<f32>,
};

@group(1)
@binding(0)
var<uniform> transform: TransformData;

@vertex
fn vs_entity(
	@location(0) pos: vec3<f32>,
	@location(1) texture: vec2<f32>,
	@location(2) normal: vec3<f32>,
) -> EntityOutput {
	var position_depth: vec4<f32> = transform.depth * vec4<f32>(pos, 1.0);
	var position_depth_0: vec4<f32> = position_depth;
	position_depth_0.z=0.0;
	var position: vec4<f32> = transform.transform * mix(position_depth,position_depth_0,transform.use_depth);

	var result: EntityOutput;
	result.normal = (transform.transform * mix(vec4<f32>(normal,0.0),vec4<f32>(0.0,0.0,1.0,0.0),transform.use_depth.z)).xyz;
	result.texture=texture;
	result.view = position.xyz - r_data.cam_pos.xyz;
	var screen_position: vec4<f32> = r_data.proj * r_data.view * position;
	result.position = mix(screen_position,position_depth,transform.use_depth);
	return result;
}

struct SquareOutput {
	@builtin(position) position: vec4<f32>,
	@location(2) normal: vec3<f32>,
	@location(3) view: vec3<f32>,
	@location(4) pos: vec3<f32>,
};
@vertex
fn vs_square(@builtin(vertex_index) vertex_index: u32) -> SquareOutput {
	// hacky way to draw two triangles that make a square
	let tmp1 = i32(vertex_index)/2-i32(vertex_index)/3;
	let tmp2 = i32(vertex_index)&1;
	let pos = vec3<f32>(
		(f32(tmp1) - 0.5)*1.8,
		f32(tmp2) - 0.5,
		0.0
	);

	var result: SquareOutput;
	result.normal = vec3<f32>(0.0,0.0,1.0);
	result.pos = (transform.transform * vec4<f32>(pos, 1.0)).xyz;
	result.view = result.pos - r_data.cam_pos.xyz;
	result.position = r_data.proj * r_data.view * transform.transform * vec4<f32>(pos, 1.0);
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

fn modulo_euclidean (a: f32, b: f32) -> f32 {
	var m = a % b;
	if (m < 0.0) {
		if (b < 0.0) {
			m -= b;
		} else {
			m += b;
		}
	}
	return m;
}

@fragment
fn fs_ground(vertex: GroundOutput) -> @location(0) vec4<f32> {
	let dir = vec3<f32>(-1.0)+vec3<f32>(modulo_euclidean(vertex.pos.x/16.,1.0),0.0,modulo_euclidean(vertex.pos.z/16.,1.0))*2.0;
	return vec4<f32>(textureSample(r_texture, r_sampler, dir).rgb, 1.0);
}

@fragment
fn fs_checkered(vertex: SquareOutput) -> @location(0) vec4<f32> {
	let voxel_parity: f32 = f32(
		u32(modulo_euclidean(vertex.pos.x,2.0)<1.0)
		^ u32(modulo_euclidean(vertex.pos.y,2.0)<1.0)
		//^ u32(modulo_euclidean(vertex.pos.z,2.0)<1.0)
	);

	let incident = normalize(vertex.view);
	let normal = normalize(vertex.normal);
	let d = dot(normal, incident);
	let reflected = incident - 2.0 * d * normal;

	let texture_color = vec3<f32>(1.0)*voxel_parity;
	let reflected_color = textureSample(r_texture, r_sampler, reflected).rgb;
	return vec4<f32>(mix(vec3<f32>(0.1) + 0.5 * reflected_color,texture_color,1.0-pow(1.0-abs(d),2.0)), 1.0);
}

@fragment
fn fs_overwrite(vertex: SquareOutput) {}