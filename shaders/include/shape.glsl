#line 1

const float tau = 6.28318530718;

uniform float minor;
uniform float major;

vec3 rThetaToEmbedPosition(float r, float theta) {
	return vec3(
		(major + minor * cos(theta)) * cos(r),
		(major + minor * cos(theta)) * sin(r),
		minor * sin(theta)
	);
}

vec2 rThetaToRealPosition(float r, float theta) {
	vec2 torus = vec2(
		(major + minor * cos(theta)) * cos(r),
		(major + minor * cos(theta)) * sin(r)
	);

	return torus;
}

struct ChristoffelSymbols {
	float rThetaR;
	float rRTheta;
	float thetaRR;
};

ChristoffelSymbols getChristoffelSymbols(float r, float theta) {
	return ChristoffelSymbols (
		- (minor * sin(theta)) / (major + minor * cos(theta)),
		- (minor * sin(theta)) / (major + minor * cos(theta)),
		((major + minor * cos(theta)) * sin(theta)) / minor
	);
}

vec3 hsv2rgb(vec3 c) {
	vec4 k = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
	vec3 p = abs(fract(c.xxx + k.xyz) * 6.0 - k.www);
	return c.z * mix(k.xxx, clamp(p - k.xxx, 0.0, 1.0), c.y);
}

vec3 sampleBackground(float r, float theta, vec2 realPosition) {
	float a = r;
	float b = theta;
	vec3 col = hsv2rgb(vec3(a / tau, cos(b) * 0.5 + 0.5, sin(b) * 0.5 + 0.5));
	float n = 8.0;
	float s = 0.1;
	// if (mod(r / tau * n * 2.0, 1.0) < s && mod(theta / tau * n * 4.0, 1.0) < 0.5 || mod(theta / tau * n, 1.0) < s) {
	if (mod(r / tau * n, 1.0) < s || mod(theta / tau * n, 1.0) < s) {
		return 1.0 - col;
	}
	return col;
}
