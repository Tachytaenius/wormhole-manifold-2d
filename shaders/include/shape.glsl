const float tau = 6.28318530718;

uniform float wormholeThroatRadius;
uniform vec2 wormholeMouthAPosition;
uniform vec2 wormholeMouthBPosition;

uniform float gridSpacing;
uniform uint gridCells;
uniform float gridLineThickness;

vec3 rThetaToExtrinsicPosition(float r, float theta) {
	return vec3(
		wormholeThroatRadius * sqrt(r * r / (wormholeThroatRadius * wormholeThroatRadius) + 1.0) * cos(theta),
		wormholeThroatRadius * sqrt(r * r / (wormholeThroatRadius * wormholeThroatRadius) + 1.0) * sin(theta),
		wormholeThroatRadius * asinh(r / wormholeThroatRadius)
	);
}

vec2 rThetaToRealPosition(float r, float theta) {
	vec2 catenoid = vec2(
		wormholeThroatRadius * sqrt(r * r / (wormholeThroatRadius * wormholeThroatRadius) + 1.0) * cos(theta),
		wormholeThroatRadius * sqrt(r * r / (wormholeThroatRadius * wormholeThroatRadius) + 1.0) * sin(theta)
	);

	if (r >= 0.0) {
		return wormholeMouthAPosition + catenoid;
	} else {
		vec2 delta = wormholeMouthBPosition - wormholeMouthAPosition;
		vec2 direction = normalize(delta);
		vec2 parallel = direction * dot(catenoid, direction);
		vec2 perpendicular = catenoid - parallel;
		vec2 parallelFlipped = -parallel;
		vec2 catenoidFlipped = parallelFlipped + perpendicular;
		return wormholeMouthBPosition + catenoidFlipped;
	}
}

vec3 getRBasisExtrinsic(float r, float theta) {
	// float rDelta = 0.01;
	// return (rThetaToExtrinsicPosition(r + rDelta, theta) - rThetaToExtrinsicPosition(r, theta)) / rDelta;
	return vec3(
		sign(wormholeThroatRadius) * r * cos(theta) / sqrt(r * r + wormholeThroatRadius * wormholeThroatRadius),
		sign(wormholeThroatRadius) * r * sin(theta) / sqrt(r * r + wormholeThroatRadius * wormholeThroatRadius),
		abs(wormholeThroatRadius) / sqrt(wormholeThroatRadius * wormholeThroatRadius + r * r)
	);
}

vec3 getThetaBasisExtrinsic(float r, float theta) {
	// float thetaDelta = 0.01; // :3
	// return (rThetaToExtrinsicPosition(r, theta + thetaDelta) - rThetaToExtrinsicPosition(r, theta)) / thetaDelta;
	return vec3(
		-wormholeThroatRadius * sqrt(r * r / (wormholeThroatRadius * wormholeThroatRadius) + 1) * sin(theta),
		wormholeThroatRadius * sqrt(r * r / (wormholeThroatRadius * wormholeThroatRadius) + 1) * cos(theta),
		0.0
	);
}

// Not necessarily r and theta input
vec3 intrinsicToExtrinsicTangent(vec3 e1, vec3 e2, vec2 v) {
	return v.x * e1 + v.y * e2;
}

// Not necessarily r and theta output
vec2 extrinsicToIntrinsicTangent(vec3 e1, vec3 e2, vec3 v) {
	float uu = dot(e1, e1);
	float uv = dot(e1, e2);
	float vv = dot(e2, e2);
	float tu = dot(v, e1);
	float tv = dot(v, e2);

	float denominator = uu * vv - uv * uv;
	return vec2(
		(tu * vv - tv * uv) / denominator,
		(tv * uu - tu * uv) / denominator
	);
}

vec3 extrinsicToRealTangent(vec3 v, bool negative) {
	if (!negative) {
		return v;
	}
	vec2 delta = wormholeMouthBPosition - wormholeMouthAPosition;
	vec3 delta3D = vec3(delta.x, delta.y, 0.0);
	vec3 direction = normalize(delta3D);
	vec3 parallel = direction * dot(v, direction);
	vec3 perpendicular = v - parallel;
	vec3 parallelFlipped = -parallel;
	return parallelFlipped + perpendicular;
}

struct ChristoffelSymbols {
	float rThetaTheta;
	float thetaRTheta;
	float thetaThetaR;
};

ChristoffelSymbols getChristoffelSymbols(float r, float theta) {
	return ChristoffelSymbols (
		-r,
		r / (wormholeThroatRadius * wormholeThroatRadius + r * r),
		r / (wormholeThroatRadius * wormholeThroatRadius + r * r)
	);
}

vec3 hue2rgb(float hue) {
	return clamp(
		vec3(
			abs(hue * 6.0 - 3.0) - 1.0,
			2.0 - abs(hue * 6.0 - 2.0),
			2.0 - abs(hue * 6.0 - 4.0)
		),
		0.0, 1.0
	);
}

vec3 hsv2rgb(vec3 hsv) {
	return ((hue2rgb(hsv.x) - 1.0) * hsv.y + 1.0) * hsv.z;
}

vec3 sampleBackground(float r, float theta, vec2 realPosition, bool curvedMode) {
	// vec2 rg = realPosition / (gridSpacing * gridCells);
	vec2 cellPos = mod(realPosition, vec2(gridSpacing));
	float hue = mod(theta / tau, 1.0);
	float saturation = r > 0.0 ? 0.8 : 0.4;
	float value = max(0.1, 1.0 - abs(r) / 200.0);
	vec3 baseColour = hsv2rgb(vec3(hue, saturation, value));
	if (!curvedMode) {
		baseColour = vec3(0.0);
	}
	if (min(cellPos.x, cellPos.y) < gridLineThickness) {
		return baseColour + 0.5;
	}
	return baseColour;
}
