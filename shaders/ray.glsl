const float tau = 6.28318530718;

uniform layout(rgba8) image2D rayMap;

uniform int stepCount;
uniform float stepSize;

uniform vec2 cameraPosition;
uniform vec2 cameraForward;

uniform float gridSpacing;
uniform uint gridCells;
uniform float gridLineThickness;

uniform float wormholeThroatRadius;
uniform vec2 wormholeMouthAPosition;
uniform vec2 wormholeMouthBPosition;

vec3 rThetaToEmbedPosition(float r, float theta) {
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

vec3 getRBasisEmbed(float r, float theta) {
	float rDelta = 0.1;
	return (rThetaToEmbedPosition(r + rDelta, theta) - rThetaToEmbedPosition(r, theta)) / rDelta;
}

vec3 getThetaBasisEmbed(float r, float theta) {
	float thetaDelta = 0.1; // :3
	return (rThetaToEmbedPosition(r, theta + thetaDelta) - rThetaToEmbedPosition(r, theta)) / thetaDelta;
}

// Not necessarily r and theta input
vec3 intrinsicToEmbedTangent(vec3 e1, vec3 e2, vec2 v) {
	return v.x * e1 + v.y * e2;
}

// Not necessarily r and theta output
vec2 embedToIntrinsicTangent(vec3 e1, vec3 e2, vec3 v) {
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

vec3 sampleBackground(vec2 position) {
	vec2 rg = position.xy / (gridSpacing * gridCells);
	// vec2 rg = vec2(1.0, 0.0);
	// if (
	// 	distance(position, wormholeMouthAPosition) >
	// 	distance(position, wormholeMouthBPosition)
	// ) {
	// 	rg.rg = rg.gr;
	// }
	vec2 cellPos = mod(position.xy, vec2(gridSpacing));
	if (min(cellPos.x, cellPos.y) < gridLineThickness) {
		return vec3(rg, 1.0);
	}
	return vec3(rg, 0.0);
}

vec3 rotate(vec3 v, vec3 axis, float angle) {
	return v + sin(angle) * cross(axis, v) + (1.0 - cos(angle)) * cross(axis, cross(axis, v));
}

layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;
void computemain() {
	ivec2 rayMapSize = imageSize(rayMap); // Wrapped texture filtering is present on the x axis so optionally using less angles would be less convenient
	int rayMapX = int(gl_GlobalInvocationID.x);
	if (rayMapX >= rayMapSize.x) {
		return;
	}

	vec3 colourHere = vec3(0.0);
	vec2 currentPosition = cameraPosition;
	float initialR = cameraPosition.x;
	float initialTheta = cameraPosition.y;
	vec3 initialRBasis = getRBasisEmbed(initialR, initialTheta);
	vec3 initialThetaBasis = getThetaBasisEmbed(initialR, initialTheta);
	vec3 initialNormal = normalize(cross(initialRBasis, initialThetaBasis)); // Same TODO as the Lua side
	float rotateAngle = float(rayMapX) / float(rayMapSize.x) * tau + tau / 4.0;
	vec3 forwardDirectionEmbed = intrinsicToEmbedTangent(initialRBasis, initialThetaBasis, cameraForward);
	vec3 currentDirectionEmbed = rotate(forwardDirectionEmbed, initialNormal, rotateAngle);
	for (int stepNumber = 0; stepNumber < stepCount; stepNumber++) {
		// Sample and store
		vec2 currentRealPosition = rThetaToRealPosition(currentPosition.x, currentPosition.y);
		colourHere = sampleBackground(currentRealPosition);
		imageStore(
			rayMap,
			ivec2(rayMapX, stepNumber),
			vec4(colourHere, 1.0)
		);

		// Move (direction is parallel transported) (TODO: What happens if we don't use the Christoffel symbols?)
		float r = currentPosition.x;
		float theta = currentPosition.y;
		vec3 rBasis = getRBasisEmbed(r, theta);
		vec3 thetaBasis = getThetaBasisEmbed(r, theta);
		vec3 stepEmbed = currentDirectionEmbed * stepSize;
		vec2 stepIntrinsic = embedToIntrinsicTangent(rBasis, thetaBasis, stepEmbed);
		vec2 newPosition = currentPosition + stepIntrinsic;
		float newR = newPosition.x;
		float newTheta = newPosition.y;
		float stepR = stepIntrinsic.x;
		float stepTheta = stepIntrinsic.y;
		ChristoffelSymbols christoffelSymbols = getChristoffelSymbols(r, theta);
		float newStepR = stepR - christoffelSymbols.rThetaTheta * stepTheta * stepTheta;
		float newStepTheta = stepTheta - 2.0 * christoffelSymbols.thetaRTheta * stepR * stepTheta; // thetaRTheta is thetaThetaR, and we are moving our displacement vector so the two Christoffel symbol terms are the same, hence double
		vec2 newStep = vec2(newStepR, newStepTheta);
		vec3 newRBasis = getRBasisEmbed(newR, newTheta);
		vec3 newThetaBasis = getThetaBasisEmbed(newR, newTheta);
		currentDirectionEmbed = normalize(intrinsicToEmbedTangent(newRBasis, newThetaBasis, newStep));
		currentPosition = newPosition;
	}
	for (int y = stepCount; y < imageSize(rayMap).y; y++) {
		imageStore(
			rayMap,
			ivec2(rayMapX, y),
			vec4(colourHere, 1.0)
		);
	}
}
