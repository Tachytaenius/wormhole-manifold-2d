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

uniform bool initialModeCurved;
uniform float curvedToFlatR;
uniform float flatToCurvedRho;

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

vec2 rotate(vec2 v, float a) {
	float s = sin(a);
	float c = cos(a);
	mat2 m = mat2(c, s, -s, c);
	return m * v;
}

vec3 rotate(vec3 v, vec3 axis, float angle) {
	return v + sin(angle) * cross(axis, v) + (1.0 - cos(angle)) * cross(axis, cross(axis, v));
}

vec2 flip(vec2 v) { // TODO: Rename
	vec2 mouthDirection = normalize(wormholeMouthBPosition - wormholeMouthAPosition);
	vec2 parallel = mouthDirection * dot(v, mouthDirection);
	vec2 perpendicular = v - parallel;
	vec2 parallelFlipped = -parallel;
	return parallelFlipped + perpendicular;
}

vec4 rk4StateDeriv(float t, vec4 state) {
	vec2 pos = state.xy;
	vec2 vel = state.zw;
	ChristoffelSymbols christoffelSymbols = getChristoffelSymbols(pos.x, pos.y);
	return vec4(
		vel,
		vec2(
			-christoffelSymbols.rThetaTheta * vel.y * vel.y,
			-(
				christoffelSymbols.thetaRTheta * vel.x * vel.y +
				christoffelSymbols.thetaThetaR * vel.y * vel.x
			)
		)
	);
}

float vectorLengthTangent(vec2 position, vec2 tangent) {
	return sqrt(tangent.x * tangent.x + (position.x * position.x + wormholeThroatRadius * wormholeThroatRadius) * tangent.y * tangent.y);
}

vec2 normaliseTangentVector(vec2 position, vec2 tangent) {
	return tangent / vectorLengthTangent(position, tangent);
}

layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;
void computemain() {
	ivec2 rayMapSize = imageSize(rayMap); // Wrapped texture filtering is present on the x axis so optionally using less angles would be less convenient
	int rayMapX = int(gl_GlobalInvocationID.x);
	if (rayMapX >= rayMapSize.x) {
		return;
	}

	float rotateAngle = float(rayMapX) / float(rayMapSize.x) * tau + tau / 4.0;

	bool currentModeCurved = initialModeCurved;
	vec2 currentPosition;
	vec2 velocityIntrinsic;
	vec2 currentDirectionFlat;
	if (currentModeCurved) {
		float initialR = cameraPosition.x;
		float initialTheta = cameraPosition.y;
		vec3 initialRBasis = getRBasisExtrinsic(initialR, initialTheta);
		vec3 initialThetaBasis = getThetaBasisExtrinsic(initialR, initialTheta);
		vec3 initialNormal = normalize(cross(initialRBasis, initialThetaBasis));
		vec3 forwardDirectionExtrinsic = intrinsicToExtrinsicTangent(initialRBasis, initialThetaBasis, cameraForward);
		currentPosition = cameraPosition;
		vec3 currentDirectionExtrinsic = rotate(forwardDirectionExtrinsic, initialNormal, rotateAngle) * stepSize;
		velocityIntrinsic = extrinsicToIntrinsicTangent(initialRBasis, initialThetaBasis, currentDirectionExtrinsic);
	} else {
		currentPosition = cameraPosition;
		currentDirectionFlat = rotate(cameraForward, rotateAngle);
	}
	vec3 colourHere = vec3(0.0);
	for (int stepNumber = 0; stepNumber < stepCount; stepNumber++) {
		vec2 currentRealPosition;
		if (currentModeCurved) {
			currentRealPosition = rThetaToRealPosition(currentPosition.x, currentPosition.y);
		} else {
			currentRealPosition = currentPosition;
		}

		// Sample and store
		colourHere = sampleBackground(currentRealPosition);
		imageStore(
			rayMap,
			ivec2(rayMapX, stepNumber),
			vec4(colourHere, 1.0)
		);

		if (currentModeCurved) {
			// Move (direction is parallel transported)
			float r = currentPosition.x;
			float theta = currentPosition.y;

			// Euler
			vec2 newPosition = currentPosition + velocityIntrinsic; // Velocity is already at step size
			float velR = velocityIntrinsic.x;
			float velTheta = velocityIntrinsic.y;
			ChristoffelSymbols christoffelSymbols = getChristoffelSymbols(r, theta);
			float newVelR = velR - christoffelSymbols.rThetaTheta * velTheta * velTheta;
			float newVelTheta = velTheta - 2.0 * christoffelSymbols.thetaRTheta * velR * velTheta; // thetaRTheta is thetaThetaR, and we are moving our velocity vector so the two Christoffel symbol terms are the same, hence double
			velocityIntrinsic = vec2(newVelR, newVelTheta);
			currentPosition = newPosition;

			// Runge-Kutta 4 (for when the surrounding code was different; this is not maintained)
			// vec3 rBasis = getRBasisExtrinsic(r, theta);
			// vec3 thetaBasis = getThetaBasisExtrinsic(r, theta);
			// vec2 velocity = extrinsicToIntrinsicTangent(rBasis, thetaBasis, currentDirectionExtrinsic); // Intrinsic space
			// float timeStepSize = stepSize / stepCount;
			// vec4 state = vec4(currentPosition, velocity);
			// for (int i = 1; i <= stepCount; i++) {
			// 	float t = float(i) * timeStepSize;
			// 	// Runge-Kutta 4
			// 	vec4 k1 = rk4StateDeriv(t, state) * timeStepSize;
			// 	vec4 k2 = rk4StateDeriv(t + timeStepSize / 2.0, state + k1 / 2.0) * timeStepSize;
			// 	vec4 k3 = rk4StateDeriv(t + timeStepSize / 2.0, state + k2 / 2.0) * timeStepSize;
			// 	vec4 k4 = rk4StateDeriv(t + timeStepSize, state + k3) * timeStepSize;
			// 	state += (k1 + k2 * 2.0 + k3 * 2.0 + k4) / 6.0;
			// }
			// vec2 pos = state.xy;
			// vec2 vel = state.zw;
			// float newR = pos.x;
			// float newTheta = pos.y;
			// vec3 newRBasis = getRBasisExtrinsic(newR, newTheta);
			// vec3 newThetaBasis = getThetaBasisExtrinsic(newR, newTheta);
			// currentPosition = pos;
			// currentDirectionExtrinsic = normalize(intrinsicToExtrinsicTangent(newRBasis, newThetaBasis, vel));
		} else {
			// Move (we're in flat space)
			currentPosition += currentDirectionFlat * stepSize;
		}

		// Check for leaving current wormhole region
		if (currentModeCurved) {
			float r = currentPosition.x;
			if (abs(r) > curvedToFlatR) {
				float theta = currentPosition.y;

				vec3 rBasis = getRBasisExtrinsic(currentPosition.x, currentPosition.y);
				vec3 thetaBasis = getThetaBasisExtrinsic(currentPosition.x, currentPosition.y);
				vec3 currentDirectionExtrinsic = intrinsicToExtrinsicTangent(rBasis, thetaBasis, velocityIntrinsic);
				currentDirectionFlat = normalize(extrinsicToRealTangent(currentDirectionExtrinsic, r < 0.0).xy);
				currentPosition = rThetaToRealPosition(r, theta);
				currentModeCurved = false;
			}
		}

		// Check for entering a wormhole region
		if (!currentModeCurved) {
			float aDistance = distance(currentPosition, wormholeMouthAPosition);
			float bDistance = distance(currentPosition, wormholeMouthBPosition);
			vec2 mouthPosition = aDistance < bDistance ? wormholeMouthAPosition : wormholeMouthBPosition;

			vec2 delta = currentPosition - mouthPosition;
			float rho = length(delta);

			if (rho < flatToCurvedRho) {
				float rSign;
				vec2 direction = currentDirectionFlat;
				if (aDistance < bDistance) {
					rSign = 1.0;
				} else {
					rSign = -1.0;
					delta = flip(delta);
					direction = flip(direction);
				}

				float theta = atan(delta.y, delta.x);
				float r = sqrt(rho * rho - wormholeThroatRadius * wormholeThroatRadius) * rSign;
				vec3 rBasis = getRBasisExtrinsic(r, theta);
				vec3 thetaBasis = getThetaBasisExtrinsic(r, theta);

				currentPosition = vec2(r, theta);

				vec3 forwardsExtrinsic = normalize(vec3(direction, 0.0)) * stepSize;
				velocityIntrinsic = extrinsicToIntrinsicTangent(rBasis, thetaBasis, forwardsExtrinsic);

				currentModeCurved = true;
			}
		}
	}
	for (int y = stepCount; y < imageSize(rayMap).y; y++) {
		imageStore(
			rayMap,
			ivec2(rayMapX, y),
			vec4(colourHere, 1.0)
		);
	}
}
