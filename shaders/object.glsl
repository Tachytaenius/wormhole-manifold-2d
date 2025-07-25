#line 1

#ifdef VERTEX

uniform mat4 modelToClip;

layout(location = 0) in vec2 VertexPosition;

out vec2 vertexPosition;
out vec3 vertexPosition3D;

void vertexmain() {
	vertexPosition = VertexPosition;
	vertexPosition3D = rThetaToExtrinsicPosition(VertexPosition.x, VertexPosition.y);
	gl_Position = modelToClip * vec4(vertexPosition3D, 1.0);
}

#endif

#ifdef PIXEL

uniform sampler2D overlay;

uniform float wormholeCutoffR;

in vec2 vertexPosition;
in vec3 vertexPosition3D;

out vec4 fragmentColour;

void pixelmain() {
	vec2 overlayTextureCoords = vec2(
		vertexPosition.x / wormholeCutoffR * 0.5 + 0.5,
		vertexPosition.y / tau
	);
	vec4 overlayHere = Texel(overlay, overlayTextureCoords);
	vec2 realPosition = rThetaToRealPosition(vertexPosition.x, vertexPosition.y);
	fragmentColour = vec4(
		mix(
			sampleBackground(vertexPosition.x, vertexPosition.y, realPosition, true),
			overlayHere.rgb,
			overlayHere.a
		),
		1.0
	);
}

#endif
