#line 1
// Requires include/shape.glsl

// const float tau = 6.2831853071796; // No definition

#ifdef VERTEX

uniform mat4 modelToClip;

layout(location = 0) in vec2 VertexPosition;

out vec2 vertexPosition;
out vec3 vertexPosition3D;

void vertexmain() {
	vertexPosition = VertexPosition;
	vertexPosition3D = rThetaToEmbedPosition(VertexPosition.x, VertexPosition.y);
	gl_Position = modelToClip * vec4(vertexPosition3D, 1.0);
}

#endif

#ifdef PIXEL

uniform sampler2D overlay;

in vec2 vertexPosition;
in vec3 vertexPosition3D;

out vec4 fragmentColour;

void pixelmain() {
	vec4 overlayHere = Texel(overlay, vertexPosition / tau);
	fragmentColour = vec4(
		mix(
			sampleBackground(vertexPosition.x, vertexPosition.y, vertexPosition3D.xy),
			overlayHere.rgb,
			overlayHere.a
		),
		1.0
	);
}

#endif
