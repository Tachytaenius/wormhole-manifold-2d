const float tau = 6.28318530718;

uniform float stepSize;
uniform int stepCount;
uniform sampler2D rayMap;

out vec4 fragmentColour;

void pixelmain() {
	vec2 screen = love_PixelCoord.xy - love_ScreenSize.xy / 2.0;
	float angle = atan(screen.y, screen.x);
	float dist = length(screen);
	float rayMapX = angle / tau;
	float rayMapY = dist / (stepSize * stepCount);
	fragmentColour = Texel(rayMap, vec2(rayMapX, rayMapY));
}
