local vec3 = require("lib.mathsies").vec3

local consts = {}

consts.tau = math.pi * 2

consts.zoom = 1.5 -- CHanges size of the camera indicator too, but doesn't matter
consts.stepSize = 0.5
consts.wormholeCutoffGradient = 0.05
consts.wormholeCutoffRhoMultiplier = 0.01

consts.forwardVector = vec3(0, 0, 1)
consts.upVector = vec3(0, 1, 0)
consts.rightVector = vec3(1, 0, 0)

consts.objectVertexFormat = {
	{name = "VertexPosition", format = "floatvec2", location = 0}
}

return consts
