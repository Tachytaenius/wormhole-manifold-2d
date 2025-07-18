-- TODO: More varied background? So that it is easier to see just how warped your view becomes
-- TODO: Match geodesics on Wikipedia page for Ellis wormhole. What am I missing?? Lightlike/timelike/spacelike geodesics...?

local mathsies = require("lib.mathsies")
local vec2 = mathsies.vec2
local vec3 = mathsies.vec3
local quat = mathsies.quat
local mat4 = mathsies.mat4

local consts = require("consts")

local camera
local embedCamera
local wormhole
local time

local surfaceMesh, surfaceResolution
local objectCanvas
local objectOverlayTexture

local rayShader
local sceneShader
local objectShader

local rayMap
local dummyTexture

local stepCount

local overviewMode

local function euler(t, x, dt, f)
	x = x + f(t, x) * dt
	return x
end

local function rk4(t, x, dt, f)
	-- Not using divisions since the x value might not support it
	local k1 = f(t, x) * dt
	local k2 = f(t + dt * 0.5, x + k1 * 0.5) * dt
	local k3 = f(t + dt * 0.5, x + k2 * 0.5) * dt
	local k4 = f(t + dt, x + k3) * dt
	x = x + (k1 + k2 * 2 + k3 * 2 + k4) * (1 / 6)
	return x
end

local function sign(x)
	return x < 0 and -1 or x == 0 and 0 or 1
end

local function asinh(x)
	return math.log(x + math.sqrt(x ^ 2 + 1))
end

local function rThetaToExtrinsicPosition(r, theta)
	return vec3( -- Ellis wormhole (catenoid)
		wormhole.throatRadius * math.sqrt(r ^ 2 / wormhole.throatRadius ^ 2 + 1) * math.cos(theta),
		wormhole.throatRadius * math.sqrt(r ^ 2 / wormhole.throatRadius ^ 2 + 1) * math.sin(theta),
		wormhole.throatRadius * asinh(r / wormhole.throatRadius)
	)
end

local function vectorLengthTangent(position, tangent)
	-- Line element of the metric tensor
	local r, theta = vec2.components(position)
	local dR, dTheta = vec2.components(tangent)
	return math.sqrt(dR ^ 2 + (r ^ 2 + wormhole.throatRadius ^ 2) * dTheta ^ 2)
end

local function normaliseTangentVector(position, tangent)
	return tangent / vectorLengthTangent(position, tangent)
end

local function innerProduct(position, tangentA, tangentB)
	-- From the metric tensor
	local r, theta = vec2.components(position)
	local dRA, dThetaA = vec2.components(tangentA)
	local dRB, dThetaB = vec2.components(tangentB)
	return dRA * dRB + (wormhole.throatRadius ^ 2 + r ^ 2) * dThetaA * dThetaB
end

local function perpendicularTangentVector(position, tangent) -- Can be negated for the opposite direction
	local conversion = math.sqrt(wormhole.throatRadius ^ 2 + position.x ^ 2)
	-- Inner product of return value with original tangent should be, mathematically, 0
	return vec2(-tangent.y * conversion, tangent.x / conversion)
end

-- Wormhole cutoff rho grows linearly as wormhole throat radius does (all else being constant), which is nice
local function getWormholeCutoffRho(inside)
	-- rho = sqrt(r ^ 2 + wormhole.throatRadius ^ 2)
	-- Given z=f(rho), the following is when f's derivative reaches the wormhole cutoff gradient
	-- z from rho is wormhole.throatRadius * acosh(r / wormhole.throatRadius)
	local cutoff = (wormhole.throatRadius * math.sqrt(1 + consts.wormholeCutoffGradient ^ 2)) / consts.wormholeCutoffGradient
	if inside then
		return cutoff * (1 + consts.wormholeCutoffRhoMultiplier)
	else
		return cutoff * (1 - consts.wormholeCutoffRhoMultiplier)
	end
end

local function rThetaToRealPosition(r, theta)
	local catenoid = vec2(
		wormhole.throatRadius * math.sqrt(r ^ 2 / wormhole.throatRadius ^ 2 + 1) * math.cos(theta),
		wormhole.throatRadius * math.sqrt(r ^ 2 / wormhole.throatRadius ^ 2 + 1) * math.sin(theta)
	)

	if r >= 0 then
		return wormhole.mouthAPosition + catenoid
	else
		local delta = wormhole.mouthBPosition - wormhole.mouthAPosition
		local direction = vec2.normalise(delta)
		local parallel = direction * vec2.dot(catenoid, direction)
		local perpendicular = catenoid - parallel
		local parallelFlipped = -parallel
		local catenoidFlipped = parallelFlipped + perpendicular
		return wormhole.mouthBPosition + catenoidFlipped
	end
end

local function getRBasisIntrinsic(r, theta)
	return vec2(1, 0)
end

local function getThetaBasisIntrinsic(r, theta) -- Will have a length of 1, according to the line element
	return vec2(0, 1 / math.sqrt(wormhole.throatRadius ^ 2 + r ^ 2))
end

local function getRBasisExtrinsic(r, theta)
	-- local rDelta = 0.0001
	-- return (rThetaToExtrinsicPosition(r + rDelta, theta) - rThetaToExtrinsicPosition(r, theta)) / rDelta
	return vec3(
		sign(wormhole.throatRadius) * r * math.cos(theta) / math.sqrt(r * r + wormhole.throatRadius * wormhole.throatRadius),
		sign(wormhole.throatRadius) * r * math.sin(theta) / math.sqrt(r * r + wormhole.throatRadius * wormhole.throatRadius),
		math.abs(wormhole.throatRadius) / math.sqrt(wormhole.throatRadius * wormhole.throatRadius + r * r)
	)
end

local function getThetaBasisExtrinsic(r, theta)
	-- local thetaDelta = 0.0001 -- :3
	-- return (rThetaToExtrinsicPosition(r, theta + thetaDelta) - rThetaToExtrinsicPosition(r, theta)) / thetaDelta
	return vec3(
		-wormhole.throatRadius * math.sqrt(r * r / (wormhole.throatRadius * wormhole.throatRadius) + 1) * math.sin(theta),
		wormhole.throatRadius * math.sqrt(r * r / (wormhole.throatRadius * wormhole.throatRadius) + 1) * math.cos(theta),
		0
	)
end

-- Not necessarily r and theta input
local function intrinsicToExtrinsicTangent(e1, e2, v)
	return v.x * e1 + v.y * e2
end

-- Not necessarily r and theta output
local function extrinsicToIntrinsicTangent(e1, e2, v)
	-- Simplified from a 4D->3D version from somebody else's work
	local uu = vec3.dot(e1, e1)
	local uv = vec3.dot(e1, e2)
	local vv = vec3.dot(e2, e2)
	local tu = vec3.dot(v, e1)
	local tv = vec3.dot(v, e2)

	local denominator = uu * vv - uv * uv
	return vec2(
		(tu * vv - tv * uv) / denominator,
		(tv * uu - tu * uv) / denominator
	)
end

local function changeIntrinsicBasesTangent(e1, e2, v)
	local uu = vec2.dot(e1, e1)
	local uv = vec2.dot(e1, e2)
	local vv = vec2.dot(e2, e2)
	local tu = vec2.dot(v, e1)
	local tv = vec2.dot(v, e2)

	local denominator = uu * vv - uv * uv
	return vec2(
		(tu * vv - tv * uv) / denominator,
		(tv * uu - tu * uv) / denominator
	)
end

local function extrinsicToRealTangent(v, negative)
	if negative then
		local delta = wormhole.mouthBPosition - wormhole.mouthAPosition
		local delta3D = vec3(delta.x, delta.y, 0)
		local direction = vec3.normalise(delta3D)
		local parallel = direction * vec2.dot(v, direction)
		local perpendicular = v - parallel
		local parallelFlipped = -parallel
		return parallelFlipped + perpendicular
	end
	return vec3.clone(v)
end

local function getChristoffelSymbols(r, theta) -- Second kind
	-- The nonzero ones. Found in a paper about the Ellis wormhole. We use theta as their phi, and assume their theta is at tau / 4
	-- We also exclude ones using their theta because we never move that coordinate
	-- https://physicspages.com/pdf/Relativity/Christoffel%20symbols%20for%20wormhole%20metric.pdf
	local christoffelRThetaTheta = -r
	local christoffelThetaRTheta = r / (wormhole.throatRadius ^ 2 + r ^ 2)
	local christoffelThetaThetaR = christoffelThetaRTheta
	return
		christoffelRThetaTheta,
		christoffelThetaRTheta,
		christoffelThetaThetaR
end

local function normaliseOrZero2(v) -- 2 for vec2
	local magnitude = #v
	if magnitude == 0 then
		return vec2()
	end
	return v / magnitude
end

local function normaliseOrZero3(v) -- 3 for vec3
	local magnitude = #v
	if magnitude == 0 then
		return vec3()
	end
	return v / magnitude
end

local function limitVectorLength3(v, m)
	local l = #v
	if l > m then
		return normaliseOrZero3(v) * m
	end
	return vec3.clone(v)
end

local function moveVectorToTarget2(current, target, rate, dt) -- 2 for vec2
	local currentToTarget = target - current
	local direction = normaliseOrZero2(currentToTarget)
	local distance = #currentToTarget
	local newCurrentToTarget = direction * math.max(0, distance - rate * dt)
	return target - newCurrentToTarget
end

local function handleVelocity(current, target, dt, acceleration, maxSpeed)
	return moveVectorToTarget2(current, target, acceleration, dt)
end

function love.load()
	rayShader = love.graphics.newComputeShader(
		love.filesystem.read("shaders/include/shape.glsl") ..
		love.filesystem.read("shaders/ray.glsl")
	)
	objectShader = love.graphics.newShader(
		love.filesystem.read("shaders/include/shape.glsl") ..
		love.filesystem.read("shaders/object.glsl")
	)
	sceneShader = love.graphics.newShader("shaders/scene.glsl")
	local width, height = love.graphics.getWidth()
	local centreToCorner = #vec2(width, height) / 2 -- Order of operations doesn't matter
	local padding = 4
	stepCount = math.ceil((centreToCorner + padding) / consts.stepSize)
	rayMap = love.graphics.newCanvas(3072, stepCount, {computewrite = true})
	rayMap:setFilter("linear", "linear")
	rayMap:setWrap("repeat", "clamp") -- x is angle, y is distance
	dummyTexture = love.graphics.newImage(love.image.newImageData(1, 1))
	objectCanvas = love.graphics.newCanvas(400, 400)
	objectOverlayTexture = love.graphics.newCanvas(4096, 4096, {computewrite = true})
	objectOverlayTexture:setFilter("nearest")

	wormhole = {
		mouthAPosition = vec2(100, 200),
		mouthBPosition = vec2(1700, 200),
		throatRadius = 30
	}
	local position = vec2(40, 0)
	local forward = normaliseTangentVector(position, vec2(1, 0))
	-- Commented out are the (not-going-to-be-maintained) initial conditions for a spiralling into the wormhole and coming out again, found from working in GeodesicViewer
	-- Timestep variation causes varied results even with rk4 with 200 steps.
	-- GeodesicViewer can be found at https://github.com/tauzero7/GeodesicViewer
	-- local r, theta = vec2.components(position)
	-- local rBasisIntrinsic = getRBasisIntrinsic(r, theta)
	-- local thetaBasisIntrinsic = getThetaBasisIntrinsic(r, theta)
	-- local forwardCartesian = changeIntrinsicBasesTangent(rBasisIntrinsic, thetaBasisIntrinsic, forward)
	-- local forwardCartesianRotated = vec2.rotate(forwardCartesian, math.rad(216.87100000))
	-- forward = forwardCartesianRotated.x * rBasisIntrinsic + forwardCartesianRotated.y * thetaBasisIntrinsic
	camera = {
		mode = "curved", -- "curved" or "flat"
		position = position, -- r and theta
		forward = forward, -- Change in r and theta, should be length 1
		velocity = vec2(0, 0), -- Change in r and theta over time
		angularVelocity = 0,
		maxSpeed = 250,
		acceleration = 750,
		maxAngularSpeed = 2,
		angularAcceleration = 10
	}
	embedCamera = {
		position = vec3(0, 100, 200),
		orientation = quat.fromAxisAngle(vec3(consts.tau / 2 * 0.8, 0, 0)),
		speed = 200,
		angularSpeed = 2,
		verticalFOV = math.rad(70),
		farPlaneDistance = 10000,
		nearPlaneDistance = 0.1
	}
	local minimum = getWormholeCutoffRho(true) * 2 * 1.1 -- Factor of 2 because there are two regions' radii, and extra factor is to force some padding between them
	local distance = vec2.distance(wormhole.mouthAPosition, wormhole.mouthBPosition)
	assert(
		distance > minimum,
		"Wormhole cutoff regions overlap, minimum distance given settings is " .. minimum .. ", current distance is " .. distance
	)
	overviewMode = false

	local vertices = {}
	surfaceResolution = 256
	local function v(x, y)
		x = (x * 2 - 1) * math.sqrt(getWormholeCutoffRho(true) ^ 2 - wormhole.throatRadius ^ 2)
		y = y * consts.tau
		vertices[#vertices + 1] = {x, y}
	end
	for x = 0, surfaceResolution - 1 do
		for y = 0, surfaceResolution - 1 do
			local x1, y1 = x / surfaceResolution, y / surfaceResolution
			local x2, y2 = (x + 1) / surfaceResolution, (y + 1) / surfaceResolution

			v(x1, y1)
			v(x2, y1)
			v(x1, y2)

			v(x2, y2)
			v(x1, y2)
			v(x2, y1)
		end
	end
	surfaceMesh = love.graphics.newMesh(consts.objectVertexFormat, vertices, "triangles")
	
	time = 0
end

function love.keypressed(key)
	if key == "space" then
		overviewMode = not overviewMode
	end
end

local stateMetatable
stateMetatable = {
	__add = function(a, b) -- b is expected to be another such array
		local new = setmetatable({}, stateMetatable)
		for i, av in ipairs(a) do
			new[i] = av + b[i]
		end
		return new
	end,
	__mul = function(a, b) -- b is expected to be a scalar
		local new = setmetatable({}, stateMetatable)
		for i, v in ipairs(a) do
			new[i] = v * b
		end
		return new
	end
}

function love.update(dt)
	if love.keyboard.isDown("r") then
		-- Rotate to face photon sphere (er, circle)
		
	end

	local moveEmbedCamera = love.keyboard.isDown("lshift")

	if moveEmbedCamera then
		local translation = vec3()
		if love.keyboard.isDown("d") then
			translation = translation + consts.rightVector
		end
		if love.keyboard.isDown("a") then
			translation = translation - consts.rightVector
		end
		if love.keyboard.isDown("e") then
			translation = translation + consts.upVector
		end
		if love.keyboard.isDown("q") then
			translation = translation - consts.upVector
		end
		if love.keyboard.isDown("w") then
			translation = translation + consts.forwardVector
		end
		if love.keyboard.isDown("s") then
			translation = translation - consts.forwardVector
		end
		embedCamera.position = embedCamera.position + vec3.rotate(normaliseOrZero3(translation), embedCamera.orientation) * embedCamera.speed * dt

		local rotation = vec3()
		if love.keyboard.isDown("k") then
			rotation = rotation + consts.rightVector
		end
		if love.keyboard.isDown("i") then
			rotation = rotation - consts.rightVector
		end
		if love.keyboard.isDown("l") then
			rotation = rotation + consts.upVector
		end
		if love.keyboard.isDown("j") then
			rotation = rotation - consts.upVector
		end
		if love.keyboard.isDown("u") then
			rotation = rotation + consts.forwardVector
		end
		if love.keyboard.isDown("o") then
			rotation = rotation - consts.forwardVector
		end
		local rotationQuat = quat.fromAxisAngle(limitVectorLength3(rotation, embedCamera.angularSpeed * dt))
		embedCamera.orientation = quat.normalise(embedCamera.orientation * rotationQuat) -- Normalise to prevent numeric drift
	end

	local targetVelocity = vec2()
	local angularDisplacment = 0
	if not moveEmbedCamera then
		local speedMultiplier = love.keyboard.isDown("lctrl") and 0.01 or 1

		-- Get relative translation
		local translation = vec2()
		if love.keyboard.isDown("w") then
			translation.y = translation.y - 1
		end
		if love.keyboard.isDown("a") then
			translation.x = translation.x - 1
		end
		if love.keyboard.isDown("s") then
			translation.y = translation.y + 1
		end
		if love.keyboard.isDown("d") then
			translation.x = translation.x + 1
		end
		if #translation == 0 then
			targetVelocity = vec2()
		else
			targetVelocity = vec2.normalise(translation) * camera.maxSpeed * speedMultiplier
		end

		-- Get relative rotation
		local rotation = 0
		if love.keyboard.isDown(",") then
			rotation = rotation - 1
		end
		if love.keyboard.isDown(".") then
			rotation = rotation + 1
		end
		local targetAngularVelocity = rotation * camera.maxAngularSpeed * speedMultiplier
		local angleDifference = targetAngularVelocity - camera.angularVelocity
		local newAngleDistance = math.max(0, math.abs(angleDifference) - camera.angularAcceleration * dt)
		camera.angularVelocity = targetAngularVelocity - sign(angleDifference) * newAngleDistance
		angularDisplacment = camera.angularVelocity * dt
	end

	if camera.mode == "curved" then
		-- Get where we are
		local r, theta = vec2.components(camera.position)

		local rBasisIntrinsic = getRBasisIntrinsic(r, theta)
		local thetaBasisIntrinsic = getThetaBasisIntrinsic(r, theta)

		-- Rotate if needed. Avoid unnecessary back-and-forth conversion that may cause numeric drift when not rotating
		if angularDisplacment ~= 0 then
			local forwardCartesian = changeIntrinsicBasesTangent(rBasisIntrinsic, thetaBasisIntrinsic, camera.forward)
			local forwardCartesianRotated = vec2.rotate(forwardCartesian, angularDisplacment)
			camera.forward = forwardCartesianRotated.x * rBasisIntrinsic + forwardCartesianRotated.y * thetaBasisIntrinsic
		end

		-- Accelerate
		local rBasis = getRBasisExtrinsic(r, theta)
		local thetaBasis = getThetaBasisExtrinsic(r, theta)
		local normal = vec3.normalise(vec3.cross(rBasis, thetaBasis))
		local forwardExtrinsic = intrinsicToExtrinsicTangent(rBasis, thetaBasis, camera.forward)
		local rightExtrinsic = vec3.cross(normal, forwardExtrinsic) -- No normalisation required
		local velocityExtrinsic = intrinsicToExtrinsicTangent(rBasis, thetaBasis, camera.velocity)
		local relativeVelocity = extrinsicToIntrinsicTangent(rightExtrinsic, -forwardExtrinsic, velocityExtrinsic) -- Same space as targetVelocity. Not r and theta this time. Since y is forwards, we swapped forwardExtrinsic and rightExtrinsic and negated forwardExtrinsic
		local newRelativeVelocity = handleVelocity(relativeVelocity, targetVelocity, dt, camera.acceleration, camera.maxSpeed)
		if relativeVelocity ~= newRelativeVelocity then -- To avoid unnecessary conversions which may cause numeric drift
			local newVelocityExtrinsic = intrinsicToExtrinsicTangent(rightExtrinsic, -forwardExtrinsic, newRelativeVelocity)
			local newVelocity = extrinsicToIntrinsicTangent(rBasis, thetaBasis, newVelocityExtrinsic)
			camera.velocity = newVelocity
		end

		if #camera.velocity > 0 then -- Not sure length of r and theta vectors have much meaning besides checking whether they're a zero vector
			local method = rk4
			local steps = 200

			local stepSize = dt / steps
			-- Parallel transported tangent vectors (such as the forward vector) are updated separately after position and velocity as this seems to maintain accuracy
			local state = setmetatable({
				camera.position.x, camera.position.y,
				camera.velocity.x, camera.velocity.y
			}, stateMetatable)
			local parallelTransportState = setmetatable({
				camera.forward.x, camera.forward.y
			}, stateMetatable)
			for i = 1, steps do
				local t = i * stepSize
				state = method(t, state, stepSize, function(t, state)
					local pos = vec2(state[1], state[2])
					local vel = vec2(state[3], state[4])
					local christoffelRThetaTheta, christoffelThetaRTheta, christoffelThetaThetaR = getChristoffelSymbols(pos.x, pos.y)
					return setmetatable({
						-- Position derivative
						vel.x, vel.y,

						-- Velocity derivative
						-christoffelRThetaTheta * vel.y * vel.y,
						-(
							christoffelThetaRTheta * vel.x * vel.y +
							christoffelThetaThetaR * vel.y * vel.x
						)
					}, stateMetatable)
				end)
				parallelTransportState = method(t, parallelTransportState, stepSize, function(t, parallelTransportState)
					local pos = vec2(state[1], state[2])
					local vel = vec2(state[3], state[4])
					local christoffelRThetaTheta, christoffelThetaRTheta, christoffelThetaThetaR = getChristoffelSymbols(pos.x, pos.y)

					local forward = vec2(parallelTransportState[1], parallelTransportState[2])
					return setmetatable({-- Forward vector derivative
						-christoffelRThetaTheta * forward.y * vel.y,
						-(
							christoffelThetaRTheta * forward.x * vel.y +
							christoffelThetaThetaR * forward.y * vel.x
						)
					}, stateMetatable)
				end)
			end
			camera.position = vec2(state[1], state[2] % consts.tau)
			camera.velocity = vec2(state[3], state[4])
			camera.forward = normaliseTangentVector(camera.position, vec2(parallelTransportState[1], parallelTransportState[2])) -- Normalise to prevent numeric drift
		end
	elseif camera.mode == "flat" then
		camera.angle = (camera.angle + angularDisplacment) % consts.tau
		local targetVelocityRotated = vec2.rotate(targetVelocity, camera.angle + consts.tau / 4)
		camera.velocity = handleVelocity(camera.velocity, targetVelocityRotated, dt, camera.acceleration, camera.maxSpeed)
		camera.position = camera.position + camera.velocity * dt
	end

	-- Check for leaving current wormhole region
	if camera.mode == "curved" then
		local r = camera.position.x
		local rho = math.sqrt(r ^ 2 + wormhole.throatRadius ^ 2)
		local cuttoffRho = getWormholeCutoffRho(true)
		if rho > cuttoffRho then
			local theta = camera.position.y
			local rBasis = getRBasisExtrinsic(r, theta)
			local thetaBasis = getThetaBasisExtrinsic(r, theta)

			local orientationReal = extrinsicToRealTangent(
				intrinsicToExtrinsicTangent(rBasis, thetaBasis, camera.forward),
				r < 0
			)
			local orientationFlat = vec2(orientationReal.x, orientationReal.y) -- Should not end up as a zero vector this far out
			camera.angle = vec2.toAngle(orientationFlat)

			camera.position = rThetaToRealPosition(r, theta)

			local velocityReal = extrinsicToRealTangent(
				intrinsicToExtrinsicTangent(rBasis, thetaBasis, camera.velocity),
				r < 0
			)
			camera.velocity = vec2(velocityReal.x, velocityReal.y)

			camera.forward = nil
			camera.mode = "flat"
		end
	end

	-- Check for entering a wormhole region
	if camera.mode == "flat" then
		local aDistance = vec2.distance(camera.position, wormhole.mouthAPosition)
		local bDistance = vec2.distance(camera.position, wormhole.mouthBPosition)
		local mouthPosition = aDistance < bDistance and wormhole.mouthAPosition or wormhole.mouthBPosition

		local delta = camera.position - mouthPosition
		local rho = vec2.length(delta)
		local cutoffRho = getWormholeCutoffRho(false)

		if rho < cutoffRho then
			local rSign
			-- Along with delta above, forwardsFlat and velocity get flipped depending on which wormhole region you are entering
			local forwardsFlat = vec2.fromAngle(camera.angle)
			local velocity = camera.velocity
			if aDistance < bDistance then
				rSign = 1
			else
				rSign = -1
				local function flip(v)
					local mouthDirection = vec2.normalise(wormhole.mouthBPosition - wormhole.mouthAPosition)
					local parallel = mouthDirection * vec2.dot(v, mouthDirection)
					local perpendicular = v - parallel
					local parallelFlipped = -parallel
					return parallelFlipped + perpendicular
				end
				delta = flip(delta)
				forwardsFlat = flip(forwardsFlat)
				velocity = flip(velocity)
			end

			local theta = vec2.toAngle(delta)
			local r = math.sqrt(rho ^ 2 - wormhole.throatRadius ^ 2) * rSign
			local rBasis = getRBasisExtrinsic(r, theta)
			local thetaBasis = getThetaBasisExtrinsic(r, theta)

			camera.position = vec2(r, theta)

			forwardsFlat = vec3(forwardsFlat.x, forwardsFlat.y, 0)
			local intrinsicPreNormalise = extrinsicToIntrinsicTangent(rBasis, thetaBasis, forwardsFlat)
			local extrinsicNormalised = vec3.normalise(intrinsicToExtrinsicTangent(rBasis, thetaBasis, intrinsicPreNormalise))
			camera.forward = extrinsicToIntrinsicTangent(rBasis, thetaBasis, extrinsicNormalised)

			local speed = #velocity
			local velocityZ0 = vec3(velocity.x, velocity.y, 0)
			local intrinsicPreLength = extrinsicToIntrinsicTangent(rBasis, thetaBasis, velocityZ0)
			local extrinsicLengthCorrected = speed * normaliseOrZero3(intrinsicToExtrinsicTangent(rBasis, thetaBasis, intrinsicPreLength))
			camera.velocity = extrinsicToIntrinsicTangent(rBasis, thetaBasis, extrinsicLengthCorrected)

			camera.angle = nil
			camera.mode = "curved"
		end
	end

	time = time + dt
end

function love.draw()
	love.graphics.setCanvas(objectOverlayTexture)
	love.graphics.clear()
	love.graphics.setPointSize(5)
	love.graphics.points((camera.position.x / math.sqrt(getWormholeCutoffRho(true) ^ 2 - wormhole.throatRadius ^ 2) * 0.5 + 0.5) * objectOverlayTexture:getWidth(), camera.position.y / consts.tau * objectOverlayTexture:getHeight())
	love.graphics.setCanvas()

	if overviewMode then
		love.graphics.translate(love.graphics.getWidth() / 2, love.graphics.getHeight() / 2)
		love.graphics.setPointSize(5)

		local orientationLineLength = 50
		if camera.mode == "curved" then
			local cameraPositionReal = rThetaToRealPosition(vec2.components(camera.position))
			love.graphics.translate(-cameraPositionReal.x, -cameraPositionReal.y)
			love.graphics.points(cameraPositionReal.x, cameraPositionReal.y)
			local r, theta = vec2.components(camera.position)
			local orientationExtrinsic = extrinsicToRealTangent(
				intrinsicToExtrinsicTangent(
					getRBasisExtrinsic(r, theta),
					getThetaBasisExtrinsic(r, theta),
					camera.forward
				),
				r < 0
			)
			local orientationFlat = vec2(orientationExtrinsic.x, orientationExtrinsic.y)
			if #orientationFlat > 0 then -- Prevent visual glitches
				local added = cameraPositionReal + orientationFlat * orientationLineLength
				love.graphics.line(
					cameraPositionReal.x, cameraPositionReal.y,
					added.x, added.y
				)
			end
		elseif camera.mode == "flat" then
			love.graphics.translate(-camera.position.x, -camera.position.y)
			love.graphics.points(camera.position.x, camera.position.y)
			local added = camera.position + vec2.fromAngle(camera.angle) * orientationLineLength
			love.graphics.line(
				camera.position.x, camera.position.y,
				added.x, added.y
			)
		end

		love.graphics.circle("line", wormhole.mouthAPosition.x, wormhole.mouthAPosition.y, wormhole.throatRadius)
		love.graphics.circle("line", wormhole.mouthBPosition.x, wormhole.mouthBPosition.y, wormhole.throatRadius)
		love.graphics.origin()
	else
		local rayMapWidth, rayMapHeight = rayMap:getDimensions()
		rayShader:send("rayMap", rayMap)
		rayShader:send("stepCount", stepCount)
		rayShader:send("stepSize", consts.stepSize)
		rayShader:send("gridSpacing", 32)
		rayShader:send("gridCells", 60) -- Per axis
		rayShader:send("gridLineThickness", 4)
		rayShader:send("wormholeThroatRadius", wormhole.throatRadius)
		rayShader:send("wormholeMouthAPosition", {vec2.components(wormhole.mouthAPosition)})
		rayShader:send("wormholeMouthBPosition", {vec2.components(wormhole.mouthBPosition)})
		rayShader:send("curvedToFlatR", math.sqrt(getWormholeCutoffRho(true) ^ 2 - wormhole.throatRadius ^ 2))
		rayShader:send("flatToCurvedRho", getWormholeCutoffRho(false))

		rayShader:send("overlay", objectOverlayTexture)
		rayShader:send("overlaySize", {objectOverlayTexture:getDimensions()})
		rayShader:send("drawToOverlay", true)
		rayShader:send("drawOverlayToRay", true)
		rayShader:send("overlayDistanceLineSteps", 4)
		rayShader:send("maxOverlayDistanceLines", 8)

		if camera.mode == "curved" then
			rayShader:send("initialModeCurved", true)
			rayShader:send("cameraPosition", {vec2.components(camera.position)}) -- r and theta
			rayShader:send("cameraForward", {vec2.components(camera.forward)}) -- r and theta change
		else
			rayShader:send("initialModeCurved", false)
			rayShader:send("cameraPosition", {vec2.components(camera.position)})
			rayShader:send("cameraForward", {vec2.components(vec2.fromAngle(camera.angle))})
		end
		love.graphics.dispatchThreadgroups(
			rayShader,
			math.ceil(rayMapWidth / rayShader:getLocalThreadgroupSize())
		)

		love.graphics.setShader(sceneShader)
		sceneShader:send("stepSize", consts.stepSize)
		sceneShader:send("stepCount", stepCount)
		sceneShader:send("rayMap", rayMap)
		love.graphics.draw(dummyTexture, 0, 0, 0, love.graphics.getDimensions())
		love.graphics.setShader()
	end

	love.graphics.setCanvas({objectCanvas, depth = true})
	love.graphics.setDepthMode("lequal", true)
	love.graphics.clear(0, 0, 0)
	local worldToCamera = mat4.camera(embedCamera.position, embedCamera.orientation)
	local worldToCameraStationary = mat4.camera(vec3(), embedCamera.orientation)
	local cameraToClip = mat4.perspectiveLeftHanded(
		objectCanvas:getWidth() / objectCanvas:getHeight(),
		embedCamera.verticalFOV,
		embedCamera.farPlaneDistance,
		embedCamera.nearPlaneDistance
	)
	local worldToClip = cameraToClip * worldToCamera
	local clipToSky = mat4.inverse(cameraToClip * worldToCameraStationary)
	love.graphics.setShader(objectShader)
	objectShader:send("wormholeThroatRadius", wormhole.throatRadius)
	objectShader:send("wormholeMouthAPosition", {vec2.components(wormhole.mouthAPosition)})
	objectShader:send("wormholeMouthBPosition", {vec2.components(wormhole.mouthBPosition)})
	objectShader:send("modelToClip", {mat4.components(worldToClip)})
	if objectShader:hasUniform("time") then
		objectShader:send("time", time)
	end
	objectShader:send("overlay", objectOverlayTexture)
	objectShader:send("gridSpacing", 32)
	objectShader:send("gridCells", 60) -- Per axis
	objectShader:send("gridLineThickness", 4)
	objectShader:send("wormholeCutoffR", math.sqrt(getWormholeCutoffRho(true) ^ 2 - wormhole.throatRadius ^ 2))
	love.graphics.draw(surfaceMesh)
	love.graphics.setCanvas()
	love.graphics.setShader()
	love.graphics.setDepthMode("always", false)
	love.graphics.draw(objectCanvas)
	love.graphics.rectangle("line", 0, 0, objectCanvas:getDimensions())

	love.graphics.print(love.timer.getFPS())
end
