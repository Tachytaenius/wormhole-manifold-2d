local mathsies = require("lib.mathsies")
local vec2 = mathsies.vec2
local vec3 = mathsies.vec3
local quat = mathsies.quat

local consts = require("consts")

local camera
local wormhole
local time

local rayShader
local sceneShader

local rayMap
local dummyTexture

local stepCount

local overviewMode

local function sign(x)
	return x < 0 and -1 or x == 0 and 0 or 1
end

local function asinh(x)
	return math.log(x + math.sqrt(x ^ 2 + 1))
end

local function rThetaToEmbedPosition(r, theta)
	return vec3( -- Ellis wormhole (catenoid)
		wormhole.throatRadius * math.sqrt(r ^ 2 / wormhole.throatRadius ^ 2 + 1) * math.cos(theta),
		wormhole.throatRadius * math.sqrt(r ^ 2 / wormhole.throatRadius ^ 2 + 1) * math.sin(theta),
		wormhole.throatRadius * asinh(r / wormhole.throatRadius)
	)
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

local function getRBasisEmbed(r, theta)
	local rDelta = 0.0001
	return (rThetaToEmbedPosition(r + rDelta, theta) - rThetaToEmbedPosition(r, theta)) / rDelta
end

local function getThetaBasisEmbed(r, theta)
	local thetaDelta = 0.0001 -- :3
	return (rThetaToEmbedPosition(r, theta + thetaDelta) - rThetaToEmbedPosition(r, theta)) / thetaDelta
end

-- Not necessarily r and theta input
local function intrinsicToEmbedTangent(e1, e2, v)
	return v.x * e1 + v.y * e2
end

-- Not necessarily r and theta output
local function embedToIntrinsicTangent(e1, e2, v)
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

local function embedToRealTangent(v, negative)
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
	rayShader = love.graphics.newComputeShader("shaders/ray.glsl")
	sceneShader = love.graphics.newShader("shaders/scene.glsl")
	local width, height = love.graphics.getWidth()
	local centreToCorner = #vec2(width, height) / 2 -- Order of operations doesn't matter
	local padding = 4
	stepCount = math.ceil((centreToCorner + padding) / consts.stepSize)
	rayMap = love.graphics.newCanvas(3072, stepCount, {computewrite = true})
	rayMap:setFilter("linear", "linear")
	rayMap:setWrap("repeat", "clamp") -- x is angle, y is distance
	dummyTexture = love.graphics.newImage(love.image.newImageData(1, 1))

	camera = {
		mode = "curved", -- "curved" or "flat"
		position = vec2(0, 0), -- r and theta
		forward = vec2(1, 0), -- Change in r and theta, should be length 1 when converted to embed space
		velocity = vec2(0, 0), -- Change in r and theta over time
		angularVelocity = 0,
		maxSpeed = 250,
		acceleration = 750,
		maxAngularSpeed = 2,
		angularAcceleration = 10
	}
	wormhole = {
		mouthAPosition = vec2(100, 200),
		mouthBPosition = vec2(1700, 200),
		throatRadius = 30
	}
	local minimum = getWormholeCutoffRho(true) * 2 * 1.1 -- Factor of 2 because there are two regions' radii, and extra factor is to force some padding between them
	local distance = vec2.distance(wormhole.mouthAPosition, wormhole.mouthBPosition)
	assert(
		distance > minimum,
		"Wormhole cutoff regions overlap, minimum distance given settings is " .. minimum .. ", current distance is " .. distance
	)
	overviewMode = false
	time = 0
end

function love.keypressed(key)
	if key == "space" then
		overviewMode = not overviewMode
	end
end

function love.update(dt)
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
	local targetVelocity
	if #translation == 0 then
		targetVelocity = vec2()
	else
		targetVelocity = vec2.normalise(translation) * camera.maxSpeed
	end

	-- Get relative rotation
	local rotation = 0
	if love.keyboard.isDown(",") then
		rotation = rotation - 1
	end
	if love.keyboard.isDown(".") then
		rotation = rotation + 1
	end
	local targetAngularVelocity = rotation * camera.maxAngularSpeed
	local angleDifference = targetAngularVelocity - camera.angularVelocity
	local newAngleDistance = math.max(0, math.abs(angleDifference) - camera.angularAcceleration * dt)
	camera.angularVelocity = targetAngularVelocity - sign(angleDifference) * newAngleDistance
	local angularDisplacment = camera.angularVelocity * dt

	if camera.mode == "curved" then
		-- Get where we are
		local r, theta = vec2.components(camera.position)

		-- Get r and theta bases as well as normal vector of surface in embed space
		local rBasis = getRBasisEmbed(r, theta)
		local thetaBasis = getThetaBasisEmbed(r, theta)
		local normal = vec3.normalise(vec3.cross(rBasis, thetaBasis))

		-- Rotate if needed. Avoid unnecessary back-and-forth conversion that may cause numeric drift when not rotating
		if angularDisplacment ~= 0 then
			local forwardEmbed = intrinsicToEmbedTangent(rBasis, thetaBasis, camera.forward)
			local axisAngle = normal * angularDisplacment
			local quaternion = quat.fromAxisAngle(axisAngle)
			local rotated = vec3.rotate(forwardEmbed, quaternion)
			local normalised = vec3.normalise(rotated) -- Avoid numeric drift of magnitude
			local forwardIntrinsic = embedToIntrinsicTangent(rBasis, thetaBasis, normalised)
			camera.forward = forwardIntrinsic
		end

		-- Accelerate
		local forwardEmbed = intrinsicToEmbedTangent(rBasis, thetaBasis, camera.forward)
		local rightEmbed = vec3.cross(normal, forwardEmbed)
		local velocityEmbed = intrinsicToEmbedTangent(rBasis, thetaBasis, camera.velocity)
		local relativeVelocity = embedToIntrinsicTangent(rightEmbed, -forwardEmbed, velocityEmbed) -- Same space as targetVelocity. Not r and theta this time. Since y is forwards, we swapped forwardEmbed and rightEmbed and negated forwardEmbed
		local newRelativeVelocity = handleVelocity(relativeVelocity, targetVelocity, dt, camera.acceleration, camera.maxSpeed)
		if relativeVelocity ~= newRelativeVelocity then -- To avoid unnecessary conversions which may cause numeric drift
			local newVelocityEmbed = intrinsicToEmbedTangent(rightEmbed, -forwardEmbed, newRelativeVelocity)
			local newVelocity = embedToIntrinsicTangent(rBasis, thetaBasis, newVelocityEmbed)
			camera.velocity = newVelocity
		end

		if #camera.velocity > 0 then -- Not sure length of r and theta vectors have much meaning besides checking whether they're a zero vecotr
			-- Get displacement in embed space
			local displacementEmbed = intrinsicToEmbedTangent(rBasis, thetaBasis, camera.velocity * dt)

			-- Get r and theta movement from displacement
			local rDisplacement, thetaDisplacement = vec2.components(embedToIntrinsicTangent(rBasis, thetaBasis, displacementEmbed))
			local newR = r + rDisplacement
			local newTheta = theta + thetaDisplacement

			local christoffelRThetaTheta, christoffelThetaRTheta, christoffelThetaThetaR = getChristoffelSymbols(r, theta)
			local function parallelTransportRTheta(dR, dTheta)
				local newDR = dR - christoffelRThetaTheta * thetaDisplacement * dTheta
				local newDTheta = dTheta - (
					christoffelThetaRTheta * rDisplacement * dTheta +
					christoffelThetaThetaR * thetaDisplacement * dR
				)
				return vec2(newDR, newDTheta)
			end

			camera.position = vec2(newR, newTheta % consts.tau)
			camera.velocity = parallelTransportRTheta(vec2.components(camera.velocity))
			local forwardNormalisedIntrinsic = embedToIntrinsicTangent(rBasis, thetaBasis, vec3.normalise(forwardEmbed)) -- Normalised to prevent numeric drift
			camera.forward = parallelTransportRTheta(vec2.components(forwardNormalisedIntrinsic))
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
			local rBasis = getRBasisEmbed(r, theta)
			local thetaBasis = getThetaBasisEmbed(r, theta)

			local orientationReal = embedToRealTangent(
				intrinsicToEmbedTangent(rBasis, thetaBasis, camera.forward),
				r < 0
			)
			local orientationFlat = vec2(orientationReal.x, orientationReal.y) -- Should not end up as a zero vector this far out
			camera.angle = vec2.toAngle(orientationFlat)

			camera.position = rThetaToRealPosition(r, theta)

			local velocityReal = embedToRealTangent(
				intrinsicToEmbedTangent(rBasis, thetaBasis, camera.velocity),
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
			local rBasis = getRBasisEmbed(r, theta)
			local thetaBasis = getThetaBasisEmbed(r, theta)

			camera.position = vec2(r, theta)

			forwardsFlat = vec3(forwardsFlat.x, forwardsFlat.y, 0)
			local intrinsicPreNormalise = embedToIntrinsicTangent(rBasis, thetaBasis, forwardsFlat)
			local embedNormalised = vec3.normalise(intrinsicToEmbedTangent(rBasis, thetaBasis, intrinsicPreNormalise))
			camera.forward = embedToIntrinsicTangent(rBasis, thetaBasis, embedNormalised)

			local speed = #velocity
			local velocityZ0 = vec3(velocity.x, velocity.y, 0)
			local intrinsicPreLength = embedToIntrinsicTangent(rBasis, thetaBasis, velocityZ0)
			local embedLengthCorrected = speed * normaliseOrZero3(intrinsicToEmbedTangent(rBasis, thetaBasis, intrinsicPreLength))
			camera.velocity = embedToIntrinsicTangent(rBasis, thetaBasis, embedLengthCorrected)

			camera.angle = nil
			camera.mode = "curved"
		end
	end

	time = time + dt
end

function love.draw()
	if overviewMode then
		love.graphics.translate(love.graphics.getWidth() / 2, love.graphics.getHeight() / 2)
		love.graphics.setPointSize(5)

		local orientationLineLength = 50
		if camera.mode == "curved" then
			local cameraPositionReal = rThetaToRealPosition(vec2.components(camera.position))
			love.graphics.translate(-cameraPositionReal.x, -cameraPositionReal.y)
			love.graphics.points(cameraPositionReal.x, cameraPositionReal.y)
			local r, theta = vec2.components(camera.position)
			local orientationEmbed = embedToRealTangent(
				intrinsicToEmbedTangent(
					getRBasisEmbed(r, theta),
					getThetaBasisEmbed(r, theta),
					camera.forward
				),
				r < 0
			)
			local orientationFlat = vec2(orientationEmbed.x, orientationEmbed.y)
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

		if camera.mode == "curved" then
			rayShader:send("initialModeCurved", true)
			rayShader:send("cameraPosition", {vec2.components(camera.position)}) -- r and theta
			rayShader:send("cameraForward", {vec2.components(camera.forward)}) -- r and theta change
			love.graphics.dispatchThreadgroups(
				rayShader,
				math.ceil(rayMapWidth / rayShader:getLocalThreadgroupSize())
			)
		else
			rayShader:send("initialModeCurved", false)
			rayShader:send("cameraPosition", {vec2.components(camera.position)})
			rayShader:send("cameraForward", {vec2.components(vec2.fromAngle(camera.angle))})
			love.graphics.dispatchThreadgroups(
				rayShader,
				math.ceil(rayMapWidth / rayShader:getLocalThreadgroupSize())
			)
		end

		love.graphics.setShader(sceneShader)
		sceneShader:send("stepSize", consts.stepSize)
		sceneShader:send("stepCount", stepCount)
		sceneShader:send("rayMap", rayMap)
		love.graphics.draw(dummyTexture, 0, 0, 0, love.graphics.getDimensions())
		love.graphics.setShader()

		-- love.graphics.draw(rayMap)
	end

	love.graphics.print(love.timer.getFPS())
end
