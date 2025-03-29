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

local function getWormholeCutoffR()
	-- Given z=f(r), the following is when f's derivative reaches the wormhole cutoff gradient
	-- z from r is just the z coordinate in rThetaToEmbedPosition
	return (wormhole.throatRadius ^ 2 - wormhole.throatRadius ^ 2 * consts.wormholeCutoffGradient ^ 2) / consts.wormholeCutoffGradient
end

-- Wormhole cutoff rho grows linearly as wormhole throat radius does (all else being constant), which is nice
local function getWormholeCutoffRho()
	-- rho = sqrt(r ^ 2 + wormhole.throatRadius ^ 2)
	-- Given z=f(rho), the following is when f's derivative reaches the wormhole cutoff gradient
	-- z from rho is wormhole.throatRadius * acosh(r / wormhole.throatRadius)
	return (wormhole.throatRadius * math.sqrt(1 + consts.wormholeCutoffGradient ^ 2)) / consts.wormholeCutoffGradient
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
		position = vec2(0, 0), -- r and theta
		forward = vec2(1, 0), -- Change in r and theta, should be length 1 when converted to embed space
		velocity = vec2(0, 0), -- Change in r and theta over time
		maxSpeed = 200,
		acceleration = 150,
		angularSpeed = 2
	}
	wormhole = {
		mouthAPosition = vec2(300, 200),
		mouthBPosition = vec2(300, 600),
		throatRadius = 50
	}
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
		translation.y = translation.y + 1
	end
	if love.keyboard.isDown("a") then
		translation.x = translation.x - 1
	end
	if love.keyboard.isDown("s") then
		translation.y = translation.y - 1
	end
	if love.keyboard.isDown("d") then
		translation.x = translation.x + 1
	end
	local relativeAcceleration
	if #translation == 0 then
		relativeAcceleration = vec2()
	else
		relativeAcceleration = vec2.normalise(translation) * camera.acceleration
	end

	-- Get relative rotation
	local rotation = 0
	if love.keyboard.isDown(",") then
		rotation = rotation - 1
	end
	if love.keyboard.isDown(".") then
		rotation = rotation + 1
	end
	local angularVelocity = rotation * camera.angularSpeed
	local angularDisplacment = angularVelocity * dt

	-- Get where we are
	local r, theta = vec2.components(camera.position)

	-- Get r and theta bases as well as normal vector of surface in embed space
	local rBasis = getRBasisEmbed(r, theta)
	local thetaBasis = getThetaBasisEmbed(r, theta)
	local normal = vec3.normalise(vec3.cross(rBasis, thetaBasis)) -- TODO: What if the vectors being crossed are parallel (or imprecision causes so)?

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

	local forwardEmbed = intrinsicToEmbedTangent(rBasis, thetaBasis, camera.forward)

	-- Accelerate if needed
	if #relativeAcceleration > 0 then
		local rightEmbed = vec3.cross(normal, forwardEmbed)
		local accelerationEmbed = intrinsicToEmbedTangent(rightEmbed, forwardEmbed, relativeAcceleration) -- Not r and theta this time. Since y is forwards, we swapped forwardEmbed and rightEmbed
		camera.velocity = camera.velocity + embedToIntrinsicTangent(rBasis, thetaBasis, accelerationEmbed * dt)
	end

	if #camera.velocity > 0 then -- Not sure length of r and theta vectors have much meaning besides checking whether you're moving or not moving.
		-- Get displacement in embed space
		local displacementEmbed = intrinsicToEmbedTangent(rBasis, thetaBasis, camera.velocity * dt)

		-- Get r and theta movement from displacement
		local rDisplacement, thetaDisplacement = vec2.components(embedToIntrinsicTangent(rBasis, thetaBasis, displacementEmbed))
		local newR = r + rDisplacement
		local newTheta = theta + thetaDisplacement

		-- Get bases at destination
		local newRBasis = getRBasisEmbed(newR, newTheta)
		local newThetaBasis = getThetaBasisEmbed(newR, newTheta)

		local christoffelRThetaTheta, christoffelThetaRTheta, christoffelThetaThetaR = getChristoffelSymbols(r, theta)
		local function parallelTransportRTheta(dR, dTheta)
			local newDR = dR - christoffelRThetaTheta * thetaDisplacement * dTheta
			local newDTheta = dTheta - (
				christoffelThetaRTheta * rDisplacement * dTheta +
				christoffelThetaThetaR * thetaDisplacement * dR
			)
			return vec2(newDR, newDTheta)
		end

		local function parallelTransportEmbedTangent(v)
			-- TODO: Maybe parallel transport component coming off the surface, too? embedToIntrinsicTangent destroys it
			-- TODO: Maintain length explicitly (probably sacrificing direction accuracy)
			local intrinsic = embedToIntrinsicTangent(rBasis, thetaBasis, v)
			local movedIntrinsic = parallelTransportRTheta(vec3.components(intrinsic))
			return intrinsicToEmbedTangent(newRBasis, newThetaBasis, movedIntrinsic) -- Note that we are using the new bases on this line
		end

		camera.position = vec2(newR, newTheta % consts.tau)
		camera.velocity = parallelTransportRTheta(vec2.components(camera.velocity))
		local forwardNormalisedIntrinsic = embedToIntrinsicTangent(rBasis, thetaBasis, vec3.normalise(forwardEmbed)) -- Normalised to prevent numeric drift
		camera.forward = parallelTransportRTheta(vec2.components(forwardNormalisedIntrinsic))
	end

	time = time + dt
end

function love.draw()
	if overviewMode then
		local cameraPositionReal = rThetaToRealPosition(vec2.components(camera.position))
		love.graphics.translate(-cameraPositionReal.x, -cameraPositionReal.y)
		love.graphics.translate(love.graphics.getWidth() / 2, love.graphics.getHeight() / 2)
		love.graphics.setPointSize(5)
		love.graphics.points(cameraPositionReal.x, cameraPositionReal.y)
		local r, theta = vec2.components(camera.position)
		local orientationEmbed = 
		embedToRealTangent(
			intrinsicToEmbedTangent(
				getRBasisEmbed(r, theta),
				getThetaBasisEmbed(r, theta),
				camera.forward
			),
			r < 0
		)
		local orientationFlat = vec2(orientationEmbed.x, orientationEmbed.y)
		if #orientationFlat > 0 then -- Prevent visual glitches
			local added = cameraPositionReal + orientationFlat * 50
			love.graphics.line(
				cameraPositionReal.x, cameraPositionReal.y,
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
		rayShader:send("cameraPosition", {vec2.components(camera.position)})
		rayShader:send("cameraForward", {vec2.components(camera.forward)})
		rayShader:send("gridSpacing", 32)
		rayShader:send("gridCells", 32) -- Per axis
		rayShader:send("gridLineThickness", 4)
		rayShader:send("wormholeThroatRadius", wormhole.throatRadius)
		rayShader:send("wormholeMouthAPosition", {vec2.components(wormhole.mouthAPosition)})
		rayShader:send("wormholeMouthBPosition", {vec2.components(wormhole.mouthBPosition)})
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

		-- love.graphics.draw(rayMap)
	end

	love.graphics.print(love.timer.getFPS())
end
