-- Using this version of love.update, there is no acceleration; the velocity snaps from standstill to max speed and back.

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
	local relativeVelocity
	if #translation == 0 then
		relativeVelocity = vec2()
	else
		relativeVelocity = vec2.normalise(translation) * camera.speed
	end
	local relativeDisplacement = relativeVelocity * dt

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

	if #relativeDisplacement > 0 then
		-- Get displacement in embed space
		local forwardEmbed = intrinsicToEmbedTangent(rBasis, thetaBasis, camera.forward)
		local rightEmbed = vec3.cross(normal, forwardEmbed)
		local displacementEmbed = intrinsicToEmbedTangent(rightEmbed, forwardEmbed, relativeDisplacement) -- Not r and theta this time. Since y is forwards, we swapped forwardEmbed and rightEmbed

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
		local forwardNormalisedIntrinsic = embedToIntrinsicTangent(rBasis, thetaBasis, vec3.normalise(forwardEmbed)) -- Normalised to prevent numeric drift
		camera.forward = parallelTransportRTheta(vec2.components(forwardNormalisedIntrinsic))
	end

	time = time + dt
end
