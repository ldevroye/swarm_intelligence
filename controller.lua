---- MINE

---------------------------------------------------------------------------
-- global variables
TARGET_DIST = 80 -- the target distance between robots, in cm
EPSILON = 50 -- a coefficient to increase the force of the repulsion/attraction function
WHEEL_SPEED = 10 -- max wheel speed

ACCEPTED_DIST = 10 -- range of accepted distance around the target distance
NEIGHBORS_AT_TARG_DIST = 4 -- minimum number of neighbors that must be at the right distance for the grouping condition to be verified
FLOCKING_TRIGGER_THRESHOLD = 50 -- number of consecutive timesteps in which the grouping condition must hold before switching to flocking
ORIENTATION_STEPS = 50 -- number of steps to rotate toward the light before moving
BLACK_FLOOR_STEPS = 10 -- number of consecutive timesteps on black before committing to the target zone
FLOCKING_CONDITION = 0
BEHAVIOR_STATE = 0 -- 0 = orient to light, 1 = grouping, 2 = tunnel, 3 = flocking on black
STATE_ORIENT = 0
STATE_GROUPING = 1
STATE_TUNNEL = 2
STATE_FLOCKING = 3
orientation_counter = 0
flocking_trigger_counter = 0
black_floor_counter = 0


LOG_FILE = "tunnel.log"
logf = io.open(LOG_FILE, "w")
current_step = 0;
ID = "fb1"

---------------------------------------------------------------------------

---------------------------------------------------------------------------
--Step function
function step()
	robot.colored_blob_omnidirectional_camera.enable()
	robot.range_and_bearing.set_data(1, BEHAVIOR_STATE) -- advertise our current phase to nearby robots
	lj_vector = ProcessRAB_LJ() -- then we compute the angle to follow, using the other robots as input, see function code for details
	leader_vector = ProcessRABLeaders() -- pull toward robots that already reached the tunnel or black floor
	light_vector = ComputeVectorToLight() -- we compute the vector towards the light source
	obstacle_vector = ComputeVectorFromProximity() -- we compute a repulsion vector away from nearby obstacles
	ground_vector, black_ground_count = ProcessGround() -- use the floor sensors to bias motion into the black target area
	robot.range_and_bearing.set_data(2, black_ground_count > 0 and 1 or 0) -- mark robots that already see black
	total_vector = {0,0}

	to_log = string.format("light {%.4f, %.4f}\n", light_vector[1], light_vector[2])
	add_log(to_log)

	if(BEHAVIOR_STATE == STATE_ORIENT) then
		-- first phase: move toward the light while staying away from nearby obstacles
		obstacle_tangent = ComputeObstacleTangent(obstacle_vector)
		total_vector[1] = light_vector[1] + 0.5 * obstacle_vector[1] + 0.45 * obstacle_tangent[1] + 0.20 * leader_vector[1]
		total_vector[2] = light_vector[2] + 0.5 * obstacle_vector[2] + 0.45 * obstacle_tangent[2] + 0.20 * leader_vector[2]
		orientation_counter = orientation_counter + 1
		if(orientation_counter >= ORIENTATION_STEPS) then
			BEHAVIOR_STATE = STATE_GROUPING
		end
	elseif(BEHAVIOR_STATE == STATE_GROUPING) then
		-- second phase: keep the pack together while continuing to use obstacle edges as a guide
		obstacle_tangent = ComputeObstacleTangent(obstacle_vector)
		total_vector[1] = lj_vector[1] - 0.20 * light_vector[1] + 0.45 * obstacle_tangent[1] + 0.60 * leader_vector[1]
		total_vector[2] = lj_vector[2] - 0.20 * light_vector[2] + 0.45 * obstacle_tangent[2] + 0.60 * leader_vector[2]
		if(FLOCKING_CONDITION == 1) then
			flocking_trigger_counter = flocking_trigger_counter + 1
			if(flocking_trigger_counter >= FLOCKING_TRIGGER_THRESHOLD) then
				BEHAVIOR_STATE = STATE_TUNNEL
			end
		else
			flocking_trigger_counter = 0
		end
	elseif(BEHAVIOR_STATE == STATE_TUNNEL) then
		-- third phase: use obstacle repulsion and floor cues to find a gap instead of pushing through it
		total_vector[1] = lj_vector[1] - 0.55 * light_vector[1] + 1.30 * obstacle_vector[1] + 0.80 * ground_vector[1] + 0.40 * leader_vector[1]
		total_vector[2] = lj_vector[2] - 0.55 * light_vector[2] + 1.30 * obstacle_vector[2] + 0.80 * ground_vector[2] + 0.40 * leader_vector[2]
		if(black_ground_count > 0) then
			black_floor_counter = black_floor_counter + 1
		else
			black_floor_counter = 0
		end
		if(black_floor_counter >= BLACK_FLOOR_STEPS) then
			BEHAVIOR_STATE = STATE_FLOCKING
		end
	else
		-- final phase: stay compact on the black floor while continuing to avoid obstacles
		total_vector[1] = lj_vector[1] + 0.90 * ground_vector[1] + 1.10 * obstacle_vector[1] - 0.15 * light_vector[1]
		total_vector[2] = lj_vector[2] + 0.90 * ground_vector[2] + 1.10 * obstacle_vector[2] - 0.15 * light_vector[2]
		if(black_ground_count == 0) then
			BEHAVIOR_STATE = STATE_TUNNEL
			black_floor_counter = 0
		end
	end

	target_angle = math.atan2(total_vector[2],total_vector[1]) -- compute the angle from the vector
	speeds = ComputeSpeedFromAngle(target_angle) -- we now compute the wheel speed necessary to go in the direction of the target angle
	if(BEHAVIOR_STATE == STATE_ORIENT) then
		robot.wheels.set_velocity(0.8 * speeds[1], 0.8 * speeds[2]) -- move slowly while centering between light and obstacles
	elseif(BEHAVIOR_STATE == STATE_TUNNEL) then
		robot.wheels.set_velocity(0.7 * speeds[1], 0.7 * speeds[2]) -- slow down to stay aligned with the tunnel entrance
	else
		robot.wheels.set_velocity(speeds[1],speeds[2]) -- actuate wheels to move
	end
	robot.range_and_bearing.clear_data() -- forget about all received messages for next step
    current_step = current_step + 1
    if (current_step%1==0) then
		to_format = "id=%s, step=%d, state=%d, flocking_condition=%d, flocking_counter=%d\n"
		log = string.format(to_format, robot.id, current_step, BEHAVIOR_STATE, FLOCKING_CONDITION, flocking_trigger_counter)
		add_log(log)
	end
end

---------------------------------------------------------------------------
--This function computes the vector (normalized) that points towards the light source
function ComputeVectorToLight()
	light_v = {0,0}
	for i = 1, 24 do 
		-- we calculate the x and y components given length and angle
		vec = {
			x = robot.light[i].value * math.cos(robot.light[i].angle),
			y = robot.light[i].value * math.sin(robot.light[i].angle)
		}
		-- we sum the vectors into a variable called accumul
		light_v[1] = light_v[1] + vec.x
		light_v[2] = light_v[2] + vec.y
	end
	len = math.sqrt(light_v[1] * light_v[1] + light_v[2] * light_v[2])
	-- we normalize the vector
	if(len ~= 0) then 
		light_v[1] = light_v[1] / len
   		light_v[2] = light_v[2] / len
	end
	return light_v
end	

---------------------------------------------------------------------------
-- This function computes a repulsion vector from nearby obstacles using the proximity sensors.
function ComputeVectorFromProximity()
	prox_v = {0,0}
	for i = 1, 24 do
		prox_v[1] = prox_v[1] - robot.proximity[i].value * math.cos(robot.proximity[i].angle)
		prox_v[2] = prox_v[2] - robot.proximity[i].value * math.sin(robot.proximity[i].angle)
	end
	len = math.sqrt(prox_v[1] * prox_v[1] + prox_v[2] * prox_v[2])
	if(len ~= 0) then
		prox_v[1] = prox_v[1] / len
		prox_v[2] = prox_v[2] / len
	end
	return prox_v
end

---------------------------------------------------------------------------
-- This function computes a vector toward robots that have already advanced.
function ProcessRABLeaders()
	leader_v = {0,0}
	for i = 1, #robot.range_and_bearing do
		if(robot.range_and_bearing[i].data[1] >= STATE_TUNNEL) then
			weight = 1 / math.max(robot.range_and_bearing[i].range, 1)
			leader_v[1] = leader_v[1] + math.cos(robot.range_and_bearing[i].horizontal_bearing) * weight
			leader_v[2] = leader_v[2] + math.sin(robot.range_and_bearing[i].horizontal_bearing) * weight
		end
	end
	len = math.sqrt(leader_v[1] * leader_v[1] + leader_v[2] * leader_v[2])
	if(len ~= 0) then
		leader_v[1] = leader_v[1] / len
		leader_v[2] = leader_v[2] / len
	end
	return leader_v
end

---------------------------------------------------------------------------
-- This function computes a tangent vector around obstacles so the swarm can
-- move along the white-zone obstacle field instead of pushing straight through it.
function ComputeObstacleTangent(obstacle_v)
	tangent_v = {0,0}
	len = math.sqrt(obstacle_v[1] * obstacle_v[1] + obstacle_v[2] * obstacle_v[2])
	if(len ~= 0) then
		tangent_v[1] = obstacle_v[2]
		tangent_v[2] = -obstacle_v[1]
	end
	return tangent_v
end

---------------------------------------------------------------------------
-- This function computes a vector toward the black floor using the motor-ground sensors.
function ProcessGround()
	ground_v = {0,0}
	black_count = 0
	for i = 1, 4 do
		if(robot.motor_ground[i].value == 0) then
			black_count = black_count + 1
			ground_v[1] = ground_v[1] + robot.motor_ground[i].offset.x
			ground_v[2] = ground_v[2] + robot.motor_ground[i].offset.y
		end
	end
	len = math.sqrt(ground_v[1] * ground_v[1] + ground_v[2] * ground_v[2])
	if(len ~= 0) then
		ground_v[1] = ground_v[1] / len
		ground_v[2] = ground_v[2] / len
	end
	return ground_v, black_count
end

---------------------------------------------------------------------------
-- This function returns the strongest light reading seen by the sensors.
function GetLightStrength()
	strength = robot.light[1].value
	for i = 2, 24 do 
        strength = math.max(strength, robot.light[i].value)
    end

    return strength
end

---------------------------------------------------------------------------
--This function computes the necessary wheel speed to go in the direction of the desired angle.
function ComputeSpeedFromAngle(angle)
    dotProduct = 0.0;
    KProp = 20;
    wheelsDistance = 0.14;

    -- if the target angle is behind the robot, we just rotate, no forward motion
    if angle > math.pi/2 or angle < -math.pi/2 then
        dotProduct = 0.0;
    else
    -- else, we compute the projection of the forward motion vector with the desired angle
        forwardVector = {math.cos(0), math.sin(0)}
        targetVector = {math.cos(angle), math.sin(angle)}
        dotProduct = forwardVector[1]*targetVector[1]+forwardVector[2]*targetVector[2]
    end

	 -- the angular velocity component is the desired angle scaled linearly
    angularVelocity = KProp * angle;
    -- the final wheel speeds are compute combining the forward and angular velocities, with different signs for the left and right wheel.
    speeds = {dotProduct * WHEEL_SPEED - angularVelocity * wheelsDistance, dotProduct * WHEEL_SPEED + angularVelocity * wheelsDistance}

    return speeds
end
---------------------------------------------------------------------------

---------------------------------------------------------------------------
-- In this function, we take all distances of the other robots and apply the lennard-jones potential.
-- We then sum all these vectors to obtain the final angle to follow in order to go to the place with the minimal potential
function ProcessRAB_LJ()
	FLOCKING_CONDITION = 0
	sum_vector = {0,0}
	neighbors_in_range_counter = 0
	for i = 1,#robot.range_and_bearing do -- for each robot seen
		lj_value = ComputeLennardJones(robot.range_and_bearing[i].range) -- compute the lennard-jones value
		sum_vector[1] = sum_vector[1] + math.cos(robot.range_and_bearing[i].horizontal_bearing)*lj_value -- sum the x components of the vectors
		sum_vector[2] = sum_vector[2] + math.sin(robot.range_and_bearing[i].horizontal_bearing)*lj_value -- sum the y components of the vectors
		if(robot.range_and_bearing[i].range < TARGET_DIST + ACCEPTED_DIST and robot.range_and_bearing[i].range > TARGET_DIST - ACCEPTED_DIST) then
			neighbors_in_range_counter = neighbors_in_range_counter + 1
			if(neighbors_in_range_counter >= NEIGHBORS_AT_TARG_DIST) then
				FLOCKING_CONDITION = 1
			end
		end		
	end

	to_log = string.format("#neighbors=%d\n", neighbors_in_range_counter)
	add_log(to_log)
	
	return sum_vector


end
---------------------------------------------------------------------------

---------------------------------------------------------------------------
-- This function take the distance and compute the lennard-jones potential.
-- The parameters are defined at the top of the script
function ComputeLennardJones(distance)
   return -(4*EPSILON/distance * (math.pow(TARGET_DIST/distance,4) - math.pow(TARGET_DIST/distance,2)));
end
---------------------------------------------------------------------------

---------------------------------------------------------------------------
-- This function computes the global potential.
-- In this case the global potential is simply the distance to the center of the arena
function ComputeGlobalPotential(distance)
   return distance;
end


--nothing to init
function init()
	reset()
end

--nothing to reset
function reset()
	robot.colored_blob_omnidirectional_camera.enable()
	BEHAVIOR_STATE = STATE_ORIENT
	orientation_counter = 0
	flocking_trigger_counter = 0
	black_floor_counter = 0
    logf = io.open(LOG_FILE, "w")
    if (logf) then
        logf:write("controller started\n")
        logf:flush()
    end
end

--nothing to destroy
function destroy()
end

function add_log(log)
	logf = io.open(LOG_FILE, "a")
	if (robot.id==ID and logf) then
		logf:write(log)
		logf:flush()
	end
end