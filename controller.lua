---- MINE

---------------------------------------------------------------------------
-- global variables
TARGET_DIST = 80 -- the target distance between robots, in cm
EPSILON = 50 -- a coefficient to increase the force of the repulsion/attraction function
WHEEL_SPEED = 5 -- max wheel speed

ACCEPTED_DIST = 10 -- range of accepted distance around the target distance
NEIGHBORS_AT_TARG_DIST = 4 -- minimum number of neighbors that must be at the right distance for the grouping condition to be verified
FLOCKING_TRIGGER_THRESHOLD = 50 -- number of consecutive timesteps in which the grouping condition must hold before switching to flocking
ORIENTATION_STEPS = 50 -- number of steps to rotate toward the light before moving
FLOCKING_CONDITION = 0
BEHAVIOR_STATE = 0 -- 0 = orient to light, 1 = grouping, 2 = flocking
STATE_ORIENT = 0
STATE_GROUPING = 1
STATE_FLOCKING = 2
orientation_counter = 0
flocking_trigger_counter = 0


LOG_FILE = "tunnel.log"
logf = io.open(LOG_FILE, "w")
current_step = 0;

---------------------------------------------------------------------------

---------------------------------------------------------------------------
--Step function
function step()
	robot.colored_blob_omnidirectional_camera.enable()
	robot.range_and_bearing.set_data(1,1) -- first we send something, to make sure the other robots see us
	lj_vector = ProcessRAB_LJ() -- then we compute the angle to follow, using the other robots as input, see function code for details
	light_vector = ComputeVectorToLight() -- we compute the vector towards the light source
	light_strength = GetLightStrength()
	total_vector = {0,0}

	if(BEHAVIOR_STATE == STATE_ORIENT) then
		-- first phase: rotate toward the light without moving
		total_vector[1] = light_vector[1]
		total_vector[2] = light_vector[2]
		orientation_counter = orientation_counter + 1
		if(orientation_counter >= ORIENTATION_STEPS) then
			BEHAVIOR_STATE = STATE_GROUPING
		end
	elseif(BEHAVIOR_STATE == STATE_GROUPING) then
		-- second phase: group while staying near the light
		total_vector[1] = lj_vector[1] + 0.3 * light_vector[1]
		total_vector[2] = lj_vector[2] + 0.3 * light_vector[2]
		if(FLOCKING_CONDITION == 1) then
			flocking_trigger_counter = flocking_trigger_counter + 1
			if(flocking_trigger_counter >= FLOCKING_TRIGGER_THRESHOLD) then
			BEHAVIOR_STATE = STATE_FLOCKING
			end
		else
			flocking_trigger_counter = 0
		end
	else
		-- third phase: flock once grouped near the light
		total_vector[1] = lj_vector[1] + 0.5 * light_vector[1]
		total_vector[2] = lj_vector[2] + 0.5 * light_vector[2]
	end

	target_angle = math.atan2(total_vector[2],total_vector[1]) -- compute the angle from the vector
	speeds = ComputeSpeedFromAngle(target_angle) -- we now compute the wheel speed necessary to go in the direction of the target angle
	if(BEHAVIOR_STATE == STATE_ORIENT) then
		robot.wheels.set_velocity(0, 0) -- stay still while orienting toward the light
	else
		robot.wheels.set_velocity(speeds[1],speeds[2]) -- actuate wheels to move
	end
	robot.range_and_bearing.clear_data() -- forget about all received messages for next step
    current_step = current_step + 1
    logf = io.open(LOG_FILE, "a")
	if (tonumber(robot.id) == 1 and logf and current_step%50==0) then
		logf:write(string.format("light_strength=%.4f state=%d flocking_condition=%d flocking_counter=%d\n", light_strength, BEHAVIOR_STATE, FLOCKING_CONDITION, flocking_trigger_counter))
		logf:flush()
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
    logf = io.open(LOG_FILE, "w")
    if (logf) then
        logf:write("controller started\n")
        logf:flush()
    end
end

--nothing to destroy
function destroy()
end