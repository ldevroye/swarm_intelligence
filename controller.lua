---- MINE

---------------------------------------------------------------------------
-- global variables
TARGET_DIST = 80 -- the target distance between robots, in cm
EPSILON = 50 -- a coefficient to increase the force of the repulsion/attraction function
WHEEL_SPEED = 10 -- max wheel speed

ACCEPTED_DIST = 10 -- range of accepted distance around the target distance
NEIGHBORS_AT_TARG_DIST = 4 -- minimum number of neighbors that must be at the right distance for the grouping condition to be verified
FLOCKING_TRIGGER_THRESHOLD = 80 -- number of consecutive timesteps in which the grouping condition must hold before switching to the black-zone phase
ORIENTATION_STEPS = 50 -- number of steps to rotate toward the light before moving
BLACK_FLOOR_STEPS = 10 -- number of consecutive timesteps on black before committing to the target zone
OBSTACLE_FRONT_THRESHOLD = 0.02 -- proximity threshold for deciding that an obstacle is directly in front
OBSTACLE_CONTACT_THRESHOLD = 1.2 -- summed front proximity needed before attempting to lock
OBSTACLE_APPROACH_STEPS = 8 -- number of timesteps spent pushing into the obstacle before locking
OBSTACLE_LOCK_STEPS = 8 -- number of timesteps spent closing the gripper before turning
OBSTACLE_TURN_SPEED = 3 -- wheel speed used while turning with a gripped obstacle
OBSTACLE_TURN_STEPS = 20 -- number of timesteps to rotate about 90 degrees while holding an obstacle
OBSTACLE_RELEASE_STEPS = 20 -- number of timesteps to back away after releasing the obstacle
OBSTACLE_COOLDOWN_STEPS = 50 -- number of timesteps to ignore new grab attempts after a release
FLOCKING_CONDITION = 0
BEHAVIOR_STATE = 0 -- 0 = orient to light, 1 = grouping, 2 = tunnel, 3 = black zone
STATE_ORIENT = 0
STATE_GROUPING = 1
STATE_TUNNEL = 2
STATE_BLACK_ZONE = 3
orientation_counter = 0
flocking_trigger_counter = 0
black_floor_counter = 0
obstacle_state = 0 -- 0 = none, 1 = approach, 2 = lock, 3 = turn, 4 = release
obstacle_counter = 0
obstacle_turn_sign = 1
obstacle_cooldown = 0
obstacle_contact = 0


ID = robot.id
directory="logs/"
LOG_FILE = directory..ID..".log"
logf = io.open(LOG_FILE, "w")
current_step = 0;

---------------------------------------------------------------------------

---------------------------------------------------------------------------
--Step function
function step()
	LogStepStart()
	if(SetupStep()) then
		return
	end
	lj_vector = ProcessRAB_LJ() -- then we compute the angle to follow, using the other robots as input, see function code for details
	leader_vector = ProcessRABLeaders() -- pull toward robots that already reached the tunnel or black floor
	light_vector = ComputeVectorToLight() -- we compute the vector towards the light source
	obstacle_vector = ComputeVectorFromProximity() -- we compute a repulsion vector away from nearby obstacles
	ground_vector, black_ground_count = ProcessGround() -- use the floor sensors to bias motion into the black target area
	robot.range_and_bearing.set_data(2, black_ground_count > 0 and 1 or 0) -- mark robots that already see black
	total_vector = {0,0}

	if(BEHAVIOR_STATE == STATE_ORIENT) then
		HandleOrientState()
	elseif(BEHAVIOR_STATE == STATE_GROUPING) then
		HandleGroupingState()
	elseif(BEHAVIOR_STATE == STATE_TUNNEL) then
		HandleTunnelState()
	else
		HandleBlackZoneState()
	end

	target_angle = math.atan2(total_vector[2],total_vector[1]) -- compute the angle from the vector
	speeds = ComputeSpeedFromAngle(target_angle) -- we now compute the wheel speed necessary to go in the direction of the target angle
	if(BEHAVIOR_STATE == STATE_ORIENT) then
		robot.wheels.set_velocity(0.8 * speeds[1], 0.8 * speeds[2]) -- move slowly while centering between light and obstacles
	elseif(BEHAVIOR_STATE == STATE_TUNNEL) then
		robot.wheels.set_velocity(0.85 * speeds[1], 0.85 * speeds[2]) -- keep advancing while still staying aligned
	elseif(BEHAVIOR_STATE == STATE_BLACK_ZONE) then
		robot.wheels.set_velocity(0.05 * speeds[1], 0.05 *speeds[2]) -- actuate wheels to move
	else
		robot.wheels.set_velocity(speeds[1], speeds[2])
	end
	robot.range_and_bearing.clear_data() -- forget about all received messages for next step
	current_step = current_step + 1
end

---------------------------------------------------------------------------
-- Log the start of each step once, before any setup or state handling.
function LogStepStart()
	to_log = string.format("id=%s, step=%d, state=%d, flocking_condition=%d\n", robot.id, current_step, BEHAVIOR_STATE, FLOCKING_CONDITION)
	add_log(to_log)
end

---------------------------------------------------------------------------
-- Prepare the step, including obstacle handling if a grab maneuver is active.
function SetupStep()
	robot.colored_blob_omnidirectional_camera.enable()
	robot.range_and_bearing.set_data(1, BEHAVIOR_STATE) -- advertise our current phase to nearby robots
	if(obstacle_cooldown > 0) then
		obstacle_cooldown = obstacle_cooldown - 1
	end
	front_obstacle, front_left, front_right = ProcessFrontObstacle()
	if(BEHAVIOR_STATE == STATE_TUNNEL and obstacle_state == 0 and obstacle_cooldown == 0 and front_obstacle) then
		obstacle_state = 1
		obstacle_counter = 0
		obstacle_contact = 0
		if(front_left >= front_right) then
			obstacle_turn_sign = -1
		else
			obstacle_turn_sign = 1
		end
	end
	if(HandleObstacle()) then
		robot.range_and_bearing.clear_data()
		current_step = current_step + 1
		return true
	end
	return false
end

---------------------------------------------------------------------------
-- Handle the orient state.
function HandleOrientState()
	obstacle_tangent = ComputeObstacleTangent(obstacle_vector)
	total_vector[1] = 0.90 * lj_vector[1] - 0.35 * light_vector[1] + 0.10 * obstacle_vector[1] + 0.05 * obstacle_tangent[1] + 0.10 * leader_vector[1]
	total_vector[2] = 0.90 * lj_vector[2] - 0.35 * light_vector[2] + 0.10 * obstacle_vector[2] + 0.05 * obstacle_tangent[2] + 0.10 * leader_vector[2]
	orientation_counter = orientation_counter + 1
	if(orientation_counter >= ORIENTATION_STEPS) then
		BEHAVIOR_STATE = STATE_GROUPING
	end
end

---------------------------------------------------------------------------
-- Handle the grouping state.
function HandleGroupingState()
	obstacle_tangent = ComputeObstacleTangent(obstacle_vector)
	total_vector[1] = 1.20 * lj_vector[1] - 0.15 * light_vector[1] + 0.08 * obstacle_tangent[1] + 0.20 * leader_vector[1]
	total_vector[2] = 1.20 * lj_vector[2] - 0.15 * light_vector[2] + 0.08 * obstacle_tangent[2] + 0.20 * leader_vector[2]
	if(black_ground_count > 0) then
		BEHAVIOR_STATE = STATE_BLACK_ZONE
		black_floor_counter = 0
		return
	end
	if(FLOCKING_CONDITION == 1) then
		flocking_trigger_counter = flocking_trigger_counter + 1
		if(flocking_trigger_counter >= FLOCKING_TRIGGER_THRESHOLD) then
			BEHAVIOR_STATE = STATE_TUNNEL
		end
	else
		flocking_trigger_counter = 0
	end
end

---------------------------------------------------------------------------
-- Handle the tunnel state.
function HandleTunnelState()
	--TODO: this state still relies on the black-floor trigger counter to switch away.
	total_vector[1] = lj_vector[1] - 1.05 * light_vector[1] + 0.05 * obstacle_vector[1] + 0.45 * ground_vector[1] + 0.15 * leader_vector[1]
	total_vector[2] = lj_vector[2] - 1.05 * light_vector[2] + 0.05 * obstacle_vector[2] + 0.45 * ground_vector[2] + 0.15 * leader_vector[2]
	if(black_ground_count > 0) then
		black_floor_counter = black_floor_counter + 1
	else
		black_floor_counter = 0
	end
	if(black_floor_counter >= BLACK_FLOOR_STEPS) then
		BEHAVIOR_STATE = STATE_BLACK_ZONE
	end
end

---------------------------------------------------------------------------
-- Handle the black-zone state.
function HandleBlackZoneState()
	total_vector[1] = 1.10 * ground_vector[1] + 0.03 * obstacle_vector[1]
	total_vector[2] = 1.10 * ground_vector[2] + 0.03 * obstacle_vector[2]
	if(black_ground_count == 0) then
		BEHAVIOR_STATE = STATE_TUNNEL
		black_floor_counter = 0
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
-- This function executes the obstacle-grabbing maneuver and returns true while it is active.
function HandleObstacle()
	if(obstacle_state == 0) then
		-- No grab sequence in progress.
		return false
	end
	if(obstacle_state == 1) then
		-- Move forward to make contact before attempting to attach.
		robot.turret.set_position_control_mode()
		robot.turret.set_rotation(0)
		robot.wheels.set_velocity(WHEEL_SPEED, WHEEL_SPEED)
		robot.gripper.unlock()
		if(front_obstacle) then
			obstacle_contact = obstacle_contact + front_left + front_right
		else
			obstacle_contact = 0
		end
		obstacle_counter = obstacle_counter + 1
		if(obstacle_counter >= OBSTACLE_APPROACH_STEPS and obstacle_contact >= OBSTACLE_CONTACT_THRESHOLD) then
			-- Enough sustained contact: switch to the locking phase.
			obstacle_state = 2
			obstacle_counter = 0
		end
	elseif(obstacle_state == 2) then
		-- Stop and close the gripper while the turret is passive.
		robot.wheels.set_velocity(0,0)
		robot.turret.set_position_control_mode()
		robot.turret.set_rotation(0)
		robot.gripper.lock_positive()
		obstacle_counter = obstacle_counter + 1
		if(obstacle_counter >= OBSTACLE_LOCK_STEPS) then
			-- Once locked, rotate the robot away from the obstacle.
			obstacle_state = 3
			obstacle_counter = 0
		end
	elseif(obstacle_state == 3) then
		-- Turn about 90 degrees while carrying the obstacle.
		robot.turret.set_passive_mode()
		robot.wheels.set_velocity(obstacle_turn_sign * -OBSTACLE_TURN_SPEED, obstacle_turn_sign * OBSTACLE_TURN_SPEED)
		obstacle_counter = obstacle_counter + 1
		if(obstacle_counter >= OBSTACLE_TURN_STEPS) then
			-- Finished turning: move to the release phase.
			obstacle_state = 4
			obstacle_counter = 0
		end
	else
		-- Back away briefly, then release and cool down before the next grab.
		robot.turret.set_position_control_mode()
		robot.turret.set_rotation(0)
		robot.wheels.set_velocity(-1, -1)
		robot.gripper.unlock()
		obstacle_counter = obstacle_counter + 1
		if(obstacle_counter >= OBSTACLE_RELEASE_STEPS) then
			obstacle_state = 0
			obstacle_counter = 0
			obstacle_cooldown = OBSTACLE_COOLDOWN_STEPS
			obstacle_contact = 0
		end
	end
	return true
end

---------------------------------------------------------------------------
-- This function checks whether a cylinder-like obstacle is directly in front of the robot.
function ProcessFrontObstacle()
	front_obstacle = false
	front_left = 0
	front_right = 0
	front_max = 0
	for i = 1, 24 do
		if(math.abs(robot.proximity[i].angle) < 2 * math.pi / 3 and robot.proximity[i].value > front_max) then
			front_max = robot.proximity[i].value
		end
		if(robot.proximity[i].angle >= 0 and robot.proximity[i].angle < 2 * math.pi / 3) then
			front_left = front_left + robot.proximity[i].value
		elseif(robot.proximity[i].angle < 0 and robot.proximity[i].angle > -2 * math.pi / 3) then
			front_right = front_right + robot.proximity[i].value
		end
	end
	if(front_max > OBSTACLE_FRONT_THRESHOLD) then
		front_obstacle = true
	end
	return front_obstacle, front_left, front_right
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
	robot.turret.set_position_control_mode()
	robot.turret.set_rotation(0)
	robot.gripper.unlock()
	BEHAVIOR_STATE = STATE_ORIENT
	orientation_counter = 0
	flocking_trigger_counter = 0
	black_floor_counter = 0
	obstacle_state = 0
	obstacle_counter = 0
	obstacle_turn_sign = 1
	obstacle_cooldown = 0
	obstacle_contact = 0
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