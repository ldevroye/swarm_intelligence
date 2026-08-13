---- hihi

---------------------------------------------------------------------------
-- global variables
TARGET_DIST = 80 -- the target distance between robots, in cm
EPSILON = 50 -- a coefficient to increase the force of the repulsion/attraction function
WHEEL_SPEED = 6 -- max wheel speed

ACCEPTED_DIST = 10 -- range of accepted distance around the target distance
NEIGHBORS_AT_TARG_DIST = 3 -- minimum number of neighbors that must be at the right distance for the grouping condition to be verified
FLOCKING_TRIGGER_THRESHOLD = 40 -- number of consecutive timesteps in which the grouping condition must hold before switching to the black-zone phase
ORIENTATION_STEPS = 100 -- number of steps to rotate toward the light before moving
BLACK_FLOOR_STEPS = 10 -- number of consecutive timesteps on black before committing to the target zone
OBSTACLE_FRONT_THRESHOLD = 0.08 -- proximity threshold for deciding that an obstacle is directly in front
OBSTACLE_CONTACT_THRESHOLD = 1.2 -- summed front proximity needed before attempting to lock
OBSTACLE_CLOSE_THRESHOLD = 0.6 -- individual proximity reading needed to treat an obstacle as very close
OBSTACLE_APPROACH_STEPS = 1 -- number of timesteps spent nudging toward the obstacle before locking
OBSTACLE_LOCK_STEPS = 2 -- number of timesteps spent closing the gripper before turning
OBSTACLE_TURN_SPEED = 2 -- wheel speed used while turning with a gripped obstacle
OBSTACLE_TURN_STEPS = 4 -- number of timesteps to rotate about 90 degrees while holding an obstacle
OBSTACLE_RELEASE_STEPS = 3 -- number of timesteps spent releasing the obstacle before resuming normal motion
OBSTACLE_COOLDOWN_STEPS = 120 -- number of timesteps to ignore new grab attempts after a release
OBSTACLE_MAX_STEPS = 40 -- maximum total timesteps allowed for one obstacle sequence
BLACK_RANDOM_WALK_STEPS = 75 -- number of timesteps to keep one random heading in the black zone
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
obstacle_elapsed = 0
black_walk_counter = 0
black_walk_angle = 0


ID = robot.id
directory="logs/"
LOG_FILE = directory..ID..".log"
logf = nil
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
	light_vector = ComputeVectorToLight() -- we compute the vector towards the light source
	obstacle_vector = ComputeVectorFromProximity() -- we compute a repulsion vector away from nearby obstacles
	obstacle_tangent = ComputeObstacleTangent(obstacle_vector)
	leader_vector = ProcessRABLeaders() -- grouping robots can pull toward the local swarm structure
	close_obstacle_tangent, close_obstacle_count = ComputeCloseObstacleVectorTangent()

	leader_vector = {0,0}
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
	if(BEHAVIOR_STATE == STATE_TUNNEL or BEHAVIOR_STATE == STATE_BLACK_ZONE) then
		speeds = ComputeMoveTowardTargetSpeeds(total_vector)
	else
		target_angle = math.atan2(total_vector[2],total_vector[1]) -- compute the angle from the vector
		speeds = ComputeSpeedFromAngle(target_angle) -- we now compute the wheel speed necessary to go in the direction of the target angle
	end
	if(BEHAVIOR_STATE == STATE_ORIENT) then
		robot.wheels.set_velocity(0.8 * speeds[1], 0.8 * speeds[2]) -- move slowly while centering between light and obstacles
	elseif(BEHAVIOR_STATE == STATE_GROUPING) then
		robot.wheels.set_velocity(0.85 * speeds[1], 0.85 * speeds[2]) -- keep advancing while still staying aligned
	elseif(BEHAVIOR_STATE == STATE_TUNNEL) then
		robot.wheels.set_velocity(0.95 * speeds[1], 0.95 * speeds[2]) -- move faster through the tunnel while keeping forward bias
	elseif(BEHAVIOR_STATE == STATE_BLACK_ZONE) then
		robot.wheels.set_velocity(0.85 * speeds[1], 0.85 *speeds[2]) -- keep moving with enough authority to stay in the zone
	else
		robot.wheels.set_velocity(speeds[1], speeds[2])
	end
	robot.range_and_bearing.clear_data() -- forget about all received messages for next step
end

---------------------------------------------------------------------------
-- Log the start of each step once, before any setup or state handling.
function LogStepStart()
	to_log = string.format("step=%d, state=%d, flocking_condition=%d", current_step, BEHAVIOR_STATE, FLOCKING_CONDITION)
	add_log(to_log)
end

---------------------------------------------------------------------------
-- Prepare the step, including obstacle handling if a grab maneuver is active.
function SetupStep()
	current_step = current_step + 1
	robot.colored_blob_omnidirectional_camera.enable()
	ground_vector, black_ground_count = ProcessGround() -- use the floor sensors to bias motion into the black target area
	robot.leds.set_all_colors("blue")
	if(BEHAVIOR_STATE == STATE_BLACK_ZONE) then
		robot.leds.set_single_color(13, "green")
	end
	robot.range_and_bearing.set_data(1, BEHAVIOR_STATE) -- advertise our current phase to nearby robots
	robot.range_and_bearing.set_data(2, BEHAVIOR_STATE == STATE_BLACK_ZONE and 1 or 0) -- explicitly mark robots that are in the black zone
	beacon_vector, beacon_seen = ProcessBlackZoneBeacon()
	if(beacon_seen > 0) then
		add_log("beacon robot seen")
	end
	if(black_ground_count == 0 and beacon_seen > 0) then
		add_log("going towards it")
		if(obstacle_state ~= 0) then
			robot.turret.set_position_control_mode()
			robot.turret.set_rotation(0)
			robot.gripper.unlock()
			obstacle_state = 0
			obstacle_counter = 0
			obstacle_contact = 0
			obstacle_elapsed = 0
			obstacle_cooldown = OBSTACLE_COOLDOWN_STEPS
		end
		front_obstacle, front_left, front_right, front_centered, front_grabbable = ProcessFrontObstacle()
		total_vector = {beacon_vector[1], beacon_vector[2]}
		beacon_avoid_vector = ComputeVectorFromProximity()
		total_vector[1] = total_vector[1] + 0.75 * beacon_avoid_vector[1]
		total_vector[2] = total_vector[2] + 0.75 * beacon_avoid_vector[2]
		if(front_obstacle or close_obstacle_count > 0) then
			beacon_tangent = ComputeCloseObstacleVectorTangent()
			total_vector[1] = total_vector[1] + 0.95 * beacon_tangent[1]
			total_vector[2] = total_vector[2] + 0.95 * beacon_tangent[2]
		end
		speeds = ComputeMoveTowardTargetSpeeds(total_vector)
		robot.wheels.set_velocity(speeds[1], speeds[2])
		robot.range_and_bearing.clear_data()
		
		return true
	end

	if(obstacle_cooldown > 0) then
		obstacle_cooldown = obstacle_cooldown - 1
	end
	front_obstacle, front_left, front_right, front_centered, front_grabbable = ProcessFrontObstacle()
	if((BEHAVIOR_STATE == STATE_TUNNEL or BEHAVIOR_STATE == STATE_BLACK_ZONE) and obstacle_state == 0 and obstacle_cooldown == 0 and front_obstacle and front_grabbable) then
		obstacle_state = 1
		obstacle_counter = 0
		obstacle_contact = 0
		obstacle_elapsed = 0
		if(math.abs(front_left - front_right) < 0.15) then
			obstacle_turn_sign = (math.random() < 0.5) and -1 or 1
		elseif(front_left >= front_right) then
			obstacle_turn_sign = -1
		else
			obstacle_turn_sign = 1
		end
	end
	if(HandleObstacle()) then
		robot.range_and_bearing.clear_data()
		return true
	end
	return false
end

---------------------------------------------------------------------------
-- Handle the orient state.
function HandleOrientState()
	total_vector[1] = 0.90 * lj_vector[1] + 0.35 * light_vector[1] + 0.10 * obstacle_vector[1] + 0.05 * obstacle_tangent[1]
	total_vector[2] = 0.90 * lj_vector[2] + 0.35 * light_vector[2] + 0.10 * obstacle_vector[2] + 0.05 * obstacle_tangent[2]
	orientation_counter = orientation_counter + 1
	if(orientation_counter >= ORIENTATION_STEPS) then
		BEHAVIOR_STATE = STATE_GROUPING
	end
end

---------------------------------------------------------------------------
-- Handle the grouping state.
function HandleGroupingState()
	leader_vector = ProcessRABLeaders() -- grouping robots can pull toward the local swarm structure
	obstacle_tangent = ComputeObstacleTangent(obstacle_vector)
	total_vector[1] = 1.20 * lj_vector[1] - 0.3 * light_vector[1] + 0.08 * obstacle_tangent[1] + 0.10 * leader_vector[1]
	total_vector[2] = 1.20 * lj_vector[2] - 0.3 * light_vector[2] + 0.08 * obstacle_tangent[2] + 0.10 * leader_vector[2]

	if(obstacle_state == 0 and close_obstacle_count >= 3) then
		-- Add a sideways correction without losing the main target drive.
		total_vector[1] = total_vector[1] + 0.2 * close_obstacle_tangent[1]
		total_vector[2] = total_vector[2] + 0.2 * close_obstacle_tangent[2]
	end

	if(black_ground_count > 0) then
		BEHAVIOR_STATE = STATE_BLACK_ZONE
		black_floor_counter = 0
		black_walk_counter = 0
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
	TARGET_DIST=60
	-- total_vector[1] = lj_vector[1] - 1.05 * light_vector[1] + 0.05 * obstacle_vector[1] + 0.45 * ground_vector[1] + 0.15 * leader_vector[1]
	total_vector[1] = lj_vector[1] - 1.05 * light_vector[1] + 1.50 * ground_vector[1] + 0.1 * obstacle_vector[1]
	total_vector[2] = lj_vector[2] - 1.05 * light_vector[2] + 1.50 * ground_vector[2] + 0.1 * obstacle_vector[2]
	
	if(obstacle_state == 0 and close_obstacle_count >= 3) then
		-- Add a sideways correction without losing the main target drive.
		total_vector[1] = total_vector[1] + 0.2 * close_obstacle_tangent[1]
		total_vector[2] = total_vector[2] + 0.2 * close_obstacle_tangent[2]
	end

	if(black_ground_count > 0) then
		black_floor_counter = black_floor_counter + 1
	else
		black_floor_counter = 0
	end
	if(black_floor_counter >= BLACK_FLOOR_STEPS) then
		BEHAVIOR_STATE = STATE_BLACK_ZONE
		black_walk_counter = 0
	end
end

---------------------------------------------------------------------------
-- Handle the black-zone state.
function HandleBlackZoneState()
	if(obstacle_state == 0) then
		if(black_walk_counter <= 0) then
			black_walk_counter = BLACK_RANDOM_WALK_STEPS
			black_walk_angle = robot.random.uniform(-math.pi, math.pi)
		end
		black_walk_counter = black_walk_counter - 1
		if(black_ground_count > 0) then
			total_vector[1] = 2.50 * ground_vector[1] + 0.35 * math.cos(black_walk_angle) + 0.20 * obstacle_tangent[1]
			total_vector[2] = 2.50 * ground_vector[2] + 0.35 * math.sin(black_walk_angle) + 0.20 * obstacle_tangent[2]
		else
			total_vector[1] = 4.00 * ground_vector[1] + 0.12 * math.cos(black_walk_angle) + 0.20 * obstacle_tangent[1]
			total_vector[2] = 4.00 * ground_vector[2] + 0.12 * math.sin(black_walk_angle) + 0.20 * obstacle_tangent[2]
		end
	else
		total_vector[1] = 1.45 * light_vector[1]
		total_vector[2] = 1.45 * light_vector[2]
	end
	if(black_ground_count > 0) then
		black_floor_counter = 0
	else
		black_floor_counter = black_floor_counter + 1
		if(black_floor_counter >= BLACK_FLOOR_STEPS) then
			BEHAVIOR_STATE = STATE_TUNNEL
			black_floor_counter = 0
		end
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
-- This function computes a repulsion vector only from very close obstacles.
function ComputeCloseObstacleVectorTangent()
	close_prox_v = {0,0}
	tangent_v = {0,0}
	close_count = 0

	for i = 1, 24 do
		if(robot.proximity[i].value >= OBSTACLE_CLOSE_THRESHOLD) then
			close_count = close_count + 1
			close_prox_v[1] = close_prox_v[1] - robot.proximity[i].value * math.cos(robot.proximity[i].angle)
			close_prox_v[2] = close_prox_v[2] - robot.proximity[i].value * math.sin(robot.proximity[i].angle)
		end
	end
	if(close_count < 2) then
		return tangent_v, close_count
	end
	len = math.sqrt(close_prox_v[1] * close_prox_v[1] + close_prox_v[2] * close_prox_v[2])
	if(len ~= 0) then
		tangent_v[1] = close_prox_v[2]
		tangent_v[2] = -close_prox_v[1]
	end
	len = math.sqrt(tangent_v[1] * tangent_v[1] + tangent_v[2] * tangent_v[2])
	if(len ~= 0) then
		tangent_v[1] = tangent_v[1] / len
		tangent_v[2] = tangent_v[2] / len
	end
	return tangent_v, close_count
end

---------------------------------------------------------------------------
-- This function executes the obstacle-grabbing maneuver and returns true while it is active.
function HandleObstacle()
	if(obstacle_state == 0) then
		-- No grab sequence in progress.
		return false
	end
	if(obstacle_state == 1) then
		HandleObstacleApproach()
	elseif(obstacle_state == 2) then
		HandleObstacleLock()
	elseif(obstacle_state == 3) then
		HandleObstacleCarry()
	else
		HandleObstacleRelease()
	end
	if(ShouldAbortObstacleSequence()) then
		ResetObstacleSequence()
		return false
	end
	return true
end

---------------------------------------------------------------------------
-- Reset obstacle-related actuators and counters after a release or timeout.
function ResetObstacleSequence()
	robot.turret.set_position_control_mode()
	robot.turret.set_rotation(0)
	robot.gripper.unlock()
	robot.wheels.set_velocity(0,0)
	obstacle_state = 0
	obstacle_counter = 0
	obstacle_contact = 0
	obstacle_elapsed = 0
	obstacle_cooldown = OBSTACLE_COOLDOWN_STEPS
end

---------------------------------------------------------------------------
-- Abort the sequence if it has run for too long, except for the black-zone carry phase.
function ShouldAbortObstacleSequence()
	return obstacle_state ~= 0 and obstacle_elapsed >= OBSTACLE_MAX_STEPS and not (BEHAVIOR_STATE == STATE_BLACK_ZONE and obstacle_state >= 3)
end

---------------------------------------------------------------------------
-- Approach an obstacle until it is centered and close enough to lock.
function HandleObstacleApproach()
	robot.turret.set_position_control_mode()
	robot.turret.set_rotation(0)
	robot.wheels.set_velocity(0.6 * WHEEL_SPEED, 0.6 * WHEEL_SPEED)
	robot.gripper.unlock()
	if(front_obstacle) then
		obstacle_contact = obstacle_contact + front_left + front_right
	else
		obstacle_contact = 0
	end
	obstacle_counter = obstacle_counter + 1
	obstacle_elapsed = obstacle_elapsed + 1
	if((obstacle_counter >= OBSTACLE_APPROACH_STEPS and obstacle_contact >= OBSTACLE_CONTACT_THRESHOLD) or (front_close_count >= 2 and front_obstacle)) then
		obstacle_state = 2
		obstacle_counter = 0
	end
end

---------------------------------------------------------------------------
-- Close the gripper once the obstacle is sufficiently aligned.
function HandleObstacleLock()
	robot.wheels.set_velocity(0,0)
	robot.turret.set_position_control_mode()
	robot.turret.set_rotation(0)
	robot.gripper.lock_positive()
	obstacle_counter = obstacle_counter + 1
	obstacle_elapsed = obstacle_elapsed + 1
	if(obstacle_counter >= OBSTACLE_LOCK_STEPS) then
		obstacle_state = 3
		obstacle_counter = 0
	end
end

---------------------------------------------------------------------------
-- Carry the obstacle until the black-zone robot reaches grey or until the
-- normal turn-and-release sequence finishes.
function HandleObstacleCarry()
	robot.turret.set_passive_mode()
	if(BEHAVIOR_STATE == STATE_BLACK_ZONE) then
		speeds = ComputePushTowardLightSpeeds()
		robot.wheels.set_velocity(speeds[1], speeds[2])
		obstacle_counter = obstacle_counter + 1
		obstacle_elapsed = obstacle_elapsed + 1
		if(black_ground_count == 0) then
			obstacle_state = 4
			obstacle_counter = 0
		end
	else
		robot.wheels.set_velocity(obstacle_turn_sign * -OBSTACLE_TURN_SPEED, obstacle_turn_sign * OBSTACLE_TURN_SPEED)
		obstacle_counter = obstacle_counter + 1
		obstacle_elapsed = obstacle_elapsed + 1
		if(obstacle_counter >= OBSTACLE_TURN_STEPS) then
			obstacle_state = 4
			obstacle_counter = 0
		end
	end
end

---------------------------------------------------------------------------
-- Release the obstacle, or keep pushing it in the black zone until grey is seen.
function HandleObstacleRelease()
	robot.turret.set_position_control_mode()
	robot.turret.set_rotation(0)
	if(BEHAVIOR_STATE == STATE_BLACK_ZONE and black_ground_count > 0) then
		speeds = ComputePushTowardLightSpeeds()
		robot.wheels.set_velocity(speeds[1], speeds[2])
		obstacle_counter = obstacle_counter + 1
		obstacle_elapsed = obstacle_elapsed + 1
	else
		robot.wheels.set_velocity(0, 0)
		robot.gripper.unlock()
		obstacle_counter = obstacle_counter + 1
		obstacle_elapsed = obstacle_elapsed + 1
		if(obstacle_counter >= OBSTACLE_RELEASE_STEPS or black_ground_count == 0) then
			obstacle_state = 0
			obstacle_counter = 0
			obstacle_cooldown = OBSTACLE_COOLDOWN_STEPS
			obstacle_contact = 0
		end
	end
end

---------------------------------------------------------------------------
-- This function checks whether a cylinder-like obstacle is directly in front of the robot.
function ProcessFrontObstacle()
	front_obstacle = false
	front_centered = false
	front_grabbable = false
	front_left = 0
	front_right = 0
	front_max = 0
	front_close_count = 0
	for i = 1, 24 do
		if(math.abs(robot.proximity[i].angle) < 2 * math.pi / 3 and robot.proximity[i].value > front_max) then
			front_max = robot.proximity[i].value
		end
		if(robot.proximity[i].angle >= 0 and robot.proximity[i].angle < 2 * math.pi / 3) then
			front_left = front_left + robot.proximity[i].value
			if(robot.proximity[i].value >= OBSTACLE_CLOSE_THRESHOLD) then
				front_close_count = front_close_count + 1
			end
		elseif(robot.proximity[i].angle < 0 and robot.proximity[i].angle > -2 * math.pi / 3) then
			front_right = front_right + robot.proximity[i].value
			if(robot.proximity[i].value >= OBSTACLE_CLOSE_THRESHOLD) then
				front_close_count = front_close_count + 1
			end
		end
	end
	if(front_max > OBSTACLE_FRONT_THRESHOLD) then
		front_obstacle = true
	end
	front_sum = front_left + front_right
	if(front_obstacle and front_sum > 0) then
		front_balance = math.abs(front_left - front_right) / front_sum
		if(front_balance < 0.35 and front_sum >= OBSTACLE_CONTACT_THRESHOLD) then
			front_centered = true
		end
	end
	if(front_obstacle and (front_sum >= OBSTACLE_CONTACT_THRESHOLD * 0.6 or front_max >= OBSTACLE_FRONT_THRESHOLD * 1.8)) then
		front_grabbable = true
	end
	if(not front_grabbable and front_centered and front_sum >= OBSTACLE_CONTACT_THRESHOLD * 0.4) then
		front_grabbable = true
	end
	return front_obstacle, front_left, front_right, front_centered, front_grabbable
end

---------------------------------------------------------------------------
-- This function computes a vector toward robots that have already advanced.
function ProcessRABLeaders()
	leader_v = {0,0}
	for i = 1, #robot.range_and_bearing do
		if(robot.range_and_bearing[i].data[1] >= STATE_TUNNEL) then
			leader_v[1] = leader_v[1] + math.cos(robot.range_and_bearing[i].horizontal_bearing)
			leader_v[2] = leader_v[2] + math.sin(robot.range_and_bearing[i].horizontal_bearing)
		end
	end
	len = math.sqrt(leader_v[1] * leader_v[1] + leader_v[2] * leader_v[2])
	if(len ~= 0) then
		leader_v[1] = leader_v[1] / len
		leader_v[2] = leader_v[2] / len
	end
	return leader_v
end

--------------------------------------------------------------------------
-- This function computes a direct vector toward the first black-zone beacon seen by the camera.
function ProcessBlackZoneBeacon()
	beacon_v = {0,0}
	for i = 1, #robot.colored_blob_omnidirectional_camera do
		blob = robot.colored_blob_omnidirectional_camera[i]
		if(blob.color.green > 200 and blob.color.red < 80 and blob.color.blue < 80) then
			beacon_v[1] = math.cos(blob.angle)
			beacon_v[2] = math.sin(blob.angle)
			return beacon_v, 1
		end
	end
	return beacon_v, 0
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

	-- clamp wheel speeds to allowed range
	for i = 1, 2 do
		if speeds[i] > WHEEL_SPEED then
			speeds[i] = WHEEL_SPEED
		elseif speeds[i] < -WHEEL_SPEED then
			speeds[i] = -WHEEL_SPEED
		end
	end

	return speeds
end

---------------------------------------------------------------------------
-- This function computes a forward-biased push toward the light so the robot
-- keeps moving while carrying an obstacle in the black zone.
function ComputePushTowardLightSpeeds()
	forward_speed = 0.65 * WHEEL_SPEED
	steer_speed = 0.30 * WHEEL_SPEED
	steer = light_vector[2]
	left_speed = forward_speed - steer_speed * steer
	right_speed = forward_speed + steer_speed * steer
	if(left_speed < 0.25 * WHEEL_SPEED) then
		left_speed = 0.25 * WHEEL_SPEED
	elseif(left_speed > WHEEL_SPEED) then
		left_speed = WHEEL_SPEED
	end
	if(right_speed < 0.25 * WHEEL_SPEED) then
		right_speed = 0.25 * WHEEL_SPEED
	elseif(right_speed > WHEEL_SPEED) then
		right_speed = WHEEL_SPEED
	end
	return {left_speed, right_speed}
end

---------------------------------------------------------------------------
-- This function computes a forward-biased motion toward a target vector so
-- the robot keeps moving while still steering around obstacles.
function ComputeMoveTowardTargetSpeeds(target_vector)
	target_angle = math.atan2(target_vector[2], target_vector[1])
	forward_speed = 0.70 * WHEEL_SPEED
	steer = target_angle
	if(steer > math.pi/2) then
		steer = math.pi/2
	elseif(steer < -math.pi/2) then
		steer = -math.pi/2
	end
	turn_speed = 0.45 * WHEEL_SPEED
	left_speed = forward_speed - turn_speed * steer
	right_speed = forward_speed + turn_speed * steer
	if(left_speed < 0.20 * WHEEL_SPEED) then
		left_speed = 0.20 * WHEEL_SPEED
	elseif(left_speed > WHEEL_SPEED) then
		left_speed = WHEEL_SPEED
	end
	if(right_speed < 0.20 * WHEEL_SPEED) then
		right_speed = 0.20 * WHEEL_SPEED
	elseif(right_speed > WHEEL_SPEED) then
		right_speed = WHEEL_SPEED
	end
	return {left_speed, right_speed}
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
	if(neighbors_in_range_counter < NEIGHBORS_AT_TARG_DIST) then
		sum_vector[1] = 0.7 * sum_vector[1]
		sum_vector[2] = 0.7 * sum_vector[2]
	end

	to_log = string.format("#neighbors=%d", neighbors_in_range_counter)
	add_log(to_log)
	
	return sum_vector


end
---------------------------------------------------------------------------

---------------------------------------------------------------------------
-- This function take the distance and compute the lennard-jones potential.
-- The parameters are defined at the top of the script
function ComputeLennardJones(distance)
	if(distance == nil) then
		return 0
	end
	-- avoid division by zero / extremely small distances
	if(distance < 0.01) then
		distance = 0.01
	end
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
	obstacle_elapsed = 0
	black_walk_counter = 0
	black_walk_angle = 0
	-- ensure logs directory exists and open per-robot log
	os.execute("mkdir -p "..directory)
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
		logf:write(log.."\n")
		logf:flush()
	end
end