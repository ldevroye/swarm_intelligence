---------------------------------------------------------------------------
-- global variables
TARGET_DIST = 80 -- the target distance between robots, in cm
EPSILON = 50 -- a coefficient to increase the force of the repulsion/attraction function
WHEEL_SPEED = 10 -- max wheel speed

ACCEPTED_DIST = 10 -- range of accepted distance around the target distance
NEIGHBORS_AT_TARG_DIST = 3 -- minimum number of neighbors that must be at the right distance for the grouping condition to be verified
FLOCKING_TRIGGER_THRESHOLD = 40 -- number of consecutive timesteps in which the grouping condition must hold before switching to the black-zone phase
ORIENTATION_STEPS = 50 -- number of steps to rotate toward the light before moving
BLACK_FLOOR_STEPS = 10 -- number of consecutive timesteps on black before committing to the target zone
OBSTACLE_FRONT_THRESHOLD = 0.08 -- proximity threshold for deciding that an obstacle is directly in front
OBSTACLE_CONTACT_THRESHOLD = 0.8-- summed front proximity needed before attempting to lock
OBSTACLE_CLOSE_THRESHOLD = 0.1 -- individual proximity reading needed to treat an obstacle as very close
OBSTACLE_APPROACH_STEPS = 1 -- number of timesteps spent nudging toward the obstacle before locking
OBSTACLE_LOCK_STEPS = 2 -- number of timesteps spent closing the gripper before turning
OBSTACLE_TURN_SPEED = 2 -- wheel speed used while turning with a gripped obstacle
OBSTACLE_TURN_STEPS = 4 -- number of timesteps to rotate about 90 degrees while holding an obstacle
OBSTACLE_RELEASE_STEPS = 3 -- number of timesteps spent releasing the obstacle before resuming normal motion
OBSTACLE_COOLDOWN_STEPS = 120 -- number of timesteps to ignore new grab attempts after a release
OBSTACLE_MAX_STEPS = 40 -- maximum total timesteps allowed for one obstacle sequence
WALL_SENSOR_THRESHOLD = 0.30 -- proximity value considered part of a wall contact band
WALL_MIN_ACTIVE_SENSORS = 12 -- if many front sensors are active, treat contact as wall-like
FRONT_BOT_BLOCK_RANGE = 45 -- if a robot is this close and centered, do not attempt a grab
FRONT_BOT_BLOCK_BEARING = 0.45 -- forward bearing cone (radians) for robot blocking checks
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

--- logs in "logs/"robot.id".log" -> "logs/fb1.log"
ID = robot.id
directory="logs/"
LOG_FILE = directory..ID..".log"
logf = nil
current_step = 0;

---------------------------------------------------------------------------
--
--	Author: Louis Devroye (523920) Louis.Devroye@ulb.be
--	Date: 14/08/26
--	Tunneling 2nd session project - 'Swarm Intelligence, INFO-H-414' ULB
--
---------------------------------------------------------------------------
-- overall workflow
-- setup facts
-- build local force vectors
-- choose phase
-- turn vector into wheel speeds
-- keep motion alive and log state
-- lj_vector : social pull from nearby robots
-- light_vector : direction to light source
-- obstacle_vector : repulsion from nearby obstacles
-- ground_vector : push toward black target floor
-- total_vector : weighted sum used for steering
---------------------------------------------------------------------------
-- Expected behavior :
-- Phase 1: orienting -> grouping in the middle with a nudge towards the obstacles
-- Phase 2: grouping -> wandering a bit while avoiding obstacles and try to search for a group
-- Phase 3: tunneling (loose flocking) -> wandering more while avoiding obstacles and trying to influence the Phase 2 bots into coming along
-- Phase 4: Cleaning -> Random walk 
-- TODO: Phase 4 obstacles cleaning by :
--		-- random walk
--		-- hit an obstacle
--		-- grab it, walk towards light for X step
--		-- wait until either ground sensor is not full black or X step elapsed
--		-- spin towards light for the obstacle to be in the grey zone
--		-- rince
--		-- repeat
--
-- In addition to that, the bots display their states via the LEDs.
-- A bot in the black zone is a 'beacon' that any bot , not in blackzone, will go towards it (while avoiding obstacles).
-- Also, the obstacle handling is introduced but non workin as of for now
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

	target_angle = math.atan2(total_vector[2],total_vector[1]) -- compute the angle from the vector
	speeds = ComputeSpeedFromAngle(target_angle) -- we now compute the wheel speed necessary to go in the direction of the target angle
	final_left = speeds[1]
	final_right = speeds[2]
	
	if(BEHAVIOR_STATE == STATE_ORIENT) then
		final_left = 0.8 * speeds[1]
		final_right = 0.8 * speeds[2] -- move slowly while centering between light and obstacles
	elseif(BEHAVIOR_STATE == STATE_TUNNEL) then
		final_left = 0.85 * speeds[1]
		final_right = 0.85 * speeds[2] -- keep advancing while still staying aligned
	elseif(BEHAVIOR_STATE == STATE_BLACK_ZONE) then
		final_left = 0.5 * speeds[1]
		final_right = 0.5 * speeds[2] -- actuate wheels to move
	end


	-- low-speed fallback
	-- fuzz inducer
	if(BEHAVIOR_STATE ~= STATE_BLACK_ZONE and math.abs(final_left) + math.abs(final_right) < 1.0) then
		if(robot.random.uniform() < 0.5) then
			final_left = 0.5 * WHEEL_SPEED
			final_right = -0.5 * WHEEL_SPEED
		else
			final_left = -0.5 * WHEEL_SPEED
			final_right = 0.5 * WHEEL_SPEED
		end
	end

	robot.wheels.set_velocity(final_left, final_right)
	robot.range_and_bearing.clear_data() -- forget about all received messages for next step
end

---------------------------------------------------------------------------
-- Log the start of each step once, before any setup or state handling.
function LogStepStart()
	to_log = string.format("step=%d, state=%d, flocking_condition=%d", current_step, BEHAVIOR_STATE, FLOCKING_CONDITION)
	add_log(to_log)
end

---------------------------------------------------------------------------
-- tick counter
-- sample floor and front obstacle
-- update phase from black floor and cooldown
-- broadcast phase to neighbors
-- steer toward beacon if visible
-- start local grab flow if needed
function SetupStep()
	current_step = current_step + 1
	robot.colored_blob_omnidirectional_camera.enable()
	ground_vector, black_ground_count = ProcessGround() -- use floor sensors to bias motion into the black target area and detect black/grey edges
	edge_detected = black_ground_count < 4 and black_ground_count > 0
	front_obstacle, front_left, front_right, front_centered, front_grabbable, front_robot_block, front_wall_block = ProcessFrontObstacle()
	
	-- leave black zone state when black floor disappears
	if(BEHAVIOR_STATE == STATE_BLACK_ZONE and black_ground_count == 0) then
		BEHAVIOR_STATE = STATE_TUNNEL
		black_floor_counter = 0
		black_walk_counter = 0
	end

	-- cooldown timer for obstacle retries
	if(obstacle_cooldown > 0) then
		obstacle_cooldown = obstacle_cooldown - 1
	end

	-- count dark-floor persistence before committing to target area
	if(BEHAVIOR_STATE ~= STATE_BLACK_ZONE and black_ground_count > 0) then
		black_floor_counter = black_floor_counter + 1
	else
		black_floor_counter = 0
	end

	-- black-zone transition
	if(black_floor_counter >= BLACK_FLOOR_STEPS) then
		BEHAVIOR_STATE = STATE_BLACK_ZONE
		black_floor_counter = 0
	end

	robot.leds.set_all_colors("blue")
	if(BEHAVIOR_STATE == STATE_BLACK_ZONE) then
		robot.leds.set_single_color(13, "green")
	elseif(BEHAVIOR_STATE == STATE_TUNNEL) then
		robot.leds.set_single_color(13, "orange")
	else 
		robot.leds.set_single_color(13, "blue")
	end

	robot.range_and_bearing.set_data(1, BEHAVIOR_STATE) -- advertise our current phase to nearby robots
	robot.range_and_bearing.set_data(2, BEHAVIOR_STATE == STATE_BLACK_ZONE and 1 or 0) -- explicitly mark robots that are in the black zone
	beacon_vector, beacon_seen = ProcessBlackZoneBeacon()
	close_obstacle_tangent, close_obstacle_count = ComputeCloseObstacleVectorTangent(beacon_vector)
	if(beacon_seen > 0) then
		add_log("beacon robot seen")
	end

	-- beacon override
	-- weak dark floor + beacon seen => aim at target beacon
	if(black_ground_count < 2 and beacon_seen > 0) then
		add_log("going towards it")
		if(obstacle_state ~= 0) then
			ResetObstacleSequence()
		end
		total_vector = {0,0}
		if(front_obstacle and close_obstacle_count > 0) then
			total_vector[1] = beacon_vector[1] + 1.2 * close_obstacle_tangent[1]
			total_vector[2] = beacon_vector[2] + 1.2 * close_obstacle_tangent[2]
		end
		target_angle = math.atan2(total_vector[2], total_vector[1])
		speeds = ComputeSpeedFromAngle(target_angle) 
		robot.wheels.set_velocity(speeds[1], speeds[2])
		robot.range_and_bearing.clear_data()
		
		return true
	end

	-- start grab only in black zone
	-- safe front, centered obstacle, no cooldown remaining
	if(BEHAVIOR_STATE == STATE_BLACK_ZONE and obstacle_state == 0 and obstacle_cooldown == 0 and front_obstacle and front_grabbable) then
		obstacle_state = 1
		obstacle_counter = 0
		obstacle_contact = 0
		obstacle_elapsed = 0
		if(front_left >= front_right) then
			obstacle_turn_sign = -1
		else
			obstacle_turn_sign = 1
		end
	end

	-- TODO: fix handling obstacles to clear black zone
	if(false) then 
		if(HandleObstacle()) then
			robot.range_and_bearing.clear_data()
			return true
		end
	else
		ResetObstacleSequence()
	end

	return false
end

function ResetObstacleSequence()
	robot.turret.set_position_control_mode()
	robot.turret.set_rotation(0)
	robot.gripper.unlock()
	obstacle_state = 0
	obstacle_counter = 0
	obstacle_contact = 0
	obstacle_elapsed = 0
	obstacle_black_counter = 0
	obstacle_cooldown = OBSTACLE_COOLDOWN_STEPS
end

---------------------------------------------------------------------------
-- social pull + light pull + small obstacle tangent
-- keep heading stable before grouping starts
function HandleOrientState()
	obstacle_tangent = ComputeObstacleTangent(obstacle_vector)
	total_vector[1] = 0.9 * lj_vector[1] + 0.35 * light_vector[1] + 0.10 * obstacle_vector[1] + 0.05 * obstacle_tangent[1]
	total_vector[2] = 0.90 * lj_vector[2] + 0.35 * light_vector[2] + 0.10 * obstacle_vector[2] + 0.05 * obstacle_tangent[2]
	orientation_counter = orientation_counter + 1
	-- fixed timing gate
	if(orientation_counter >= ORIENTATION_STEPS) then
		BEHAVIOR_STATE = STATE_GROUPING
	end
end

---------------------------------------------------------------------------
-- keep target spacing from neighbors
-- pull back from light to avoid overshoot
-- add sideways obstacle drift when needed
-- move to tunnel once cluster holds
function HandleGroupingState()
	leader_vector = ProcessRABLeaders() -- grouping robots can pull toward the local swarm structure
	total_vector[1] = 0.6 * lj_vector[1] - 0.5 * light_vector[1] + 0.2 * leader_vector[1]
	total_vector[2] = 0.6 * lj_vector[2] - 0.5 * light_vector[2] + 0.2 * leader_vector[2]

	if(front_obstacle) then
		close_tangent, nbr = ComputeCloseObstacleVectorTangent(total_vector)
		-- add sideways drift to slip around local obstacle without losing main drive
		total_vector[1] = total_vector[1] + 0.5 * close_tangent[1]
		total_vector[2] = total_vector[2] + 0.5 * close_tangent[2]
	end

	-- swarm stability trigger
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
-- push toward black zone while staying with swarm
-- keep small lateral obstacle drift
function HandleTunnelState()
	TARGET_DIST=60
	-- main drive
	-- swarm attraction + light repulsion + leader pull
	-- total_vector[1] = lj_vector[1] - 1.05 * light_vector[1] + 0.05 * obstacle_vector[1] + 0.45 * ground_vector[1] + 0.15 * leader_vector[1]
	total_vector[1] = 0.4 * lj_vector[1] - 1.05 * light_vector[1] + 0.10 * leader_vector[1]
	total_vector[2] = 0.4 * lj_vector[2] - 1.05 * light_vector[2] + 0.10 * leader_vector[1]
	
	if(front_obstacle) then
		-- sideways nudge around obstacle while keeping heading
		close_tangent, nbr = ComputeCloseObstacleVectorTangent(total_vector)
		total_vector[1] = total_vector[1] + 0.3 * close_tangent[1]
		total_vector[2] = total_vector[2] + 0.3 * close_tangent[2]
	end
end

---------------------------------------------------------------------------
-- commit to dark floor when long often enough
-- drift with random heading until wall contact
-- fall back to tunnel if dark floor disappears
function HandleBlackZoneState()
	if(black_ground_count < 4 and not IsWallAhead()) then
		-- leave heading as soon as dark floor fades
		total_vector = ground_vector
		obstacle_counter = 0
		return
	end

	-- random direction in a bounded cone for drift
	if(obstacle_counter == 0 or obstacle_counter >= 100) then
		obstacle_counter = 0
		obstacle_contact = (robot.random.uniform() - 0.5) * (math.pi / 2)
	end

	obstacle_counter = obstacle_counter + 1
	total_vector[1] = math.cos(obstacle_contact)
	total_vector[2] = math.sin(obstacle_contact)

	if(IsWallAhead()) then
		-- push away from wall contact, then renormalize
		total_vector[1] = total_vector[1] + 1.5 * obstacle_vector[1]
		total_vector[2] = total_vector[2] + 1.5 * obstacle_vector[2]
		len = math.sqrt(total_vector[1] * total_vector[1] + total_vector[2] * total_vector[2])
		if(len ~= 0) then
			total_vector[1] = total_vector[1] / len
			total_vector[2] = total_vector[2] / len
		end
	end
end

---------------------------------------------------------------------------
-- light vector
-- sum sensor contributions
-- each sensor gives a weighted direction to the light
-- normalize to unit vector for steering
function ComputeVectorToLight()
	light_v = {0,0}
	for i = 1, 24 do 
		-- sensor direction * light intensity = local contribution
		vec = {
			x = robot.light[i].value * math.cos(robot.light[i].angle),
			y = robot.light[i].value * math.sin(robot.light[i].angle)
		}
		-- sum x and y contributions from all sensors
		light_v[1] = light_v[1] + vec.x
		light_v[2] = light_v[2] + vec.y
	end
	len = math.sqrt(light_v[1] * light_v[1] + light_v[2] * light_v[2])
	-- unit vector keeps only direction, not brightness
	if(len ~= 0) then 
		light_v[1] = light_v[1] / len
   		light_v[2] = light_v[2] / len
	end
	return light_v
end	

---------------------------------------------------------------------------
-- proximity vector
-- each close sensor pushes away from that direction
-- sum all pushes, then normalize to a unit vector
function ComputeVectorFromProximity()
	prox_v = {0,0}
	for i = 1, 24 do
		-- sensor value is strength of obstacle on that side
		-- minus sign makes the vector point away from the obstacle
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
-- close obstacle tangent
-- keep only near obstacles
-- turn sideways around them instead of pushing straight through
function ComputeCloseObstacleVectorTangent()
	close_prox_v = {0,0}
	tangent_v = {0,0}
	close_count = 0

	for i = 1, 24 do
		if(robot.proximity[i].value >= OBSTACLE_CLOSE_THRESHOLD) then
			close_count = close_count + 1
			-- use strong local repulsion to build a side-slip vector
			close_prox_v[1] = close_prox_v[1] - robot.proximity[i].value * math.cos(robot.proximity[i].angle)
			close_prox_v[2] = close_prox_v[2] - robot.proximity[i].value * math.sin(robot.proximity[i].angle)
		end
	end
	if(close_count < 2) then
		return tangent_v, close_count
	end
	len = math.sqrt(close_prox_v[1] * close_prox_v[1] + close_prox_v[2] * close_prox_v[2])
	if(len ~= 0) then
		-- rotate repulsion by 90 degrees to get tangent drift
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
-- obstacle flow
-- entry point for grab, lock, push and release sequence
-- returns true only while a sequence is active
function HandleObstacle()
	if (true) then
		return false
	end

	if(obstacle_state == 0) then
		-- no active sequence
		return false
	end

	if(obstacle_state == 1) then
		HandleObstacleApproach()
	elseif(obstacle_state == 2) then
		HandleObstacleLock()
	elseif(obstacle_state == 3) then
		HandleObstaclePushToLight()
	else
		HandleObstacleRelease()
	end
	if(ShouldAbortObstacleSequence()) then
		ResetObstacleSequence()
		robot.wheels.set_velocity(0,0)

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
	obstacle_state = 0
	obstacle_counter = 0
	obstacle_contact = 0
	obstacle_elapsed = 0
	obstacle_black_counter = 0
	obstacle_cooldown = OBSTACLE_COOLDOWN_STEPS
end

---------------------------------------------------------------------------
-- obstacle abort gate
-- stop long or invalid grab attempts
-- keep black-zone carry phase alive only for a bounded time
function ShouldAbortObstacleSequence()
	if(obstacle_state == 0) then
		return false
	end

	if (black_ground_count < 4) then 
		return true
	elseif(BEHAVIOR_STATE == STATE_BLACK_ZONE and obstacle_elapsed >= OBSTACLE_BLACK_MAX_STEPS) then
		return true
	elseif (BEHAVIOR_STATE ~= STATE_BLACK_ZONE) then
		return true
	end

	return false
end

---------------------------------------------------------------------------
-- slow advance to obstacle
-- lock only once centered and strong enough
function HandleObstacleApproach()
	robot.turret.set_position_control_mode()
	robot.turret.set_rotation(0)
	robot.wheels.set_velocity(0.6 * WHEEL_SPEED, 0.6 * WHEEL_SPEED)
	robot.gripper.unlock()
	if(front_robot_block or front_wall_block) then
		ResetObstacleSequence()
		return
	end
	if(front_obstacle) then
		-- accumulate front pressure to judge alignment and grip quality
		obstacle_contact = obstacle_contact + front_left + front_right
	else
		obstacle_contact = 0
	end
	obstacle_counter = obstacle_counter + 1
	obstacle_elapsed = obstacle_elapsed + 1
	if((obstacle_counter >= OBSTACLE_APPROACH_STEPS and obstacle_contact >= OBSTACLE_CONTACT_THRESHOLD) or (front_close_count >= 2 and front_obstacle)) then
		-- enough pressure and alignment -> lock
		obstacle_state = 2
		obstacle_counter = 0
	end
end

---------------------------------------------------------------------------
-- stop motion
-- close gripper
-- wait a fixed number of steps before turning
function HandleObstacleLock()
	robot.wheels.set_velocity(0,0)
	robot.turret.set_position_control_mode()
	robot.turret.set_rotation(0)
	robot.gripper.lock_positive()
	obstacle_counter = obstacle_counter + 1
	obstacle_elapsed = obstacle_elapsed + 1
	if(obstacle_counter >= OBSTACLE_LOCK_STEPS) then
		-- lock completed, switch to carry turn
		obstacle_state = 3
		obstacle_counter = 0
	end
end

---------------------------------------------------------------------------
-- keep obstacle while turning toward light
-- quit on light hit or timeout
function HandleObstaclePushToLight()
	robot.turret.set_passive_mode()
	speeds = ComputePushTowardLightSpeeds()
	robot.wheels.set_velocity(speeds[1], speeds[2])
	obstacle_counter = obstacle_counter + 1
	obstacle_elapsed = obstacle_elapsed + 1
	obstacle_black_counter = obstacle_black_counter + 1
	if(GetLightStrength() >= OBSTACLE_LIGHT_THRESHOLD or 
	  obstacle_counter >= OBSTACLE_PUSH_STEPS or 
	  black_ground_count < 4) then
		-- release when target reached or push budget is spent
		obstacle_state = 4
		obstacle_counter = 0
	end
	
end

---------------------------------------------------------------------------
-- stop, unlock, reset cooldown
function HandleObstacleRelease()

	-- TODO: add spinning towards light so obstacle gets put in gray zone
	robot.turret.set_position_control_mode()
	robot.turret.set_rotation(0)
	robot.wheels.set_velocity(0, 0)
	robot.gripper.unlock()
	obstacle_state = 0
	obstacle_counter = 0
	obstacle_cooldown = OBSTACLE_COOLDOWN_STEPS
	obstacle_contact = 0
	obstacle_black_counter = 0
end

---------------------------------------------------------------------------
-- front obstacle scan
-- look at 24 proximity sensors in front arc
-- decide if obstacle is in front, centered, grab-safe, and wall-like
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
		-- balance close to 0 means obstacle is centered in front
		front_balance = math.abs(front_left - front_right) / front_sum
		if(front_balance < 0.35 and front_sum >= OBSTACLE_CONTACT_THRESHOLD) then
			front_centered = true
		end
	end
	if(front_centered and front_close_count <= 5) then
		front_grabbable = true
	end

	robot_block = IsRobotAhead()
	wall_block = IsWallAhead()
	if(robot_block or wall_block) then
		front_grabbable = false
	end
	return front_obstacle, front_left, front_right, front_centered, front_grabbable, robot_block, wall_block
end

---------------------------------------------------------------------------
-- wall gate
-- too many active front sensors means broad wall contact
-- heuristic as there is no sensor for walls specificly
function IsWallAhead()
	active_count = 0
	for i = 1, 24 do
		if(math.abs(robot.proximity[i].angle) < math.pi / 2 and robot.proximity[i].value >= WALL_SENSOR_THRESHOLD) then
			active_count = active_count + 1
		end
	end
	if((front_left + front_right) >= 1.5 and (front_left > 0.55 or front_right > 0.55)) then
		return true
	end
	return active_count >= WALL_MIN_ACTIVE_SENSORS
end

---------------------------------------------------------------------------
-- robot block gate
-- do not grab when another robot sits in front
function IsRobotAhead()
	for i = 1, #robot.range_and_bearing do
		range = robot.range_and_bearing[i].range
		bearing = robot.range_and_bearing[i].horizontal_bearing
		if(range ~= nil and bearing ~= nil and range <= FRONT_BOT_BLOCK_RANGE and math.abs(bearing) <= FRONT_BOT_BLOCK_BEARING) then
			return true
		end
	end
	return false
end

---------------------------------------------------------------------------
-- leader vector
-- pull toward robots already in tunnel or black zone
-- use bearing angle as direction, then normalize
function ProcessRABLeaders()
	leader_v = {0,0}
	for i = 1, #robot.range_and_bearing do
		if(robot.range_and_bearing[i].data[1] >= STATE_TUNNEL) then
			-- bearing gives direction to leader, unit vector keeps heading only
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
-- beacon vector
-- first green beacon seen by camera becomes heading target
function ProcessBlackZoneBeacon()
	beacon_v = {0,0}
	for i = 1, #robot.colored_blob_omnidirectional_camera do
		blob = robot.colored_blob_omnidirectional_camera[i]
		if(blob.color.green > 200 and blob.color.red < 80 and blob.color.blue < 80) then
			-- beacon angle gives exact heading to green target
			beacon_v[1] = math.cos(blob.angle)
			beacon_v[2] = math.sin(blob.angle)
			return beacon_v, 1
		end
	end
	return beacon_v, 0
end

---------------------------------------------------------------------------
-- obstacle tangent
-- rotate repulsion by 90 degrees
-- this keeps the robot sliding around obstacles instead of hitting head-on
function ComputeObstacleTangent(obstacle_v)
	tangent_v = {0,0}
	len = math.sqrt(obstacle_v[1] * obstacle_v[1] + obstacle_v[2] * obstacle_v[2])
	if(len ~= 0) then
		-- 90 deg turn : x,y -> y,-x
		tangent_v[1] = obstacle_v[2]
		tangent_v[2] = -obstacle_v[1]
	end
	return tangent_v
end

---------------------------------------------------------------------------
-- ground vector
-- active black sensors produce a pull toward the dark patch
-- sensor offset gives direction of the black region
function ProcessGround()
	ground_v = {0,0}
	black_count = 0
	for i = 1, 4 do
		if(robot.motor_ground[i].value == 0) then
			black_count = black_count + 1
			-- sensor offset points to the black patch relative to robot frame
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
-- max light strength
-- keep strongest sensor value as the local light intensity
function GetLightStrength()
	strength = robot.light[1].value
	for i = 2, 24 do 
        strength = math.max(strength, robot.light[i].value)
    end

    return strength
end

---------------------------------------------------------------------------
-- taken from exercices
-- wheel speed from angle
-- dotProduct = forward_dir · target_dir
-- if target is behind, rotate in place
-- angularVelocity = KProp * angle
-- left and right wheels get opposite angular terms
function ComputeSpeedFromAngle(angle)
    dotProduct = 0.0;
    KProp = 20;
    wheelsDistance = 0.14;

    -- behind target => no forward drive, rotate only
    if angle > math.pi/2 or angle < -math.pi/2 then
        dotProduct = 0.0;
    else
    -- forward_dir · target_dir keeps only the forward component
        forwardVector = {math.cos(0), math.sin(0)}
        targetVector = {math.cos(angle), math.sin(angle)}
        dotProduct = forwardVector[1]*targetVector[1]+forwardVector[2]*targetVector[2]
    end

	 -- linear angular error term
    angularVelocity = KProp * angle;
    -- left and right wheels use opposite angular signs to steer toward target
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

---------------------------------------------------------------------------
-- taken from exercices
-- lj swarm force
-- for each neighbor, apply lennard-jones potential
-- sum vectors to get net attraction/repulsion to nearby robots
function ProcessRAB_LJ()
	FLOCKING_CONDITION = 0
	sum_vector = {0,0}
	neighbors_in_range_counter = 0
	for i = 1,#robot.range_and_bearing do -- for each robot seen
		lj_value = ComputeLennardJones(robot.range_and_bearing[i].range)
		-- direction to neighbor * lj force gives x and y social pull
		sum_vector[1] = sum_vector[1] + math.cos(robot.range_and_bearing[i].horizontal_bearing)*lj_value
		sum_vector[2] = sum_vector[2] + math.sin(robot.range_and_bearing[i].horizontal_bearing)*lj_value
		if(robot.range_and_bearing[i].range < TARGET_DIST + ACCEPTED_DIST and robot.range_and_bearing[i].range > TARGET_DIST - ACCEPTED_DIST) then
			neighbors_in_range_counter = neighbors_in_range_counter + 1
			if(neighbors_in_range_counter >= NEIGHBORS_AT_TARG_DIST) then
				FLOCKING_CONDITION = 1
			end
		end		
	end
	if(neighbors_in_range_counter < NEIGHBORS_AT_TARG_DIST) then
		-- weak local flocking when too few neighbors are in target band
		sum_vector[1] = 0.7 * sum_vector[1]
		sum_vector[2] = 0.7 * sum_vector[2]
	end

	to_log = string.format("#neighbors=%d", neighbors_in_range_counter)
	add_log(to_log)
	
	return sum_vector


end
---------------------------------------------------------------------------
-- taken from exercices
-- lj math
-- repulsion when distance is too small
-- attraction when distance is a bit larger than target
-- formula is based on target distance and epsilon scaling
function ComputeLennardJones(distance)
	if(distance == nil) then
		return 0
	end
	-- avoid division by zero / extremely small distances
	if(distance < 0.01) then
		distance = 0.01
	end
	-- inv distance term makes force stronger as neighbors get closer
	-- the squared and fourth-power terms create a soft attraction around target distance
	return -(4*EPSILON/distance * (math.pow(TARGET_DIST/distance,4) - math.pow(TARGET_DIST/distance,2)));
end
---------------------------------------------------------------------------

---------------------------------------------------------------------------
-- global potential
-- trivial placeholder for center-distance heuristic
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


-----------------------------------------
-- small logging function to append to the file
function add_log(log)
	logf = io.open(LOG_FILE, "a")
	if (robot.id==ID and logf) then
		logf:write(log.."\n")
		logf:flush()
	end
end