-- Simple controller: random walk on white floor, then switch to a fixed run-away target when gray is seen.

RANDOM_STATE = 0
RUN_STATE = 1

STATE = RANDOM_STATE
turn_steps = 0
turn_dir = 1
walk_steps = 40
target_x = 1
target_y = 0

WALK_SPEED = 6
RUN_SPEED = 10
OBSTACLE_THRESHOLD = 0.08
AVOIDANCE_THRESHOLD = 0.15
RANDOM_TURN_PROB = 0.10
RANDOM_TURN_STEPS = 12
RANDOM_WALK_STEPS = 40
GRAY_THRESHOLD = 0.99

function init()
   STATE = RANDOM_STATE
   turn_steps = 0
   turn_dir = 1
   walk_steps = RANDOM_WALK_STEPS
   target_x = 1
   target_y = 0
   robot.gripper.unlock()
end

function reset()
   STATE = RANDOM_STATE
   turn_steps = 0
   turn_dir = 1
   walk_steps = RANDOM_WALK_STEPS
   target_x = 1
   target_y = 0
   robot.gripper.unlock()
end

function destroy()
end

function step()
   robot.gripper.unlock()

   if STATE == RANDOM_STATE then
      if SeeGray() then
         StoreEscapeTarget()
         STATE = RUN_STATE
      else
         WhiteRandomWalk()
      end
   elseif STATE == RUN_STATE then
      if SeeWhite() then
         STATE = RANDOM_STATE
         turn_steps = 0
         walk_steps = RANDOM_WALK_STEPS
      else
         RunToTarget()
      end
   end

   UpdateLEDs()
end

function SeeGray()
   for i = 1, 4 do
      if robot.motor_ground[i].value < GRAY_THRESHOLD then
         return true
      end
   end
   return false
end

function SeeWhite()
   for i = 1, 4 do
      if robot.motor_ground[i].value < GRAY_THRESHOLD then
         return false
      end
   end
   return true
end

function WhiteRandomWalk()
   local avoid_x, avoid_y = ComputeAvoidanceVector()
   local avoid_length = math.sqrt(avoid_x * avoid_x + avoid_y * avoid_y)

   if avoid_length > AVOIDANCE_THRESHOLD then
      local avoid_angle = math.atan2(avoid_y, avoid_x)
      local left = WALK_SPEED * math.cos(avoid_angle) - WALK_SPEED * math.sin(avoid_angle)
      local right = WALK_SPEED * math.cos(avoid_angle) + WALK_SPEED * math.sin(avoid_angle)
      robot.wheels.set_velocity(left, right)
      return
   end

   if turn_steps > 0 then
      turn_steps = turn_steps - 1
      robot.wheels.set_velocity(turn_dir * WALK_SPEED, -turn_dir * WALK_SPEED)
      return
   end

   if walk_steps > 0 then
      walk_steps = walk_steps - 1
      robot.wheels.set_velocity(WALK_SPEED, WALK_SPEED)
   else
      if robot.random.uniform() < RANDOM_TURN_PROB then
         turn_steps = RANDOM_TURN_STEPS
         if robot.random.bernoulli() == 1 then
            turn_dir = 1
         else
            turn_dir = -1
         end
         walk_steps = RANDOM_WALK_STEPS
         robot.wheels.set_velocity(turn_dir * WALK_SPEED, -turn_dir * WALK_SPEED)
      else
         walk_steps = RANDOM_WALK_STEPS
         robot.wheels.set_velocity(WALK_SPEED, WALK_SPEED)
      end
   end
end

function ComputeAvoidanceVector()
   local avoid_x = 0
   local avoid_y = 0

   for i = 1, #robot.proximity do
      avoid_x = avoid_x - robot.proximity[i].value * math.cos(robot.proximity[i].angle)
      avoid_y = avoid_y - robot.proximity[i].value * math.sin(robot.proximity[i].angle)
   end

   return avoid_x, avoid_y
end

function UpdateLEDs()
   if STATE == RANDOM_STATE then
      robot.leds.set_all_colors("red")
   else
      robot.leds.set_all_colors("green")
   end
end

function StoreEscapeTarget()
   local push_x = 0
   local push_y = 0

   for i = 1, #robot.proximity do
      push_x = push_x - robot.proximity[i].value * math.cos(robot.proximity[i].angle)
      push_y = push_y - robot.proximity[i].value * math.sin(robot.proximity[i].angle)
   end

   local length = math.sqrt(push_x * push_x + push_y * push_y)
   if length < 0.0001 then
      target_x = 1
      target_y = 0
   else
      target_x = push_x / length
      target_y = push_y / length
   end
end

function RunToTarget()
   local angle = math.atan2(target_y, target_x)
   local left = RUN_SPEED * math.cos(angle) - RUN_SPEED * math.sin(angle)
   local right = RUN_SPEED * math.cos(angle) + RUN_SPEED * math.sin(angle)
   robot.wheels.set_velocity(left, right)
end