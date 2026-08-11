--[[ PATTERN FORMATION

     o -- o
    / \  / \
  o -- o -- o
   \ /  \ /
    o -- o

The goal of this exercise is to let the robot move in order to form an hexagonal pattern.
In order to do this the robots try to position themselves in order to minimize a potential
field computed using the Lennard-Jones potential.
Simplifying a lot, the Lennard-Jones potential is a model for the interaction between atoms:
- if two atoms are too close, they will be subject to a repulsion force, pushing them away from one another;
- if they are too far away, they will be subject to an attraction force, pushing them close to each other;
- if they are at the right distance, they will be subject to no force, leaving them there;
This force can be used to let robots move to a position in which the distance between all robots is equal.

The trick is that a robot computes the virtual force "created" by that the other robots seen.

]]
---------------------------------------------------------------------------
-- global variables
SIGMA = 80 -- the target distance between robots, in cm
EPSILON = 50 -- a coefficient to increase the force of the repulsion/attraction function
WHEEL_SPEED = 5 -- max wheel speed
---------------------------------------------------------------------------

---------------------------------------------------------------------------
--Step function
function step()

end

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

    -- the function returns an array where speeds[1] contains the velocity for the left wheel, and speeds[2] contains the velocity for the right wheel
    return speeds
end

function init()
end

function reset()
end

function destroy()
end
