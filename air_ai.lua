--[[
   Air AI with configurable state machines

   Documentation at github repository:
]]--



-- #############################################################################
-- #############################################################################
-- quick stack library, adapted from http://www.lua.org/pil/11.4.html
-- #############################################################################
-- #############################################################################

Stack = {}
function Stack.new ()
   return {top = -1}
end
function Stack.push (stack, value)
   local top = stack.top + 1
   stack.top = top
   stack[top] = value
end
function Stack.stack (bottom, top)
   if bottom == top then
	  return
   end
   for tidx = 0, top.top do
	  Stack.push(bottom, top[tidx])
   end
end
function Stack.pop (stack)
   if Stack.empty(stack) then return nil end
   local top = stack.top
   local value = stack[top]
   stack.top = top - 1
   stack[top] = nil
   return value
end
function Stack.peek (stack)
   if Stack.empty(stack) then return nil end
   return stack[stack.top]
end
function Stack.empty (stack)
   return stack.top < 0
end
function Stack.len (stack)
   return stack.top + 1
end
function Stack.peekiter(stack)
   local i = stack.top + 1
   return function()
	  i = i - 1
	  if i >= 0 then return stack[i] end
   end
end

-- #############################################################################
-- #############################################################################
-- PID library
-- #############################################################################
-- #############################################################################


-- #############################################################################
-- #############################################################################
-- the grammar
-- #############################################################################
-- #############################################################################


-- the planner for this AI is basically the Motion Grammar of Dantam et al
-- (http://www.golems.org/papers/dantam2013motion.pdf), with a slight
-- modification for available libraries. Since we don't have an efficient CFG
-- parser, even though our grammar is probably LL-1 we can't use the trick where
-- we make sensory percepts terminals in our grammar and then predicates on them
-- are part of productions and allow them to drive the parse tree (LL-1 means
-- that the next symbol unambiguously determines the next production, so if
-- productions include predicates on percepts and the grammar is LL-1 then at
-- each percept we can follow from it to the next production). Instead, our
-- nonterminal symbols are associated with functions that take the state of the
-- craft and return a member of the set of productions for that symbol. Which is
-- *almost* identical, but we lose a lot of the guarantees about correctness
-- because our percepts are not directly part of the grammar.

-- each symbol is either a literal or a production. a literal returns a
-- collection of setpoints for the PIDs in the controller to attempt to
-- achieve. a production returns set of symbols to be pushed onto the top of the
-- stack. each symbol is augmented with some "goal" information that gives it
-- more specifics for its behavior. for example, we might give a "levelflight"
-- terminal goals for altitude and yaw angle and it'd issue orders to the
-- controller layer to bring the craft to roll = 0 and hold that heading. We
-- might give an "attackrun" production a goal for the location of the target
-- and it'd realize it's pointing in exactlyt he wrong direction, then create
-- and push onto the stack symbols for "immelman", followed by pushing a copy of
-- itself it'd be going in the right direction, so it'd push productions for
-- "evasive movement", "aim weapons", "hard turn", "evasive movement".

-- in classic CFG tradition, the stack starts with a single, special, symbol, S,
-- which is the root production for the grammar. You can change S to get
-- *extremely* configurable behavior for your aircraft. I provide two defaults,
-- one that imitates LoSboccacc's cinematic gun-aiming AI and one that sort of
-- imitates Madwand's air ai (which imitates the built-in air ai, sort of).

symbols = {
   S = function(state, goals)
	  return {
		 production = {
			{ symbol = "levelflight", goals = { altitude = 150 } },
			{ symbol = "missileevade", goals = { } },
			{ symbol = "S", goals = { } },
		 },
		 controls = nil,
	  }
   end,
   levelflight = function(state, goals)
	  -- levelflight is a terminal; it only provides a control output
	  local production = { }
	  -- the goals for levelflight are { altitude, yaw, speed }. this is
	  -- entirely a controls task, so we pass these straight on to the
	  -- controller.
	  local controls = {
		 alt = goals.altitude or state.GetConstructPosition.x,
		 yaw = goals.yaw or state.GetConstructYaw,
		 fwd = goals.speed or 1,
	  }
	  return { production = production, controls = controls }
   end,
   missileevade = function(state, goals)
	  -- production: if no missiles around, produces { }, if missiles around,
	  -- produces symbols for evading the missile.
	  return nil
   end,
}

-- the controller is a two-stage process that takes targets (setpoints) for the
-- craft's orientation and position in space and issues actuator commands to
-- control the craft to achieve the setpoints. the first stage takes the targets
-- and outputs a three-dimensional rigid-body velocity. the second stage takes a
-- three-dimension rigid-body velocity and sends commands to actuators
-- (propulsion, spinblocks) that will accelerate the craft to achieve those
-- velocities. there are also a few oddball actuations, things like "aim
-- weapons" and "set balloons", that are handles specially.

function Update(I)
   
end
