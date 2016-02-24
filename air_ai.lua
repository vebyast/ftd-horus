--[[
   Air AI run by context-free "motion grammar" via pushdown automaton

   Documentation in file.
]]--


function MakeConfigs()
   return {
	  turn = {
		 err_angle = 10,		-- degrees
	  },
   }
end


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
-- misc library functions
-- #############################################################################
-- #############################################################################

-- copied off https://gist.github.com/balaam/3122129
-- does an in-place array reverse.
function reverse(tbl)
   for i=1, math.floor(#tbl / 2) do
	  tbl[i], tbl[#tbl - i + 1] = tbl[#tbl - i + 1], tbl[i]
   end
end

function joinlit(args)
   out = ""
   for idx,val in ipairs(args) do
	  out = out .. val .. "	"
   end
   return out
end

-- #############################################################################
-- #############################################################################
-- the grammar
-- #############################################################################
-- #############################################################################

-- the planner for this AI is basically the Motion Grammar of Dantam et al
-- (http://www.golems.org/papers/dantam2013motion.pdf), with a slight
-- modification for library availability. We don't really have the
-- infrastructure in place to conveniently parse CFGs, even for LL-1 grammars,
-- nor do we have a library suitable for the tokenization step in section V-B of
-- Dantam where we separate the state space of the controllable system into
-- disjoint regions which cover the configuration space. Instead, we make
-- nonterminal symbols functions from the state of the craft to a production for
-- that symbol. Which is *close*, but we lose a lot of the guarantees about
-- correctness because we don't have the disjoint cover into percept literals
-- and the resulting provably safe policy.

-- A symbol's production can return a control and a list of symbol-goal pairs. A
-- control is a collection of setpoints for the PIDs in the high-level
-- controller to attempt to achieve during this tick. The list of symbol-goal
-- pairs is pushed onto the top of the stack. We augment each symbol with goals
-- to allow longer-term planning. There's a subtlety here due to controls only
-- lasting for a single tick; symbols corresponding to sustained maneuvers will
-- have multiple productions they can output, some of which (those that
-- correspond to the manuever being sustained) ending in a repetition of the
-- symbol itself with its own goals. A number of nonterminals include something
-- like "hold your current heading" or "turn to relative +60 degrees heading",
-- which are handled by having this re-stacking of the symbol include some
-- information computed and passed along by the first parse that stores the
-- initial heading.

-- Additionally, some maneuvers need to handle potential failure - for example,
-- if a gun run can't line up before it reaches minimum safe distance or becomes
-- too predictable - and a stack machine can't just start randomly popping
-- symbols off its stack, as we'd have to do if we wanted to "unwind" the stack
-- to handle an error. This is restrictive, but the way to deal with this is
-- basically to design your nonterminals so they don't stack up symbols past
-- things that can fail. If you need a maneuver to be able to fail, then don't
-- assume it'll succeed and issue symbols after/below it; instead, split it up
-- and give it goals that let it make the proper decision if it
-- succeds. Maintain don't-repeat-yourself by factoring symbols properly:
-- instead of writing "fail-safe immelman that completes into a gun run" and
-- "fail-safe immelman that completes into a missile evade" duplicates, write
-- "gun run following from possibly-failed immelman" and "missile evade
-- following from possibly-failed immelman", and handle the immelman failing in
-- the following nonterminal.

-- note, though, that there's no _actual_ distinction between terminals and
-- nonterminals, and you can very well have a production that returns a controls
-- block and both will be happily executed. That may be useful for some very
-- simple sustained near-terminals; for example, a half-second max-power turn
-- for gunfire evasion might issue both a control and its own symbol until it's
-- done.

-- in classic CFG tradition, the stack starts with a single, special, symbol, S,
-- which is the root production for the grammar. You can change the productions
-- on S to get *extremely* configurable behavior for your aircraft.

-- note to self: left-handed coordinate system, {right, up, forward}

-- convention: for convenience, on all returned arrays of symbols we stack
-- starting from the end of the array, allowing us to write our productions from
-- top to bottom in the order they'll be popped off the stack.

-- convention: heading, elevation, upangle, north, east, and altitude indicate
-- *global* control dimensions, while yaw, pitch, roll, right, forward, and up
-- indicate *local* control dimensions. Dimensions prefixed with "d" are
-- velocities.

-- CONTROLS
-- 
--   Controls passed to the controller are an array of { dim = dimension,
--   setpoint = value, effort = value } tuples. The controller is a two-stage
--   process. The first stage receives the control setpoints and converts them
--   to normalized local velocities before mixing them together into a single,
--   _fully-specified_ normalized velocity covering all six of the degrees of
--   freedom for a rigid body in R^3. The second stage takes that and turns it
--   into actuation.
-- 
--   Specifying a relative command has an issue with the controller having to
--   figure out relative to _what_, so if you want to do relative movements,
--   compute a target in the nonterminal and hold on to it along as a
--   goal. Specifying an absolute velocity has an issue with having to know how
--   fast the craft can go in any given direction, and has some issues with
--   drift, so if you want a velocity, handle it by sending global position
--   targets that are computed an accumulator in the goals.
-- 
--   Effort indicates how strongly the controller should prioritize competing
--   objectives. Note that almost every command pair will have competition, if
--   only between linear and rotational goals and between global goals and
--   components of local goals. effort is given out proportionally, that is, if
--   you have one target with effort 1 and one control with effort 2, the
--   effort-2 one will get roughly 2/3 of the controller's energy, while the
--   effort-1 one will get 1/3. Specifying a dimension multiple times results in
--   their targets being mixed together according to their efforts.
--
--   The global linear setpoint commands (north, east, alt) are decomposed into
--   local targets. Magnitudes are computed obviously, while effort is
--   distributed along with magnitude to avoid a single "alt" on a perfectly
--   upright craft halving the effect of a "forward". These local targets are
--   then fed to a PID to produce normalized (interval [0, 1]) local
--   velocities. These local velocities are then mixed with the local linear
--   velocity commands that were specified (forward, right, up) according to all
--   the efforts, normalized into [0, 1] again, and sent on.
--
--   C is a convenience function that makes it less boilerplaty to build control
--   command tuples. its signature is (dim, setpoint, effort); if effort is
--   unspecified it defaults to 1. this lets us build examples like:
-- 
--   { C("pitch", 0, .5),
--     C("altitude", 200),
--     C("roll", 0),
--     C("heading", init_heading),
--     C("forward", 1), }
--
--   orders the controller to maintain straight, level forward flight at 200
--   meters. We specify forward to keep the plane going forward; without this
--   it'd just hover there. The heading is stored by the CFG on the first
--   instance of that symbol and passed along through the goals table so that we
--   don't drift.
-- 
--   { C("north", init_north),
--     C("east", init_east),
--     C("pitch", 90),
--     C("forward", 1), }
--
--   Orders the controller to ascend at maximum rate without side to side
--   movement. Again, instead of trying to implement side-to-side commands, we
--   store an initial position, pass it through the goals table, and specify a
--   global absolute north and east target.
--

-- Note: The controller doesn't know how to orient the aircraft to achieve
-- maximum thrust. Remember to point your aircraft in the direction you want to
-- go! Or don't.

-- Note: The controller doesn't know if any of your degrees of freedom are
-- unactuated!

-- Note: most terminals assume that, if controller has a stable region that's
-- smaller than the entire configuration space (that is, if there are vehicle
-- configurations with setpoints that the controller will fail to achieve and
-- end up spinning infinitely), productions will get the vehicle into a safe
-- region before issuing this symbol.


function MakeInitMainStack()
   s = Stack.new()
   Stack.push(s, { symbol = "S", goals = { } })
   return s
end


-- two reasons to put this in a function. first, so syntax errors don't produce
-- silent crashes on start. second, so we can put C somewhere without it mucking
-- up the global scope.
function MakeSymbols ()
   function C(dim, setpoint, effort)
	  return {
		 dim = dim,
		 setpoint = setpoint,
		 effort = effort or 1,
	  }
   end
   
   return {
	  S = function(I, goals, config)
		 return {
			controls = nil,		-- this is a nonterminal
			production = {
			   -- we produce a levelflight nonterminal
			   { symbol = "levelflight", goals = { altitude = 150, } },
			   -- and then we recurse on S to loop infinitely
			   { symbol = "S", goals = { } },
			},
		 }
	  end,
	  -- turn = function (I, goals, config)
	  -- 	 -- turn takes a vector and turns so that that vector is the forward
	  -- 	 -- vector. config.error specifies the angular error before we count
	  -- 	 -- this as having been achieved. goals.upvector, if present, specifies
	  -- 	 -- a roll value to attempt to fix the last rotational degree of
	  -- 	 -- freedom.
	  -- 	 local dir = goals.dir
	  -- 	 return {
	  -- 		production = nil, 	-- terminal
	  -- 		controls = {
			   
	  -- 		},
	  -- 	 }
	  -- end,
	  levelflight = function(I, goals, config)
		 -- flies straight and level at a given speed.
		 --
		 -- goals:
		 --   altitude: global altitude setpoint. optional.
		 --   heading: global heading setpoint. optional.
		 --   forward: local normalized forward velocity. if unspecified
		 --     defaults to 1.
		 return {
			production = nil,		-- this is a terminal
			controls = {
			   goals.altitude and C("alt", goals.altitude) or nil,
			   goals.heading and C("heading", goals.yaw) or nil,
			   C("roll", 0),	-- stay level
			   C("forward", goals.forward or 1),	-- thrust local forward
			},
		 }
	  end,
   }
end

function ControlHigh(I, hightarget)
   -- note to self: use pairs instead of ipairs because some of our termianls
   -- handle optional goals in a way that introduces nils into the returned
   -- controls array, which breaks lua's ipairs functionality. so, yeah, use
   -- pairs.
   for k,v in pairs(hightarget) do I:Log(joinlit({"hightarget: ", k, v.dim, v.setpoint, v.effort})) end
   return {
	  droll = 0,
	  dpitch = 0,
	  dyaw = 0,
	  dforward = 0,
	  dright = 0,
	  dup = 0,
   }
end

function ControlLow(I, lowtarget)
   for k,v in pairs(lowtarget) do I:Log(joinlit({"lowtarget: ", k, v})) end
end

inited = false
mainstack = nil
function Update(I)
   I:ClearLogs()
   I:Log("foo")
   if not inited then
	  configs = MakeConfigs()
	  symbols = MakeSymbols()
	  mainstack = MakeInitMainStack()
	  inited = true
   end
   
   local hightarget
   repeat
	  local stacktop = Stack.pop(mainstack)
	  local output = symbols[stacktop.symbol](I, stacktop.goals, configs[stacktop.symbol])
	  local disp = ""
	  if output.production then
		 reverse(output.production)
		 for _,symbolpair in pairs(output.production) do
			Stack.push(mainstack, symbolpair) -- And that's it. Formal langauge theory, folks!! Simplest code ever.
			disp = symbolpair.symbol .. ", " .. disp
		 end
	  end
	  I:Log(joinlit({"symbol: ", stacktop.symbol, " -> ", disp}))
	  hightarget = output.controls
   until output.controls

   local lowtarget = ControlHigh(I, hightarget)
   ControlLow(I, lowtarget)
end
