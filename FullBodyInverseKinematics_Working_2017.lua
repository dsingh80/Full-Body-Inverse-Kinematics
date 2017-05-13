
--  CONSTANTS  --
local FastWait = game:GetService("RunService").RenderStepped

local Character = script.Parent
local HumanoidRootPart = Character.HumanoidRootPart
local Torso = Character.Torso
local NewTorso = Character.newTorso
local URA, ULA = Character.URA, Character.ULA
local LRA, LLA = Character.LRA, Character.LLA
local URL, ULL = Character.URL, Character.ULL
local LRL, LLL = Character.LRL, Character.LLL

local Hip = HumanoidRootPart.RootJoint
local Neck = Torso.Neck
local RS = NewTorso["Right Shoulder"]
local LS = NewTorso["Left Shoulder"]
local RE = URA["Right Elbow"]
local LE = ULA["Left Elbow"]
local RH = NewTorso["Right Hip"]
local LH = NewTorso["Left Hip"]
local RK = URL["Right Knee"]
local LK = ULL["Left Knee"]

local Targets = {
	RA = game.Workspace.RA_Target,
	LA = game.Workspace.LA_Target,
	RL = game.Workspace.RL_Target,
	LL = game.Workspace.LL_Target,
	Hip = game.Workspace.HipController,
	Neck = game.Workspace.NeckController
}





Hip.C0 = CFrame.new(0,-NewTorso.Size.Y/2, 0)
Hip.C1 = CFrame.new(0,-NewTorso.Size.Y/2, 0)

local defHip, defN, defRS, defLS, defRE, defLE, defRH, defLH, defRK, defLK = Hip.C0, Neck.C0, RS.C1, LS.C1, RE.C1, LE.C1, RH.C1, LH.C1, RK.C1, LK.C1


--  Useful Functions  --
function resetJoints(pow, style, speed)
	local animationMode = _G.AnimationMode
	for i=1, (speed or 10) do
		if _G.AnimationMode ~= animationMode then break end
		RS.C1 = RS.C1:lerp(defRS, style and style(pow, i/(speed or 10)) or i/(speed or 10))
		LS.C1 = LS.C1:lerp(defLS, style and style(pow, i/(speed or 10)) or i/(speed or 10))

		RH.C1 = RH.C1:lerp(defRH, style and style(pow, i/(speed or 10)) or i/(speed or 10))
		LH.C1 = LH.C1:lerp(defLH, style and style(pow, i/(speed or 10)) or i/(speed or 10))

		RE.C1 = RE.C1:lerp(defRE, style and style(pow, i/(speed or 10)) or i/(speed or 10))
		LE.C1 = LE.C1:lerp(defLE, style and style(pow, i/(speed or 10)) or i/(speed or 10))

		RK.C1 = RK.C1:lerp(defRK, style and style(pow, i/(speed or 10)) or i/(speed or 10))
		LK.C1 = LK.C1:lerp(defLK, style and style(pow, i/(speed or 10)) or i/(speed or 10))

		Neck.C0 = Neck.C0:lerp(defN, style and style(pow, i/(speed or 10)) or i/(speed or 10))
		Hip.C0 = Hip.C0:lerp(defHip, style and style(pow, i/(speed or 10)) or i/(speed or 10))
		FastWait:wait()
	end
end

function getGround(relativeToTorso)
	--[[   RETURNS A VECTOR3 OF POSITION OF GROUND   ]]--
	local downVector;
	--Check if the Torso rotation should be factored in(maybe cuz weird gravity? idk)
	if relativeToTorso == true then
		downVector = ((Torso.CFrame*CFrame.new(0,-3,0)).p - Torso.Position) * 100
	else
		downVector = Vector3.new(0,-100,0)
	end

	local _, groundPos = workspace:FindPartOnRayWithIgnoreList(
		Ray.new(Torso.Position, downVector),
		Character:GetChildren()
	)
	
	return groundPos
end


--------------============----------------===============--------------==========----------
--------------============------------------START SCRIPT---------------==========----------
--------------============----------------===============--------------==========----------

local URALen,LRALen = 1.5, 1.5                  -- length of each segment of arm
local ULALen,LLALen = 1.5, 1.5                  -- length of each segment of arm

-- The "backwards" parameter exists from an older method
-- I was testing FABRIK against this algorithm
--[[ RIGHT ARM ]]--
function getRSCF(backwards)
	if backwards == true then
		--base it off of LRA
		return LRA.CFrame * CFrame.new(0, 0, LRA.Size.Z/2)
	else
		--base it off of torso
		return Torso.CFrame * CFrame.new(Torso.Size.X/2 + URA.Size.X/2, Torso.Size.Y/2-URALen/4, 0)
	end
end

function getRECF(backwards)
	if backwards == true then
		--base it off of itself
		return LRA.CFrame*CFrame.new(0,0,-LRA.Size.Z/2)
	else
		--base it off of URA
		return URA.CFrame*CFrame.new(0,0, -URA.Size.Z/2)
	end
end

--[[ LEFT ARM ]]--
function getLSCF(backwards)
	if backwards == true then
		--base it off of LRA
		return LLA.CFrame * CFrame.new(0, 0, LLA.Size.Z/2)
	else
		--base it off of torso
		return Torso.CFrame * CFrame.new(-Torso.Size.X/2 - ULA.Size.X/2, Torso.Size.Y/2-URALen/4, 0)
	end
end

function getLECF(backwards)
	if backwards == true then
		--base it off of itself
		return LLA.CFrame*CFrame.new(0,0,-LLA.Size.Z/2)
	else
		--base it off of URA
		return ULA.CFrame*CFrame.new(0,0, -ULA.Size.Z/2)
	end
end

--[[ RIGHT LEG ]]--
function getRHCF()
	return Torso.CFrame * CFrame.new(Torso.Size.X/2 - URL.Size.X/2, -Torso.Size.Y/2, 0)
end

function getRKCF()
	return URL.CFrame*CFrame.new(0, -URL.Size.Y/2, 0)
end

function getRLEND()
	return LRL.CFrame * CFrame.new(0,-LRL.Size.Y/2,0)
end

--[[ LEFT LEG ]]--
function getLHCF()
	return Torso.CFrame * CFrame.new(-Torso.Size.X/2 + ULL.Size.X/2, -Torso.Size.Y/2, 0)
end

function getLKCF()
	return ULL.CFrame*CFrame.new(0, -ULL.Size.Y/2, 0)
end

function getLLEND()
	return LLL.CFrame * CFrame.new(0,-LLL.Size.Y/2,0)
end

local Constraints = {
--[[    
	JOINT = {
		AXIS = {POSITIVE_CONSTRAINT,   NEGATIVECONSTRAINT},
	}
]]--
	RH = {
		X = {math.pi/4, -(5/6)*math.pi},
		Y = {math.pi/4, -math.pi/2},
		Z = {math.pi/3, -math.pi/4}
	},
	LH = {
		X = {math.pi/4, -(5/6)*math.pi},
		Y = {math.pi/2, -math.pi/4},
		Z = {math.pi/4, -math.pi/3}
	},
	RS = {
		X = {math.pi/2, -(5/6)*math.pi},
		Y = {math.pi/2, -math.pi/3},
		Z = {math.pi/4, -math.pi/3}
	},
	LS = {
		X = {math.pi/4, -(5/6)*math.pi},
		Y = {math.pi/3, -math.pi/2},
		Z = {math.pi/3, -math.pi/4}
	},
	N = {
		X = {1.25*math.pi/3, -3.25*math.pi/4},
		Y = {math.pi/3, -math.pi/3},
		Z = {1.25*math.pi/3, -1.25*math.pi/3},
	}
}

	
function updateRL(END_EFFECTOR)
	if END_EFFECTOR == nil then return end
	
	local RETURN_VALUE = false
	
	local TARGET_VECTOR = getRHCF():inverse() * END_EFFECTOR
	local TARGET_DIST = TARGET_VECTOR.magnitude

	local RH_X, RH_Y, RH_Z = 0;
	local RK_X, RK_Y, RK_Z = 0;
	
	RH_Z = math.atan2(TARGET_VECTOR.Y, -TARGET_VECTOR.X)
	RH_Y = math.atan2(-TARGET_VECTOR.Z, TARGET_VECTOR.X)
		
	-- Offsetting the axis to have proper angle values
	if RH_Z > 0 then
		RH_Z = RH_Z - math.pi/2
	else 
		RH_Z = RH_Z + math.pi/2
	end
	
	if RH_Y < 0 then
		RH_Y = RH_Y + math.pi/2
	else
		RH_Y = RH_Y - math.pi/2
	end
	

	-- Apply Constraints
	if RH_Z > Constraints.RH.Z[1] then
		RH_Z = Constraints.RH.Z[1]
	elseif RH_Z < Constraints.RH.Z[2] then
		RH_Z = Constraints.RH.Z[2]
	end
	
	if RH_Y > Constraints.RH.Y[1] then
		RH_Y = Constraints.RH.Y[1]
	elseif RH_Y < Constraints.RH.Y[2] then
		RH_Y = Constraints.RH.Y[2]
	end
	
	local YZ_RATIO; --Used to make the leg move more naturally
	if RH_Z < 0 then
		YZ_RATIO = math.abs((Constraints.RH.Z[2]-RH_Z)/Constraints.RH.Z[2])
		RH_Z = RH_Z * YZ_RATIO
		RH_Y = RH_Y * (1-YZ_RATIO)
	else
		YZ_RATIO = math.abs((Constraints.RH.Z[1]-RH_Z)/Constraints.RH.Z[1])
		RH_Z = RH_Z * YZ_RATIO
		RH_Y = RH_Y * (1-YZ_RATIO)
	end
	
	if TARGET_DIST <= URLLen+ LRLLen then
		-- TARGET WITHIN REACH --
		local theta = math.atan2(TARGET_VECTOR.Z, -TARGET_VECTOR.Y) --math.atan2(TARGET_VECTOR.Y, -TARGET_VECTOR.Z) + math.pi/2
		local theta1 = LRLLen^2 - URLLen^2 - TARGET_DIST^2
		theta1 = theta1 / (-2 * URLLen * TARGET_DIST)
		theta1 = math.acos(theta1)
		RH_X = theta - theta1
		
		local theta2 = TARGET_DIST^2 - URLLen^2 - LRLLen^2
		theta2 = theta2 / (-2 * URLLen * LRLLen)
		theta2 = math.acos(theta2)
		RK_X = math.pi - theta2 -- theta2 - math.pi
		
		--print("RH_X: ", math.deg(RH_X))
		
		RETURN_VALUE = true
	else
		-- TARGET OUT OF REACH --
		--print("Target out of reach")
		RH_X = math.atan2(TARGET_VECTOR.Z, -TARGET_VECTOR.Y) --math.atan2(TARGET_VECTOR.Y, -TARGET_VECTOR.Z) + math.pi/2
		RK_X = 0

		RETURN_VALUE = false
	end
	
	if RH_X > Constraints.RH.X[1] then
		RH_X = 0
		RK_X = 0
		RH_Y = 0
		RH_Z = 0
	elseif RH_X < Constraints.RH.X[2] then
		RH_X = 0
		RK_X = 0
		RH_Y = 0
		RH_Z = 0
	end
		
	
	RH.C1 = defRH * CFrame.fromEulerAnglesXYZ(RH_X,-RH_Y, RH_Z)
	RK.C1 = defRK * CFrame.fromEulerAnglesXYZ(RK_X, 0, 0)
	
	return RETURN_VALUE
end

function updateLL(END_EFFECTOR)
	if END_EFFECTOR == nil then return end
	
	local TARGET_VECTOR = getLHCF():inverse() * END_EFFECTOR
	local TARGET_DIST = TARGET_VECTOR.magnitude

	local LH_X, LH_Y, LH_Z = 0;
	local LK_X, LK_Y, LK_Z = 0;
	
	LH_Z = math.atan2(TARGET_VECTOR.Y, -TARGET_VECTOR.X)
	LH_Y = math.atan2(-TARGET_VECTOR.Z, TARGET_VECTOR.X)
	-- Offsetting the axis to have proper angle values
	if LH_Z > 0 then
		LH_Z = LH_Z - math.pi/2
	else 
		LH_Z = LH_Z + math.pi/2
	end
	
	if LH_Y < 0 then
		LH_Y = LH_Y + math.pi/2
	else
		LH_Y = LH_Y - math.pi/2
	end
	
	-- Apply Constraints
	if LH_Z > Constraints.LH.Z[1] then
		LH_Z = Constraints.LH.Z[1]
	elseif LH_Z < Constraints.LH.Z[2] then
		LH_Z = Constraints.LH.Z[2]
	end
	
	if LH_Y > Constraints.LH.Y[1] then
		LH_Y = Constraints.LH.Y[1]
	elseif LH_Y < Constraints.LH.Y[2] then
		LH_Y = Constraints.LH.Y[2]
	end
	
	
	
	local YZ_RATIO; --Used to make the leg move more naturally
	if LH_Z < 0 then
		YZ_RATIO = math.abs((Constraints.LH.Z[2]-LH_Z)/Constraints.LH.Z[2])
		LH_Z = LH_Z * YZ_RATIO
		LH_Y = LH_Y * (1-YZ_RATIO)
	else
		YZ_RATIO = math.abs((Constraints.LH.Z[1]-LH_Z)/Constraints.LH.Z[1])
		LH_Z = LH_Z * YZ_RATIO
		LH_Y = LH_Y * (1-YZ_RATIO)
	end

	if TARGET_DIST <= URLLen+ LRLLen then
		-- TARGET WITHIN REACH --
		local theta = math.atan2(TARGET_VECTOR.Y, TARGET_VECTOR.Z) + math.pi/2
		local theta1 = LRLLen^2 - URLLen^2 - TARGET_DIST^2
		theta1 = theta1 / (-2 * URLLen * TARGET_DIST)
		theta1 = math.acos(theta1)
		LH_X = theta - theta1
		
		local theta2 = TARGET_DIST^2 - URLLen^2 - LRLLen^2
		theta2 = theta2 / (-2 * URLLen * LRLLen)
		theta2 = math.acos(theta2)
		LK_X = math.pi - theta2
	else
		-- TARGET OUT OF REACH --
		--print("Target out of reach")
		LH_X = math.atan2(TARGET_VECTOR.Y, TARGET_VECTOR.Z) + math.pi/2
		LK_X = 0
	end
	
	if LH_X > Constraints.LH.X[1] then
		LH_X = 0
		LK_X = 0
		LH_Y = 0
		LH_Z = 0
	elseif LH_X < Constraints.LH.X[2] then
		LH_X = 0
		LK_X = 0
		LH_Y = 0
		LH_Z = 0
	end
	
	LH.C1 = defLH * CFrame.fromEulerAnglesXYZ(LH_X - HIP_LEAN,-LH_Y, LH_Z)
	LK.C1 = defLK * CFrame.fromEulerAnglesXYZ(LK_X, 0, 0)
end


function updateRA(END_EFFECTOR)
	if END_EFFECTOR == nil then return end
	
	local RETURN_VALUE = false
	
	-- Get the vector between target and shoulder
	local TARGET_VECTOR = getRSCF():inverse() * END_EFFECTOR
	local TARGET_DIST = TARGET_VECTOR.magnitude

	local RS_X, RS_Y, RS_Z = 0;
	local RE_X, RE_Y, RE_Z = 0;
	
	RS_Z = math.atan2(TARGET_VECTOR.X, -TARGET_VECTOR.Y)
	RS_Y = math.atan2(TARGET_VECTOR.X, -TARGET_VECTOR.Z)

	-- Offsetting the axis to have proper angle values
	if RS_Z > math.pi/2 then
		RS_Z = RS_Z - math.pi/2
	else 
		RS_Z = RS_Z + math.pi/2
	end

	-- Apply Constraints
	if RS_Z > Constraints.RS.Z[1] then
		RS_Z = Constraints.RS.Z[1]
	elseif RS_Z < Constraints.RS.Z[2] then
		RS_Z = Constraints.RS.Z[2]
	end
	
	if RS_Y > Constraints.RS.Y[1] then
		RS_Y = Constraints.RS.Y[1]
	elseif RS_Y < Constraints.RS.Y[2] then
		RS_Y = Constraints.RS.Y[2]
	end
	
	local YZ_RATIO; --Used to move more naturally
	if RS_Z < 0 then
		YZ_RATIO = math.abs((Constraints.RS.Z[2]-RS_Z)/Constraints.RS.Z[2])
		RS_Z = RS_Z * YZ_RATIO
		RS_Y = RS_Y * (1-YZ_RATIO)
	else
		YZ_RATIO = math.abs((Constraints.RS.Z[1]-RS_Z)/Constraints.RS.Z[1])
		RS_Z = RS_Z * YZ_RATIO
		RS_Y = RS_Y * (1-YZ_RATIO)
	end
	
	if TARGET_DIST <= URALen+ LRALen then
		-- TARGET WITHIN REACH --
		local theta = math.atan2(TARGET_VECTOR.Z, -TARGET_VECTOR.Y)--math.atan2(TARGET_VECTOR.Y, TARGET_VECTOR.Z) + math.pi/2
		local theta1 = LRALen^2 - URALen^2 - TARGET_DIST^2
		theta1 = theta1 / (-2 * URALen * TARGET_DIST)
		theta1 = math.acos(theta1)
		RS_X = theta + theta1
		
		local theta2 = TARGET_DIST^2 - URALen^2 - LRALen^2
		theta2 = theta2 / (-2 * URALen * LRALen)
		theta2 = math.acos(theta2)
		RE_X = theta2 - math.pi --math.pi - theta2
		
		--print("RS_X: ", math.deg(theta))
		RETURN_VALUE = true
	else
		-- TARGET OUT OF REACH --
		--print("Target out of reach")
		RS_X = math.atan2(TARGET_VECTOR.Z, -TARGET_VECTOR.Y)--math.atan2(TARGET_VECTOR.Y, -TARGET_VECTOR.Z) + math.pi/2
		RE_X = 0

		RETURN_VALUE = false
	end
	
	-- Apply constraints
	if RS_X > Constraints.RS.X[1] then
		RS_X = 0
		RE_X = 0
		RS_Y = 0
		RS_Z = 0
	elseif RS_X < Constraints.RS.X[2] then
		RS_X = 0
		RE_X = 0
		RS_Y = 0
		RS_Z = 0
	end
	
	RS.C1 = defRS * CFrame.fromEulerAnglesXYZ(RS_X, RS_Y, RS_Z)
	RE.C1 = defRE * CFrame.fromEulerAnglesXYZ(RE_X, 0, 0)
	
	return RETURN_VALUE
end

function updateLA(END_EFFECTOR)
	if END_EFFECTOR == nil then return end
	
	local RETURN_VALUE = false
	
	local TARGET_VECTOR = getLSCF():inverse() * END_EFFECTOR
	local TARGET_DIST = TARGET_VECTOR.magnitude

	-- New Euler Angle values
	local LS_X, LS_Y, LS_Z = 0;
	local LE_X, LE_Y, LE_Z = 0;

	LS_Z = math.atan2(TARGET_VECTOR.Y, -TARGET_VECTOR.X)
	LS_Y = math.atan2(TARGET_VECTOR.X, -TARGET_VECTOR.Z)
	
	-- Offsetting the axis to have proper angle values
	if LS_Z > math.pi/2 then
		LS_Z = LS_Z - math.pi/2
	else 
		LS_Z = LS_Z + math.pi/2
	end

	-- Apply Constraints
	if LS_Z > Constraints.LS.Z[1] then
		LS_Z = Constraints.LS.Z[1]
	elseif LS_Z < Constraints.LS.Z[2] then
		LS_Z = Constraints.LS.Z[2]
	end
	
	if LS_Y > Constraints.LS.Y[1] then
		LS_Y = Constraints.LS.Y[1]
	elseif LS_Y < Constraints.LS.Y[2] then
		LS_Y = Constraints.LS.Y[2]
	end
	
	local YZ_RATIO; --Used to move more naturally
	if LS_Z < 0 then
		YZ_RATIO = math.abs((Constraints.LS.Z[2]-LS_Z)/Constraints.LS.Z[2])
		LS_Z = LS_Z * YZ_RATIO
		LS_Y = LS_Y * (1-YZ_RATIO)
	else
		YZ_RATIO = math.abs((Constraints.LS.Z[1]-LS_Z)/Constraints.LS.Z[1])
		LS_Z = LS_Z * YZ_RATIO
		LS_Y = LS_Y * (1-YZ_RATIO)
	end
	
	if TARGET_DIST <= ULALen+ LLALen then
		-- TARGET WITHIN LEACH --
		
		-- Law of Cosines
		local theta = math.atan2(TARGET_VECTOR.Y, TARGET_VECTOR.Z)
		if theta > math.pi/2 then
			theta = theta - 3*math.pi/2
		else
			theta = theta + math.pi/2
		end
		local theta1 = LLALen^2 - ULALen^2 - TARGET_DIST^2
		theta1 = theta1 / (-2 * ULALen * TARGET_DIST)
		theta1 = math.acos(theta1)
		LS_X = theta + theta1
		
		local theta2 = TARGET_DIST^2 - ULALen^2 - LLALen^2
		theta2 = theta2 / (-2 * ULALen * LLALen)
		theta2 = math.acos(theta2)
		LE_X = theta2 - math.pi

		RETURN_VALUE = true
	else
		-- TARGET OUT OF LEACH --
		--print("Target out of reach")
		LS_X = math.atan2(TARGET_VECTOR.Y, TARGET_VECTOR.Z) + math.pi/2
		LE_X = 0

		RETURN_VALUE = false
	end
	
	if LS_X > Constraints.LS.X[1] then
		LS_X = 0
		LE_X = 0
		LS_Y = 0
		LS_Z = 0
	elseif LS_X < Constraints.LS.X[2] then
		LS_X = 0
		LE_X = 0
		LS_Y = 0
		LS_Z = 0
	end
	
	LS.C1 = defLS * CFrame.fromEulerAnglesXYZ(LS_X, LS_Y, LS_Z)
	LE.C1 = defLE * CFrame.fromEulerAnglesXYZ(LE_X, 0, 0)
	
	return RETURN_VALUE
end



local defNeck = Neck.C0
function updateNeck(Neck_Target)
	-- Calculate neck angle based on Neck_Target part
	local TARGET_CFRAME = Torso.CFrame:inverse() * Neck_Target.CFrame
	local comp = {TARGET_CFRAME:components()}
	
	-- remove x,y,z components
	table.remove(comp, 1)
	table.remove(comp, 1)
	table.remove(comp, 1)
	
	Neck.C0 = CFrame.new(defNeck.X, defNeck.Y, defNeck.Z, unpack(comp))
end
------------------------------=========-----------------========-------------------------------
--------------------------=============EVENT CONNECTIONS============---------------------------
------------------------------=========-----------------========-------------------------------
resetJoints()

Targets.Hip.CFrame = HumanoidRootPart.CFrame * Hip.C0
Targets.Neck.CFrame = Torso.CFrame * Neck.C0

function updateJoints()
	Hip.C0 = HumanoidRootPart.CFrame:inverse() * Targets.Hip.CFrame
	updateNeck(Targets.Neck)
	updateRA(Targets.RA.Position)
	updateLA(Targets.LA.Position)
	updateRL(Targets.RL.Position)
	updateLL(Targets.LL.Position)
end

Targets.RA.Changed:connect(function()
	updateJoints()
end)
Targets.LA.Changed:connect(function()
	updateJoints()
end)
Targets.RL.Changed:connect(function()
	updateJoints()
end)
Targets.LL.Changed:connect(function()
	updateJoints()
end)
Targets.Hip.Changed:connect(function()
	Targets.Neck.CFrame = Torso.CFrame * Neck.C0
	updateJoints()
end)
Targets.Neck.Changed:connect(function()
	updateJoints()
end)
