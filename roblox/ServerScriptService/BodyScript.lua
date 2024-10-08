MULTI = false
EXECUTE = false
BRUTEFORCE = false
LOAD = true
LOAD_DATA = {
	{
		["Mass"] = 10000,
		["Pos"] = Vector3.new(0, 0, 0),
		["Velo"] = Vector3.new(0, 0, 0),
		["Accel"] = Vector3.new(0, 0, 0)
	}, {
		["Mass"] = 1,
		["Pos"] = Vector3.new(200, 200, 200),
		["Velo"] = Vector3.new(0, 0, 0),
		["Accel"] = Vector3.new(0, 0, 0)
	}, 
}
game.ReplicatedStorage.RemoteEvent.OnServerEvent:Connect(function(player, val)
	EXECUTE = val
end)

local HttpService = game:GetService("HttpService")


local Workspace = game:GetService("Workspace")
local ReplicatedStorage = game:GetService("ReplicatedStorage")
local Bodies = Workspace.Bodies
local SimBodies = ReplicatedStorage.SimBodies


local table_points = {}

local bf = {} -- VALEURS BRUTE FORCE
local bh = {} -- VALEURS BARNES-HUT
local posdiff = {} -- VALEURS DIFFERENCE DE POSITION

-- CONSTANTES PHYSIQUES
local limit = 1024
local g = 6.673e-5
local dt = 1e-2
local min_mass = 10
local max_mass = 100
local max_velo = 5

-- CONSTANTES SIMULATION
local gfactor = 2
local theta = 0.9
local epsilon = 10

-- VALEURS SIMULATION
local tick_ = 0
local maxtick = 5000
local tries = 3
local nbcorps = 20
local maxcorps = 25

-- OCTREE LIB
local octree = {
	Otype = 0,
	body = nil,
	region = nil,
	node = nil
}

function octree:new(o)
	o.parent = self
	return o
end

function createEmpty(region)
	return octree:new{
		Otype = 0,
		region = region
	}
end

function createLeaf(body, region)
	return octree:new{
		Otype = 1,
		body = body,
		region = region
	}
end

function createNode(node, region)
	return octree:new{
		Otype = 2,
		node = node,
		region = region
	}
end

local root = createEmpty(
	Region3.new(
		Vector3.new(-limit, -limit, -limit),
		Vector3.new(limit, limit, limit)
	)
)

-- NODE LIB
local node = {
	center_mass = nil,
	mass = 0,
	children = {}
}

function node:new(o)
	o.parent = self
	return o
end

-- PARCOURS AFFICHAGE CONSOLE
function recursive(tree, s)
	local r = s.."[TYPE "..tree.Otype.." SIZE "..tostring(tree.region.Size.X).." X "..tostring(tree.region.CFrame.X).." Y "..tostring(tree.region.CFrame.Y).." Z "..tostring(tree.region.CFrame.Z).."]"
	if tree.Otype == 1 then
		r = r.." [BODY "..tree.body.Name.."]"
	elseif tree.Otype == 2 then
		for i = 1, 8 do
			r = r.."\n"..recursive(tree.node.children[i], s.." ")
		end
	end
	return r
end

function drawTree(t)
	print("\n[ROOT!]\n"..recursive(t, ""))
end

-- FONCTIONS
---- UTILITAIRE
function nthBit(n, pos)
	local masque = bit32.lshift(1, pos)
	return bit32.band(n, masque) ~= 0
end

function randomPos()
	return Vector3.new(
		math.random(-limit / 4, limit / 4),
		math.random(-limit / 4, limit / 4),
		math.random(-limit / 4, limit / 4)
	)
end

function randomVelo()
	return Vector3.new(
		math.random(-max_velo, max_velo),
		math.random(-max_velo, max_velo),
		math.random(-max_velo, max_velo)
	)
end

function avgTable(t)
	local res = 0
	for i = 1, #t do
		res += t[i]
	end
	return res / #t
end

function avgTableVector3(t1, t2)
	local res = 0
	for i = 1, #t1 do
		res += (t1[i] - t2[i]).Magnitude
	end
	return res / #t1
end

function tableOfVector3(vect)
	return { ["X"] = vect.X, ["Y"] = vect.Y, ["Z"] = vect.Z }
end

---- CALCULS
function forceVect(body1, pos2, mass2)
	local dist = (body1:GetAttribute("Pos") - pos2).Magnitude
	local force = g * body1:GetAttribute("Mass") * mass2 / (math.sqrt(dist ^ 2 + epsilon ^ 2) ^ gfactor)
	local dir = (pos2 - body1:GetAttribute("Pos")).Unit
	return force * dir
end

---- OCTREE UTILITAIRES
function generateEmpty(region)
	local res = {}
	for i = 1, 8 do
		res[i] = createEmpty(subRegion(region, i))
	end
	return res
end

function getSubtree(vect, region)
	local res = 0
	if region.CFrame.X < vect.X then res += 1 end
	if region.CFrame.Y < vect.Y then res += 2 end
	if region.CFrame.Z < vect.Z then res += 4 end
	return res + 1
end

function subRegion(region, n)
	local center = region.CFrame
	local size = region.Size:Abs()

	local sx, sy, sz = size.X/2, size.Y/2, size.Z/2
	local mx, my, mz = center.X, center.Y, center.Z

	if nthBit(n - 1, 0) then
		mx += sx
	else
		mx -= sx
	end
	if nthBit(n - 1, 1) then
		my += sy
	else
		my -= sy
	end
	if nthBit(n - 1, 2) then
		mz += sz
	else
		mz -= sz
	end

	local center = Vector3.new(mx, my, mz)
	return Region3.new(center - size / 4, center + size / 4)
end

function insertInOctree(tree, body)
	if tree.Otype == 2 then
		local somme_masses = tree.node.mass + body:GetAttribute("Mass")
		tree.node.center_mass = Vector3.new(
			(tree.node.mass * tree.node.center_mass.X + body:GetAttribute("Mass") * body:GetAttribute("Pos").X) / somme_masses,
			(tree.node.mass * tree.node.center_mass.Y + body:GetAttribute("Mass") * body:GetAttribute("Pos").Y) / somme_masses,
			(tree.node.mass * tree.node.center_mass.Z + body:GetAttribute("Mass") * body:GetAttribute("Pos").Z) / somme_masses
		)
		tree.node.mass += body:GetAttribute("Mass")
		local i = getSubtree(body:GetAttribute("Pos"), tree.region)
		tree.node.children[i] = insertInOctree(tree.node.children[i], body)
		return tree
	elseif tree.Otype == 1 then
		if body.Name ~= tree.body.Name then
			local newSubtree = nil
			local s = insertInOctree(createNode(node:new{
				center_mass = Vector3.new(0, 0, 0),
				mass = 0,
				children = generateEmpty(tree.region)
			}, tree.region), tree.body)
			return insertInOctree(s, body)
		end
	elseif tree.Otype == 0 then
		return createLeaf(body, tree.region)
	else
		warn("insertInOctree: Otype is illegal")
	end
	return tree
end

function buildOctree(reset)
	root = if reset then createEmpty(
		Region3.new(
			Vector3.new(-limit, -limit, -limit),
			Vector3.new(limit, limit, limit)
		)
	) else root
	for i = 1, #table_points do
		root = insertInOctree(root, table_points[i])
	end
end

---- BRUTE FORCE
function bruteForce(universe)
	local as = {}
	
	for i = 1, #universe do
		for j = 1, #universe do
			if as[j] == nil then as[j] = Vector3.new(0, 0, 0) end
			if i ~= j then
				local a = forceVect(universe[j], universe[i]:GetAttribute("Pos"), universe[i]:GetAttribute("Mass"))
				as[i] = as[i] + a / universe[i]:GetAttribute("Mass")
			end
		end
	end

	return as
end

---- DEPLACEMENT
function updatePos(universe, dt)
	for i = 1, #universe do
		universe[i]:SetAttribute("Pos", universe[i]:GetAttribute("Pos") + universe[i]:GetAttribute("Velo") * dt + universe[i]:GetAttribute("Accel") * (dt^2)/2)
	end
end

function updateVelo(universe, accel, dt)
	for i = 1, #universe do
		universe[i]:SetAttribute("Velo", universe[i]:GetAttribute("Velo") + (universe[i]:GetAttribute("Accel") + accel[i]) * dt / 2)
	end
end

function updateAccel(universe, accel)
	for i = 1, #universe do
		universe[i]:SetAttribute("Accel", accel[i])
	end
end

function updateWorkspace(universe)
	for i = 1, #universe do
		universe[i].Position = universe[i]:GetAttribute("Pos")
	end
end

function updateAccTree(tree, body)
	if tree.Otype == 0 then
		return Vector3.new(0, 0, 0)
	elseif tree.Otype == 1 and tree.body.Name == body.Name then
		return Vector3.new(0, 0, 0)
	elseif tree.Otype == 1 then
		return forceVect(body, tree.body:GetAttribute("Pos"), tree.body:GetAttribute("Mass"))
	elseif tree.Otype == 2 then
		if  (body:GetAttribute("Pos") - tree.node.center_mass).Magnitude <= math.abs(tree.region.Size.X) * theta then
			return forceVect(body, tree.node.center_mass, tree.node.mass)
		else
			local r = Vector3.new(0, 0, 0)
			for i = 1, 8 do
				r = r + updateAccTree(tree.node.children[i], body)
			end
			return r
		end
	else
		warn("updateAccTree: Otype is illegal")
	end
end

function updateAllAcc(tree, universe)
	local as = {}

	for i = 1, #universe do
		as[i] = updateAccTree(tree, universe[i]) / universe[i]:GetAttribute("Mass")
	end
	
	return as
end

function applyAlgorithm()
	if BRUTEFORCE then
		return bruteForce(table_points)
	else
		local as = updateAllAcc(root, table_points)
		buildOctree(true)
		return as
	end
end

function moveForwardLeapfrog(moveWorkspace)
	updatePos(table_points, dt)
	local as = applyAlgorithm()
	updateVelo(table_points, as, dt)
	updateAccel(table_points, as)
	if moveWorkspace then updateWorkspace(table_points) end
end

---- INITIALISATION
function createBodies(qte, from_loaded)
	print("Créé "..if from_loaded then #LOAD_DATA else qte.." corps")
	for i = 1, if from_loaded then #LOAD_DATA else qte do
		local part = Instance.new("Part")
		local pos = randomPos()
		part.Name = "Corps "..i
		part.Shape = Enum.PartType.Ball
		part.Anchored = true
		part.Size = Vector3.new(5, 5, 5)
		part.Color = Color3.fromRGB(255, 0, 0)
		if from_loaded then
			part.Position = LOAD_DATA[i].Pos
			part:SetAttribute("Mass", LOAD_DATA[i].Mass)
			part:SetAttribute("Pos", LOAD_DATA[i].Pos)
			part:SetAttribute("Velo", LOAD_DATA[i].Velo)
			part:SetAttribute("Accel", LOAD_DATA[i].Accel)
		else
			part.Position = pos
			part:SetAttribute("Mass", math.random(min_mass, max_mass))
			part:SetAttribute("Pos", pos)
			part:SetAttribute("Velo", randomVelo())
			part:SetAttribute("Accel", Vector3.new(0, 0, 0))
		end
		part.Parent = SimBodies
	end
end

---- DEBUT DE SESSION
function copieSimBodies()
	for i, v in pairs(SimBodies:GetChildren()) do
		local clone = v:Clone()
		clone.Parent = Bodies
		table.insert(table_points, clone)
	end
end

function sendToServer(bft, bht, delta)
	local data = {
		["g"] = g,
		["dt"] = dt,
		["theta"] = theta,
		["epsilon"] = epsilon,
		["gfactor"] = gfactor,

		["ticks"] = maxtick,
		["tries"] = tries,
		["bodies"] = {},

		["bruteForceTime"] = bft,
		["barnesHutTime"] = bht,
		["deltaDist"] = delta
	}

	for i, v in pairs(SimBodies:GetChildren()) do
		table.insert(data["bodies"], {
			["Pos"] = tableOfVector3(v:GetAttribute("Pos")),
			["Velo"] = tableOfVector3(v:GetAttribute("Velo")),
			["Accel"] = tableOfVector3(v:GetAttribute("Accel")),
			["Mass"] = v:GetAttribute("Mass")
		})
	end

	local encoded = HttpService:JSONEncode(data)

	local success, result = pcall(function()
		HttpService:PostAsync("http://localhost:3035/", encoded, Enum.HttpContentType.ApplicationJson)
	end)

	print(success, result)

	if success then
		print("Data sent to server")
	else
		warn("Could not send to server")
	end
end

function lanceNoMulti()
	createBodies(nbcorps, LOAD)
	copieSimBodies()

	if not BRUTEFORCE then
		buildOctree(false)
	end
	
	while true do
		wait(0.00005)
		if EXECUTE then
			tick_ += 1
			moveForwardLeapfrog(true)
		end
	end
end

function lanceMulti()
	if nbcorps < 2 then nbcorps = 2 end
	if nbcorps > maxcorps then
		MULTI = false
		return
	end
	
	local temps_bf = {}
	local pos_bf = {}
	local temps_bh = {}
	local pos_bh = {}
	
	SimBodies:ClearAllChildren()
	createBodies(nbcorps)
	
	local i = 0
	
	BRUTEFORCE = true
	while i < tries do
		wait(1)
		tick_ = 0
		
		Bodies:ClearAllChildren()
		table.clear(table_points)
		copieSimBodies()
		
		local starttime = os.clock()
		while tick_ < maxtick do
			tick_ += 1
			moveForwardLeapfrog(false)
		end
		local endtime = os.clock()
		
		table.insert(temps_bf, endtime - starttime)
		if i == 0 then
			for j = 1, #table_points do
				table.insert(pos_bf, table_points[j]:GetAttribute("Pos"))
			end
		end
		
		i += 1
	end
	
	BRUTEFORCE = false
	while i < tries * 2 do
		wait(1)
		tick_ = 0

		Bodies:ClearAllChildren()
		table.clear(table_points)
		copieSimBodies()
		root = createEmpty(
			Region3.new(
				Vector3.new(-limit, -limit, -limit),
				Vector3.new(limit, limit, limit)
			)
		)

		local starttime = os.clock()
		buildOctree(false)
		while tick_ < maxtick do
			tick_ += 1
			moveForwardLeapfrog(false)
		end
		local endtime = os.clock()

		table.insert(temps_bh, endtime - starttime)
		if i == tries then
			for j = 1, #table_points do
				table.insert(pos_bh, table_points[j]:GetAttribute("Pos"))
			end
		end
		
		i += 1
	end
	
	local bf_temps_moyen = avgTable(temps_bf)
	local bh_temps_moyen = avgTable(temps_bh)
	local norme_diff_moy = avgTableVector3(pos_bf, pos_bh)
	
	table.insert(bf, bf_temps_moyen)
	table.insert(bh, bh_temps_moyen)
	table.insert(posdiff, norme_diff_moy)
	
	sendToServer(bf_temps_moyen, bh_temps_moyen, norme_diff_moy)
	
	nbcorps += 1
	return lanceMulti()
end

---- AFFICHAGE
local Players = game:GetService("Players")
local function onPlayerAdded (player)
	local data = player.PlayerGui:WaitForChild("DataGui")
	data.Frame.dt.Text = "dt : "..dt.."s"
	while true do
		wait(0.01)
		data.Frame.Corps.Text = "Corps : "..(if not MULTI and LOAD then #LOAD_DATA else nbcorps)
		data.Frame.Methode.Text = "Méthode : "..(if BRUTEFORCE then "Force Brute" else "Barnes-Hut")..(if MULTI then " MULTI" else "")
		data.Frame.Pause.Text = (if EXECUTE or MULTI then "En cours" else "En pause")
		data.Frame.Tick.Text = "Tick : "..tick_
		data.Frame.LTBF.Text = "Dernier temps moyen Brute Force : "..(if #bf == 0 then "N/A" else bf[#bf].."s")
		data.Frame.LTBH.Text = "Dernier temps moyen Barnes-Hut : "..(if #bh == 0 then "N/A" else bh[#bh].."s")
		data.Frame.LPBF.Text = "Dernière distance moyenne : "..(if #posdiff == 0 then "N/A" else posdiff[#posdiff].."studs")
	end
end
Players.PlayerAdded:Connect(onPlayerAdded)
for _, player in ipairs(Players:GetPlayers()) do
	onPlayerAdded(player)
end


-- MAIN
Workspace.Centre:Destroy()
Workspace.Limites.Transparency = 1

if MULTI then
	lanceMulti()
else
	lanceNoMulti()
end
