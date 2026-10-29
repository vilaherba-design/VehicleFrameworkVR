Scriptname VehicleFrameworkVR:Combat Hidden
{Called by the plugin (VehicleFrameworkVR.dll) while the player drives: an enemy that has already seen the driver is told to fight them.
Seated in a vehicle the player is seen but the AI does not start combat on its own; this is not called per frame (at most every few seconds per actor).
The actor comes as its form id (an int: the plugin's Papyrus call packs plain values only).}

Function Engage(int aiActorFormId) global
	Actor akActor = Game.GetForm(aiActorFormId) as Actor
	Actor player = Game.GetPlayer()
	If akActor && !akActor.IsDead() && akActor.GetCombatTarget() != player
		akActor.StartCombat(player)
	EndIf
EndFunction

; Diagnostics (bLogAIProbe): what the engine itself says about an enemy and the player, handed back to the plugin's log (ReportCombat).
Function Report(int aiActorFormId) global
	Actor akActor = Game.GetForm(aiActorFormId) as Actor
	Actor player = Game.GetPlayer()
	If !akActor || !player
		Return
	EndIf
	int targetId = 0
	Actor target = akActor.GetCombatTarget()
	If target
		targetId = target.GetFormID()
	EndIf
	VehicleFrameworkVR:Probe.ReportCombat(aiActorFormId, akActor.GetCombatState(), targetId, akActor.IsWeaponDrawn(), akActor.HasDetectionLOS(player), player.IsDetectedBy(akActor), akActor.IsDead(), akActor.GetDistance(player))
EndFunction

