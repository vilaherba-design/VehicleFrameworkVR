Scriptname VehicleFrameworkVR:Turret Hidden
{Called by the plugin (VehicleFrameworkVR.dll) when a vehicle turret starts and stops firing: its fire sound is a loop (the vanilla turret's
WPNRobotTurretLowTechFire is an automatic-fire loop at 420 rpm), started at the player (the turret sits over the driver's head) and stopped when the
trigger is let go. Forms and instances come as ints (the plugin's Papyrus call packs plain values only). Two calls per burst, nothing per frame.}

Function StartFire(int aiSoundFormId) global
	Sound fireSound = Game.GetForm(aiSoundFormId) as Sound
	If fireSound
		VehicleFrameworkVR:Probe.TurretSoundStarted(fireSound.Play(Game.GetPlayer()))
	EndIf
EndFunction

; One shot of a single-shot weapon (a cannon, a rocket): the sound plays once at the player. One call per shot.
Function PlayShot(int aiSoundFormId) global
	Sound shotSound = Game.GetForm(aiSoundFormId) as Sound
	If shotSound
		shotSound.Play(Game.GetPlayer())
	EndIf
EndFunction

Function StopFire(int aiInstance) global
	If aiInstance != 0
		Sound.StopInstance(aiInstance)
	EndIf
EndFunction
