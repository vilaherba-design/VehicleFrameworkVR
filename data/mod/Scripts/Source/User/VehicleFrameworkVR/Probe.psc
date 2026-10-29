Scriptname VehicleFrameworkVR:Probe Native Hidden
{Native diagnostics of the Vehicle Framework VR plugin (VehicleFrameworkVR.dll). Its own script: "Native" is a keyword, so VehicleFrameworkVR:Native cannot be compiled.}

; Called by VehicleFrameworkVR:Combat.Report: writes one "AIProbe: engine says" line in the plugin's log.
Function ReportCombat(int aiActorFormId, int aiCombatState, int aiTargetFormId, bool abWeaponDrawn, bool abHasLOS, bool abDetected, bool abDead, float afDistance) global native

; Called by VehicleFrameworkVR:ChargingStation on power, load, unload and scrap: vehicles parked near a powered station fill their battery.
Function SetChargerPowered(ObjectReference akStation, bool abPowered) global native

; Called by VehicleFrameworkVR:Turret.StartFire with the instance of the turret's fire loop, which the plugin stops when the trigger is let go.
Function TurretSoundStarted(int aiInstance) global native
