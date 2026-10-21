Scriptname VehicleFrameworkVR:TestGrenadeMarker extends ObjectReference
{Placed by the test grenade's explosion (Placed Object). Waits 5 seconds, asks the plugin to spawn the test
vehicle here, then removes itself. One event and one timer: no polling.}

float Property SpawnDelay = 5.0 Auto Const
{Seconds between the grenade landing and the vehicle appearing.}

Event OnInit()
	StartTimer(SpawnDelay, 1)
EndEvent

Event OnTimer(int aiTimerID)
	If aiTimerID == 1
		If !VehicleFrameworkVR:Native.SpawnTestVehicleAt(Self)
			Debug.Notification("Vehicle Framework VR plugin is not loaded")
		EndIf
		Disable()
		Delete()
	EndIf
EndEvent
