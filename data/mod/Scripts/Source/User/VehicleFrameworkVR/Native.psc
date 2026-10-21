Scriptname VehicleFrameworkVR:Native Native Hidden
{Native functions of the Vehicle Framework VR plugin (VehicleFrameworkVR.dll).}

; Spawn the test vehicle (sTestVehicleProfile in the INI) at the marker's position, facing away from the player.
; Returns false when the plugin is not loaded. The plugin reads the position immediately and spawns on the next
; frame, so the marker can be deleted right after the call.
bool Function SpawnTestVehicleAt(ObjectReference akMarker) global native

; Version of the loaded plugin, e.g. "0.1.0".
string Function GetVersion() global native
