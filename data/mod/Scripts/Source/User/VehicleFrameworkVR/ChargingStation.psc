Scriptname VehicleFrameworkVR:ChargingStation extends ObjectReference
{The vehicle charging station (VFVR_ChargingStation, built at a settlement's workshop). Tells the plugin (VehicleFrameworkVR.dll) when the
station gets or loses power, loads, unloads or is scrapped: a vehicle standing still near a powered station fills its battery.}

Function Report(bool abPowered)
	VehicleFrameworkVR:Probe.SetChargerPowered(self, abPowered)
EndFunction

Event OnLoad()
	Report(IsPowered())
EndEvent

Event OnUnload()
	Report(false)
EndEvent

Event OnPowerOn(ObjectReference akPowerGenerator)
	Report(true)
EndEvent

Event OnPowerOff()
	Report(false)
EndEvent

Event OnWorkshopObjectPlaced(ObjectReference akReference)
	Report(IsPowered())
EndEvent

Event OnWorkshopObjectDestroyed(ObjectReference akActionRef)
	Report(false)
EndEvent
