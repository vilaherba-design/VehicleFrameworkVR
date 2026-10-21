#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "model/VehicleSpec.h"

namespace vfvr::model
{
    namespace
    {
        using json = nlohmann::json;

        void readFloat(const json& j, const char* key, float& out)
        {
            if (const auto it = j.find(key); it != j.end()) {
                if (!it->is_number()) {
                    throw std::runtime_error(std::string("'") + key + "' must be a number");
                }
                out = it->get<float>();
            }
        }

        void readBool(const json& j, const char* key, bool& out)
        {
            if (const auto it = j.find(key); it != j.end()) {
                out = it->get<bool>();
            }
        }

        void readString(const json& j, const char* key, std::string& out)
        {
            if (const auto it = j.find(key); it != j.end()) {
                out = it->get<std::string>();
            }
        }

        // A position in game units -> meters.
        void readGamePoint(const json& j, const char* key, Vec3& out)
        {
            if (const auto it = j.find(key); it != j.end()) {
                if (!it->is_array() || it->size() != 3) {
                    throw std::runtime_error(std::string("'") + key + "' must be [x, y, z] in game units");
                }
                out = Vec3{ (*it)[0].get<float>(), (*it)[1].get<float>(), (*it)[2].get<float>() } * GAME_TO_HAVOK;
            }
        }

        // A plain 3-vector (a direction: no unit conversion).
        void readVector(const json& j, const char* key, Vec3& out)
        {
            if (const auto it = j.find(key); it != j.end()) {
                if (!it->is_array() || it->size() != 3) {
                    throw std::runtime_error(std::string("'") + key + "' must be [x, y, z]");
                }
                out = Vec3{ (*it)[0].get<float>(), (*it)[1].get<float>(), (*it)[2].get<float>() };
            }
        }

        const json& section(const json& root, const char* key)
        {
            static const json empty = json::object();
            const auto it = root.find(key);
            return it != root.end() && it->is_object() ? *it : empty;
        }
    }

    std::vector<Vec3> VehicleSpec::doorHandlePoints() const
    {
        if (!doorHandles.empty()) {
            return doorHandles;
        }
        if (hull.explicitBox) {
            const float side = exitPoint.x < driverEye.x ? hull.min.x : hull.max.x;
            return { Vec3{ side, driverEye.y, 0.5f * (driverEye.z + hull.min.z) } };
        }
        // No hull box: halfway from the seat to the exit point, about half a metre below the eyes.
        return { Vec3{ 0.5f * (driverEye.x + exitPoint.x), driverEye.y, driverEye.z - 0.45f } };
    }

    bool VehicleSpec::ensureWheels(const Vec3& boundsMin, const Vec3& boundsMax)
    {
        if (!wheels.empty()) {
            return false;
        }
        const float centerX = (boundsMin.x + boundsMax.x) * 0.5f;
        const float centerY = (boundsMin.y + boundsMax.y) * 0.5f;
        const float halfWidth = (boundsMax.x - boundsMin.x) * 0.5f;
        const float halfLength = (boundsMax.y - boundsMin.y) * 0.5f;
        const float x = (std::max)(halfWidth - autoWheelInsetX, 0.3f);
        const float y = halfLength * autoWheelbaseFactor;
        const float staticCompression = mass * 9.81f / (4.0f * suspension.springRate);
        const float autoHeight = (std::max)(tyre.radius - staticCompression - autoGroundClearance, 0.05f);
        const float z = boundsMin.z + (autoHubHeight >= 0.0f ? autoHubHeight : autoHeight);
        const bool front = driveLayout == "fwd" || driveLayout == "awd";
        const bool rear = driveLayout != "fwd";
        wheels = {
            { "FL", { centerX - x, centerY + y, z }, true, front, false, -1.0f },
            { "FR", { centerX + x, centerY + y, z }, true, front, false, 1.0f },
            { "RL", { centerX - x, centerY - y, z }, false, rear, true, -1.0f },
            { "RR", { centerX + x, centerY - y, z }, false, rear, true, 1.0f },
        };
        return true;
    }

    float VehicleSpec::rollInertia() const
    {
        if (lean.rollInertia > 0.0f) {
            return lean.rollInertia;
        }
        const float width = hull.explicitBox ? hull.max.x - hull.min.x : 0.7f;
        const float height = hull.explicitBox ? hull.max.z - hull.min.z : 1.1f;
        return mass * (width * width + height * height) / 12.0f;
    }

    float VehicleSpec::pitchInertia() const
    {
        if (lean.pitchInertia > 0.0f) {
            return lean.pitchInertia;
        }
        const float length = hull.explicitBox ? hull.max.y - hull.min.y : 4.4f;
        const float height = hull.explicitBox ? hull.max.z - hull.min.z : 1.1f;
        return mass * (length * length + height * height) / 12.0f;
    }

    const PartSpec* findPart(const VehicleSpec& spec, const std::string& slot, const std::string& id)
    {
        for (const auto& part : spec.parts) {
            if (part.slot == slot && part.id == id) {
                return &part;
            }
        }
        return nullptr;
    }

    VehicleSpec parseVehicleSpec(const std::string& jsonText, const std::map<std::string, std::string>& selection)
    {
        json root;
        try {
            root = json::parse(jsonText, nullptr, true, true); // allow comments
        } catch (const json::parse_error& e) {
            throw std::runtime_error(std::string("JSON syntax error: ") + e.what());
        }

        VehicleSpec spec;
        try {
            // Parts: collect them, then merge the selected ones into the document.
            if (const auto it = root.find("parts"); it != root.end() && it->is_object()) {
                for (const auto& [slot, list] : it->items()) {
                    if (!list.is_array()) {
                        continue;
                    }
                    for (const auto& p : list) {
                        PartSpec part;
                        part.slot = slot;
                        readString(p, "id", part.id);
                        readString(p, "name", part.name);
                        if (part.name.empty()) {
                            part.name = part.id;
                        }
                        part.patchJson = p.contains("patch") ? p["patch"].dump() : "{}";
                        if (!part.id.empty()) {
                            spec.parts.push_back(std::move(part));
                        }
                    }
                }
            }
            for (const auto& [slot, id] : selection) {
                const auto* part = findPart(spec, slot, id);
                if (!part) {
                    throw std::runtime_error("no part '" + id + "' for slot '" + slot + "'");
                }
                root.merge_patch(json::parse(part->patchJson));
                spec.selection[slot] = id;
            }

            readString(root, "id", spec.id);
            readString(root, "displayName", spec.displayName);
            readString(root, "baseForm", spec.baseForm);
            readString(root, "fallbackBaseForm", spec.fallbackBaseForm);
            readString(root, "carrierForm", spec.carrierForm);
            readString(root, "modelForward", spec.modelForward);
            readFloat(root, "mass", spec.mass);
            readString(root, "driveLayout", spec.driveLayout);
            readFloat(root, "hullClearance", spec.hullClearance);
            readFloat(root, "maxGroundSlopeDeg", spec.maxGroundSlopeDeg);
            readBool(root, "boxHull", spec.boxHull);

            {
                const auto& damage = section(root, "damage");
                readFloat(damage, "maxHealth", spec.damage.maxHealth);
                const auto& resistance = section(damage, "resistance");
                readFloat(resistance, "ballistic", spec.damage.ballisticResistance);
                readFloat(resistance, "explosion", spec.damage.explosionResistance);
                readFloat(resistance, "melee", spec.damage.meleeResistance);
                readFloat(resistance, "collision", spec.damage.collisionResistance);
                const auto& multipliers = section(damage, "multipliers");
                readFloat(multipliers, "ballistic", spec.damage.ballisticMultiplier);
                readFloat(multipliers, "explosion", spec.damage.explosionMultiplier);
                readFloat(multipliers, "melee", spec.damage.meleeMultiplier);
                readFloat(damage, "unknownAttackDamage", spec.damage.unknownAttackDamage);
                readBool(damage, "occupantProtected", spec.damage.occupantProtected);
                readBool(damage, "hitsWhileParked", spec.damage.hitsWhileParked);
                readBool(damage, "selfDamage", spec.damage.selfDamage);
                const auto& collision = section(damage, "collision");
                readFloat(collision, "minDeltaV", spec.damage.collisionMinDeltaV);
                readFloat(collision, "factor", spec.damage.collisionFactor);
                readFloat(collision, "exponent", spec.damage.collisionExponent);
                readFloat(collision, "maxShare", spec.damage.collisionMaxShare);
                readFloat(collision, "cooldownSeconds", spec.damage.collisionCooldownSeconds);
                const auto& smoke = section(damage, "smoke");
                readFloat(smoke, "damagedBelow", spec.damage.damagedBelow);
                if (const auto it = smoke.find("points"); it != smoke.end() && it->is_array()) {
                    spec.damage.smokePoints.clear();
                    for (const auto& p : *it) {
                        if (!p.is_array() || p.size() != 3) {
                            throw std::runtime_error("'damage.smoke.points' entries must be [x, y, z] in game units");
                        }
                        spec.damage.smokePoints.push_back(Vec3{ p[0].get<float>(), p[1].get<float>(), p[2].get<float>() } * GAME_TO_HAVOK);
                    }
                }
                readString(damage, "onZeroHealth", spec.damage.onZeroHealth);
                const auto& repair = section(damage, "repair");
                readFloat(repair, "seconds", spec.damage.repairSeconds);
                readFloat(repair, "healthShare", spec.damage.repairHealthShare);
                readString(repair, "item", spec.damage.repairItem);
                if (const auto it = repair.find("point"); it != repair.end()) {
                    if (!it->is_array() || it->size() != 3) {
                        throw std::runtime_error("'damage.repair.point' must be [x, y, z] in game units");
                    }
                    spec.damage.repairPoint = Vec3{ (*it)[0].get<float>(), (*it)[1].get<float>(), (*it)[2].get<float>() } * GAME_TO_HAVOK;
                    spec.damage.hasRepairPoint = true;
                }
                if (spec.damage.maxHealth <= 0.0f) {
                    throw std::runtime_error("'damage.maxHealth' must be above 0");
                }
                for (const float r : { spec.damage.ballisticResistance, spec.damage.explosionResistance, spec.damage.meleeResistance, spec.damage.collisionResistance }) {
                    if (r < 0.0f) {
                        throw std::runtime_error("'damage.resistance' values must be 0 or more");
                    }
                }
                if (spec.damage.damagedBelow < 0.0f || spec.damage.damagedBelow > 1.0f) {
                    throw std::runtime_error("'damage.smoke.damagedBelow' must be between 0 and 1");
                }
                if (spec.damage.onZeroHealth != "disable") {
                    throw std::runtime_error("'damage.onZeroHealth' must be \"disable\" (the only state so far)");
                }
            }
            {
                const auto& radiation = section(root, "radiation");
                readFloat(radiation, "resistance", spec.radiation.resistance);
                readFloat(radiation, "capacity", spec.radiation.capacity);
                readFloat(radiation, "drainThreshold", spec.radiation.drainThreshold);
                readFloat(radiation, "drainPerRad", spec.radiation.drainPerRad);
                readFloat(radiation, "recoverPerSecond", spec.radiation.recoverPerSecond);
                if (spec.radiation.resistance < 0.0f || spec.radiation.capacity < 0.0f) {
                    throw std::runtime_error("'radiation.resistance' and 'radiation.capacity' must be 0 or more");
                }
            }
            readFloat(section(root, "wheelVisual"), "maxAboveMesh", spec.wheelMaxAboveMesh);
            readFloat(section(root, "wheelVisual"), "lift", spec.wheelVisualLift);
            readString(section(root, "wheelVisual"), "set", spec.wheelSet);

            const auto& hull = section(root, "hull");
            if (hull.contains("min") && hull.contains("max")) {
                readGamePoint(hull, "min", spec.hull.min);
                readGamePoint(hull, "max", spec.hull.max);
                spec.hull.explicitBox = true;
            }
            readFloat(hull, "rampLength", spec.hull.rampLength);
            readFloat(hull, "rampHeight", spec.hull.rampHeight);
            readFloat(hull, "sideInset", spec.hull.sideInset);

            if (const auto it = root.find("wheels"); it != root.end() && it->is_array()) {
                for (const auto& w : *it) {
                    WheelSpec wheel;
                    readString(w, "name", wheel.name);
                    readGamePoint(w, "hub", wheel.hub);
                    readBool(w, "steer", wheel.steer);
                    readBool(w, "drive", wheel.drive);
                    readBool(w, "handbrake", wheel.handbrake);
                    readFloat(w, "radius", wheel.radius);
                    spec.wheels.push_back(wheel);
                }
            }

            // Every hub moves by the same offset (game units): a lift kit or taller tyres lower the wheel centres in the body.
            Vec3 hubOffset;
            readGamePoint(root, "hubOffset", hubOffset);
            for (auto& wheel : spec.wheels) {
                wheel.hub = wheel.hub + hubOffset;
            }
            // Which wheels are driven, when the document says so (overrides the per-wheel flags): "rwd", "fwd" or "awd".
            if (root.contains("driveLayout")) {
                const auto layout = root["driveLayout"].get<std::string>();
                const bool front = layout == "fwd" || layout == "awd";
                const bool rear = layout != "fwd";
                for (auto& wheel : spec.wheels) {
                    wheel.drive = wheel.hub.y >= 0.0f ? front : rear;
                }
            }

            const auto& autoWheels = section(root, "autoWheels");
            float insetGame = spec.autoWheelInsetX * HAVOK_TO_GAME;
            readFloat(autoWheels, "insetX", insetGame);
            spec.autoWheelInsetX = insetGame * GAME_TO_HAVOK;
            readFloat(autoWheels, "wheelbaseFactor", spec.autoWheelbaseFactor);
            float clearanceGame = spec.autoGroundClearance * HAVOK_TO_GAME;
            readFloat(autoWheels, "clearance", clearanceGame);
            spec.autoGroundClearance = clearanceGame * GAME_TO_HAVOK;
            if (autoWheels.contains("hubHeight")) {
                float hubGame = 0.0f;
                readFloat(autoWheels, "hubHeight", hubGame);
                spec.autoHubHeight = hubGame * GAME_TO_HAVOK;
            }

            const auto& sus = section(root, "suspension");
            readFloat(sus, "restLength", spec.suspension.restLength);
            readFloat(sus, "springRate", spec.suspension.springRate);
            readFloat(sus, "damperBump", spec.suspension.damperBump);
            readFloat(sus, "damperRebound", spec.suspension.damperRebound);
            readFloat(sus, "maxForce", spec.suspension.maxForce);
            readFloat(sus, "antiRollFront", spec.suspension.antiRollFront);
            readFloat(sus, "antiRollRear", spec.suspension.antiRollRear);
            readFloat(sus, "frictionLift", spec.suspension.frictionLift);
            readFloat(sus, "bumpStopStart", spec.suspension.bumpStopStart);
            readFloat(sus, "bumpStopRate", spec.suspension.bumpStopRate);
            readFloat(sus, "bumpStopDamping", spec.suspension.bumpStopDamping);

            const auto& tyre = section(root, "tyre");
            readFloat(tyre, "radius", spec.tyre.radius);
            readFloat(tyre, "wheelInertia", spec.tyre.wheelInertia);
            readFloat(tyre, "grip", spec.tyre.gripAsphalt);
            readFloat(tyre, "longB", spec.tyre.longB);
            readFloat(tyre, "longC", spec.tyre.longC);
            readFloat(tyre, "latB", spec.tyre.latB);
            readFloat(tyre, "latC", spec.tyre.latC);
            readFloat(tyre, "rollingResistance", spec.tyre.rollingResistance);
            readFloat(tyre, "minSlipSpeed", spec.tyre.minSlipSpeed);

            const auto& eng = section(root, "engine");
            if (const auto it = eng.find("torqueCurve"); it != eng.end()) {
                spec.engine.torqueCurve.clear();
                for (const auto& point : *it) {
                    spec.engine.torqueCurve.emplace_back(point.at(0).get<float>(), point.at(1).get<float>());
                }
            }
            if (const auto it = eng.find("gears"); it != eng.end()) {
                spec.engine.gears = it->get<std::vector<float>>();
            }
            if (const auto it = eng.find("cylinders"); it != eng.end() && it->is_number_integer()) {
                spec.engine.cylinders = it->get<int>();
            }
            readFloat(eng, "idleRpm", spec.engine.idleRpm);
            readFloat(eng, "maxRpm", spec.engine.maxRpm);
            readFloat(eng, "reverseRatio", spec.engine.reverseRatio);
            readFloat(eng, "finalDrive", spec.engine.finalDrive);
            readFloat(eng, "efficiency", spec.engine.efficiency);
            readFloat(eng, "shiftUpRpm", spec.engine.shiftUpRpm);
            readFloat(eng, "shiftDownRpm", spec.engine.shiftDownRpm);
            readFloat(eng, "shiftTime", spec.engine.shiftTime);
            readFloat(eng, "engineBrake", spec.engine.engineBrake);
            readFloat(eng, "tractionControlSlip", spec.engine.tractionControlSlip);
            const auto& snd = section(eng, "sound");
            readFloat(snd, "gain", spec.engine.sound.gain);
            readFloat(snd, "pitch", spec.engine.sound.pitch);
            readFloat(snd, "oddDecay", spec.engine.sound.oddDecay);
            readFloat(snd, "evenDecay", spec.engine.sound.evenDecay);
            readFloat(snd, "rumble", spec.engine.sound.rumble);
            readFloat(snd, "lumpiness", spec.engine.sound.lumpiness);
            readFloat(snd, "noise", spec.engine.sound.noise);
            readFloat(snd, "brightness", spec.engine.sound.brightness);
            readFloat(snd, "whine", spec.engine.sound.whine);
            readFloat(snd, "whineOrder", spec.engine.sound.whineOrder);

            const auto& brakes = section(root, "brakes");
            readFloat(brakes, "maxTorque", spec.brakes.maxTorque);
            readFloat(brakes, "frontBias", spec.brakes.frontBias);
            readFloat(brakes, "handbrakeTorque", spec.brakes.handbrakeTorque);
            readFloat(brakes, "holdSpeed", spec.brakes.holdSpeed);
            readFloat(brakes, "absSlip", spec.brakes.absSlip);

            const auto& steer = section(root, "steering");
            readFloat(steer, "maxAngleDeg", spec.steering.maxAngleDeg);
            readFloat(steer, "reduceAtSpeed", spec.steering.reduceAtSpeed);
            readFloat(steer, "reducedFactor", spec.steering.reducedFactor);
            readFloat(steer, "speedDegPerSec", spec.steering.speedDegPerSec);
            readFloat(steer, "maxLateralG", spec.steering.maxLateralG);
            readFloat(steer, "yawLimitG", spec.steering.yawLimitG);
            readGamePoint(steer, "pivot", spec.steering.pivot);
            readVector(steer, "axis", spec.steering.axis);

            if (const auto it = root.find("lean"); it != root.end() && it->is_object()) {
                const auto& lean = *it;
                spec.lean.enabled = true;
                readBool(lean, "enabled", spec.lean.enabled);
                readFloat(lean, "maxLeanDeg", spec.lean.maxLeanDeg);
                readFloat(lean, "follow", spec.lean.follow);
                readFloat(lean, "frequencyHz", spec.lean.frequencyHz);
                readFloat(lean, "damping", spec.lean.damping);
                readFloat(lean, "uprightBelow", spec.lean.uprightBelow);
                readFloat(lean, "armLength", spec.lean.armLength);
                readFloat(lean, "rollInertia", spec.lean.rollInertia);
                readFloat(lean, "maxTorque", spec.lean.maxTorque);
                readFloat(lean, "fallDeg", spec.lean.fallDeg);
                readFloat(lean, "targetSeconds", spec.lean.targetSeconds);
                readFloat(lean, "throttleCutAtMax", spec.lean.throttleCutAtMax);
                readFloat(lean, "throttleCutStartDeg", spec.lean.throttleCutStartDeg);
                readFloat(lean, "pitchDamping", spec.lean.pitchDamping);
                readFloat(lean, "airLevelHz", spec.lean.airLevelHz);
                readFloat(lean, "pitchMaxTorque", spec.lean.pitchMaxTorque);
                readFloat(lean, "pitchInertia", spec.lean.pitchInertia);
            }

            const auto& aero = section(root, "aero");
            readFloat(aero, "dragCoefficient", spec.aero.dragCoefficient);
            readFloat(aero, "topSpeedKmh", spec.aero.topSpeedKmh);

            const auto& lights = section(root, "lights");
            readString(lights, "form", spec.lightForm);
            readFloat(lights, "pitchDownDeg", spec.lightPitchDownDeg);
            readFloat(lights, "yawDeg", spec.lightYawDeg);
            if (const auto it = lights.find("points"); it != lights.end() && it->is_array()) {
                for (const auto& p : *it) {
                    if (!p.is_array() || p.size() != 3) {
                        throw std::runtime_error("'lights.points' entries must be [x, y, z] in game units");
                    }
                    spec.lightPoints.push_back(Vec3{ p[0].get<float>(), p[1].get<float>(), p[2].get<float>() } * GAME_TO_HAVOK);
                }
            }

            const auto& exhaust = section(root, "exhaust");
            readString(exhaust, "form", spec.exhaustForm);
            if (const auto it = exhaust.find("points"); it != exhaust.end() && it->is_array()) {
                spec.exhaustPoints.clear();
                for (const auto& p : *it) {
                    if (!p.is_array() || p.size() != 3) {
                        throw std::runtime_error("'exhaust.points' entries must be [x, y, z] in game units");
                    }
                    spec.exhaustPoints.push_back(Vec3{ p[0].get<float>(), p[1].get<float>(), p[2].get<float>() } * GAME_TO_HAVOK);
                }
            }

            const auto& seat = section(root, "seat");
            readGamePoint(seat, "driverEye", spec.driverEye);
            readFloat(seat, "leanFollow", spec.seatLeanFollow);
            readGamePoint(seat, "exit", spec.exitPoint);
            if (const auto it = seat.find("doorHandles"); it != seat.end() && it->is_array()) {
                spec.doorHandles.clear();
                for (const auto& p : *it) {
                    if (!p.is_array() || p.size() != 3) {
                        throw std::runtime_error("'seat.doorHandles' entries must be [x, y, z] in game units");
                    }
                    spec.doorHandles.push_back(Vec3{ p[0].get<float>(), p[1].get<float>(), p[2].get<float>() } * GAME_TO_HAVOK);
                }
            }
            readFloat(seat, "interactRadius", spec.interactRadius);

            if (const auto it = root.find("trunk"); it != root.end() && it->is_object()) {
                if (it->find("point") == it->end() || it->find("capacity") == it->end()) {
                    throw std::runtime_error("'trunk' needs \"point\": [x, y, z] in game units and \"capacity\" (carry weight)");
                }
                readGamePoint(*it, "point", spec.trunkPoint);
                readFloat(*it, "capacity", spec.trunkCapacity);
                spec.hasTrunk = spec.trunkCapacity > 0.0f;
            }
        } catch (const json::exception& e) {
            throw std::runtime_error(std::string("JSON value error: ") + e.what());
        }

        if (spec.mass <= 0.0f || spec.tyre.radius <= 0.0f || spec.suspension.restLength <= 0.0f) {
            throw std::runtime_error("mass, tyre.radius and suspension.restLength must be positive");
        }
        return spec;
    }

    VehicleSpec loadVehicleSpecFile(const std::string& path, const std::map<std::string, std::string>& selection)
    {
        std::ifstream file(path);
        if (!file) {
            throw std::runtime_error("cannot open '" + path + "'");
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        return parseVehicleSpec(buffer.str(), selection);
    }
}
