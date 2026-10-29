#include "ui/GridMenu.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>
#include <optional>

#include "api/FRIKApiV2.h"
#include "common/MatrixUtils.h"
#include "f4vr/PlayerNodes.h"
#include "model/MenuGrid.h"
#include "physics/HavokBridge.h"
#include "telemetry/HangWatchdog.h"
#include "render/PrimitiveDrawRenderer.h"
#include "vrcf/VRControllersHaptic.h"
#include "vrcf/VRControllersManager.h"
#include "vrcf/VRControllersSuppressor.h"
#include "vrui/UIManager.h"
#include "vrui/UIPanelStyle.h"
#include "vrui/UITextPanel.h"
#include "vrui/UIUtils.h"

namespace vfvr::ui
{
    namespace
    {
        using common::MatrixUtils;

        constexpr auto FRIK_POSE_TAG = "VehicleFrameworkVR_Menu";
        constexpr auto SUPPRESS_KEY = "VehicleFrameworkVR_Menu";

        // Layout in vrui units, before the panel's scale: buttons, the space between them, the size of their text and of the title's.
        constexpr float BUTTON_WIDTH = 9.0f;
        constexpr float BUTTON_HEIGHT = 1.5f;
        constexpr float BAR_BUTTON_WIDTH = BUTTON_WIDTH * 0.6f;
        constexpr float CELL_GAP = 0.9f; // between rows, and between the buttons of the bottom row (about 3 cm at the default size)
        constexpr float COLUMN_GAP = 1.5f; // between the two columns
        constexpr float SECTION_GAP = 1.4f; // between the title, the grid and the bottom row
        constexpr float BUTTON_TEXT = 0.55f; // capitals' height; vrui's default for a text button is 0.30
        constexpr float TITLE_TEXT = 0.5f;

        // The scale that makes the panel look its default size (text about twice as tall as in 0.1.82) at a given distance: 2.4 at 65 units.
        constexpr float SCALE_PER_UNIT_OF_DISTANCE = 2.4f / 65.0f;

        // Fingertip near enough to the panel for the hand to take the pointing pose (game units).
        constexpr float POINT_POSE_FRONT = 20.0f;
        constexpr float POINT_POSE_MARGIN = 8.0f;

        constexpr int FLASH_FRAMES = 6; // a pressed button stays lit for this many frames before the menu closes
        constexpr float LASER_IDLE_LENGTH = 40.0f; // the beam's length when it does not meet the panel
        constexpr float LASER_HALF_WIDTH = 0.12f;
        constexpr float LASER_DOT = 0.5f;

        constexpr std::array HANDS = { vrcf::Hand::Primary, vrcf::Hand::Offhand };

        render::PrimitiveDrawRenderer& laserLayer()
        {
            static render::PrimitiveDrawRenderer instance("VehicleFrameworkVR menu laser", render::DRAW_ORDER_DEFAULT, false);
            return instance;
        }

        // Our own node for the panel to hang from: kept out of the scene graph, its world transform is written every frame from the room node,
        // so the panel stays where it was put instead of following the head. Built once and kept for the life of the game (vrui holds
        // NiPointers to its attach node, so it must never be freed). The engine code it needs is looked up by address library ID first: a
        // missing ID would end the game.
        RE::NiNode* menuAnchorNode()
        {
            static RE::NiNode* node = []() -> RE::NiNode* {
                for (const std::uint64_t id : { 390064, 1048271, 1031341, 146310, 277807, 1414337, 343176, 652767 }) {
                    if (!physics::hasAddressId(id)) {
                        logger::warn("Menu: address library ID {} missing: the panel follows the head instead of staying put", id);
                        return nullptr;
                    }
                }
                auto* created = new RE::NiNode(0);
                created->IncRefCount();
                created->name = "VFVR_MenuAnchor";
                return created;
            }();
            return node;
        }

        RE::NiPoint3 row(const RE::NiMatrix3& m, const int i)
        {
            return { m.entry[i][0], m.entry[i][1], m.entry[i][2] };
        }

        // Where the head faces, level: its forward axis without the vertical part; looking straight down or up, its up axis instead.
        std::optional<RE::NiPoint3> levelForward(const RE::NiAVObject* head)
        {
            for (const int axis : { 1, 2 }) {
                const RE::NiPoint3 v = row(head->world.rotate, axis);
                const float length = std::hypot(v.x, v.y);
                if (length > 0.2f) {
                    return RE::NiPoint3(v.x / length, v.y / length, 0.0f);
                }
            }
            return std::nullopt;
        }

        // A level frame at `position` facing `forward` (rows: right, forward, up - the codebase's rotation convention).
        RE::NiTransform levelFrame(const RE::NiPoint3& position, const RE::NiPoint3& forward)
        {
            RE::NiTransform t;
            t.translate = position;
            t.scale = 1.0f;
            t.rotate = MatrixUtils::getIdentityMatrix();
            t.rotate.entry[0][0] = forward.y;
            t.rotate.entry[0][1] = -forward.x;
            t.rotate.entry[0][2] = 0.0f;
            t.rotate.entry[1][0] = forward.x;
            t.rotate.entry[1][1] = forward.y;
            t.rotate.entry[1][2] = 0.0f;
            t.rotate.entry[2][0] = 0.0f;
            t.rotate.entry[2][1] = 0.0f;
            t.rotate.entry[2][2] = 1.0f;
            return t;
        }

        frik::api::FRIKApiV2::Hand frikHand(const int hand)
        {
            return hand == 0 ? frik::api::FRIKApiV2::Hand::Primary : frik::api::FRIKApiV2::Hand::Offhand;
        }

        RE::NiNode* wandNode(const int hand)
        {
            const auto* nodes = f4vr::getVRPlayerNodes();
            if (!nodes) {
                return nullptr;
            }
            return hand == 0 ? nodes->primaryWandNode : nodes->secondaryWandNode;
        }
    }

    bool GridMenu::open(const std::string& title, const std::vector<std::string>& labels, const Settings& settings)
    {
        if (!f4cf::vrui::g_uiManager || labels.empty()) {
            return false;
        }
        const HangStep step("menu panel: opening");
        close();
        _title = title;
        _labels = labels;
        _settings = settings;
        _page = 0;
        _result = NONE;
        _turnPage = 0;
        _flashFrames = 0;
        for (auto& hand : _hands) {
            hand.touch = {};
            hand.trigger = {}; // starts as held: a trigger already pulled presses nothing until let go
            hand.hover = -1;
            hand.laserOn = false;
        }
        if (!placeAnchor()) {
            return false;
        }
        build();
        if (_root && _settings.laser) {
            suppressTriggers(true);
            _releaseTriggersWhenUp = false;
        }
        return _root != nullptr;
    }

    void GridMenu::close()
    {
        if (_root && f4cf::vrui::g_uiManager) {
            f4cf::vrui::g_uiManager->detachElement(_root, true);
        }
        const bool wasOpen = _root != nullptr;
        _root.reset();
        _buttons.clear();
        _rects.clear();
        for (int hand = 0; hand < 2; ++hand) {
            setPointing(hand, false);
            _hands[static_cast<std::size_t>(hand)].laserOn = false;
        }
        if (wasOpen) {
            laserLayer().publish(render::PrimitiveDraw{});
        }
        // The game gets the triggers back once they are let go, so the pull that chose does not also fire the weapon.
        if (_triggersSuppressed) {
            _releaseTriggersWhenUp = true;
        }
    }

    // Puts the panel's frame level in front of the eyes. On our own node it is kept in the room node's space; without it, on the HMD node.
    bool GridMenu::placeAnchor()
    {
        const auto* nodes = f4vr::getVRPlayerNodes();
        RE::NiNode* own = menuAnchorNode();
        if (own && nodes && nodes->hmdNode && nodes->roomNode) {
            if (const auto forward = levelForward(nodes->hmdNode)) {
                const RE::NiTransform frame = levelFrame(nodes->hmdNode->world.translate, *forward);
                _anchorInRoom = MatrixUtils::worldToLocalTransform(nodes->roomNode->world, frame);
                own->world = frame;
                _ownAnchor = true;
                return true;
            }
        }
        _ownAnchor = false;
        return f4cf::vrui::UIUtils::getHMDAttachNode() != nullptr;
    }

    // The panel moves with the room (the player walking, the vehicle driving, snap turns) and comes round in front again when the head has turned
    // too far from it.
    void GridMenu::followHead()
    {
        if (!_ownAnchor) {
            return;
        }
        const auto* nodes = f4vr::getVRPlayerNodes();
        RE::NiNode* own = menuAnchorNode();
        if (!own || !nodes || !nodes->roomNode || !nodes->hmdNode) {
            return;
        }
        own->world = MatrixUtils::localToWorldTransform(nodes->roomNode->world, _anchorInRoom);
        const auto forward = levelForward(nodes->hmdNode);
        if (!forward) {
            return;
        }
        const RE::NiPoint3 panelForward = row(own->world.rotate, 1);
        const float cosine = std::clamp(forward->x * panelForward.x + forward->y * panelForward.y, -1.0f, 1.0f);
        if (std::acos(cosine) * 180.0f / std::numbers::pi_v<float> > _settings.followAngle) {
            placeAnchor();
            for (auto& hand : _hands) {
                hand.touch = {};
            }
            logger::info("Menu panel: the head turned away, the panel comes round in front again");
        }
    }

    void GridMenu::build()
    {
        using namespace f4cf::vrui;
        using model::MenuGrid;
        if (_root) {
            g_uiManager->detachElement(_root, true);
            _root.reset();
        }
        _buttons.clear();
        _rects.clear();
        const int count = static_cast<int>(_labels.size());
        const int pages = MenuGrid::pages(count);
        _page = std::clamp(_page, 0, pages - 1);
        const int rows = MenuGrid::rowsOn(count, _page);
        const int columns = MenuGrid::countOn(count, _page) > MenuGrid::ROWS ? 2 : 1;
        const float scale = _settings.scale * _settings.distance * SCALE_PER_UNIT_OF_DISTANCE;

        std::vector<std::string> titleLines;
        std::string heading = pages > 1 ? std::format("{} ({}/{})", _title, _page + 1, pages) : _title;
        for (std::size_t start = 0; start <= heading.size();) {
            const auto end = heading.find('\n', start);
            const auto line = heading.substr(start, end == std::string::npos ? std::string::npos : end - start);
            if (!line.empty()) {
                titleLines.push_back(line);
            }
            if (end == std::string::npos) {
                break;
            }
            start = end + 1;
        }

        // Everything is placed by hand around the panel's centre (vrui's nested layouts centre a column on its top edge, which put the bottom
        // row over the grid's fourth row): the title, then the grid, then a row of its own for the page buttons and "Cancel".
        const float gridWidth = static_cast<float>(columns) * BUTTON_WIDTH + static_cast<float>(columns - 1) * COLUMN_GAP;
        const float titleHeight = titleLines.empty() ? 0.0f : static_cast<float>(titleLines.size()) * TITLE_TEXT * 1.6f + 0.5f;
        const float gridHeight = static_cast<float>(rows) * BUTTON_HEIGHT + static_cast<float>(rows - 1) * CELL_GAP;
        const float totalHeight = titleHeight + (titleHeight > 0.0f ? SECTION_GAP : 0.0f) + gridHeight + SECTION_GAP + BUTTON_HEIGHT;
        const float top = totalHeight * 0.5f;

        auto root = std::make_shared<UIContainer>("VFVR_Menu", UIContainerLayout::Manual, 0.0f, scale);
        const auto place = [&](const std::shared_ptr<UIElement>& element, const float x, const float z) {
            element->setPosition(x * scale, 0.0f, z * scale);
            root->addElement(element);
        };
        const auto addButton =
            [&](const std::string& name, const std::string& label, const float width, const float x, const float z, const Action action, const int entry, const bool enabled) {
                auto button = std::make_shared<UIButtonPanel>(name, width, BUTTON_HEIGHT);
                button->setMiddleText(label);
                button->setTextOnlyHeight(BUTTON_TEXT);
                button->setDisabled(!enabled);
                place(button, x, z);
                _buttons.push_back(Button{ .panel = button, .name = name, .action = action, .entry = entry });
                _rects.push_back(
                    model::MenuRect{ .x = x * scale, .z = z * scale, .halfWidth = width * 0.5f * scale, .halfHeight = BUTTON_HEIGHT * 0.5f * scale, .enabled = enabled });
            };

        if (!titleLines.empty()) {
            auto title = std::make_shared<UITextPanel>("VFVR_MenuTitle", (std::max)(gridWidth, 3.0f * BAR_BUTTON_WIDTH + 2.0f * CELL_GAP), titleHeight);
            title->setStyle(F4VR_PANEL_STYLE);
            title->setTextHeight(TITLE_TEXT);
            title->setContent([titleLines](std::vector<TextRow>& lines) {
                for (const auto& line : titleLines) {
                    lines.push_back({ .text = line });
                }
            });
            place(title, 0.0f, top - titleHeight * 0.5f);
        }

        // Two columns, filled top to bottom: entry i of the page goes to column i / 12, row i % 12.
        const float gridTop = top - titleHeight - (titleHeight > 0.0f ? SECTION_GAP : 0.0f);
        for (int column = 0; column < columns; ++column) {
            for (int r = 0; r < rows; ++r) {
                const int entry = MenuGrid::entryAt(count, _page, column, r);
                if (entry < 0) {
                    break;
                }
                const float x = -gridWidth * 0.5f + BUTTON_WIDTH * 0.5f + static_cast<float>(column) * (BUTTON_WIDTH + COLUMN_GAP);
                const float z = gridTop - BUTTON_HEIGHT * 0.5f - static_cast<float>(r) * (BUTTON_HEIGHT + CELL_GAP);
                addButton(std::format("VFVR_MenuItem{}", entry), _labels[static_cast<std::size_t>(entry)], BUTTON_WIDTH, x, z, Action::Entry, entry, true);
            }
        }

        // The bottom row: "< Previous" and "Next >" when there are pages, and "Cancel", centred under the grid.
        const float barZ = -top + BUTTON_HEIGHT * 0.5f;
        const int barCount = pages > 1 ? 3 : 1;
        float barX = -(static_cast<float>(barCount) * BAR_BUTTON_WIDTH + static_cast<float>(barCount - 1) * CELL_GAP) * 0.5f + BAR_BUTTON_WIDTH * 0.5f;
        if (pages > 1) {
            addButton("VFVR_MenuPrevious", "< Previous", BAR_BUTTON_WIDTH, barX, barZ, Action::Previous, -1, _page > 0);
            barX += BAR_BUTTON_WIDTH + CELL_GAP;
        }
        addButton("VFVR_MenuCancel", "Cancel", BAR_BUTTON_WIDTH, barX, barZ, Action::Cancel, -1, true);
        if (pages > 1) {
            barX += BAR_BUTTON_WIDTH + CELL_GAP;
            addButton("VFVR_MenuNext", "Next >", BAR_BUTTON_WIDTH, barX, barZ, Action::Next, -1, _page + 1 < pages);
        }

        // The panel's centre `distance` ahead of the eyes, `side` to their right (negative = left) and `height` above them, square to where the
        // head faced when it opened.
        root->setPosition(_settings.side, _settings.distance, _settings.height);
        RE::NiNode* attach = _ownAnchor ? menuAnchorNode() : UIUtils::getHMDAttachNode();
        if (!attach) {
            return;
        }
        g_uiManager->attachElement(root, attach);
        _root = root;
        logger::info("Menu panel: {} buttons, scale {:.2f}, {:.0f} units ahead, {}",
            _buttons.size(),
            scale,
            _settings.distance,
            _ownAnchor ? "placed in the room" : "following the head (no own anchor node)");
    }

    void GridMenu::setLook(Button& button, const Look look)
    {
        if (button.look == look) {
            return;
        }
        button.look = look;
        auto style = f4cf::vrui::F4VR_BUTTON_STYLE;
        if (look == Look::Hovered) {
            style.background = render::Color::rgba(10, 250, 120, 80);
            style.color = render::Color::rgba(225, 255, 235);
            style.borderThicknessUnits = 0.14f;
        } else if (look == Look::Pressed) {
            style.background = render::Color::rgba(10, 250, 120, 230);
            style.color = render::Color::rgba(5, 25, 12);
            style.borderColor = render::Color::rgba(225, 255, 235);
            style.borderThicknessUnits = 0.14f;
        }
        button.panel->setStyle(style);
    }

    void GridMenu::setPointing(const int hand, const bool point)
    {
        auto& state = _hands[static_cast<std::size_t>(hand)];
        if (state.pointing == point) {
            return;
        }
        state.pointing = point;
        using frik::api::FRIKApiV2;
        if (!FRIKApiV2::inst) {
            return;
        }
        if (point) {
            FRIKApiV2::inst->setHandPose(FRIK_POSE_TAG, frikHand(hand), FRIKApiV2::HandPoseKind::Pointing, FRIKApiV2::HAND_POSE_PRIORITY_DEFAULT);
        } else {
            FRIKApiV2::inst->clearHandPose(FRIK_POSE_TAG, frikHand(hand));
        }
    }

    void GridMenu::suppressTriggers(const bool suppress)
    {
        if (_triggersSuppressed == suppress) {
            return;
        }
        _triggersSuppressed = suppress;
        for (const auto hand : HANDS) {
            vrcf::VRControllersSuppress.setSuppressed(SUPPRESS_KEY, hand, vr::k_EButton_SteamVR_Trigger, suppress);
        }
    }

    // Each hand: the button under its fingertip, or else under its beam. A push of the fingertip through a button, or a pull of the trigger
    // while the beam is on one, presses it; only the first press of the frame counts.
    void GridMenu::pointHands(int& pressed, int& pressedBy)
    {
        using frik::api::FRIKApiV2;
        RE::NiNode* attach = _ownAnchor ? menuAnchorNode() : f4cf::vrui::UIUtils::getHMDAttachNode();
        if (!attach) {
            return;
        }
        const RE::NiTransform& frame = attach->world;
        const float planeY = _settings.distance;
        float halfWidth = 0.0f;
        float halfHeight = 0.0f;
        for (const auto& r : _rects) {
            halfWidth = (std::max)(halfWidth, std::abs(r.x) + r.halfWidth);
            halfHeight = (std::max)(halfHeight, std::abs(r.z) + r.halfHeight);
        }
        const bool frik = FRIKApiV2::inst && FRIKApiV2::inst->isSkeletonReady();

        for (int hand = 0; hand < 2; ++hand) {
            auto& state = _hands[static_cast<std::size_t>(hand)];
            state.laserOn = false;
            if (hand == 1 && !_settings.bothHands) {
                state.hover = -1;
                setPointing(hand, false);
                continue;
            }
            int hover = -1;
            int pressedNow = -1;

            // The fingertip: on the button under it while it is within a few centimetres of the panel's plane.
            bool nearPanel = false;
            if (frik) {
                const RE::NiPoint3 tip = MatrixUtils::worldToLocalPoint(frame, FRIKApiV2::inst->getIndexFingerTipPosition(frikHand(hand)));
                const float x = tip.x;
                const float z = tip.z - _settings.height;
                const float depth = tip.y - planeY;
                const int under = model::hitMenuRect(_rects, x, z);
                nearPanel =
                    depth > -POINT_POSE_FRONT && depth < state.touch.hoverBehind && std::abs(x) < halfWidth + POINT_POSE_MARGIN && std::abs(z) < halfHeight + POINT_POSE_MARGIN;
                hover = model::MenuTouch::hovered(under, depth, state.touch.hoverFront, state.touch.hoverBehind);
                pressedNow = state.touch.update(under, depth);
            }
            setPointing(hand, nearPanel);

            // The beam from the controller, along its forward axis, while the fingertip is not on a button.
            const bool triggerPulled = state.trigger.update(vrcf::VRControllers.getAxisValue(HANDS[static_cast<std::size_t>(hand)], vrcf::Axis::Trigger).x);
            if (_settings.laser) {
                if (const RE::NiNode* wand = wandNode(hand)) {
                    const RE::NiPoint3 from = wand->world.translate;
                    const RE::NiPoint3 direction = row(wand->world.rotate, 1);
                    const RE::NiPoint3 o = MatrixUtils::worldToLocalPoint(frame, from);
                    const RE::NiPoint3 d = MatrixUtils::worldToLocalPoint(frame, from + direction) - o;
                    float x = 0.0f;
                    float z = 0.0f;
                    const bool meets = !nearPanel && model::rayOnMenuPlane(o.x, o.y, o.z, d.x, d.y, d.z, planeY, x, z) && std::abs(x) < halfWidth + POINT_POSE_MARGIN &&
                                       std::abs(z - _settings.height) < halfHeight + POINT_POSE_MARGIN;
                    if (meets) {
                        const int under = model::hitMenuRect(_rects, x, z - _settings.height);
                        if (hover < 0) {
                            hover = under;
                        }
                        if (pressedNow < 0 && triggerPulled && under >= 0) {
                            pressedNow = under;
                        }
                        state.laserOn = true;
                        state.laserFrom = from;
                        state.laserTo = MatrixUtils::localToWorldPoint(frame, RE::NiPoint3(x, planeY - 0.2f, z));
                    } else if (!nearPanel) {
                        state.laserOn = true;
                        state.laserFrom = from;
                        state.laserTo = from + direction * LASER_IDLE_LENGTH;
                    }
                }
            }

            if (hover != state.hover && hover >= 0) {
                vrcf::VRHaptics.trigger(HANDS[static_cast<std::size_t>(hand)], vrcf::HapticPattern::Tick, 0.6f);
            }
            state.hover = hover;
            if (pressedNow >= 0 && pressed < 0) {
                pressed = pressedNow;
                pressedBy = hand;
            }
        }
    }

    void GridMenu::drawLasers() const
    {
        render::PrimitiveDraw frame;
        const auto* nodes = f4vr::getVRPlayerNodes();
        const RE::NiPoint3 viewer = nodes && nodes->hmdNode ? nodes->hmdNode->world.translate : RE::NiPoint3{};
        frame.viewerPosition = viewer;
        RE::NiNode* attach = _ownAnchor ? menuAnchorNode() : f4cf::vrui::UIUtils::getHMDAttachNode();
        for (const auto& hand : _hands) {
            if (!hand.laserOn) {
                continue;
            }
            const RE::NiPoint3 along = hand.laserTo - hand.laserFrom;
            const RE::NiPoint3 side = MatrixUtils::vec3Cross(along, viewer - hand.laserFrom);
            RE::NiPoint3 offset;
            if (!MatrixUtils::tryVec3Norm(side, offset)) {
                continue;
            }
            offset = offset * LASER_HALF_WIDTH;
            const bool onButton = hand.hover >= 0;
            const auto color = onButton ? render::Color::rgba(120, 255, 170, 220) : render::Color::rgba(10, 250, 120, 110);
            frame.addQuad(hand.laserFrom - offset, hand.laserTo - offset, hand.laserTo + offset, hand.laserFrom + offset, color);
            if (attach && MatrixUtils::vec3Len(along) < LASER_IDLE_LENGTH - 0.5f) {
                // The dot where the beam meets the panel, flat on it.
                const RE::NiPoint3 right = row(attach->world.rotate, 0) * LASER_DOT;
                const RE::NiPoint3 up = row(attach->world.rotate, 2) * LASER_DOT;
                frame.addQuad(hand.laserTo - right + up, hand.laserTo + right + up, hand.laserTo + right - up, hand.laserTo - right - up, render::Color::rgba(225, 255, 235, 255));
            }
        }
        laserLayer().publish(std::move(frame));
    }

    int GridMenu::update()
    {
        // Closed by a trigger: the game gets the triggers back once both are let go.
        if (!_root && _releaseTriggersWhenUp) {
            const bool held = std::ranges::any_of(HANDS, [](const vrcf::Hand hand) {
                return vrcf::VRControllers.getAxisValue(hand, vrcf::Axis::Trigger).x > 0.3f;
            });
            if (!held) {
                _releaseTriggersWhenUp = false;
                suppressTriggers(false);
            }
        }
        if (_root) {
            const HangStep step("menu panel: following the head");
            followHead();
        }
        if (f4cf::vrui::g_uiManager) {
            const HangStep step("menu panel: UI manager frame (layout, drawing)");
            f4cf::vrui::g_uiManager->onFrameUpdate(&_adapter);
        }
        if (!_root) {
            return NONE;
        }

        // A pressed button flashes for a few frames, then the menu closes with its result.
        if (_flashFrames > 0) {
            if (--_flashFrames == 0) {
                const int result = _result;
                _result = NONE;
                close();
                return result;
            }
            drawLasers();
            return NONE;
        }

        int pressed = -1;
        int pressedBy = -1;
        {
            const HangStep step("menu panel: fingers and beams");
            pointHands(pressed, pressedBy);
        }
        for (std::size_t i = 0; i < _buttons.size(); ++i) {
            const bool hovered = std::ranges::any_of(_hands, [i](const Hand& hand) {
                return hand.hover == static_cast<int>(i);
            });
            setLook(_buttons[i], hovered ? Look::Hovered : Look::Normal);
        }

        if (pressed >= 0 && static_cast<std::size_t>(pressed) < _buttons.size()) {
            auto& button = _buttons[static_cast<std::size_t>(pressed)];
            logger::info("Menu panel: '{}' pressed with the {} hand", button.name, pressedBy == 0 ? "primary" : "other");
            vrcf::VRHaptics.trigger(HANDS[static_cast<std::size_t>(pressedBy)], vrcf::HapticPattern::Click);
            switch (button.action) {
            case Action::Entry:
                _result = button.entry;
                break;
            case Action::Cancel:
                _result = CANCELLED;
                break;
            case Action::Previous:
                _turnPage = -1;
                break;
            case Action::Next:
                _turnPage = 1;
                break;
            }
            if (_result != NONE) {
                setLook(button, Look::Pressed);
                _flashFrames = FLASH_FRAMES;
            }
        }
        if (_turnPage != 0) {
            _page += _turnPage;
            _turnPage = 0;
            for (auto& hand : _hands) {
                hand.touch = {};
            }
            const HangStep step("menu panel: turning the page");
            build();
        }
        const HangStep step("menu panel: drawing the beams");
        drawLasers();
        return NONE;
    }
}
