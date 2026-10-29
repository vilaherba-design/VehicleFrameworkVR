#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "model/MenuPointer.h"
#include "vrui/UIButtonPanel.h"
#include "vrui/UIContainer.h"
#include "vrui/UIModAdapter.h"

namespace vfvr::ui
{
    // The vehicle and wheel menus as a panel of buttons ahead of the player (f4cf vrui panels, drawn by the overlay renderer: no Scaleform): two
    // columns of twelve, "Previous"/"Next" past 24 entries and "Cancel" in a row of their own under them.
    //
    // The panel is placed once when it opens, level and square to where the head faces, and then stays put in the player's room (it moves with
    // the player and the vehicle, not with the head), so leaning or reaching towards it does not push it away. It only comes round in front again
    // when the head turns further than fMenuFollowAngle from it.
    //
    // Either hand can choose: touching a button with the index fingertip (FRIK) or pointing the controller at it (a laser beam shows where) and
    // pulling the trigger. Only one button can be under a finger or a beam at a time, the one under it lights up, and a button presses once per
    // touch or pull. vrui's own press detection is not used: it tests a sphere as wide as the button around its centre, which with rows this
    // close took in three or four buttons at once.
    class GridMenu
    {
    public:
        static constexpr int NONE = -1; // nothing chosen yet
        static constexpr int CANCELLED = -2;

        struct Settings
        {
            float distance = 65.0f; // ahead of the eyes, game units
            float height = -12.0f; // of the panel's centre above the eyes (negative = below)
            float side = -7.0f; // of the panel's centre to the right of the eyes (negative = left), game units
            float scale = 1.0f; // apparent size: 1 is the default, at any distance
            bool laser = true; // aim with the controllers and press with the trigger
            bool bothHands = true; // false: the primary hand only
            float followAngle = 50.0f; // degrees the head can turn away before the panel comes round
        };

        // [game] Shows the menu: `title` above the buttons (lines split at '\n').
        bool open(const std::string& title, const std::vector<std::string>& labels, const Settings& settings);
        // [game] Removes it (also when the result has been taken).
        void close();

        bool isOpen() const
        {
            return _root != nullptr;
        }

        // [game] Every frame, open or not: drives the panel (placement, pointing, presses, drawing). Returns the chosen entry once, CANCELLED, or NONE.
        int update();

    private:
        // vrui asks for an interaction bone; the menu tests the fingers itself, so it gets one far away (and vrui never points the hands).
        class Adapter : public f4cf::vrui::UIModAdapter
        {
        public:
            RE::NiPoint3 getInteractionBoneWorldPosition() override
            {
                return { 0.0f, 0.0f, -100000.0f };
            }

            void setInteractionHandPointing(bool, bool) override
            {}
        };

        enum class Action
        {
            Entry,
            Previous,
            Next,
            Cancel
        };

        enum class Look
        {
            Normal,
            Hovered,
            Pressed
        };

        struct Button
        {
            std::shared_ptr<f4cf::vrui::UIButtonPanel> panel;
            std::string name;
            Action action = Action::Entry;
            int entry = -1;
            Look look = Look::Normal;
        };

        struct Hand
        {
            model::MenuTouch touch;
            model::MenuTrigger trigger;
            int hover = -1;
            bool pointing = false; // FRIK pointing pose set on this hand
            bool laserOn = false; // the beam is drawn (it meets the panel)
            RE::NiPoint3 laserFrom;
            RE::NiPoint3 laserTo;
        };

        void build();
        bool placeAnchor();
        void followHead();
        void pointHands(int& pressed, int& pressedBy);
        void setPointing(int hand, bool point);
        void drawLasers() const;
        void setLook(Button& button, Look look);
        void suppressTriggers(bool suppress);

        Adapter _adapter;
        std::shared_ptr<f4cf::vrui::UIContainer> _root;
        std::vector<Button> _buttons;
        std::vector<model::MenuRect> _rects;
        std::array<Hand, 2> _hands{}; // 0 primary, 1 offhand
        std::vector<std::string> _labels;
        std::string _title;
        Settings _settings;
        int _page = 0;
        int _result = NONE;
        int _turnPage = 0; // a page button pressed: rebuilt on the next frame
        int _flashFrames = 0; // the pressed button stays lit this many frames before the menu closes
        bool _ownAnchor = false; // placed on our own node in the room (else on the HMD node, following the head)
        RE::NiTransform _anchorInRoom; // the panel's frame in the room node's space
        bool _triggersSuppressed = false;
        bool _releaseTriggersWhenUp = false; // closed by a trigger: the game gets the triggers back once they are let go
    };
}
