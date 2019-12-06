#include "rvinci/gui_elements/waypoints_control_panel.h"

#include <ros/ros.h>

#include <OgreFontManager.h>
#include <OgreMaterialManager.h>
#include <OgrePanelOverlayElement.h>
#include <OgreTextAreaOverlayElement.h>
#include <OgreViewport.h>

namespace rvinci {
namespace gui_elements {

namespace {
// da Vinci viewer has a large unusable area at the bottom, so scrubber has to
// be fairly high up
constexpr Ogre::Real kPanelTop = 0.55;
constexpr Ogre::Real kPanelHeight = 0.05;
constexpr Ogre::Real kPanelPadding = 0.05;
constexpr Ogre::Real kButtonWidth = 0.05;
constexpr Ogre::Real kButtonMargin = 0.01;
constexpr Ogre::Real kButtonPadding = 0.0075;
// Relative to container
constexpr Ogre::Real kActionHintTop = 0.1;
constexpr Ogre::Real kActionHintHeight = 0.8;

constexpr Ogre::Real kPanelWidth = 1. - 2. * kPanelPadding;
constexpr Ogre::Real kAbsoluteActionHintTop = kActionHintTop * kPanelHeight;
constexpr Ogre::Real kAbsoluteActionHintHeight =
    kActionHintHeight * kPanelHeight;
} // namespace

Ogre::OverlayContainer* WaypointsControlPanel::create() {
  Ogre::OverlayManager& overlay_manager = Ogre::OverlayManager::getSingleton();

  main_panel_ = dynamic_cast<Ogre::PanelOverlayElement*>(
      overlay_manager.createOverlayElement(
          "Panel", "NasaInterfaceWaypointsControlPanel"));
  main_panel_->setPosition(kPanelPadding, kPanelTop);
  main_panel_->setDimensions(kPanelWidth, kPanelHeight);
  main_panel_->setMaterialName("Template/PartialTransparent");

  action_hint_ = dynamic_cast<Ogre::TextAreaOverlayElement*>(
      overlay_manager.createOverlayElement("TextArea", "NasaActionHint"));
  action_hint_->setPosition(0.5 - kPanelPadding, kAbsoluteActionHintTop);
  action_hint_->setDimensions(kPanelWidth, kPanelHeight);
  action_hint_->setCharHeight(kAbsoluteActionHintHeight);
  // Liberation Sans is the only font installed by default when I tested
  action_hint_->setFontName("Liberation Sans");
  action_hint_->setColour(Ogre::ColourValue(1., 1., 1.));
  action_hint_->setAlignment(Ogre::TextAreaOverlayElement::Center);
  action_hint_->setCaption("");
  main_panel_->addChild(action_hint_);

  main_panel_->addChild(
      add_here_button_.create(overlay_manager, "AddHereButton")
          .atPosition(kButtonPadding + kButtonMargin,
                      kButtonPadding - kButtonWidth / 2.)
          .withDimensions(kButtonWidth - 2 * kButtonPadding,
                          kButtonWidth - 2 * kButtonPadding)
          .withMaterial("Template/AddHereIcon")
          .withClickTopic("~rvinci_add_waypoint_here")
          .done());

  main_panel_->addChild(
      add_at_end_button_.create(overlay_manager, "AddAtEndButton")
          .atPosition(kButtonWidth + 2 * kButtonMargin,
                      kButtonPadding - kButtonWidth / 2.)
          .withDimensions(kButtonWidth - 2 * kButtonPadding,
                          kButtonWidth - 2 * kButtonPadding)
          .withMaterial("Template/AddAtEndIcon")
          .withClickTopic("~rvinci_add_waypoint_at_end")
          .done());

  return main_panel_;
}

void WaypointsControlPanel::destroy() {
  Ogre::OverlayManager& overlay_manager = Ogre::OverlayManager::getSingleton();
  if (main_panel_) {
    overlay_manager.destroyOverlayElement(main_panel_);
    main_panel_ = nullptr;
  }

  if (action_hint_) {
    overlay_manager.destroyOverlayElement(action_hint_);
    action_hint_ = nullptr;
  }
}

} // namespace gui_elements
} // namespace rvinci