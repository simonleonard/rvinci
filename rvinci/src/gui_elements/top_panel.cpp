#include "rvinci/gui_elements/top_panel.h"

#include <ros/ros.h>

#include <OgreFontManager.h>
#include <OgreMaterialManager.h>
#include <OgrePanelOverlayElement.h>
#include <OgreTextAreaOverlayElement.h>
#include <OgreViewport.h>

namespace rvinci {
namespace gui_elements {

namespace {
constexpr size_t kMaxCharacters = 28; // derived experimentally
constexpr auto kEllipsis = "...";

constexpr Ogre::Real kPanelTop = 0.0;
constexpr Ogre::Real kPanelHeight = 0.05;
constexpr Ogre::Real kPanelPadding = 0.05;

// Relative to container
constexpr Ogre::Real kWaypointNameTop = 0.1;
constexpr Ogre::Real kWaypointNameHeight = 0.8;

// Computed
constexpr Ogre::Real kPanelWidth = 1. - 2. * kPanelPadding;
constexpr Ogre::Real kAbsoluteWaypointNameTop = kWaypointNameTop * kPanelHeight;
constexpr Ogre::Real kAbsoluteWaypointNameHeight =
    kWaypointNameHeight * kPanelHeight;
} // namespace

Ogre::OverlayContainer* TopPanel::create() {
  Ogre::OverlayManager& overlay_manager = Ogre::OverlayManager::getSingleton();

  main_panel_ = dynamic_cast<Ogre::PanelOverlayElement*>(
      overlay_manager.createOverlayElement("Panel", "NasaInterfaceTopPanel"));
  main_panel_->setPosition(kPanelPadding, kPanelTop);
  main_panel_->setDimensions(kPanelWidth, kPanelHeight);
  main_panel_->setMaterialName("Template/PartialTransparent");

  waypoint_name_ = dynamic_cast<Ogre::TextAreaOverlayElement*>(
      overlay_manager.createOverlayElement("TextArea", "WaypointLabel"));
  waypoint_name_->setPosition(0.5 - kPanelPadding, kAbsoluteWaypointNameTop);
  waypoint_name_->setDimensions(kPanelWidth, kPanelHeight);
  waypoint_name_->setCharHeight(kAbsoluteWaypointNameHeight);
  // Liberation Sans is the only font installed by default when I tested
  waypoint_name_->setFontName("Liberation Sans");
  waypoint_name_->setColour(Ogre::ColourValue(1., 1., 1.));
  waypoint_name_->setAlignment(Ogre::TextAreaOverlayElement::Center);
  waypoint_name_->setCaption("Hello, World!");
  main_panel_->addChild(waypoint_name_);

  return main_panel_;
}

void TopPanel::destroy() {
  Ogre::OverlayManager& overlay_manager = Ogre::OverlayManager::getSingleton();
  if (main_panel_) {
    overlay_manager.destroyOverlayElement(main_panel_);
    main_panel_ = nullptr;
  }

  if (waypoint_name_) {
    overlay_manager.destroyOverlayElement(waypoint_name_);
    waypoint_name_ = nullptr;
  }
}
void TopPanel::setWaypointName(std::string name) {
  // Truncate name if needed
  // TODO Make this encoding-safe (with Qt QFontMetricsF::elidedText?)
  if (name.length() > kMaxCharacters) {
    // Count ellipsis as 1 char because it approximately has the width of one
    const size_t extra_chars = strlen(kEllipsis) - 1;
    name.resize(kMaxCharacters + extra_chars);
    name.replace(name.end() - strlen(kEllipsis), name.end(), kEllipsis);
  }

  if (waypoint_name_) {
    waypoint_name_->setCaption(name);
  }
}

} // namespace gui_elements
} // namespace rvinci