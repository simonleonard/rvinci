#include "rvinci/rvinci_gui.h"

#include <ros/console.h>

#include <OgreOverlayManager.h>
#include <OgrePanelOverlayElement.h>

namespace rvinci {

namespace {
constexpr double kScrubberTop = 0.8;
constexpr double kScrubberPadding = 0.02;
} // namespace

RvinciGui::~RvinciGui() {
  if (overlay_) {
    Ogre::OverlayManager::getSingleton().destroy(overlay_);
  }
};

void RvinciGui::initialize() {
  ROS_INFO("Adding the overlay");

  Ogre::OverlayManager& overlay_manager = Ogre::OverlayManager::getSingleton();

  overlay_ = overlay_manager.create("NasaInterface");
  auto panel = dynamic_cast<Ogre::PanelOverlayElement*>(
      overlay_manager.createOverlayElement("Panel", "NasaInterfacePanel"));

  panel->setPosition(kScrubberPadding, kScrubberTop);
  panel->setDimensions(1. - 2 * kScrubberPadding,
                       1. - kScrubberTop - kScrubberPadding);
  panel->setMaterialName("BaseWhite");

  overlay_->add2D(panel);
}

void RvinciGui::show() {
  assert(overlay_);
  overlay_->show();
}

void RvinciGui::hide() {
  assert(overlay_);
  overlay_->hide();
}
} // namespace rvinci