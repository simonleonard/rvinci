#include "rvinci/rvinci_gui.h"

#include <ros/console.h>

#include <OgreOverlayManager.h>
#include <OgrePanelOverlayElement.h>

namespace rvinci {

RvinciGui::~RvinciGui() {
  if (overlay_) {
    Ogre::OverlayManager::getSingleton().destroy(overlay_);
  }
};

void RvinciGui::initialize() {
  if (overlay_) return;

  ROS_INFO("Adding the overlay");

  Ogre::OverlayManager& overlay_manager = Ogre::OverlayManager::getSingleton();

  overlay_ = overlay_manager.create("NasaInterface");

  overlay_->add2D(bottom_panel_.create());
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