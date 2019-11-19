#include "rvinci/gui_elements/bottom_panel.h"

#include <OgreMaterialManager.h>
#include <OgrePanelOverlayElement.h>
#include <OgreViewport.h>

namespace rvinci {
namespace gui_elements {

namespace {
constexpr Ogre::Real kScrubberHeight = 0.05;
constexpr Ogre::Real kScrubberPadding = 0.01;
constexpr Ogre::Real kScrubberBarHeight = 0.1;

constexpr Ogre::Real kScrubberWidth = 1. - 2. * kScrubberPadding;
constexpr Ogre::Real kScrubberTop = 1 - kScrubberHeight - kScrubberPadding;
constexpr Ogre::Real kAbsoluteScrubberBarHeight =
    kScrubberHeight * kScrubberBarHeight;

} // namespace

Ogre::OverlayContainer* BottomPanel::create() {
  Ogre::OverlayManager& overlay_manager = Ogre::OverlayManager::getSingleton();

  auto panel = dynamic_cast<Ogre::PanelOverlayElement*>(
      overlay_manager.createOverlayElement("Panel", "NasaInterfacePanel"));
  panel->setPosition(kScrubberPadding, kScrubberTop);
  panel->setDimensions(kScrubberWidth, kScrubberHeight);

  auto bar = dynamic_cast<Ogre::PanelOverlayElement*>(
      overlay_manager.createOverlayElement("Panel", "ScrubberBarPanel"));
  // Position, but not dimensions, are relative to the containing Panel
  bar->setVerticalAlignment(Ogre::GVA_CENTER);
  bar->setPosition(0., -kAbsoluteScrubberBarHeight / 2.);
  bar->setDimensions(kScrubberWidth, kAbsoluteScrubberBarHeight);
  bar->setMaterialName("Template/Blue");
  bar->show();
  panel->addChild(bar);

  return panel;
}

} // namespace gui_elements
} // namespace rvinci