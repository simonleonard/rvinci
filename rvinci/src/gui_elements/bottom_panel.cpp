#include "rvinci/gui_elements/bottom_panel.h"

#include <ros/ros.h>

#include <OgreMaterialManager.h>
#include <OgrePanelOverlayElement.h>
#include <OgreViewport.h>

namespace rvinci {
namespace gui_elements {

namespace {
// da Vinci viewer has a large unusable area at the bottom, so scrubber has to
// be fairly high up
constexpr Ogre::Real kPanelTop = 0.6;
constexpr Ogre::Real kPanelHeight = 0.05;
constexpr Ogre::Real kPanelPadding = 0.05;
constexpr Ogre::Real kScrubberBarHeight = 0.1;
constexpr Ogre::Real kScrubberDotSize = 0.5;
constexpr Ogre::Real kButtonWidth = 0.05;
constexpr Ogre::Real kButtonMargin = 0.01;
constexpr Ogre::Real kButtonPadding = 0.0075;

constexpr Ogre::Real kPanelWidth = 1. - 2. * kPanelPadding;
constexpr Ogre::Real kAbsoluteScrubberBarHeight =
    kPanelHeight * kScrubberBarHeight;
constexpr Ogre::Real kAbsoluteScrubberDotSize = kPanelHeight * kScrubberDotSize;
constexpr Ogre::Real kScrubberBarPadding =
    kButtonWidth + 2 * kButtonMargin + kAbsoluteScrubberDotSize / 2.;
constexpr Ogre::Real kScrubberBarWidth = 1 - 2. * kScrubberBarPadding;
constexpr Ogre::Real kAbsoluteScrubberBarWidth =
    kScrubberBarWidth * kPanelWidth;
constexpr Ogre::Real kScrubberDotZero =
    kScrubberBarPadding - kAbsoluteScrubberDotSize / 2.;

} // namespace

Ogre::OverlayContainer* BottomPanel::create() {
  Ogre::OverlayManager& overlay_manager = Ogre::OverlayManager::getSingleton();

  main_panel_ = dynamic_cast<Ogre::PanelOverlayElement*>(
      overlay_manager.createOverlayElement("Panel",
                                           "NasaInterfaceBottomPanel"));
  main_panel_->setPosition(kPanelPadding, kPanelTop);
  main_panel_->setDimensions(kPanelWidth, kPanelHeight);
  main_panel_->setMaterialName("Template/PartialTransparent");

  createScrubber(overlay_manager);
  createPlayPause(overlay_manager);
  createExecute(overlay_manager);

  return main_panel_;
}
void BottomPanel::createScrubber(Ogre::OverlayManager& overlay_manager) {
  // NOTE Siblings within a container have equal z-order, and ties are resolved
  // by name order. The numbers in the names are chosen to get the correct order
  scrubber_bar_ = dynamic_cast<Ogre::PanelOverlayElement*>(
      overlay_manager.createOverlayElement("Panel", "ScrubberBar1Panel"));
  // Position, but not dimensions, are relative to the containing Panel
  scrubber_bar_->setVerticalAlignment(Ogre::GVA_CENTER);
  scrubber_bar_->setPosition(kScrubberBarPadding,
                             -kAbsoluteScrubberBarHeight / 2.);
  scrubber_bar_->setDimensions(kAbsoluteScrubberBarWidth,
                               kAbsoluteScrubberBarHeight);
  scrubber_bar_->setMaterialName("Template/Blue");
  main_panel_->addChild(scrubber_bar_);

  scrubber_dot_ = dynamic_cast<Ogre::PanelOverlayElement*>(
      overlay_manager.createOverlayElement("Panel", "ScrubberBar2Dot"));
  scrubber_dot_->setVerticalAlignment(Ogre::GVA_CENTER);
  scrubber_dot_->setPosition(kScrubberDotZero, -kAbsoluteScrubberDotSize / 2.);
  scrubber_dot_->setDimensions(kAbsoluteScrubberDotSize,
                               kAbsoluteScrubberDotSize);
  scrubber_dot_->setMaterialName("Template/ScrubberDot");
  Ogre::UserObjectBindings& dot_bindings =
      scrubber_dot_->getUserObjectBindings();
  dot_bindings.setUserAny("rvinci_interaction_mode",
                          Ogre::Any(std::string("slide_horizontal")));
  dot_bindings.setUserAny("rvinci_publish_topic",
                          Ogre::Any(std::string("set_preview_position")));
  dot_bindings.setUserAny("rvinci_slide_limit_lower",
                          Ogre::Any(scrubber_bar_->getLeft()));
  dot_bindings.setUserAny(
      "rvinci_slide_limit_upper",
      Ogre::Any(scrubber_bar_->getLeft() + scrubber_bar_->getWidth()));
  main_panel_->addChild(scrubber_dot_);
}

void BottomPanel::createPlayPause(Ogre::OverlayManager& overlay_manager) {
  play_pause_button_ = dynamic_cast<Ogre::PanelOverlayElement*>(
      overlay_manager.createOverlayElement("Panel", "PlayPauseButton"));
  play_pause_button_->setVerticalAlignment(Ogre::GVA_CENTER);
  play_pause_button_->setPosition(kButtonPadding + kButtonMargin,
                                  kButtonPadding - kButtonWidth / 2.);
  play_pause_button_->setDimensions(kButtonWidth - 2 * kButtonPadding,
                                    kButtonWidth - 2 * kButtonPadding);
  play_pause_button_->setMaterialName("Template/PlayIcon");
  Ogre::UserObjectBindings& play_pause_bindings =
      play_pause_button_->getUserObjectBindings();
  play_pause_bindings.setUserAny("rvinci_interaction_mode",
                                 Ogre::Any(std::string("click")));
  play_pause_bindings.setUserAny("rvinci_publish_topic",
                                 Ogre::Any(std::string("~rvinci_play_pause")));

  main_panel_->addChild(play_pause_button_);
}

void BottomPanel::createExecute(Ogre::OverlayManager& overlay_manager) {
  execute_button_ = dynamic_cast<Ogre::PanelOverlayElement*>(
      overlay_manager.createOverlayElement("Panel", "ExecuteButton"));
  execute_button_->setVerticalAlignment(Ogre::GVA_CENTER);
  execute_button_->setHorizontalAlignment(Ogre::GHA_RIGHT);
  execute_button_->setPosition(-(kButtonPadding + kButtonWidth - kButtonMargin),
                               kButtonPadding - kButtonWidth / 2.);
  execute_button_->setDimensions(kButtonWidth - 2 * kButtonPadding,
                                 kButtonWidth - 2 * kButtonPadding);
  execute_button_->setMaterialName("Template/UploadIcon");
  Ogre::UserObjectBindings& bindings = execute_button_->getUserObjectBindings();
  bindings.setUserAny("rvinci_interaction_mode",
                      Ogre::Any(std::string("click")));
  bindings.setUserAny("rvinci_publish_topic",
                      Ogre::Any(std::string("~rvinci_execute")));

  main_panel_->addChild(execute_button_);
}

void BottomPanel::destroy() {
  Ogre::OverlayManager& overlay_manager = Ogre::OverlayManager::getSingleton();
  if (main_panel_) {
    overlay_manager.destroyOverlayElement(main_panel_);
    main_panel_ = nullptr;
  }

  if (scrubber_bar_) {
    overlay_manager.destroyOverlayElement(scrubber_bar_);
    scrubber_bar_ = nullptr;
  }

  if (scrubber_dot_) {
    overlay_manager.destroyOverlayElement(scrubber_dot_);
    scrubber_dot_ = nullptr;
  }

  if (play_pause_button_) {
    overlay_manager.destroyOverlayElement(play_pause_button_);
    play_pause_button_ = nullptr;
  }

  if (execute_button_) {
    overlay_manager.destroyOverlayElement(execute_button_);
    execute_button_ = nullptr;
  }
}

void BottomPanel::setScrubberPosition(double position) {
  ROS_ASSERT(scrubber_dot_);
  scrubber_dot_->setLeft(kScrubberDotZero +
                         position * kAbsoluteScrubberBarWidth);
}

} // namespace gui_elements
} // namespace rvinci