#include "rvinci/gui_elements/preview_panel.h"

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

Ogre::OverlayContainer* PreviewPanel::create() {
  Ogre::OverlayManager& overlay_manager = Ogre::OverlayManager::getSingleton();

  main_panel_ = dynamic_cast<Ogre::PanelOverlayElement*>(
      overlay_manager.createOverlayElement("Panel",
                                           "NasaInterfacePreviewPanel"));
  main_panel_->setPosition(kPanelPadding, kPanelTop);
  main_panel_->setDimensions(kPanelWidth, kPanelHeight);
  main_panel_->setMaterialName("Template/PartialTransparent");

  createScrubber(overlay_manager);
  //  main_panel_->addChild()

  createPlayPause(overlay_manager);
  createExecute(overlay_manager);

  return main_panel_;
}
void PreviewPanel::createScrubber(Ogre::OverlayManager& overlay_manager) {
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
  scrubber_bar_->hide(); // Disabled by default
  main_panel_->addChild(scrubber_bar_);

  scrubber_dot_ = dynamic_cast<Ogre::PanelOverlayElement*>(
      overlay_manager.createOverlayElement("Panel", "ScrubberBar2Dot"));
  scrubber_dot_->setVerticalAlignment(Ogre::GVA_CENTER);
  scrubber_dot_->setPosition(kScrubberDotZero, -kAbsoluteScrubberDotSize / 2.);
  scrubber_dot_->setDimensions(kAbsoluteScrubberDotSize,
                               kAbsoluteScrubberDotSize);
  scrubber_dot_->setMaterialName("Template/ScrubberDot");
  scrubber_dot_->hide(); // Disabled by default
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

void PreviewPanel::createPlayPause(Ogre::OverlayManager& overlay_manager) {
  main_panel_->addChild(
      play_pause_button_.create(overlay_manager, "PlayPauseButton")
          .atPosition(kButtonPadding + kButtonMargin,
                      kButtonPadding - kButtonWidth / 2.)
          .withDimensions(kButtonWidth - 2 * kButtonPadding,
                          kButtonWidth - 2 * kButtonPadding)
          .withMaterial("Template/PlayIcon")
          .withClickTopic("~rvinci_play_pause")
          .disabled()
          .done());
}

void PreviewPanel::createExecute(Ogre::OverlayManager& overlay_manager) {
  main_panel_->addChild(
      execute_button_.create(overlay_manager, "ExecuteButton")
          .alignedRight()
          .atPosition(-(kButtonPadding + kButtonWidth - kButtonMargin),
                      kButtonPadding - kButtonWidth / 2.)
          .withDimensions(kButtonWidth - 2 * kButtonPadding,
                          kButtonWidth - 2 * kButtonPadding)
          .withMaterial("Template/UploadIcon")
          .withClickTopic("~rvinci_execute_abort")
          .disabled()
          .done());
}

void PreviewPanel::destroy() {
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
}

void PreviewPanel::setScrubberPosition(double position) {
  ROS_ASSERT(scrubber_dot_);
  scrubber_dot_->setLeft(kScrubberDotZero +
                         position * kAbsoluteScrubberBarWidth);
}

void PreviewPanel::setPreviewButtonPlaying(bool is_playing) {
  play_pause_button_.setIcon(is_playing ? "Template/PauseIcon"
                                        : "Template/PlayIcon");
}

void PreviewPanel::setExecuteAbortButtonExecuting(bool is_executing) {
  execute_button_.setIcon(is_executing ? "Template/UploadIcon"
                                       : "Template/StopIcon");
}

void PreviewPanel::setPreviewEnabled(bool preview_enabled) {
  play_pause_button_.setEnabled(preview_enabled);
  if (preview_enabled) {
    scrubber_bar_->show();
    scrubber_dot_->show();
  } else {
    scrubber_bar_->hide();
    scrubber_dot_->hide();
  }
}

void PreviewPanel::setExecuteEnabled(bool execute_enabled) {
  execute_button_.setEnabled(execute_enabled);
}

} // namespace gui_elements
} // namespace rvinci