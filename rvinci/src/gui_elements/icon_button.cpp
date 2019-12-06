#include "rvinci/gui_elements/icon_button.h"

#include <OgreOverlayManager.h>
#include <OgrePanelOverlayElement.h>
#include <OgreTechnique.h>

namespace rvinci {
namespace gui_elements {
namespace {
constexpr Ogre::Real kDisabledColorSubtract = 0.5;
} // namespace

IconButton& IconButton::create(Ogre::OverlayManager& manager,
                               const std::string& name) {
  // Make sure not to leak if we already have one
  destroy();

  panel_ = dynamic_cast<Ogre::PanelOverlayElement*>(
      manager.createOverlayElement("Panel", name));
  panel_->setVerticalAlignment(Ogre::GVA_CENTER);

  return *this;
}

IconButton::~IconButton() { destroy(); }

void IconButton::destroy() {
  if (panel_) {
    Ogre::OverlayManager::getSingleton().destroyOverlayElement(panel_);
    panel_ = nullptr;
  }
}

IconButton& IconButton::atPosition(Ogre::Real left, Ogre::Real top) {
  panel_->setPosition(left, top);
  return *this;
}

IconButton& IconButton::withDimensions(Ogre::Real width, Ogre::Real height) {
  panel_->setDimensions(width, height);
  return *this;
}

IconButton& IconButton::withMaterial(const std::string& name) {
  panel_->setMaterialName(name);
  original_color_op_ = getColorOp();

  return *this;
}

IconButton& IconButton::withClickTopic(const std::string& name) {
  panel_->getUserObjectBindings().setUserAny("rvinci_publish_topic",
                                             Ogre::Any(name));

  return *this;
}

IconButton& IconButton::alignedRight() {
  panel_->setHorizontalAlignment(Ogre::GHA_RIGHT);
  return *this;
}

IconButton& IconButton::disabled() {
  is_enabled_ = false;
  return *this;
}

Ogre::PanelOverlayElement* IconButton::done() {
  // Note setEnabled must be called even if the button is enabled already, to
  // set up rvinci_interaction_mode
  setEnabled(is_enabled_);
  return panel_;
}

void IconButton::setIcon(const std::string& material_name) {
  // Copy the color in case it's been modified by the interaction msgs
  Ogre::LayerBlendModeEx color = getColorOp();

  panel_->setMaterialName(material_name);

  // Restore color
  setColorBlendMode(color);
}

void IconButton::setEnabled(bool is_enabled) {
  // current disabled + command disabled must run, because that's how an
  // initially disabled element gets its color faded, but current enabled +
  // command enabled must not, because that could overwrite selected colors from
  // interaction_cursor_rviz. This seems fragile but I don't have time to
  // revisit it now.
  if (is_enabled_ && is_enabled) return;

  is_enabled_ = is_enabled;

  // Update interaction mode
  std::string mode = is_enabled_ ? "click" : "disabled";
  panel_->getUserObjectBindings().setUserAny("rvinci_interaction_mode",
                                             Ogre::Any(mode));

  // Update color
  if (is_enabled_) {
    setColorBlendMode(original_color_op_);
  } else {
    Ogre::LayerBlendModeEx c = original_color_op_;
    c.colourArg1.r = std::max(0.f, c.colourArg1.r - kDisabledColorSubtract);
    c.colourArg1.g = std::max(0.f, c.colourArg1.g - kDisabledColorSubtract);
    c.colourArg1.b = std::max(0.f, c.colourArg1.b - kDisabledColorSubtract);
    setColorBlendMode(c);
  }
}

void IconButton::setColorBlendMode(const Ogre::LayerBlendModeEx& color) const {
  panel_->getMaterial()
      ->getTechnique(0)
      ->getPass(0)
      ->getTextureUnitState(0)
      ->setColourOperationEx(color.operation, color.source1, color.source2,
                             color.colourArg1, color.colourArg2, color.factor);
}

const Ogre::LayerBlendModeEx& IconButton::getColorOp() const {
  return panel_->getMaterial()
      ->getTechnique(0)
      ->getPass(0)
      ->getTextureUnitState(0)
      ->getColourBlendMode();
}
} // namespace gui_elements
} // namespace rvinci