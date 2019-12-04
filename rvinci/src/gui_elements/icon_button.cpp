#include "rvinci/gui_elements/icon_button.h"

#include <OgreOverlayManager.h>
#include <OgrePanelOverlayElement.h>
#include <OgreTechnique.h>

namespace rvinci {
namespace gui_elements {

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
  return *this;
}

IconButton& IconButton::withClickTopic(const std::string& name) {
  // Setting the interaction mode here not because the method has click in the
  // name, (a button's mode is always click) but because the interaction cursor
  // will error if it sees an interaction mode without a topic.
  Ogre::UserObjectBindings& bindings = panel_->getUserObjectBindings();
  bindings.setUserAny("rvinci_interaction_mode",
                      Ogre::Any(std::string("click")));
  bindings.setUserAny("rvinci_publish_topic", Ogre::Any(name));

  return *this;
}

IconButton& IconButton::alignedRight() {
  panel_->setHorizontalAlignment(Ogre::GHA_RIGHT);
  return *this;
}

Ogre::PanelOverlayElement* IconButton::done() { return panel_; }

void IconButton::setIcon(const std::string& material_name) {
  // Copy the color in case it's been modified by the interaction msgs
  Ogre::LayerBlendModeEx color = panel_->getMaterial()
                                     ->getTechnique(0)
                                     ->getPass(0)
                                     ->getTextureUnitState(0)
                                     ->getColourBlendMode();

  panel_->setMaterialName(material_name);

  // Restore color
  // Note: DON'T save a reference to this long getter, because it
  // changes between the two calls
  panel_->getMaterial()
      ->getTechnique(0)
      ->getPass(0)
      ->getTextureUnitState(0)
      ->setColourOperationEx(color.operation, color.source1, color.source2,
                             color.colourArg1, color.colourArg2, color.factor);
}

} // namespace gui_elements
} // namespace rvinci