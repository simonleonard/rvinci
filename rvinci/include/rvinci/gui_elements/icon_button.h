#ifndef RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_ICON_BUTTON_H_
#define RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_ICON_BUTTON_H_

#include <OgreOverlayManager.h>
#include <OgrePrerequisites.h>
#include <OgreBlendMode.h>

namespace Ogre {
class PanelOverlayElement;
}

namespace rvinci {
namespace gui_elements {

class IconButton {
public:
  IconButton() = default;
  ~IconButton();

  IconButton& create(Ogre::OverlayManager& manager, const std::string& name);
  void destroy();

  IconButton& atPosition(Ogre::Real left, Ogre::Real top);
  IconButton& withDimensions(Ogre::Real width, Ogre::Real height);
  IconButton& withMaterial(const std::string& name);
  IconButton& withClickTopic(const std::string& name);
  IconButton& alignedRight();
  IconButton& disabled();
  Ogre::PanelOverlayElement* done();

  void setIcon(const std::string& material_name);
  void setEnabled(bool is_enabled);

private:
  Ogre::PanelOverlayElement* panel_ = nullptr;
  Ogre::LayerBlendModeEx original_color_op_;

  bool is_enabled_ = true;

  const Ogre::LayerBlendModeEx& getColorOp() const;
  void setColorBlendMode(const Ogre::LayerBlendModeEx& color) const;
};

} // namespace gui_elements
} // namespace rvinci

#endif // RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_ICON_BUTTON_H_
