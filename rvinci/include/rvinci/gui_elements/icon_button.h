#ifndef RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_ICON_BUTTON_H_
#define RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_ICON_BUTTON_H_

#include <OgreOverlayManager.h>
#include <OgrePrerequisites.h>

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
  Ogre::PanelOverlayElement* done();

  void setIcon(const std::string& material_name);

private:
  Ogre::PanelOverlayElement* panel_;
};

} // namespace gui_elements
} // namespace rvinci

#endif // RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_ICON_BUTTON_H_
