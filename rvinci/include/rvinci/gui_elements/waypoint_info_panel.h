#ifndef RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_WAYPOINT_INFO_PANEL_H_
#define RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_WAYPOINT_INFO_PANEL_H_

#include <OgreOverlayManager.h>

namespace Ogre {
class PanelOverlayElement;
class TextAreaOverlayElement;
}

namespace rvinci {
namespace gui_elements {

class WaypointInfoPanel {
public:
  WaypointInfoPanel() = default;
  ~WaypointInfoPanel() { destroy(); };

  Ogre::OverlayContainer* create();
  void destroy();

  void setWaypointName(std::string name);

private:
  Ogre::PanelOverlayElement* main_panel_;
  Ogre::TextAreaOverlayElement* waypoint_name_;
};

} // namespace gui_elements
} // namespace rvinci


#endif // RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_WAYPOINT_INFO_PANEL_H_
