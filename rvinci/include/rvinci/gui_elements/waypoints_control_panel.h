#ifndef RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_WAYPOINTS_CONTROL_PANEL_H_
#define RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_WAYPOINTS_CONTROL_PANEL_H_

#include <OgreOverlayManager.h>
#include "icon_button.h"

namespace Ogre {
class PanelOverlayElement;
class TextAreaOverlayElement;
}

namespace rvinci {
namespace gui_elements {

class WaypointsControlPanel {
public:
  WaypointsControlPanel() = default;
  ~WaypointsControlPanel() { destroy(); };

  Ogre::OverlayContainer* create();
  void destroy();

private:
  Ogre::PanelOverlayElement* main_panel_;
  Ogre::TextAreaOverlayElement* action_hint_;
  IconButton add_here_button_;
  IconButton add_at_end_button_;
};

} // namespace gui_elements
} // namespace rvinci


#endif // RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_WAYPOINTS_CONTROL_PANEL_H_
