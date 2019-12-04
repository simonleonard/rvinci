#ifndef RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_BOTTOM_PANEL_H_
#define RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_BOTTOM_PANEL_H_

#include <OgreOverlayManager.h>

namespace Ogre {
class PanelOverlayElement;
}

namespace rvinci {
namespace gui_elements {

class BottomPanel {
public:
  BottomPanel() = default;
  ~BottomPanel() { destroy(); };

  Ogre::OverlayContainer* create();
  void destroy();

  void setScrubberPosition(double position);

private:
  Ogre::PanelOverlayElement* main_panel_;
  Ogre::PanelOverlayElement* scrubber_bar_;
  Ogre::PanelOverlayElement* scrubber_dot_;
  Ogre::PanelOverlayElement* play_pause_button_;
  Ogre::PanelOverlayElement* execute_button_;

  void createScrubber(Ogre::OverlayManager& overlay_manager);
  void createPlayPause(Ogre::OverlayManager& overlay_manager);
  void createExecute(Ogre::OverlayManager& overlay_manager);
};

} // namespace gui_elements
} // namespace rvinci

#endif // RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_BOTTOM_PANEL_H_
