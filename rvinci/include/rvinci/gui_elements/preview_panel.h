#ifndef RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_PREVIEW_PANEL_H_
#define RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_PREVIEW_PANEL_H_

#include <OgreOverlayManager.h>

#include "icon_button.h"

namespace Ogre {
class PanelOverlayElement;
}

namespace rvinci {
namespace gui_elements {

class PreviewPanel {
public:
  PreviewPanel() = default;
  ~PreviewPanel() { destroy(); };

  Ogre::OverlayContainer* create(const Ogre::Vector3& input_scale);
  void destroy();

  void setScrubberPosition(double position);

  void setPreviewButtonPlaying(bool is_playing);

  void setExecuteAbortButtonExecuting(bool is_executing);

  void setPreviewEnabled(bool preview_enabled);

  void setExecuteEnabled(bool execute_enabled);

  void changeInputScale(Ogre::Vector3 input_scale);

private:
  Ogre::PanelOverlayElement* main_panel_;
  Ogre::PanelOverlayElement* scrubber_bar_;
  Ogre::PanelOverlayElement* scrubber_dot_;
  IconButton play_pause_button_;
  IconButton execute_button_;

  void createScrubber(Ogre::OverlayManager& overlay_manager,
                      const Ogre::Vector3& input_scale);
  void createPlayPause(Ogre::OverlayManager& overlay_manager);
  void createExecute(Ogre::OverlayManager& overlay_manager);
};

} // namespace gui_elements
} // namespace rvinci

#endif // RVINCI_INCLUDE_RVINCI_GUI_ELEMENTS_PREVIEW_PANEL_H_
