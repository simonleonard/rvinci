//
// Created by will on 11/19/19.
//

#ifndef RVINCI_INCLUDE_RVINCI_RVINCI_GUI_H_
#define RVINCI_INCLUDE_RVINCI_RVINCI_GUI_H_

namespace Ogre {
class Overlay;
}

namespace rvinci {

class RvinciGui {
public:
  ~RvinciGui();

  void initialize();

  void show();
  void hide();
private:
  Ogre::Overlay* overlay_ = nullptr;
};

}

#endif // RVINCI_INCLUDE_RVINCI_RVINCI_GUI_H_
