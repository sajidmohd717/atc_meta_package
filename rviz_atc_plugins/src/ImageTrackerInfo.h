/*
 * ImageTrackerInfo.h
 *
 *  Created on: Mar 18, 2021
 *      Author: timityjoe
 */

#ifndef RVIZ_ATC_PLUGINS_SRC_IMAGETRACKERINFO_H_
#define RVIZ_ATC_PLUGINS_SRC_IMAGETRACKERINFO_H_

#include "rviz/image/ros_image_texture.h"
#include "rviz/render_panel.h"

namespace rviz {

class ImageTrackerInfo {
public:
	//ImageTrackerInfo();
	//virtual ~ImageTrackerInfo();

	  ImageTrackerInfo():currentHeadingDegrees(0.0), commandedHeadingDegrees(0.0), hasChanged(false)
	  {

	  };

	  //render_panel_ptr->getRenderWindow()->getWidth()  - Width of Ogre Panel (not video stream)
	  //render_panel_ptr->getRenderWindow()->getHeight() - Height of Ogre Panel (not video stream)
	  RenderPanel* render_panel_ptr;

	  // imgTrkInfo.texture_ptr->getWidth()  - Width of video stream
	  // imgTrkInfo.texture_ptr->getHeight() - Height of video stream
	  ROSImageTexture* texture_ptr;

	  float currentHeadingDegrees;
	  float commandedHeadingDegrees;

	  // Set to true on mouse click in image_click_tool: processMouseEvent()
	  bool hasChanged;
};

} /* namespace rviz */

#endif /* RVIZ_ATC_PLUGINS_SRC_IMAGETRACKERINFO_H_ */
