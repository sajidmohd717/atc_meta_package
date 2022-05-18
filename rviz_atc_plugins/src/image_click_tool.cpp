/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OgreRenderWindow.h>

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include "image_click_tool.h"

// Mod by Tim:
#define DEBUG_IMG_CLICK_TOOL    1
#include "atc_utils/angles.h"


namespace rviz_plugin_tutorials
{

//----------------------------------------------------------------------------------
ImageClickTool::ImageClickTool()
  : moving_flag_node_( NULL )
  , current_flag_property_( NULL )
{
  shortcut_key_ = 'q';
}

//----------------------------------------------------------------------------------
ImageClickTool::~ImageClickTool()
{
  for( unsigned i = 0; i < flag_nodes_.size(); i++ )
  {
    scene_manager_->destroySceneNode( flag_nodes_[ i ]);
  }

}

//----------------------------------------------------------------------------------
void ImageClickTool::onInitialize()
{
#if DEBUG_IMG_CLICK_TOOL
	ROS_INFO("ImageClickTool::onInitialize() ");
#endif

  flag_resource_ = "package://rviz_plugin_tutorials/media/flag.dae";
  //flag_resource_ = "package://rviz_plugin_tutorials/media/arrow.dae";

  if( rviz::loadMeshFromResource( flag_resource_ ).isNull() )
  {
    ROS_ERROR( "ImageClickTool: failed to load model resource '%s'.", flag_resource_.c_str() );
    return;
  }

  moving_flag_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
  moving_flag_node_->attachObject( entity );
  moving_flag_node_->setVisible( false );


}

//----------------------------------------------------------------------------------
void ImageClickTool::activate()
{
#if DEBUG_IMG_CLICK_TOOL
	ROS_INFO("ImageClickTool::activate() ");
#endif

  if( moving_flag_node_ )
  {
    moving_flag_node_->setVisible( true );

    current_flag_property_ = new rviz::VectorProperty( "Flag " + QString::number( flag_nodes_.size() ));
    current_flag_property_->setReadOnly( true );
    getPropertyContainer()->addChild( current_flag_property_ );
  }



}

//----------------------------------------------------------------------------------------------
// deactivate() is called when the tool is being turned off because
// another tool has been chosen.
//
// We make the moving flag invisible, then delete the current flag
// property.  Deleting a property also removes it from its parent
// property, so that doesn't need to be done in a separate step.  If
// we didn't delete it here, it would stay in the list of flags when
// we switch to another tool.
void ImageClickTool::deactivate()
{
#if DEBUG_IMG_CLICK_TOOL
	ROS_INFO("ImageClickTool::deactivate() ");
#endif

  if( moving_flag_node_ )
  {
    moving_flag_node_->setVisible( false );
    delete current_flag_property_;
    current_flag_property_ = NULL;
  }


}

//----------------------------------------------------------------------------------------------
int ImageClickTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
#if DEBUG_IMG_CLICK_TOOL
	//ROS_INFO("ImageClickTool::processMouseEvent() ");
#endif

  if( !moving_flag_node_ )
  {
#if DEBUG_IMG_CLICK_TOOL
	ROS_INFO(" !moving_flag_node_");
#endif
    return Render;
  }
  //--------------------------------------------------------------------------
  // Top left is 0,0
  // Assumed that panel is always sufficiently wide, only height not enough. ie:
  // panelHeight > imgHeight
  // panelWidth == imgWidth
  if(rviz::ImageTrackerDisplay::isImgTrkInfoAvail && event.leftDown())
  {
	  //render_panel_ptr->getRenderWindow()->getWidth()  - Width of Ogre Panel (not video stream)
	  //render_panel_ptr->getRenderWindow()->getHeight() - Height of Ogre Panel (not video stream)
	  const float panelWidth  = rviz::ImageTrackerDisplay::imgTrkInfo.render_panel_ptr->getRenderWindow()->getWidth();
	  const float panelHeight = rviz::ImageTrackerDisplay::imgTrkInfo.render_panel_ptr->getRenderWindow()->getHeight();

	  // imgTrkInfo.texture_ptr->getWidth()  - Width of video stream
	  // imgTrkInfo.texture_ptr->getHeight() - Height of video stream
	  const float imgWidth  = rviz::ImageTrackerDisplay::imgTrkInfo.texture_ptr->getWidth();
	  const float imgHeight = rviz::ImageTrackerDisplay::imgTrkInfo.texture_ptr->getHeight();

	  const float clickedPanelHt = event.y;	// In panel size terms
	  const float clickedPanelWidth = event.x;

	  const float upsizeFactorHeight = (panelWidth/imgWidth);
	  const float upsizeFactor = upsizeFactorHeight;
//	  const float upsizeFactorWidth = (panelHeight/imgHeight);
//	  const float upsizeFactor = (upsizeFactorHeight > upsizeFactorWidth) ? (upsizeFactorHeight):(upsizeFactorWidth);

	  const float adjImgPanelHeight = imgHeight * upsizeFactor;
	  const float adjImgPanelWidth = imgWidth * upsizeFactor;
	  ROS_INFO(" adjImgPanelHeight:%.2f, imgHeight:%.2f, upsizeFactorHeight:%.2f ",
			  adjImgPanelHeight, imgHeight, upsizeFactorHeight);

	  float clickedImgHt = clickedPanelHt;
	  float clickedImgWidth = clickedPanelWidth*(1/upsizeFactor);
	  if(panelHeight > imgHeight)
	  {
		  const float deltaHeightOffset = (panelHeight - adjImgPanelHeight)/2;
		  ROS_INFO(" deltaHeightOffset:%.2f, panelHeight:%.2f, adjImgPanelHeight:%.2f",
				  deltaHeightOffset, panelHeight, adjImgPanelHeight);

		  clickedImgHt = clickedPanelHt - deltaHeightOffset;
		  clickedImgHt = (clickedImgHt > 0) ? (clickedImgHt) : (0);
		  clickedImgHt = (clickedImgHt > adjImgPanelHeight) ? (adjImgPanelHeight) : (clickedImgHt);
		  clickedImgHt = clickedImgHt * (1/upsizeFactor);	// State in image resolution terms
		  ROS_INFO(" clickedImgHt:%.2f, clickedPanelHt:%.2f, deltaHeightOffset:%.2f",
				  clickedImgHt, clickedPanelHt, deltaHeightOffset);
	  }

	  const float currentHeadingDeg = rviz::ImageTrackerDisplay::imgTrkInfo.currentHeadingDegrees;
	  const float commandedHeadingDeg = calculateCommandedHeading(currentHeadingDeg, clickedImgWidth, imgWidth);
	  rviz::ImageTrackerDisplay::imgTrkInfo.commandedHeadingDegrees = commandedHeadingDeg;


#if DEBUG_IMG_CLICK_TOOL
	ROS_INFO(" panelWidth:%.2f, ht:%.2f, imgWidth:%.2f, ht:%.2f, clickedImgWidth:%.2f, ht:%.2f, currHdgDeg:%.2f, cmdHdgDeg:%.2f\n",
			panelWidth, panelHeight, imgWidth, imgHeight, clickedImgWidth, clickedImgHt, currentHeadingDeg, commandedHeadingDeg);
#endif

	rviz::ImageTrackerDisplay::imgTrkInfo.hasChanged = true;
  }
  else
  {
#if DEBUG_IMG_CLICK_TOOL
	//ROS_INFO(" !isImgTrkInfoAvail");
#endif
  }
  //--------------------------------------------------------------------------

  return Render;

  //return -1;
}

//----------------------------------------------------------------------------------------------
float ImageClickTool::calculateCommandedHeading(const float& currentHeadingDegrees, const float& clickedImgWidth, const float& imgWidth)
{

	// For a 360 camera, in local frame of reference:
	//-180 deg -> left most 0pixels
	//   0 deg -> center (imgWidth/2)pixels
	// 180 deg -> center (imgWidth)pixels
//	const float cmdHeadingDeg_vehFrame = ((clickedImgWidth/imgWidth)*360.0) - 180.0;
//	const float cmdHeadingDeg_worldFrame = atc_utils::constrainAngle_0_360(cmdHeadingDeg_vehFrame + currentHeadingDegrees);

	// For a 360 camera, in local frame of reference:
	// 180 deg -> left most 0pixels
	//   0 deg -> center (imgWidth/2)pixels
	// -180 deg -> center (imgWidth)pixels
	const float cmdHeadingDeg_vehFrame = -1.0*(((clickedImgWidth/imgWidth)*360.0) - 180.0);
	const float cmdHeadingDeg_worldFrame = atc_utils::constrainAngle_180(cmdHeadingDeg_vehFrame + currentHeadingDegrees);

#if DEBUG_IMG_CLICK_TOOL
	ROS_INFO("ImageClickTool::calculateCommandedHeading() cmdHeadingDeg vehFrame:%.2f worldFrame:%.2f \n", cmdHeadingDeg_vehFrame, cmdHeadingDeg_worldFrame);
#endif

	return cmdHeadingDeg_worldFrame;
}

//----------------------------------------------------------------------------------------------
// This is a helper function to create a new flag in the Ogre scene and save its scene node in a list.
void ImageClickTool::makeFlag( const Ogre::Vector3& position )
{
#if DEBUG_IMG_CLICK_TOOL
	ROS_INFO("ImageClickTool::makeFlag() ");
#endif

  Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
  node->attachObject( entity );
  node->setVisible( true );
  node->setPosition( position );
  flag_nodes_.push_back( node );


}

// Loading and saving the flags
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// Tools with a fixed set of Property objects representing adjustable
// parameters are typically just created in the tool's constructor and
// added to the Property container (getPropertyContainer()).  In that
// case, the Tool subclass does not need to override load() and save()
// because the default behavior is to read all the Properties in the
// container from the Config object.
//
// Here however, we have a list of named flag positions of unknown
// length, so we need to implement save() and load() ourselves.
//
// We first save the class ID to the config object so the
// rviz::ToolManager will know what to instantiate when the config
// file is read back in.
void ImageClickTool::save( rviz::Config config ) const
{
#if DEBUG_IMG_CLICK_TOOL
	ROS_INFO("ImageClickTool::save() ");
#endif

  config.mapSetValue( "Class", getClassId() );

  // The top level of this tool's Config is a map, but our flags
  // should go in a list, since they may or may not have unique keys.
  // Therefore we make a child of the map (``flags_config``) to store
  // the list.
  rviz::Config flags_config = config.mapMakeChild( "Flags" );

  // To read the positions and names of the flags, we loop over the
  // the children of our Property container:
  rviz::Property* container = getPropertyContainer();
  int num_children = container->numChildren();
  for( int i = 0; i < num_children; i++ )
  {
    rviz::Property* position_prop = container->childAt( i );
    // For each Property, we create a new Config object representing a
    // single flag and append it to the Config list.
    rviz::Config flag_config = flags_config.listAppendNew();
    // Into the flag's config we store its name:
    flag_config.mapSetValue( "Name", position_prop->getName() );
    // ... and its position.
    position_prop->save( flag_config );
  }
}

// In a tool's load() function, we don't need to read its class
// because that has already been read and used to instantiate the
// object before this can have been called.
void ImageClickTool::load( const rviz::Config& config )
{
#if DEBUG_IMG_CLICK_TOOL
	ROS_INFO("ImageClickTool::load() ");
#endif

  // Here we get the "Flags" sub-config from the tool config and loop over its entries:
  rviz::Config flags_config = config.mapGetChild( "Flags" );
  int num_flags = flags_config.listLength();
  for( int i = 0; i < num_flags; i++ )
  {
    rviz::Config flag_config = flags_config.listChildAt( i );
    // At this point each ``flag_config`` represents a single flag.
    //
    // Here we provide a default name in case the name is not in the config file for some reason:
    QString name = "Flag " + QString::number( i + 1 );
    // Then we use the convenience function mapGetString() to read the
    // name from ``flag_config`` if it is there.  (If no "Name" entry
    // were present it would return false, but we don't care about
    // that because we have already set a default.)
    flag_config.mapGetString( "Name", &name );
    // Given the name we can create an rviz::VectorProperty to display the position:
    rviz::VectorProperty* prop = new rviz::VectorProperty( name );
    // Then we just tell the property to read its contents from the config, and we've read all the data.
    prop->load( flag_config );
    // We finish each flag by marking it read-only (as discussed
    // above), adding it to the property container, and finally making
    // an actual visible flag object in the 3D scene at the correct
    // position.
    prop->setReadOnly( true );
    getPropertyContainer()->addChild( prop );
    makeFlag( prop->getVector() );
  }
}

// End of .cpp file
// ^^^^^^^^^^^^^^^^
//
// At the end of every plugin class implementation, we end the
// namespace and then tell pluginlib about the class.  It is important
// to do this in global scope, outside our package's namespace.

} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_tutorials::ImageClickTool,rviz::Tool )
// END_TUTORIAL
