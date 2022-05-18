
#include <boost/bind.hpp>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <OgreTechnique.h>
#include <OgreCamera.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/compatibility.h>
#include <rviz/render_panel.h>
#include <rviz/validate_floats.h>

#include <sensor_msgs/image_encodings.h>


// Mod by Tim:
#define DEBUG_IMG_TRK_DISPLAY 0
#define DEBUG_IMG_TRK_INFO    0

#include "image_tracker_display.h"

namespace rviz
{

// Mod by Tim: Declare static member variables
bool ImageTrackerDisplay::isImgTrkInfoAvail = false;
ImageTrackerInfo ImageTrackerDisplay::imgTrkInfo;

//----------------------------------------------------------------------------------------------
ImageTrackerDisplay::ImageTrackerDisplay() : ImageDisplayBase(), texture_()
{
  // Mod by Tim:
  imgTrkInfo.render_panel_ptr = NULL;
  isImgTrkInfoAvail = false;
  imgTrkInfo.texture_ptr = &texture_;

  normalize_property_ = new BoolProperty(
      "Normalize Range", true,
      "If set to true, will try to estimate the range of possible values from the received images.",
      this, SLOT(updateNormalizeOptions()));

  min_property_ = new FloatProperty("Min Value", 0.0, "Value which will be displayed as black.", this,
                                    SLOT(updateNormalizeOptions()));

  max_property_ = new FloatProperty("Max Value", 1.0, "Value which will be displayed as white.", this,
                                    SLOT(updateNormalizeOptions()));

  median_buffer_size_property_ =
      new IntProperty("Median window", 5, "Window size for median filter used for computin min/max.",
                      this, SLOT(updateNormalizeOptions()));

  got_float_image_ = false;
}
//----------------------------------------------------------------------------------------------
void ImageTrackerDisplay::onInitialize()
{
#if DEBUG_IMG_TRK_DISPLAY
	ROS_INFO("ImageTrackerDisplay::onInitialize() ");
#endif

  ImageDisplayBase::onInitialize();
  {
    static uint32_t count = 0;
    std::stringstream ss;
    ss << "ImageTrackerDisplay" << count++;
    img_scene_manager_ = Ogre::Root::getSingleton().createSceneManager(Ogre::ST_GENERIC, ss.str());
  }

  img_scene_node_ = img_scene_manager_->getRootSceneNode()->createChildSceneNode();

  {
    static int count = 0;
    std::stringstream ss;
    ss << "ImageTrackerDisplayObject" << count++;

    screen_rect_ = new Ogre::Rectangle2D(true);
    screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
    screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

    ss << "Material";
    material_ = Ogre::MaterialManager::getSingleton().create(
        ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_->setSceneBlending(Ogre::SBT_REPLACE);
    material_->setDepthWriteEnabled(false);
    material_->setReceiveShadows(false);
    material_->setDepthCheckEnabled(false);

    material_->getTechnique(0)->setLightingEnabled(false);
    Ogre::TextureUnitState* tu = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureName(texture_.getTexture()->getName());
    tu->setTextureFiltering(Ogre::TFO_NONE);

    material_->setCullingMode(Ogre::CULL_NONE);
    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    screen_rect_->setBoundingBox(aabInf);
    setMaterial(*screen_rect_, material_);
    img_scene_node_->attachObject(screen_rect_);
  }

  render_panel_ = new RenderPanel();

  // Mod by Tim:
  imgTrkInfo.render_panel_ptr = render_panel_;
  //if((imgTrkInfo.render_panel_ptr != NULL) && (imgTrkInfo.render_panel_ptr != NULL))
  //{
	  ROS_INFO("ImageTrackerDisplay::onInitialize() isImgTrkInfoAvail!");
	  isImgTrkInfoAvail = true;
  //}

  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getRenderWindow()->setActive(false);

  render_panel_->resize(640, 480);
  render_panel_->initialize(img_scene_manager_, context_);

  setAssociatedWidget(render_panel_);

  render_panel_->setAutoRender(false);
  render_panel_->setOverlaysEnabled(false);
  render_panel_->getCamera()->setNearClipDistance(0.01f);

  updateNormalizeOptions();
}
//----------------------------------------------------------------------------------------------
ImageTrackerDisplay::~ImageTrackerDisplay()
{
  if (initialized())
  {
    delete render_panel_;
    delete screen_rect_;
    removeAndDestroyChildNode(img_scene_node_->getParentSceneNode(), img_scene_node_);
  }
}
//----------------------------------------------------------------------------------------------
void ImageTrackerDisplay::onEnable()
{
#if DEBUG_IMG_TRK_DISPLAY
	ROS_INFO("ImageTrackerDisplay::onEnable() ");
#endif

  ImageDisplayBase::subscribe();
  render_panel_->getRenderWindow()->setActive(true);
}
//----------------------------------------------------------------------------------------------
void ImageTrackerDisplay::onDisable()
{
#if DEBUG_IMG_TRK_DISPLAY
	ROS_INFO("ImageTrackerDisplay::onDisable() ");
#endif

  render_panel_->getRenderWindow()->setActive(false);
  ImageDisplayBase::unsubscribe();
  reset();
}
//----------------------------------------------------------------------------------------------
void ImageTrackerDisplay::updateNormalizeOptions()
{
#if DEBUG_IMG_TRK_DISPLAY
	ROS_INFO("ImageTrackerDisplay::updateNormalizeOptions() ");
#endif

  if (got_float_image_)
  {
    bool normalize = normalize_property_->getBool();

    normalize_property_->setHidden(false);
    min_property_->setHidden(normalize);
    max_property_->setHidden(normalize);
    median_buffer_size_property_->setHidden(!normalize);

    texture_.setNormalizeFloatImage(normalize, min_property_->getFloat(), max_property_->getFloat());
    texture_.setMedianFrames(median_buffer_size_property_->getInt());
  }
  else
  {
    normalize_property_->setHidden(true);
    min_property_->setHidden(true);
    max_property_->setHidden(true);
    median_buffer_size_property_->setHidden(true);
  }
}
//----------------------------------------------------------------------------------------------
void ImageTrackerDisplay::update(float wall_dt, float ros_dt)
{
#if DEBUG_IMG_TRK_DISPLAY
	ROS_INFO("ImageTrackerDisplay::update() ");
#endif

  Q_UNUSED(wall_dt)
  Q_UNUSED(ros_dt)
  try
  {
    texture_.update();

    // make sure the aspect ratio of the image is preserved
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    float img_width = texture_.getWidth();
    float img_height = texture_.getHeight();

    if (img_width != 0 && img_height != 0 && win_width != 0 && win_height != 0)
    {
      float img_aspect = img_width / img_height;
      float win_aspect = win_width / win_height;

      if (img_aspect > win_aspect)
      {
        screen_rect_->setCorners(-1.0f, 1.0f * win_aspect / img_aspect, 1.0f,
                                 -1.0f * win_aspect / img_aspect, false);
      }
      else
      {
        screen_rect_->setCorners(-1.0f * img_aspect / win_aspect, 1.0f, 1.0f * img_aspect / win_aspect,
                                 -1.0f, false);
      }
    }

    render_panel_->getRenderWindow()->update();
  }
  catch (UnsupportedImageEncoding& e)
  {
    setStatus(StatusProperty::Error, "Image", e.what());
  }

#if DEBUG_IMG_TRK_INFO
	ROS_INFO("ImageTrackerDisplay::update() width:%i  height:%i ",
			imgTrkInfo.render_panel_ptr->getRenderWindow()->getWidth(),
			imgTrkInfo.render_panel_ptr->getRenderWindow()->getHeight());

//  ROS_INFO("ImageTrackerDisplay::update() width:%i  height:%i",
//		  imgTrkInfo.texture_ptr->getWidth(),
//		  imgTrkInfo.texture_ptr->getHeight());

#endif
}
//----------------------------------------------------------------------------------------------
void ImageTrackerDisplay::reset()
{
  ImageDisplayBase::reset();
  texture_.clear();
  render_panel_->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
}
//----------------------------------------------------------------------------------------------
/* This is called by incomingMessage(). */
void ImageTrackerDisplay::processMessage(const sensor_msgs::Image::ConstPtr& msg)
{
#if DEBUG_IMG_TRK_DISPLAY
	ROS_INFO("ImageTrackerDisplay::processMessage() ");
#endif

  bool got_float_image = msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
                         msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                         msg->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
                         msg->encoding == sensor_msgs::image_encodings::MONO16;

  if (got_float_image != got_float_image_)
  {
    got_float_image_ = got_float_image;
    updateNormalizeOptions();
  }
  texture_.addMessage(msg);
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::ImageTrackerDisplay, rviz::Display)
