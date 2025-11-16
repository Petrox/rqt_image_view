/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef rqt_image_view__ImageView_H
#define rqt_image_view__ImageView_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_image_view.h>

#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/macros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

#include <opencv2/core/core.hpp>

#include <QAction>
#include <QImage>
#include <QList>
#include <QString>
#include <QSet>
#include <QSize>
#include <QWidget>

#include <vector>
#include <mutex>
#include <atomic>

namespace rqt_image_view {

class ImageView
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  ImageView();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual void shutdownPlugin();

  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

protected slots:

  virtual void setColorSchemeList();

  virtual void updateTopicList();

protected:

  // deprecated function for backward compatibility only, use getTopics() instead
  ROS_DEPRECATED virtual QList<QString> getTopicList(const QSet<QString>& message_types, const QList<QString>& transports);

  virtual QSet<QString> getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports);

  virtual void selectTopic(const QString& topic);

signals:

  // Thread-safe signal to update image from ROS callbacks
  void newImageAvailable(const QImage& image);

protected slots:

  // Thread-safe slot to update UI (runs on Qt main thread)
  void updateImageDisplay(const QImage& image);

  virtual void onTopicChanged(int index);

  virtual void onZoom1(bool checked);

  virtual void onDynamicRange(bool checked);

  virtual void onMaxRangeChanged(double value);

  virtual void onColorSchemeChanged(int index);

  virtual void saveImage();

  virtual void updateNumGridlines();

  virtual void onMousePublish(bool checked);

  virtual void onMouseLeft(int x, int y);

  virtual void onPubTopicChanged();

  virtual void onHideToolbarChanged(bool hide);

  virtual void onRotateLeft();
  virtual void onRotateRight();

protected:

  virtual void callbackImage(const sensor_msgs::Image::ConstPtr& msg);

  virtual void callbackImageHud(const sensor_msgs::Image::ConstPtr& msg);

  virtual void invertPixels(int x, int y);

  QList<int> getGridIndices(int size) const;

  virtual void overlayGrid();

  std::string parseBaseTopicName(const std::string& topic, const std::string& transport);

  void applyDimmingOverlay(cv::Mat& img, double delay_sec);

  cv::Mat processStaleHud(const cv::Mat& hud, double age_sec);

  void overlayHud(cv::Mat& main, const cv::Mat& hud);

  void generateCompositeImage();

  Ui::ImageViewWidget ui_;

  QWidget* widget_;

  image_transport::Subscriber subscriber_;

  image_transport::Subscriber hud_subscriber_;

  cv::Mat conversion_mat_;

  cv::Mat hud_conversion_mat_;

private:

  enum RotateState {
    ROTATE_0 = 0,
    ROTATE_90 = 1,
    ROTATE_180 = 2,
    ROTATE_270 = 3,

    ROTATE_STATE_COUNT
  };

  void syncRotateLabel();

  QString arg_topic_name;
  ros::Publisher pub_mouse_left_;

  bool pub_topic_custom_;

  QAction* hide_toolbar_action_;

  std::atomic<int> num_gridlines_;

  std::atomic<int> rotate_state_;

  // HUD overlay state
  ros::Time main_image_timestamp_;
  ros::Time hud_image_timestamp_;
  ros::Time last_main_update_time_;
  ros::Time last_hud_update_time_;

  // HUD configuration parameters
  double hud_trigger_delay_;
  double hud_stale_threshold_;
  double hud_disappear_threshold_;
  double main_dim_start_;
  double main_dim_end_;

  // Cached UI widget values for thread-safe access from ROS callbacks
  // These are updated from Qt slots (main thread) and read from callbacks (ROS threads)
  std::atomic<double> max_range_;
  std::atomic<bool> dynamic_range_enabled_;
  std::atomic<int> color_scheme_;

  // Thread safety: recursive_mutex protects all shared image data and timestamps
  // accessed from ROS callbacks (callbackImage, callbackImageHud) and generateCompositeImage
  // Recursive to allow callbacks to call generateCompositeImage() while holding lock
  mutable std::recursive_mutex image_mutex_;
};

}

#endif // rqt_image_view__ImageView_H
