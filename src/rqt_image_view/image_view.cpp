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

#include <rqt_image_view/image_view.h>

#include <pluginlib/class_list_macros.hpp>
#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ctime>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>

namespace rqt_image_view {

ImageView::ImageView()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
  , pub_topic_custom_(false)
  , hide_toolbar_action_(0)
  , num_gridlines_(0)
  , rotate_state_(ROTATE_0)
  , hud_trigger_delay_(0.2)
  , hud_stale_threshold_(0.5)
  , hud_disappear_threshold_(3.0)
  , main_dim_start_(1.0)
  , main_dim_end_(10.0)
{
  setObjectName("ImageView");
}

void ImageView::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  // Version identifier for custom build
  ROS_INFO("=== rqt_image_view CUSTOM BUILD with HUD overlay support (2024-11-15) ===");

  // Load HUD overlay parameters
  ros::NodeHandle pnh = getPrivateNodeHandle();
  pnh.param("hud_trigger_delay", hud_trigger_delay_, 0.2);
  pnh.param("hud_stale_threshold", hud_stale_threshold_, 0.5);
  pnh.param("hud_disappear_threshold", hud_disappear_threshold_, 3.0);
  pnh.param("main_dim_start", main_dim_start_, 1.0);
  pnh.param("main_dim_end", main_dim_end_, 10.0);

  setColorSchemeList();
  ui_.color_scheme_combo_box->setCurrentIndex(ui_.color_scheme_combo_box->findText("Gray"));

  // Initialize cached UI values for thread-safe access from ROS callbacks
  max_range_ = ui_.max_range_double_spin_box->value();
  dynamic_range_enabled_ = ui_.dynamic_range_check_box->isChecked();
  color_scheme_ = ui_.color_scheme_combo_box->itemData(ui_.color_scheme_combo_box->currentIndex()).toInt();

  // Connect slots to update cached values when UI changes
  connect(ui_.max_range_double_spin_box, SIGNAL(valueChanged(double)), this, SLOT(onMaxRangeChanged(double)));
  connect(ui_.color_scheme_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onColorSchemeChanged(int)));

  // Connect signal/slot for thread-safe image updates from ROS callbacks
  connect(this, SIGNAL(newImageAvailable(QImage)), this, SLOT(updateImageDisplay(QImage)), Qt::QueuedConnection);

  updateTopicList();
  ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText(""));
  connect(ui_.topics_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));

  ui_.refresh_topics_push_button->setIcon(QIcon::fromTheme("view-refresh"));
  connect(ui_.refresh_topics_push_button, SIGNAL(pressed()), this, SLOT(updateTopicList()));

  ui_.zoom_1_push_button->setIcon(QIcon::fromTheme("zoom-original"));
  connect(ui_.zoom_1_push_button, SIGNAL(toggled(bool)), this, SLOT(onZoom1(bool)));

  connect(ui_.dynamic_range_check_box, SIGNAL(toggled(bool)), this, SLOT(onDynamicRange(bool)));

  ui_.save_as_image_push_button->setIcon(QIcon::fromTheme("document-save-as"));
  connect(ui_.save_as_image_push_button, SIGNAL(pressed()), this, SLOT(saveImage()));

  connect(ui_.num_gridlines_spin_box, SIGNAL(valueChanged(int)), this, SLOT(updateNumGridlines()));

  // set topic name if passed in as argument
  const QStringList& argv = context.argv();
  if (!argv.empty()) {
    arg_topic_name = argv[0];
    selectTopic(arg_topic_name);
  }
  pub_topic_custom_ = false;

  ui_.image_frame->setOuterLayout(ui_.image_layout);

  QRegExp rx("([a-zA-Z/][a-zA-Z0-9_/]*)?"); //see http://www.ros.org/wiki/ROS/Concepts#Names.Valid_Names (but also accept an empty field)
  ui_.publish_click_location_topic_line_edit->setValidator(new QRegExpValidator(rx, this));
  connect(ui_.publish_click_location_check_box, SIGNAL(toggled(bool)), this, SLOT(onMousePublish(bool)));
  connect(ui_.image_frame, SIGNAL(mouseLeft(int, int)), this, SLOT(onMouseLeft(int, int)));
  connect(ui_.publish_click_location_topic_line_edit, SIGNAL(editingFinished()), this, SLOT(onPubTopicChanged()));

  connect(ui_.smooth_image_check_box, SIGNAL(toggled(bool)), ui_.image_frame, SLOT(onSmoothImageChanged(bool)));

  connect(ui_.rotate_left_push_button, SIGNAL(clicked(bool)), this, SLOT(onRotateLeft()));
  connect(ui_.rotate_right_push_button, SIGNAL(clicked(bool)), this, SLOT(onRotateRight()));

  // Make sure we have enough space for "XXX °"
  ui_.rotate_label->setMinimumWidth(
    ui_.rotate_label->fontMetrics().width("XXX°")
  );

  hide_toolbar_action_ = new QAction(tr("Hide toolbar"), this);
  hide_toolbar_action_->setCheckable(true);
  ui_.image_frame->addAction(hide_toolbar_action_);
  connect(hide_toolbar_action_, SIGNAL(toggled(bool)), this, SLOT(onHideToolbarChanged(bool)));
}

void ImageView::shutdownPlugin()
{
  subscriber_.shutdown();
  pub_mouse_left_.shutdown();
}

void ImageView::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  QString topic = ui_.topics_combo_box->currentText();
  //qDebug("ImageView::saveSettings() topic '%s'", topic.toStdString().c_str());
  instance_settings.setValue("topic", topic);
  instance_settings.setValue("zoom1", ui_.zoom_1_push_button->isChecked());
  instance_settings.setValue("dynamic_range", ui_.dynamic_range_check_box->isChecked());
  instance_settings.setValue("max_range", ui_.max_range_double_spin_box->value());
  instance_settings.setValue("publish_click_location", ui_.publish_click_location_check_box->isChecked());
  instance_settings.setValue("mouse_pub_topic", ui_.publish_click_location_topic_line_edit->text());
  instance_settings.setValue("toolbar_hidden", hide_toolbar_action_->isChecked());
  instance_settings.setValue("num_gridlines", ui_.num_gridlines_spin_box->value());
  instance_settings.setValue("smooth_image", ui_.smooth_image_check_box->isChecked());
  instance_settings.setValue("rotate", rotate_state_.load());
  instance_settings.setValue("color_scheme", ui_.color_scheme_combo_box->currentIndex());
}

void ImageView::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  bool zoom1_checked = instance_settings.value("zoom1", false).toBool();
  ui_.zoom_1_push_button->setChecked(zoom1_checked);

  bool dynamic_range_checked = instance_settings.value("dynamic_range", false).toBool();
  ui_.dynamic_range_check_box->setChecked(dynamic_range_checked);

  double max_range = instance_settings.value("max_range", ui_.max_range_double_spin_box->value()).toDouble();
  ui_.max_range_double_spin_box->setValue(max_range);

  int gridlines = instance_settings.value("num_gridlines", ui_.num_gridlines_spin_box->value()).toInt();
  num_gridlines_.store(gridlines);
  ui_.num_gridlines_spin_box->setValue(gridlines);

  QString topic = instance_settings.value("topic", "").toString();
  // don't overwrite topic name passed as command line argument
  if (!arg_topic_name.isEmpty())
  {
    arg_topic_name = "";
  }
  else
  {
    //qDebug("ImageView::restoreSettings() topic '%s'", topic.toStdString().c_str());
    selectTopic(topic);
  }

  bool publish_click_location = instance_settings.value("publish_click_location", false).toBool();
  ui_.publish_click_location_check_box->setChecked(publish_click_location);

  QString pub_topic = instance_settings.value("mouse_pub_topic", "").toString();
  ui_.publish_click_location_topic_line_edit->setText(pub_topic);

  bool toolbar_hidden = instance_settings.value("toolbar_hidden", false).toBool();
  hide_toolbar_action_->setChecked(toolbar_hidden);

  bool smooth_image_checked = instance_settings.value("smooth_image", false).toBool();
  ui_.smooth_image_check_box->setChecked(smooth_image_checked);

  int rotate = instance_settings.value("rotate", 0).toInt();
  if(rotate >= ROTATE_STATE_COUNT)
    rotate = ROTATE_0;
  rotate_state_.store(rotate);
  syncRotateLabel();

  int color_scheme = instance_settings.value("color_scheme", ui_.color_scheme_combo_box->currentIndex()).toInt();
  ui_.color_scheme_combo_box->setCurrentIndex(color_scheme);
}

void ImageView::setColorSchemeList()
{
  static const std::map<std::string, int> COLOR_SCHEME_MAP
  {
    { "Gray", -1 },  // Special case: no color map
    { "Autumn", cv::COLORMAP_AUTUMN },
    { "Bone", cv::COLORMAP_BONE },
    { "Cool", cv::COLORMAP_COOL },
    { "Hot", cv::COLORMAP_HOT },
    { "Hsv", cv::COLORMAP_HSV },
    { "Jet", cv::COLORMAP_JET },
    { "Ocean", cv::COLORMAP_OCEAN },
    { "Pink", cv::COLORMAP_PINK },
    { "Rainbow", cv::COLORMAP_RAINBOW },
    { "Spring", cv::COLORMAP_SPRING },
    { "Summer", cv::COLORMAP_SUMMER },
    { "Winter", cv::COLORMAP_WINTER }
  };

  for (const auto& kv : COLOR_SCHEME_MAP)
  {
    ui_.color_scheme_combo_box->addItem(QString::fromStdString(kv.first), QVariant(kv.second));
  }
}

void ImageView::updateTopicList()
{
  QSet<QString> message_types;
  message_types.insert("sensor_msgs/Image");
  QSet<QString> message_sub_types;
  message_sub_types.insert("sensor_msgs/CompressedImage");

  // get declared transports
  QList<QString> transports;
  image_transport::ImageTransport it(getNodeHandle());
  std::vector<std::string> declared = it.getDeclaredTransports();
  for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++)
  {
    //qDebug("ImageView::updateTopicList() declared transport '%s'", it->c_str());
    QString transport = it->c_str();

    // strip prefix from transport name
    QString prefix = "image_transport/";
    if (transport.startsWith(prefix))
    {
      transport = transport.mid(prefix.length());
    }
    transports.append(transport);
  }

  QString selected = ui_.topics_combo_box->currentText();

  // fill combo box
  QList<QString> topics = getTopics(message_types, message_sub_types, transports).values();
  topics.append("");
  qSort(topics);
  ui_.topics_combo_box->clear();
  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
  {
    QString label(*it);
    label.replace(" ", "/");
    ui_.topics_combo_box->addItem(label, QVariant(*it));
  }

  // restore previous selection
  selectTopic(selected);
}

QList<QString> ImageView::getTopicList(const QSet<QString>& message_types, const QList<QString>& transports)
{
  QSet<QString> message_sub_types;
  return getTopics(message_types, message_sub_types, transports).values();
}

QSet<QString> ImageView::getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports)
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  QSet<QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    all_topics.insert(it->name.c_str());
  }

  QSet<QString> topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    if (message_types.contains(it->datatype.c_str()))
    {
      QString topic = it->name.c_str();

      // add raw topic
      topics.insert(topic);
      //qDebug("ImageView::getTopics() raw topic '%s'", topic.toStdString().c_str());

      // add transport specific sub-topics
      for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
      {
        if (all_topics.contains(topic + "/" + *jt))
        {
          QString sub = topic + " " + *jt;
          topics.insert(sub);
          //qDebug("ImageView::getTopics() transport specific sub-topic '%s'", sub.toStdString().c_str());
        }
      }
    }
    if (message_sub_types.contains(it->datatype.c_str()))
    {
      QString topic = it->name.c_str();
      int index = topic.lastIndexOf("/");
      if (index != -1)
      {
        topic.replace(index, 1, " ");
        topics.insert(topic);
        //qDebug("ImageView::getTopics() transport specific sub-topic '%s'", topic.toStdString().c_str());
      }
    }
  }
  return topics;
}

void ImageView::selectTopic(const QString& topic)
{
  int index = ui_.topics_combo_box->findText(topic);
  if (index == -1)
  {
    // add topic name to list if not yet in
    QString label(topic);
    label.replace(" ", "/");
    ui_.topics_combo_box->addItem(label, QVariant(topic));
    index = ui_.topics_combo_box->findText(topic);
  }
  ui_.topics_combo_box->setCurrentIndex(index);
}

void ImageView::onTopicChanged(int index)
{
  subscriber_.shutdown();
  hud_subscriber_.shutdown();

  {
    // Lock mutex while resetting shared state
    std::lock_guard<std::recursive_mutex> lock(image_mutex_);

    conversion_mat_.release();
    hud_conversion_mat_.release();

    // Reset timestamps
    main_image_timestamp_ = ros::Time(0);
    hud_image_timestamp_ = ros::Time(0);
    last_main_update_time_ = ros::Time(0);
    last_hud_update_time_ = ros::Time(0);
  }

  // reset image on topic change
  ui_.image_frame->setImage(QImage());

  QStringList parts = ui_.topics_combo_box->itemData(index).toString().split(" ");
  QString topic = parts.first();
  QString transport = parts.length() == 2 ? parts.last() : "raw";

  if (!topic.isEmpty())
  {
    image_transport::ImageTransport it(getNodeHandle());
    image_transport::TransportHints hints(transport.toStdString());
    try {
      subscriber_ = it.subscribe(topic.toStdString(), 1, &ImageView::callbackImage, this, hints);
      ROS_INFO("rqt_image_view: Subscribed to main topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());

      // Subscribe to HUD topic (base topic name + "_hud")
      std::string base_topic = parseBaseTopicName(topic.toStdString(), transport.toStdString());
      std::string hud_topic = base_topic + "_hud";
      ROS_INFO("rqt_image_view: Parsed base topic from '%s' (transport '%s') -> base '%s', HUD topic '%s'",
               topic.toStdString().c_str(), transport.toStdString().c_str(), base_topic.c_str(), hud_topic.c_str());

      // Try to subscribe to HUD topic (it's okay if it doesn't exist)
      try {
        image_transport::TransportHints hud_hints("raw"); // HUD always uses raw transport
        hud_subscriber_ = it.subscribe(hud_topic, 1, &ImageView::callbackImageHud, this, hud_hints);
        ROS_INFO("rqt_image_view: Successfully subscribed to HUD topic '%s'", hud_topic.c_str());
      } catch (image_transport::TransportLoadException& e) {
        // HUD topic doesn't exist or failed to load, which is fine
        ROS_WARN("rqt_image_view: HUD topic '%s' not available: %s", hud_topic.c_str(), e.what());
      }
    } catch (image_transport::TransportLoadException& e) {
      QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
    }
  }

  onMousePublish(ui_.publish_click_location_check_box->isChecked());
}

void ImageView::updateImageDisplay(const QImage& image)
{
  // This slot runs on Qt main thread - safe to update UI
  // No mutex needed here as QImage is passed by value (deep copy)
  ui_.image_frame->setImage(image);

  if (!ui_.zoom_1_push_button->isEnabled())
  {
    ui_.zoom_1_push_button->setEnabled(true);
  }
  onZoom1(ui_.zoom_1_push_button->isChecked());
}

void ImageView::onZoom1(bool checked)
{
  if (checked)
  {
    if (ui_.image_frame->getImage().isNull())
    {
      return;
    }
    ui_.image_frame->setInnerFrameFixedSize(ui_.image_frame->getImage().size());
  } else {
    ui_.image_frame->setInnerFrameMinimumSize(QSize(80, 60));
    ui_.image_frame->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
    widget_->setMinimumSize(QSize(80, 60));
    widget_->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
  }
}

void ImageView::onDynamicRange(bool checked)
{
  dynamic_range_enabled_ = checked;  // Update cached value
  ui_.max_range_double_spin_box->setEnabled(!checked);
}

void ImageView::onMaxRangeChanged(double value)
{
  max_range_ = value;  // Update cached value for thread-safe access
}

void ImageView::onColorSchemeChanged(int index)
{
  color_scheme_ = ui_.color_scheme_combo_box->itemData(index).toInt();  // Update cached value
}

void ImageView::updateNumGridlines()
{
  num_gridlines_.store(ui_.num_gridlines_spin_box->value());
}

void ImageView::saveImage()
{
  // take a snapshot before asking for the filename
  QImage img = ui_.image_frame->getImageCopy();

  QString file_name = QFileDialog::getSaveFileName(widget_, tr("Save as image"), "image.png", tr("Image (*.bmp *.jpg *.png *.tiff)"));
  if (file_name.isEmpty())
  {
    return;
  }

  img.save(file_name);
}

void ImageView::onMousePublish(bool checked)
{
  std::string topicName;
  if(pub_topic_custom_)
  {
    topicName = ui_.publish_click_location_topic_line_edit->text().toStdString();
  } else {
    if(!subscriber_.getTopic().empty())
    {
      topicName = subscriber_.getTopic()+"_mouse_left";
    } else {
      topicName = "mouse_left";
    }
    ui_.publish_click_location_topic_line_edit->setText(QString::fromStdString(topicName));
  }

  if(checked)
  {
    pub_mouse_left_ = getNodeHandle().advertise<geometry_msgs::Point>(topicName, 1000);
  } else {
    pub_mouse_left_.shutdown();
  }
}

void ImageView::onMouseLeft(int x, int y)
{
  if(ui_.publish_click_location_check_box->isChecked() && !ui_.image_frame->getImage().isNull())
  {
    geometry_msgs::Point clickCanvasLocation;
    // Publish click location in pixel coordinates
    clickCanvasLocation.x = round((double)x/(double)ui_.image_frame->width()*(double)ui_.image_frame->getImage().width());
    clickCanvasLocation.y = round((double)y/(double)ui_.image_frame->height()*(double)ui_.image_frame->getImage().height());
    clickCanvasLocation.z = 0;

    geometry_msgs::Point clickLocation = clickCanvasLocation;

    switch(rotate_state_.load())
    {
      case ROTATE_90:
        clickLocation.x = clickCanvasLocation.y;
        clickLocation.y = ui_.image_frame->getImage().width() - clickCanvasLocation.x;
        break;
      case ROTATE_180:
        clickLocation.x = ui_.image_frame->getImage().width() - clickCanvasLocation.x;
        clickLocation.y = ui_.image_frame->getImage().height() - clickCanvasLocation.y;
        break;
      case ROTATE_270:
        clickLocation.x = ui_.image_frame->getImage().height() - clickCanvasLocation.y;
        clickLocation.y = clickCanvasLocation.x;
        break;
      default:
        break;
    }

    pub_mouse_left_.publish(clickLocation);
  }
}

void ImageView::onPubTopicChanged()
{
  pub_topic_custom_ = !(ui_.publish_click_location_topic_line_edit->text().isEmpty());
  onMousePublish(ui_.publish_click_location_check_box->isChecked());
}

void ImageView::onHideToolbarChanged(bool hide)
{
  ui_.toolbar_widget->setVisible(!hide);
}

void ImageView::onRotateLeft()
{
  int m = rotate_state_.load() - 1;
  if(m < 0)
    m = ROTATE_STATE_COUNT-1;

  rotate_state_.store(m);
  syncRotateLabel();
}

void ImageView::onRotateRight()
{
  rotate_state_.store((rotate_state_.load() + 1) % ROTATE_STATE_COUNT);
  syncRotateLabel();
}

void ImageView::syncRotateLabel()
{
  switch(rotate_state_.load())
  {
    default:
    case ROTATE_0:   ui_.rotate_label->setText("0°"); break;
    case ROTATE_90:  ui_.rotate_label->setText("90°"); break;
    case ROTATE_180: ui_.rotate_label->setText("180°"); break;
    case ROTATE_270: ui_.rotate_label->setText("270°"); break;
  }
}

void ImageView::invertPixels(int x, int y)
{
  // Could do 255-conversion_mat_.at<cv::Vec3b>(cv::Point(x,y))[i], but that doesn't work well on gray
  cv::Vec3b & pixel = conversion_mat_.at<cv::Vec3b>(cv::Point(x, y));
  if (pixel[0] + pixel[1] + pixel[2] > 3 * 127)
    pixel = cv::Vec3b(0,0,0);
  else
    pixel = cv::Vec3b(255,255,255);
}

QList<int> ImageView::getGridIndices(int size) const
{
  QList<int> indices;

  // Thread-safe: load atomic value once for consistency
  const int gridlines = num_gridlines_.load();

  // the spacing between adjacent grid lines
  float grid_width = 1.0f * size / (gridlines + 1);

  // select grid line(s) closest to the center
  float index;
  if (gridlines % 2)  // odd
  {
    indices.append(size / 2);
    // make the center line 2px wide in case of an even resolution
    if (size % 2 == 0)  // even
      indices.append(size / 2 - 1);
    index = 1.0f * (size - 1) / 2;
  }
  else  // even
  {
    index = grid_width * (gridlines / 2);
    // one grid line before the center
    indices.append(round(index));
    // one grid line after the center
    indices.append(size - 1 - round(index));
  }

  // add additional grid lines from the center to the border of the image
  int lines = (gridlines - 1) / 2;
  while (lines > 0)
  {
    index -= grid_width;
    indices.append(round(index));
    indices.append(size - 1 - round(index));
    lines--;
  }

  return indices;
}

void ImageView::overlayGrid()
{
  // vertical gridlines
  QList<int> columns = getGridIndices(conversion_mat_.cols);
  for (QList<int>::const_iterator x = columns.begin(); x != columns.end(); ++x)
  {
    for (int y = 0; y < conversion_mat_.rows; ++y)
    {
      invertPixels(*x, y);
    }
  }

  // horizontal gridlines
  QList<int> rows = getGridIndices(conversion_mat_.rows);
  for (QList<int>::const_iterator y = rows.begin(); y != rows.end(); ++y)
  {
    for (int x = 0; x < conversion_mat_.cols; ++x)
    {
      invertPixels(x, *y);
    }
  }
}

void ImageView::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{
  // Lock recursive_mutex to protect shared image data and timestamps from concurrent access
  std::lock_guard<std::recursive_mutex> lock(image_mutex_);

  try
  {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;

    if (num_gridlines_.load() > 0)
      overlayGrid();
  }
  catch (cv_bridge::Exception& e)
  {
    try
    {
      // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      if (msg->encoding == "CV_8UC3")
      {
        // assuming it is rgb
        conversion_mat_ = cv_ptr->image;
      } else if (msg->encoding == "8UC1") {
        // convert gray to rgb
        cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
      } else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
        // scale / quantify
        double min = 0;
        double max = max_range_.load();  // Thread-safe: read cached value
        if (msg->encoding == "16UC1") max *= 1000;
        if (dynamic_range_enabled_.load())  // Thread-safe: read cached value
        {
          // dynamically adjust range based on min/max in image
          cv::minMaxLoc(cv_ptr->image, &min, &max);
          if (min == max) {
            // completely homogeneous images are displayed in gray
            min = 0;
            max = 2;
          }
        }
        cv::Mat img_scaled_8u;
        cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));

        const auto color_scheme = color_scheme_.load();  // Thread-safe: read cached value
        if (color_scheme == -1) {
          cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
        } else {
          cv::Mat img_color_scheme;
          cv::applyColorMap(img_scaled_8u, img_color_scheme, color_scheme);
          cv::cvtColor(img_color_scheme, conversion_mat_, CV_BGR2RGB);
        }
      } else {
        qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
        emit newImageAvailable(QImage()); // Thread-safe: emit signal instead of direct UI access
        return;
      }
    }
    catch (cv_bridge::Exception& e)
    {
      qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
      emit newImageAvailable(QImage()); // Thread-safe: emit signal instead of direct UI access
      return;
    }
  }

  // Handle rotation (only for main image, not HUD)
  switch(rotate_state_.load())
  {
    case ROTATE_90:
    {
      cv::Mat tmp;
      cv::transpose(conversion_mat_, tmp);
      cv::flip(tmp, conversion_mat_, 1);
      break;
    }
    case ROTATE_180:
    {
      cv::Mat tmp;
      cv::flip(conversion_mat_, tmp, -1);
      conversion_mat_ = tmp;
      break;
    }
    case ROTATE_270:
    {
      cv::Mat tmp;
      cv::transpose(conversion_mat_, tmp);
      cv::flip(tmp, conversion_mat_, 0);
      break;
    }
    default:
      break;
  }

  // Store timestamp and update time for main image
  main_image_timestamp_ = msg->header.stamp;
  last_main_update_time_ = ros::Time::now();

  ROS_INFO_THROTTLE(1.0, "rqt_image_view: Main image callback - timestamp: %.3f (zero: %d), update time: %.3f",
                    main_image_timestamp_.toSec(), main_image_timestamp_.isZero(), last_main_update_time_.toSec());

  // Generate composite image with HUD overlay
  generateCompositeImage();
}

std::string ImageView::parseBaseTopicName(const std::string& topic, const std::string& transport)
{
  // Strip transport suffix and /image_* suffix to get camera base name
  // Example: "/fpv_camera/image_compressed" -> "/fpv_camera"
  // Example: "/camera/image_raw" -> "/camera"

  std::string base_topic = topic;

  // First, strip transport suffix if present (e.g., "/compressed", "/theora")
  if (!transport.empty() && transport != "raw")
  {
    std::string transport_suffix = "/" + transport;
    size_t pos = base_topic.rfind(transport_suffix);
    if (pos != std::string::npos && pos == base_topic.length() - transport_suffix.length())
    {
      base_topic = base_topic.substr(0, pos);
    }
  }

  // Now strip the /image_* suffix to get the camera base name
  // Find the last occurrence of "/image"
  size_t image_pos = base_topic.rfind("/image");
  if (image_pos != std::string::npos)
  {
    // Check if what follows "/image" is either nothing or starts with underscore or is a known suffix
    // This handles cases like /image_raw, /image_compressed, /image_rect, etc.
    std::string after_image = base_topic.substr(image_pos + 6); // 6 = length of "/image"
    if (after_image.empty() || after_image[0] == '_' || after_image[0] == '/')
    {
      base_topic = base_topic.substr(0, image_pos);
    }
  }

  return base_topic;
}

void ImageView::applyDimmingOverlay(cv::Mat& img, double delay_sec)
{
  if (delay_sec <= main_dim_start_)
  {
    return; // No dimming needed
  }

  // Calculate dimming factor: linear interpolation from 0% to 70% black
  double factor = (delay_sec - main_dim_start_) / (main_dim_end_ - main_dim_start_);
  factor = std::min(1.0, std::max(0.0, factor)); // Clamp to [0, 1]

  double overlay_alpha = factor * 0.7; // 0% to 70% black overlay

  // Apply darkening by multiplying pixel values
  img = img * (1.0 - overlay_alpha);
}

cv::Mat ImageView::processStaleHud(const cv::Mat& hud, double age_sec)
{
  if (hud.empty())
  {
    return cv::Mat(); // Return empty if input is empty
  }

  // If HUD is too old, make it disappear
  if (age_sec > hud_disappear_threshold_)
  {
    return cv::Mat(); // Return empty mat to skip HUD rendering
  }

  cv::Mat processed_hud = hud.clone();

  // If HUD is stale but not too old, apply grayscale and transparency effects
  if (age_sec > hud_stale_threshold_)
  {
    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(processed_hud, gray, cv::COLOR_RGB2GRAY);
    cv::cvtColor(gray, processed_hud, cv::COLOR_GRAY2RGB);

    // Apply 50% transparency effect (will be used during blending)
    processed_hud = processed_hud * 0.5;

    // Add "stale hud data" text in red at bottom right
    std::string text = "stale hud data";
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.7;
    int thickness = 2;
    int baseline = 0;

    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);

    // Position at bottom right corner with some padding
    cv::Point text_pos(processed_hud.cols - text_size.width - 10,
                       processed_hud.rows - 10);

    // Draw text in red (RGB: 255, 0, 0)
    cv::putText(processed_hud, text, text_pos, font_face, font_scale,
                cv::Scalar(255, 0, 0), thickness);
  }

  return processed_hud;
}

void ImageView::overlayHud(cv::Mat& main, const cv::Mat& hud)
{
  if (hud.empty() || main.empty())
  {
    return; // Nothing to overlay
  }

  // Scale HUD to match main image dimensions independently
  cv::Mat hud_scaled;
  if (hud.size() != main.size())
  {
    cv::resize(hud, hud_scaled, main.size(), 0, 0, cv::INTER_LINEAR);
  }
  else
  {
    hud_scaled = hud;
  }

  // Ensure both images have the same type
  if (hud_scaled.type() != main.type())
  {
    cv::Mat hud_converted;
    hud_scaled.convertTo(hud_converted, main.type());
    hud_scaled = hud_converted;
  }

  // Alpha blend HUD over main image
  // Using addWeighted for simple transparency blending
  // Alpha value for HUD overlay (0.7 means 70% HUD, 30% main shows through)
  double hud_alpha = 0.7;
  cv::addWeighted(main, 1.0, hud_scaled, hud_alpha, 0, main);
}

void ImageView::generateCompositeImage()
{
  // Lock recursive_mutex to protect shared image data and timestamps
  // Note: This lock is recursive-safe since callbacks already hold it
  std::lock_guard<std::recursive_mutex> lock(image_mutex_);

  cv::Mat composite;
  bool hud_filtered_out = false;

  // Handle case where we have HUD but no main image
  if (conversion_mat_.empty())
  {
    // If we have HUD data, create a black background to display it on
    if (!hud_conversion_mat_.empty() && !hud_image_timestamp_.isZero())
    {
      // When no main image, can't calculate relative age, so just display HUD
      cv::Mat processed_hud = hud_conversion_mat_.clone();

      // Create black background matching HUD size
      composite = cv::Mat::zeros(hud_conversion_mat_.rows, hud_conversion_mat_.cols, CV_8UC3);
      overlayHud(composite, processed_hud);

      // Create QImage with DEEP COPY (not shallow wrapper of composite.data)
      QImage image(composite.data, composite.cols, composite.rows, composite.step[0], QImage::Format_RGB888);
      QImage imageCopy = image.copy(); // Deep copy to avoid dangling pointer when composite goes out of scope

      // Emit signal to update UI on main thread (thread-safe)
      emit newImageAvailable(imageCopy);
    }
    return; // Nothing to display
  }

  // Start with a copy of the processed main image
  composite = conversion_mat_.clone();

  // Calculate main image age relative to current time (for dimming based on data staleness)
  double main_age = 0.0;
  if (!main_image_timestamp_.isZero())
  {
    main_age = (ros::Time::now() - main_image_timestamp_).toSec();
    applyDimmingOverlay(composite, main_age);
  }

  // Process and overlay HUD if available
  if (!hud_conversion_mat_.empty())
  {
    // Calculate HUD age:
    // - If message timestamps are valid (non-zero), use message timestamp difference (works with bag playback)
    // - If timestamps are zero, use wall clock time difference (live topics without timestamps)
    double hud_age = 0.0;

    if (!main_image_timestamp_.isZero() && !hud_image_timestamp_.isZero())
    {
      // Both timestamps valid - use message time difference
      hud_age = std::abs((main_image_timestamp_ - hud_image_timestamp_).toSec());
      ROS_INFO_THROTTLE(1.0, "rqt_image_view: Composite generation - HUD available, age (from timestamps): %.3f sec", hud_age);
    }
    else
    {
      // Timestamps not available - fall back to wall clock time
      hud_age = (ros::Time::now() - last_hud_update_time_).toSec();
      ROS_INFO_THROTTLE(1.0, "rqt_image_view: Composite generation - HUD available, age (from wall clock): %.3f sec", hud_age);
    }

    cv::Mat processed_hud = processStaleHud(hud_conversion_mat_, hud_age);

    if (!processed_hud.empty())
    {
      ROS_INFO_THROTTLE(1.0, "rqt_image_view: Overlaying HUD onto main image");
      overlayHud(composite, processed_hud);
    }
    else
    {
      ROS_INFO_THROTTLE(1.0, "rqt_image_view: HUD too old (%.3f sec), not displaying", hud_age);
      hud_filtered_out = true;
    }
  }
  else
  {
    ROS_INFO_THROTTLE(1.0, "rqt_image_view: No HUD data available");
  }

  // Draw "HUD info obsolete" message with timestamp when HUD is filtered out
  if (hud_filtered_out)
  {
    // Calculate how old the HUD is
    double hud_age = 0.0;
    if (!main_image_timestamp_.isZero() && !hud_image_timestamp_.isZero())
    {
      hud_age = std::abs((main_image_timestamp_ - hud_image_timestamp_).toSec());
    }
    else
    {
      hud_age = (ros::Time::now() - last_hud_update_time_).toSec();
    }

    // Format the message with age information
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "HUD info obsolete (%.1fs old, hidden)", hud_age);
    std::string text(buffer);

    // Add timestamp of last HUD update
    char time_buffer[128];
    time_t hud_time = hud_image_timestamp_.isZero() ? last_hud_update_time_.sec : hud_image_timestamp_.sec;
    struct tm* timeinfo = localtime(&hud_time);
    strftime(time_buffer, sizeof(time_buffer), "Last HUD: %H:%M:%S", timeinfo);
    std::string timestamp_text(time_buffer);

    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.7;
    int thickness = 2;
    cv::Scalar text_color(255, 255, 255); // White

    // Draw main message at top left
    cv::Point text_pos(10, 30);
    cv::putText(composite, text, text_pos, font_face, font_scale, text_color, thickness, cv::LINE_AA);

    // Draw timestamp below main message
    cv::Point time_pos(10, 60);
    cv::putText(composite, timestamp_text, time_pos, font_face, font_scale * 0.8, text_color, thickness - 1, cv::LINE_AA);
  }

  // Create QImage with DEEP COPY (not shallow wrapper of composite.data)
  QImage image(composite.data, composite.cols, composite.rows, composite.step[0], QImage::Format_RGB888);
  QImage imageCopy = image.copy(); // Deep copy to avoid dangling pointer when composite goes out of scope

  // Emit signal to update UI on main thread (thread-safe)
  emit newImageAvailable(imageCopy);
}

void ImageView::callbackImageHud(const sensor_msgs::Image::ConstPtr& msg)
{
  // Lock recursive_mutex to protect shared HUD data and timestamps from concurrent access
  std::lock_guard<std::recursive_mutex> lock(image_mutex_);

  try
  {
    // Convert ROS image to OpenCV format (RGB8 to match main image format)
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);

    // Store HUD image and timestamp
    hud_conversion_mat_ = cv_ptr->image.clone();
    hud_image_timestamp_ = msg->header.stamp;
    last_hud_update_time_ = ros::Time::now();

    ROS_INFO_THROTTLE(1.0, "rqt_image_view: HUD callback received image %dx%d, timestamp %.3f",
                      hud_conversion_mat_.cols, hud_conversion_mat_.rows, msg->header.stamp.toSec());

    // Trigger redraw if:
    // 1. No main image has been received yet (display HUD on black background), OR
    // 2. Main image hasn't updated recently (beyond trigger delay)
    if (last_main_update_time_.isZero())
    {
      // No main image yet - display HUD alone (happens when viewing HUD-only topic)
      ROS_INFO_ONCE("rqt_image_view: Displaying HUD on black background (no main image topic selected)");
      generateCompositeImage();
    }
    else
    {
      // Main image exists - trigger if it hasn't updated recently (use wall clock for trigger timing)
      double time_since_main = (ros::Time::now() - last_main_update_time_).toSec();
      ROS_DEBUG_THROTTLE(1.0, "rqt_image_view: HUD received, time since main update: %.3f sec (trigger delay: %.3f)",
                         time_since_main, hud_trigger_delay_);
      if (time_since_main > hud_trigger_delay_)
      {
        ROS_DEBUG("rqt_image_view: Triggering composite generation from HUD callback (main image hasn't updated recently)");
        generateCompositeImage();
      }
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception in HUD callback: %s", e.what());
  }
}
}

PLUGINLIB_EXPORT_CLASS(rqt_image_view::ImageView, rqt_gui_cpp::Plugin)
