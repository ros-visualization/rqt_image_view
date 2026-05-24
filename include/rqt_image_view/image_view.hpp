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

#ifndef RQT_IMAGE_VIEW__IMAGE_VIEW_HPP_
#define RQT_IMAGE_VIEW__IMAGE_VIEW_HPP_

#include <ui_image_view.h>

#include <atomic>
#include <memory>
#include <optional>
#include <vector>

#include <rqt_gui_cpp/plugin.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <opencv2/core/core.hpp>

#include <rqt_image_view/detail/framerate_estimator.hpp>

#include <QAction>  // NOLINT
#include <QImage>  // NOLINT
#include <QList>  // NOLINT
#include <QMutex>  // NOLINT
#include <QObject>  // NOLINT
#include <QPoint>  // NOLINT
#include <QString>  // NOLINT
#include <QSet>  // NOLINT
#include <QSize>  // NOLINT
#include <QWidget>  // NOLINT

QT_FORWARD_DECLARE_CLASS(QTimer)

namespace rqt_image_view
{
class ImageView : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  ImageView();

  virtual void initPlugin(qt_gui_cpp::PluginContext & context);

  virtual void shutdownPlugin();

  virtual void saveSettings(
    qt_gui_cpp::Settings & plugin_settings,
    qt_gui_cpp::Settings & instance_settings) const;

  virtual void restoreSettings(
    const qt_gui_cpp::Settings & plugin_settings,
    const qt_gui_cpp::Settings & instance_settings);

protected slots:
  virtual void updateTopicList();

protected:
  virtual QSet<QString> getTopics(
    const QSet<QString> & message_types,
    const QSet<QString> & message_sub_types, const QList<QString> & transports);

  virtual void selectTopic(const QString & topic);

protected slots:
  virtual void onTopicChanged(int index);

  virtual void onZoom1(bool checked);

  virtual void onDynamicRange(bool checked);

  virtual void saveImage();

  virtual void updateNumGridlines();

  virtual void onMousePublish(bool checked);

  virtual void onMouseLeft(int x, int y);

  virtual void onMouseMovedOnImage(int x, int y);

  virtual void onMouseExitedImage();

  // Throttle-timer callback that renders the most recent hover coordinates;
  // see hover_throttle_timer_ for the rate-limiting design.
  virtual void flushHoverLabel();

  virtual void onPubTopicChanged();

  virtual void onHideToolbarChanged(bool hide);

  virtual void onInfoBarToggled(bool checked);

  virtual void onRotateLeft();
  virtual void onRotateRight();

  virtual void updateInfoBarStats();

protected:
  virtual void callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  virtual void invertPixels(int x, int y);

  QList<int> getGridIndices(int size) const;

  virtual void overlayGrid();

  Ui::ImageViewWidget ui_;

  QWidget * widget_;

  image_transport::Subscriber subscriber_;

  cv::Mat conversion_mat_;

private:
  enum RotateState
  {
    ROTATE_0 = 0,
    ROTATE_90 = 1,
    ROTATE_180 = 2,
    ROTATE_270 = 3,

    ROTATE_STATE_COUNT
  };

  void syncRotateLabel();

  // Wire up the info-bar widgets, timers, and signal/slot connections.
  void setupInfoBar();

  // Reset the hover readout to the "no value" placeholder.
  void clearHoverLabel();

  QString arg_topic_name;

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_mouse_left_;

  bool pub_topic_custom_;

  QAction * hide_toolbar_action_;

  int num_gridlines_;

  // rotate_state_: written by the GUI thread (onRotateLeft/onRotateRight,
  // restoreSettings); read by the subscriber callback (callbackImage) to
  // decide how to rotate the just-arrived image. std::atomic makes the
  // cross-thread read race-free under the C++17 memory model.
  std::atomic<RotateState> rotate_state_;

  // latest_msg_ + latest_rotation_degrees_: both written by the ROS subscriber
  // callback under latest_msg_mutex_ so the hover handler reads the message
  // together with the rotation that was actually applied to it — needed
  // because the rotate buttons change rotate_state_ without re-rendering the
  // currently displayed frame.
  sensor_msgs::msg::Image::ConstSharedPtr latest_msg_;
  int latest_rotation_degrees_;
  mutable QMutex latest_msg_mutex_;

  // framerate_estimator_: written by the subscriber callback, read by the
  // info-bar timer; serialized with framerate_mutex_. Samples are pushed even
  // when cv_bridge later fails so arrival rate stays measurable on broken
  // streams — latest_msg_ in contrast only updates after a successful conversion.
  detail::FramerateEstimator framerate_estimator_;
  mutable QMutex framerate_mutex_;

  // GUI thread only; created in initPlugin and parented to `this`.
  QTimer * info_bar_stats_timer_;

  // High-polling-rate mice (multi-kHz) can fire mouseMoveEvent faster than a
  // RichText QLabel can re-layout, so we coalesce: onMouseMovedOnImage stores
  // the latest position in pending_hover_ and starts the single-shot
  // hover_throttle_timer_; flushHoverLabel() renders at most once per
  // HOVER_THROTTLE_INTERVAL_MS. GUI thread only.
  std::optional<QPoint> pending_hover_;
  QTimer * hover_throttle_timer_;
};

}  // namespace rqt_image_view

#endif  // RQT_IMAGE_VIEW__IMAGE_VIEW_HPP_
