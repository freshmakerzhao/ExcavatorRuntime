#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <QApplication>
#include <QByteArray>
#include <QPushButton>
#include <QSlider>
#include <QTabWidget>

#include "gtest/gtest.h"
#include "pluginlib/class_loader.hpp"
#include "rviz_common/panel.hpp"

TEST(MissionPanelPlugin, IsDiscoverableByRvizClassName)
{
  pluginlib::ClassLoader<rviz_common::Panel> loader(
    "rviz_common", "rviz_common::Panel");
  const std::vector<std::string> classes = loader.getDeclaredClasses();

  EXPECT_NE(
    std::find(
      classes.begin(), classes.end(),
      "airy_mission_panel/ExcavationPanel"),
    classes.end());

  qputenv("QT_QPA_PLATFORM", QByteArray("offscreen"));
  int argc = 1;
  static char application_name[] = "airy_mission_panel_plugin_test";
  char * argv[] = {application_name, nullptr};
  std::unique_ptr<QApplication> application;
  if (QApplication::instance() == nullptr) {
    application = std::make_unique<QApplication>(argc, argv);
  }

  const auto panel = loader.createSharedInstance(
    "airy_mission_panel/ExcavationPanel");
  ASSERT_NE(panel, nullptr);

  const auto * tabs = panel->findChild<QTabWidget *>("mission_panel_tabs");
  ASSERT_NE(tabs, nullptr);
  ASSERT_EQ(tabs->count(), 3);
  EXPECT_EQ(tabs->tabText(0), "Actions");
  EXPECT_EQ(tabs->tabText(1), "Logs");
  EXPECT_EQ(tabs->tabText(2), "Tests");

  auto * dig = panel->findChild<QPushButton *>("plan_follow_dig");
  auto * dump = panel->findChild<QPushButton *>("plan_follow_dump");
  ASSERT_NE(dig, nullptr);
  ASSERT_NE(dump, nullptr);
  EXPECT_EQ(dig->text(), "Plan + Follow DIG");
  EXPECT_EQ(dump->text(), "Plan + Follow DUMP");

  std::vector<QSlider *> sliders;
  for (const auto * joint : {
      "swing_joint", "boom_joint", "arm_joint", "bucket_joint"})
  {
    auto * slider = panel->findChild<QSlider *>(QString("joint_test_slider_%1").arg(joint));
    ASSERT_NE(slider, nullptr);
    sliders.push_back(slider);
  }
  EXPECT_NE(panel->findChild<QPushButton *>("joint_test_publish"), nullptr);
  auto * reset = panel->findChild<QPushButton *>("joint_test_reset");
  ASSERT_NE(reset, nullptr);
  reset->setEnabled(true);
  for (auto * slider : sliders) {
    slider->setValue(25);
  }
  reset->click();
  for (const auto * slider : sliders) {
    EXPECT_EQ(slider->value(), 0);
  }
}
