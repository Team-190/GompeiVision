#include <gtest/gtest.h>
#include "io/FieldInterface.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>

TEST(FieldInterfaceTest, BasicFieldLoad) {
  auto nt_inst = nt::NetworkTableInstance::Create();
  
  FieldInterface::initialize(nt_inst);
  EXPECT_TRUE(FieldInterface::isInitialized());

  auto fieldTable = nt_inst.GetTable("field");
  
  std::vector<nt::DoubleArrayPublisher> pubs;
  for (int i = 1; i <= 22; i++) {
    auto pub = fieldTable->GetDoubleArrayTopic("tag_" + std::to_string(i)).Publish();
    pub.Set(std::vector<double>{static_cast<double>(i), 2.0, 3.0, 1.0, 0.0, 0.0, 0.0});
    pubs.push_back(std::move(pub));
  }

  bool update_success = FieldInterface::update();
  EXPECT_TRUE(update_success);

  auto tagMap = FieldInterface::getMap();
  EXPECT_EQ(tagMap.size(), 22u);
  EXPECT_DOUBLE_EQ(tagMap[5].X().value(), 5.0);
  EXPECT_DOUBLE_EQ(tagMap[22].X().value(), 22.0);

  pubs[0].Set(std::vector<double>{1.0, 2.0}); // tag_1 has size 2 now, making it invalid
  update_success = FieldInterface::update();
  EXPECT_FALSE(update_success);
}
