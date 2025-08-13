#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <hector_gamepad_plugin_interface/blackboard.hpp>

using hector_gamepad_plugin_interface::Blackboard;

TEST(Blackboard, SetAndGet_PrimitivesAndStrings) {
  Blackboard bb;

  bb.set("i", 42);
  auto* pi = bb.try_get<int>("i");
  ASSERT_NE(pi, nullptr);
  EXPECT_EQ(*pi, 42);

  double d = 3.14;
  bb.set("d", d);
  auto* pd = bb.try_get<double>("d");
  ASSERT_NE(pd, nullptr);
  EXPECT_DOUBLE_EQ(*pd, 3.14);

  std::string s = "hello";
  bb.set("s", s);
  auto* ps1 = bb.try_get<std::string>("s");
  ASSERT_NE(ps1, nullptr);
  EXPECT_EQ(*ps1, "hello");

  bb.set("s2", std::string("world"));
  auto* ps2 = bb.try_get<std::string>("s2");
  ASSERT_NE(ps2, nullptr);
  EXPECT_EQ(*ps2, "world");
}

TEST(Blackboard, SetOverwritesAndTypeChanges) {
  Blackboard bb;

  bb.set("key", 1);
  EXPECT_TRUE(bb.contains("key"));
  EXPECT_NE(bb.try_get<int>("key"), nullptr);
  EXPECT_EQ(*bb.try_get<int>("key"), 1);

  bb.set("key", std::string("changed"));
  EXPECT_TRUE(bb.contains("key"));
  EXPECT_EQ(bb.try_get<int>("key"), nullptr);
  auto* ps = bb.try_get<std::string>("key");
  ASSERT_NE(ps, nullptr);
  EXPECT_EQ(*ps, "changed");
}

TEST(Blackboard, Emplace_ConstructsInPlaceAndReturnsRef) {
  Blackboard bb;

  auto& vec = bb.emplace<std::vector<int>>("vec", 3, 7);
  EXPECT_EQ(vec.size(), 3u);
  EXPECT_EQ(vec[0], 7);
  EXPECT_EQ(vec[1], 7);
  EXPECT_EQ(vec[2], 7);

  vec[1] = 99;
  auto* pv = bb.try_get<std::vector<int>>("vec");
  ASSERT_NE(pv, nullptr);
  EXPECT_EQ((*pv)[1], 99);

  // Re-emplace: content replaced; do not assume pointer stability
  auto& vec2 = bb.emplace<std::vector<int>>("vec", 2, 5);
  EXPECT_EQ(vec2.size(), 2u);
  EXPECT_EQ(vec2[0], 5);
  EXPECT_EQ(vec2[1], 5);
}

TEST(Blackboard, TryGet_NullptrOnMissingOrTypeMismatch) {
  Blackboard bb;
  EXPECT_EQ(bb.try_get<int>("missing"), nullptr);

  bb.set("x", 10);
  EXPECT_NE(bb.try_get<int>("x"), nullptr);
  EXPECT_EQ(bb.try_get<double>("x"), nullptr);
}

TEST(Blackboard, At_ReturnsRef_AndThrowsOnErrors) {
  Blackboard bb;

  bb.set("num", 123);
  int& r = bb.at<int>("num");
  r = 456;
  EXPECT_EQ(*bb.try_get<int>("num"), 456);

  EXPECT_THROW((void)bb.at<int>("missing"), std::out_of_range);
  EXPECT_THROW((void)bb.at<double>("num"), std::bad_any_cast);

  const Blackboard& cbb = bb;
  const int& cr = cbb.at<int>("num");
  EXPECT_EQ(cr, 456);
  EXPECT_THROW((void)cbb.at<std::string>("num"), std::bad_any_cast);
}

TEST(Blackboard, ValueOr_ReturnsStoredOrDefault_OnMissingOrMismatch) {
  Blackboard bb;

  EXPECT_EQ(bb.value_or<int>("missing", 7), 7);

  bb.set("a", 11);
  EXPECT_EQ(bb.value_or<int>("a", 7), 11);

  bb.set("b", std::string("nope"));
  EXPECT_EQ(bb.value_or<int>("b", 99), 99);

  std::string def = "default";
  EXPECT_EQ(bb.value_or<std::string>("c", def), "default");
}

TEST(Blackboard, GetOrEmplace_ReturnsExistingOrCreates_AndOverwritesWrongType) {
  Blackboard bb;

  auto& v1 = bb.get_or_emplace<std::vector<int>>("vec", 2, 3);
  EXPECT_EQ(v1.size(), 2u);
  EXPECT_EQ(v1[0], 3);
  EXPECT_EQ(v1[1], 3);

  v1[0] = 8;
  auto& v1_again = bb.get_or_emplace<std::vector<int>>("vec", 10, 99);
  EXPECT_EQ(&v1, &v1_again);
  EXPECT_EQ(v1_again[0], 8);
  EXPECT_EQ(v1_again.size(), 2u);

  bb.set("x", std::string("string"));
  auto& xi = bb.get_or_emplace<int>("x", 77);
  EXPECT_EQ(xi, 77);
  EXPECT_EQ(bb.try_get<std::string>("x"), nullptr); // replaced -> string gone
  EXPECT_NE(bb.try_get<int>("x"), nullptr);
}

TEST(Blackboard, ContainsEraseClear) {
  Blackboard bb;

  bb.set("k1", 1);
  bb.set("k2", 2);
  EXPECT_TRUE(bb.contains("k1"));
  EXPECT_TRUE(bb.contains("k2"));

  bb.erase("k1");
  EXPECT_FALSE(bb.contains("k1"));
  EXPECT_TRUE(bb.contains("k2"));
  EXPECT_EQ(bb.try_get<int>("k1"), nullptr);

  bb.clear();
  EXPECT_FALSE(bb.contains("k2"));
  EXPECT_EQ(bb.try_get<int>("k2"), nullptr);
}

TEST(Blackboard, PointerLikeTypes_SetAndEmplace_SharedPtr) {
  Blackboard bb;

  // set with shared_ptr (copyable → allowed by std::any)
  bb.set("p", std::make_shared<int>(5));
  auto* pp = bb.try_get<std::shared_ptr<int>>("p");
  ASSERT_NE(pp, nullptr);
  ASSERT_NE(pp->get(), nullptr);
  EXPECT_EQ(**pp, 5);

  // emplace shared_ptr from raw pointer
  auto& sp = bb.emplace<std::shared_ptr<int>>("q", new int(9));
  ASSERT_NE(sp.get(), nullptr);
  EXPECT_EQ(*sp, 9);

  // overwrite with another shared_ptr
  bb.set("q", std::make_shared<int>(42));
  auto* pq = bb.try_get<std::shared_ptr<int>>("q");
  ASSERT_NE(pq, nullptr);
  ASSERT_NE(pq->get(), nullptr);
  EXPECT_EQ(**pq, 42);
}

TEST(Blackboard, EmptyKeyAndWeirdKeys) {
  Blackboard bb;

  bb.set("", 123);
  EXPECT_TRUE(bb.contains(""));
  EXPECT_NE(bb.try_get<int>(""), nullptr);
  EXPECT_EQ(*bb.try_get<int>(""), 123);

  bb.set("Key", 1);
  bb.set("key", 2);
  EXPECT_EQ(*bb.try_get<int>("Key"), 1);
  EXPECT_EQ(*bb.try_get<int>("key"), 2);

  std::string long_key(10000, 'x');
  bb.set(long_key, 7);
  EXPECT_TRUE(bb.contains(long_key));
  EXPECT_EQ(*bb.try_get<int>(long_key), 7);
}

TEST(Blackboard, ReferenceStability_AfterEmplaceAndMutation) {
  Blackboard bb;

  auto& s = bb.emplace<std::string>("name", "alice");
  EXPECT_EQ(s, "alice");
  s = "bob";
  EXPECT_EQ(*bb.try_get<std::string>("name"), "bob");

  auto& s2 = bb.at<std::string>("name");
  s2.push_back('!');
  EXPECT_EQ(*bb.try_get<std::string>("name"), "bob!");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
