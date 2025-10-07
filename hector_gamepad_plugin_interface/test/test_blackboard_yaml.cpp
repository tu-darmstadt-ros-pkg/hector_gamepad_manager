#include <gtest/gtest.h>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <hector_gamepad_plugin_interface/blackboard.hpp>

// If your yaml_helper.hpp isn’t visible to tests, add minimal fallbacks here:
// But the Blackboard uses the project’s is_* helpers, so we won’t redefine them.

using hector_gamepad_plugin_interface::Blackboard;

namespace
{

// Small helpers to probe existence & type
template<typename T>
bool has_as( const Blackboard &bb, const std::string &key, T *out = nullptr )
{
  if ( auto p = bb.try_get<T>( key ) ) {
    if ( out )
      *out = *p;
    return true;
  }
  return false;
}

} // namespace

TEST( BlackboardYaml, ScalarMapBasic )
{
  const char *txt = R"(
args:
  b_true: true
  b_yes:  yes
  i:      42
  d:      3.14
  s:      "hello"
)";
  YAML::Node root = YAML::Load( txt );
  ASSERT_TRUE( root["args"] );

  Blackboard bb;
  EXPECT_TRUE( bb.set_from_yaml( root["args"], "prefix" ) );

  bool b{};
  int64_t i{};
  double d{};
  std::string s;

  EXPECT_TRUE( has_as<bool>( bb, "prefix/b_true", &b ) ) << "b_true missing/typed wrong";
  EXPECT_TRUE( b );

  EXPECT_TRUE( has_as<bool>( bb, "prefix/b_yes", &b ) );
  EXPECT_TRUE( b );

  EXPECT_TRUE( has_as<int64_t>( bb, "prefix/i", &i ) );
  EXPECT_EQ( i, 42 );

  EXPECT_TRUE( has_as<double>( bb, "prefix/d", &d ) );
  EXPECT_DOUBLE_EQ( d, 3.14 );

  EXPECT_TRUE( has_as<std::string>( bb, "prefix/s", &s ) );
  EXPECT_EQ( s, "hello" );
}

TEST( BlackboardYaml, SequenceHomogeneous )
{
  const char *txt = R"(
args:
  ints:    [1, 2, 3]
  bools:   [true, false, true]
  doubles: [1.0, 2.5, 3]   # 3 is acceptable as double
  strings: ["a", "b", "c"]
)";
  YAML::Node root = YAML::Load( txt );
  Blackboard bb;
  ASSERT_TRUE( root["args"] );
  EXPECT_TRUE( bb.set_from_yaml( root["args"], "p" ) );

  std::vector<int64_t> vi;
  std::vector<bool> vb = { false, true, false };
  std::vector<double> vd = { 0.0, 0.0, 0.0 };
  std::vector<std::string> vs = { "", "", "" };

  ASSERT_TRUE( has_as<std::vector<int64_t>>( bb, "p/ints", &vi ) );
  EXPECT_EQ( ( std::vector<int64_t>{ 1, 2, 3 } ), vi );

  ASSERT_TRUE( has_as<std::vector<bool>>( bb, "p/bools", &vb ) );
  ASSERT_EQ( 3u, vb.size() );
  EXPECT_TRUE( vb[0] );
  EXPECT_FALSE( vb[1] );
  EXPECT_TRUE( vb[2] );

  ASSERT_TRUE( has_as<std::vector<double>>( bb, "p/doubles", &vd ) );
  ASSERT_EQ( 3u, vd.size() );
  EXPECT_DOUBLE_EQ( vd[0], 1.0 );
  EXPECT_DOUBLE_EQ( vd[1], 2.5 );
  EXPECT_DOUBLE_EQ( vd[2], 3.0 );

  ASSERT_TRUE( has_as<std::vector<std::string>>( bb, "p/strings", &vs ) );
  EXPECT_EQ( ( std::vector<std::string>{ "a", "b", "c" } ), vs );
}

TEST( BlackboardYaml, SequenceMixedTransformedToStringVec )
{
  const char *txt = R"(
args:
  mixed: [1, "a"]
)";
  YAML::Node root = YAML::Load( txt );
  Blackboard bb;
  ASSERT_TRUE( root["args"] );
  // set_from_yaml should return false because mixed scalar types are not allowed
  EXPECT_TRUE( bb.set_from_yaml( root["args"], "p" ) );
  // Ensure no key was created
  ASSERT_TRUE( bb.contains( "p/mixed" ) );
  auto &v = bb.at<std::vector<std::string>>( "p/mixed" );
  EXPECT_EQ( v[0], "1" ); // int 1 transformed to string
  EXPECT_EQ( v[1], "a" ); // string "a" remains unchanged
}

TEST( BlackboardYaml, SequenceWithMapReject )
{
  const char *txt = R"(
args:
  bad: [ {a:1}, 2 ]
)";
  YAML::Node root = YAML::Load( txt );
  Blackboard bb;
  ASSERT_TRUE( root["args"] );
  EXPECT_FALSE( bb.set_from_yaml( root["args"], "p" ) );
  EXPECT_FALSE( bb.contains( "p/bad" ) );
}

TEST( BlackboardYaml, NestedMapRecursion )
{
  const char *txt = R"(
args:
  nested:
    level1:
      level2:
        a: 1
        b: "x"
      c: 1.4
)";
  YAML::Node root = YAML::Load( txt );
  Blackboard bb;
  ASSERT_TRUE( root["args"] );
  EXPECT_TRUE( bb.set_from_yaml( root["args"], "root" ) );

  int64_t a{};
  std::string b;
  double c{};
  EXPECT_TRUE( has_as<int64_t>( bb, "root/nested/level1/level2/a", &a ) );
  EXPECT_EQ( a, 1 );
  EXPECT_TRUE( has_as<std::string>( bb, "root/nested/level1/level2/b", &b ) );
  EXPECT_EQ( b, "x" );
  EXPECT_TRUE( has_as<double>( bb, "root/nested/level1/c", &c ) );
  EXPECT_DOUBLE_EQ( c, 1.4 );
}

TEST( BlackboardYaml, TopLevelSequenceAtPrefix )
{
  YAML::Node seq = YAML::Load( "[10, 20, 30]" );
  Blackboard bb;
  EXPECT_TRUE( bb.set_from_yaml( seq, "top" ) );

  std::vector<int64_t> v;
  ASSERT_TRUE( has_as<std::vector<int64_t>>( bb, "top", &v ) );
  EXPECT_EQ( ( std::vector<int64_t>{ 10, 20, 30 } ), v );
}

TEST( BlackboardYaml, EmptySequencePolicy )
{
  YAML::Node seq = YAML::Load( "[]" );
  Blackboard bb;
  EXPECT_TRUE( bb.set_from_yaml( seq, "empty" ) );

  // By policy we store empty vector<string>
  std::vector<std::string> v;
  ASSERT_TRUE( has_as<std::vector<std::string>>( bb, "empty", &v ) );
  EXPECT_TRUE( v.empty() );
}

TEST( BlackboardYaml, NullValuesAreIgnored )
{
  YAML::Node root = YAML::Load( R"(
args:
  some: null
  other: 1
)" );
  Blackboard bb;
  ASSERT_TRUE( root["args"] );
  EXPECT_TRUE( bb.set_from_yaml( root["args"], "p" ) );
  EXPECT_FALSE( bb.contains( "p/some" ) );
  int64_t other{};
  EXPECT_TRUE( has_as<int64_t>( bb, "p/other", &other ) );
  EXPECT_EQ( other, 1 );
}

TEST( BlackboardYaml, OverwriteExistingKey )
{
  Blackboard bb;
  // First set
  YAML::Node n1 = YAML::Load( "a: 1" );
  EXPECT_TRUE( bb.set_from_yaml( n1, "k" ) );
  int64_t a{};
  ASSERT_TRUE( has_as<int64_t>( bb, "k/a", &a ) );
  EXPECT_EQ( a, 1 );

  // Overwrite with double
  YAML::Node n2 = YAML::Load( "a: 2.5" );
  EXPECT_TRUE( bb.set_from_yaml( n2, "k" ) );
  double d{};
  ASSERT_TRUE( has_as<double>( bb, "k/a", &d ) );
  EXPECT_DOUBLE_EQ( d, 2.5 );
}

TEST( BlackboardYaml, NumericEdgeCases )
{
  YAML::Node n = YAML::Load( R"(
a: 1e2
b: 0x10
c: 07
d: "1.0"   # double
)" );
  Blackboard bb;
  EXPECT_TRUE( bb.set_from_yaml( n, "n" ) );

  double a{};
  ASSERT_TRUE( has_as<double>( bb, "n/a", &a ) );
  EXPECT_DOUBLE_EQ( a, 100.0 );

  // Depending on yaml-cpp, hex and octal decode as int
  int64_t b{};
  ASSERT_TRUE( has_as<int64_t>( bb, "n/b", &b ) );
  EXPECT_EQ( b, 16 );

  int64_t c{};
  ASSERT_TRUE( has_as<int64_t>( bb, "n/c", &c ) );
  EXPECT_EQ( c, 7 );

  double d;
  ASSERT_TRUE( has_as<double>( bb, "n/d", &d ) );
  EXPECT_NEAR( d, 1.0, 1e-6 ); // check double conversion
}

TEST( BlackboardYaml, BoolSynonyms )
{
  YAML::Node n = YAML::Load( R"(
t1: yes
t2: On
f1: no
f2: OFF
f3: false
f4: TRUE
f5: False
f6: true
)" );
  Blackboard bb;
  EXPECT_TRUE( bb.set_from_yaml( n, "b" ) );

  bool t1{}, t2{}, f1{}, f2{}, f3{}, f4{}, f5{}, f6{};
  ASSERT_TRUE( has_as<bool>( bb, "b/t1", &t1 ) );
  EXPECT_TRUE( t1 );
  ASSERT_TRUE( has_as<bool>( bb, "b/t2", &t2 ) );
  EXPECT_TRUE( t2 );
  ASSERT_TRUE( has_as<bool>( bb, "b/f1", &f1 ) );
  EXPECT_FALSE( f1 );
  ASSERT_TRUE( has_as<bool>( bb, "b/f2", &f2 ) );
  EXPECT_FALSE( f2 );
  ASSERT_TRUE( has_as<bool>( bb, "b/f3", &f3 ) );
  EXPECT_FALSE( f3 );
  ASSERT_TRUE( has_as<bool>( bb, "b/f4", &f4 ) );
  EXPECT_TRUE( f4 );
  ASSERT_TRUE( has_as<bool>( bb, "b/f5", &f5 ) );
  EXPECT_FALSE( f5 );
  ASSERT_TRUE( has_as<bool>( bb, "b/f6", &f6 ) );
  EXPECT_TRUE( f6 );
}

TEST( BlackboardYaml, EmptyStringScalar )
{
  YAML::Node n = YAML::Load( "s: ''" );
  Blackboard bb;
  EXPECT_TRUE( bb.set_from_yaml( n, "x" ) );
  std::string s;
  ASSERT_TRUE( has_as<std::string>( bb, "x/s", &s ) );
  EXPECT_TRUE( s.empty() );
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}