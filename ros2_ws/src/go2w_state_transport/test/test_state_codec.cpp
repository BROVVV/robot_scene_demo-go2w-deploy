#include "go2w_state_transport/state_codec.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <atomic>
#include <filesystem>
#include <thread>
#include <unistd.h>
using namespace go2w_state_transport;
static LowSample sample() {
  LowSample s;s.monotonic=10.;s.wall=1000.;s.boot_id="current-boot";s.soc=77;s.temperature=24;
  for (std::size_t i=0;i<16;++i) {s.q[i]=static_cast<float>(i)*.1234567f;s.dq[i]=-s.q[i];}
  s.quaternion={1.f,0.f,0.f,0.f};s.accelerometer={.5f,-.1f,9.8f};s.rpy={.01f,-.02f,.03f};
  return s;
}
TEST(StateCodec, FullMotorAndImuValuesSurviveSerialization) {
  const auto s=sample();const auto r=decode(encode(s),10.1,1000.1,.3,s.boot_id);
  ASSERT_TRUE(r);EXPECT_EQ(r->q,s.q);EXPECT_EQ(r->dq,s.dq);EXPECT_EQ(r->quaternion,s.quaternion);
  EXPECT_EQ(r->accelerometer,s.accelerometer);EXPECT_EQ(r->rpy,s.rpy);EXPECT_EQ(r->soc,77);
}
TEST(StateCodec, PreviousBootCannotBecomeFreshAtMatchingUptime) {
  const auto s=sample();EXPECT_FALSE(decode(encode(s),10.1,1000.1,.3,"another-boot"));
}
TEST(StateCodec, FutureMonotonicAndWallClocksAreRejected) {
  const auto s=sample();EXPECT_FALSE(decode(encode(s),9.9,1000.1,.3,s.boot_id));
  EXPECT_FALSE(decode(encode(s),10.1,999.9,.3,s.boot_id));
}
TEST(StateCodec, OldWallClockRejectsPreviousBootWithoutBootMetadata) {
  const std::string legacy=R"({"capture_monotonic":10,"capture_wall_time":100,"wheel_q":[1,2,3,4],"wheel_dq":[0,0,0,0]})";
  EXPECT_FALSE(decode(legacy,10.1,1000.,.3,"new-boot"));
  const auto valid=decode(legacy,10.1,100.1,.3,"new-boot");ASSERT_TRUE(valid);EXPECT_EQ(valid->q[12],1.f);
}
TEST(StateCodec, RejectedFuturePacketDoesNotBlockSubsequentFreshPacket) {
  auto s=sample();s.monotonic=10000.;EXPECT_FALSE(decode(encode(s),10.1,1000.1,.3,s.boot_id));
  s.monotonic=10.;EXPECT_TRUE(decode(encode(s),10.1,1000.1,.3,s.boot_id));
}
TEST(StateCodec, StaleAndInvalidFramesNeverProduceState) {
  auto s=sample();EXPECT_FALSE(decode(encode(s),11.,1000.1,.3,s.boot_id));
  EXPECT_FALSE(decode(encode(s),10.1,1001.,.3,s.boot_id));
  s.q[3]=std::numeric_limits<float>::quiet_NaN();EXPECT_FALSE(decode(encode(s),10.1,1000.1,.3,s.boot_id));
  EXPECT_FALSE(decode("not-json",10.,1000.,.3,"current-boot"));
  EXPECT_FALSE(decode(std::string(65537,' '),10.,1000.,.3,"current-boot"));
}
TEST(StateCodec, IntegerOverflowCannotWrapIntoValidBatteryValue) {
  const std::string bad=R"({"capture_monotonic":10,"capture_wall_time":1000,"wheel_q":[1,2,3,4],"wheel_dq":[0,0,0,0],"battery_soc":4294967373})";
  EXPECT_FALSE(decode(bad,10.1,1000.1,.3,"current-boot"));
}
TEST(StateCodec, AtomicReplacementNeverExposesPartialJsonToConcurrentReader) {
  char pattern[]="/tmp/go2w_state_codec_XXXXXX";
  const char * directory=::mkdtemp(pattern);ASSERT_NE(directory,nullptr);
  const auto path=std::string(directory)+"/state.json";auto s=sample();atomic_write(path,encode(s));
  std::atomic<bool> done{false};std::atomic<int> bad{0};
  std::thread reader([&]() {
    while (!done.load()) {
      auto text=bounded_read(path);
      if (!text || !decode(*text,10.1,1000.1,.3,s.boot_id)) {++bad;}
    }
  });
  for (int i=0;i<20;++i) {s.q[0]=static_cast<float>(i);atomic_write(path,encode(s));}
  done.store(true);reader.join();EXPECT_EQ(bad.load(),0);
  EXPECT_EQ(std::distance(std::filesystem::directory_iterator(directory),
    std::filesystem::directory_iterator{}),1);
  std::filesystem::remove_all(directory);
}
