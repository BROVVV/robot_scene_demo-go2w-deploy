// Passive migration evidence only. No SDK, service/action clients or motion publishers.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <map>
#include <cmath>
#include <ctime>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <stdexcept>

using json=nlohmann::json;
namespace fs=std::filesystem;
double clock_s(clockid_t which) { timespec t{};if(clock_gettime(which,&t))throw std::runtime_error("clock_gettime");return t.tv_sec+t.tv_nsec*1e-9; }
json read_json(const fs::path &p) {std::ifstream f(p);json j;f>>j;return j;}
std::string text_file(const fs::path &p) {std::ifstream f(p);return {std::istreambuf_iterator<char>(f),{}};}

class Trace final:public rclcpp::Node {
public:
 Trace(fs::path out,fs::path low,double duration):Node("migration_control_idle_passive_trace_native"),out_(out),low_(low),duration_(duration) {
  for(auto n:{"safety_trace.jsonl","resources.jsonl","collector_ready.json","collector_terminal.json"})
   if(fs::exists(out_/n))throw std::runtime_error("refusing existing trace file");
  trace_.open(out_/"safety_trace.jsonl");resources_.open(out_/"resources.jsonl");
  if(!trace_||!resources_)throw std::runtime_error("cannot open trace output");
  boot_=text_file("/proc/sys/kernel/random/boot_id");while(!boot_.empty()&&boot_.back()=='\n')boot_.pop_back();
  auto sensor=rclcpp::SensorDataQoS();
  front_=create_subscription<std_msgs::msg::Float32>("/go2w/safety/front_clearance",sensor,[this](std_msgs::msg::Float32::SharedPtr m){
   // Preserve the previous Python trace's NaN/Infinity JSON extensions exactly.
   phase();auto row=prefix("front");std::string s=row.dump();s.pop_back();
   s+=",\"value\":";if(std::isnan(m->data))s+="NaN";else if(std::isinf(m->data))s+=m->data>0?"Infinity":"-Infinity";else s+=json(m->data).dump();
   trace_<<s<<"}\n";
  });
  lidar_=create_subscription<std_msgs::msg::Bool>("/go2w/safety/lidar_fresh",10,[this](std_msgs::msg::Bool::SharedPtr m){record("lidar",m->data);});
  health_=create_subscription<std_msgs::msg::String>("/go2w/nav/localization_status",10,[this](std_msgs::msg::String::SharedPtr m){status("health",m->data);});
  geometry_=create_subscription<std_msgs::msg::String>("/go2w/lidar/geometry_status",10,[this](std_msgs::msg::String::SharedPtr m){status("geometry",m->data);});
  odom_=create_subscription<nav_msgs::msg::Odometry>("/go2w/odom/fused",sensor,[this](nav_msgs::msg::Odometry::SharedPtr m){record("odom",{{"x",m->pose.pose.position.x},{"y",m->pose.pose.position.y}});});
  start_=clock_s(CLOCK_MONOTONIC);next_resources_=start_;
  timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{tick();});
  std::ofstream(out_/"collector_ready.json")<<json{{"pid",getpid()},{"wall",clock_s(CLOCK_REALTIME)},{"boot_id",boot_},{"backend","native_readonly"}}.dump();
 }
 bool done()const{return done_;}
 void finish() {
  trace_.flush();resources_.flush();
  std::ofstream(out_/"collector_terminal.json")<<json{{"counts",counts_},{"seconds",clock_s(CLOCK_MONOTONIC)-start_},{"motion_sent",false},{"backend","native_readonly"},{"parse_errors",parse_errors_}}.dump(2);
 }
private:
 void phase() {try{phase_=read_json(out_/"phase.json").at("phase").get<std::string>();}catch(const std::exception&){} }
 json prefix(const std::string &topic){++counts_[phase_+":"+topic];return {{"wall",clock_s(CLOCK_REALTIME)},{"mono",clock_s(CLOCK_MONOTONIC)},{"phase",phase_},{"topic",topic}};}
 void record(const std::string &topic,json value){phase();auto row=prefix(topic);row["value"]=std::move(value);trace_<<row.dump()<<'\n';}
 void status(const std::string &topic,const std::string &s) {
  try{record(topic,json::parse(s));}catch(const std::exception& e){++parse_errors_;record("invalid_"+topic,{{"raw",s},{"error",e.what()}});}
 }
 struct Cpu {std::string start;double ticks;};std::map<std::string,Cpu> previous_;
 void resources(double now) {
  json rows=json::array();std::map<std::string,Cpu> current;
  for(const auto &entry:fs::directory_iterator("/proc")) {
   auto name=entry.path().filename().string();if(name.empty()||name.find_first_not_of("0123456789")!=std::string::npos)continue;
   try{
    auto stat=text_file(entry.path()/"stat");auto end=stat.rfind(')');if(end==std::string::npos)continue;
    std::istringstream stream(stat.substr(end+2));std::vector<std::string> f;std::string token;while(stream>>token)f.push_back(token);if(f.size()<22)continue;
    double ticks=std::stod(f[11])+std::stod(f[12]);current[name]={f[19],ticks};auto old=previous_.find(name);
    if(old==previous_.end()||old->second.start!=f[19]||last_resource_==0)continue;
    double cpu=100*(ticks-old->second.ticks)/sysconf(_SC_CLK_TCK)/(now-last_resource_);if(cpu<=1)continue;
    json argv=json::array();std::istringstream cmd(text_file(entry.path()/"cmdline"));for(int i=0;i<3&&std::getline(cmd,token,'\0');++i)argv.push_back(token);
    rows.push_back({{"pid",std::stoi(name)},{"cpu_percent",cpu},{"rss",std::stoll(f[21])*sysconf(_SC_PAGESIZE)},{"argv",argv}});
   }catch(const std::exception&){} // A process may exit between /proc reads.
  }
  resources_<<json{{"wall",clock_s(CLOCK_REALTIME)},{"phase",phase_},{"processes",rows}}.dump()<<'\n';resources_.flush();previous_=std::move(current);last_resource_=now;
 }
 void tick() {
  phase();double now=clock_s(CLOCK_MONOTONIC);
  if(phase_=="finished"||now-start_>=duration_){done_=true;return;}
  if(now-last_low_>=.1){last_low_=now;try{
   auto low=read_json(low_);double age=now-low.at("capture_monotonic").get<double>();
   double wall_age=clock_s(CLOCK_REALTIME)-low.at("capture_wall_time").get<double>();
   if(age>=0&&age<.5&&wall_age>=0&&wall_age<.5&&low.at("capture_boot_id")==boot_)record("low",std::move(low));
  }catch(const std::exception&){} }
  if(now>=next_resources_){resources(now);next_resources_=now+2;trace_.flush();}
 }
 fs::path out_,low_;double duration_,start_{},next_resources_{},last_low_{},last_resource_{};bool done_{};
 std::string phase_="starting",boot_;std::map<std::string,unsigned long> counts_;unsigned long parse_errors_{};
 std::ofstream trace_,resources_;rclcpp::TimerBase::SharedPtr timer_;
 rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr front_;
 rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr health_,geometry_;
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
};
int main(int argc,char**argv){
 try{
  fs::path out,low;double duration=165;
  for(int i=1;i<argc;++i){std::string arg=argv[i];if(arg=="--output-dir"&&i+1<argc)out=argv[++i];else if(arg=="--low-state"&&i+1<argc)low=argv[++i];else if(arg=="--duration"&&i+1<argc)duration=std::stod(argv[++i]);else throw std::invalid_argument("unknown/incomplete option: "+arg);}
  if(!fs::is_directory(out)||low.empty()||!std::isfinite(duration)||duration<=0||duration>180)throw std::invalid_argument("existing output dir, low-state path and duration (0,180] required");
  rclcpp::init(argc,argv);auto node=std::make_shared<Trace>(out,low,duration);rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(node);
  while(rclcpp::ok()&&!node->done())exec.spin_once(std::chrono::milliseconds(20));
  node->finish();exec.remove_node(node);node.reset();if(rclcpp::ok())rclcpp::shutdown();return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';if(rclcpp::ok())rclcpp::shutdown();return 2;}
}
