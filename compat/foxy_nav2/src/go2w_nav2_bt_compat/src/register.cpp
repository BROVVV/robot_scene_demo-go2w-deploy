#include "behaviortree_cpp_v3/bt_factory.h"
void register_compute_path_to_pose(BT::BehaviorTreeFactory &);
void register_follow_path(BT::BehaviorTreeFactory &);
void register_wait(BT::BehaviorTreeFactory &);
BT_REGISTER_NODES(factory)
{
  register_compute_path_to_pose(factory);
  register_follow_path(factory);
  register_wait(factory);
}
