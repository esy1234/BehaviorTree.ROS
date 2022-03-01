// Copyright (c) 2019 Samsung Research America
// Copyright (c) 2020 Davide Faconti
// Copyright (c) 2021 E Shengyang
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BEHAVIOR_TREE_BT_TOPIC_NODE_H
#define BEHAVIOR_TREE_BT_TOPIC_NODE_H

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

namespace BT
{

/** Helper Node to publish an MessageT inside a BT::ActionNode
 *
 * Note that the user must implement the methods:
 *
 *  - sendMessage
 */
template<class MessageT>
class RosPublisherNode : public BT::SyncActionNode
{
protected:
  RosPublisherNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(name, conf)
    , node_(nh)
  {
    publisher_init_ = false;
  }

public:
  using MessageType = MessageT;

  RosPublisherNode() = delete;

  virtual ~RosPublisherNode() = default;

  /// These ports will be added automatically if this Node is
  /// registered using RegisterRosPublisher<DeriveClass>()
  static PortsList providedPorts()
  {
    return {
      InputPort<std::string>("topic_name", "name of the ROS topic")
    };
  }

  /// User must implement this method
  // If it return false, don't publish message.
  virtual bool sendMessage(MessageType& msg) = 0;

protected:
  // The node that will be used for any ROS operations
  ros::NodeHandle& node_;

  ros::Publisher publisher_;
  bool publisher_init_;

  BT::NodeStatus tick() override
  {
    if (!publisher_init_)
    {
      std::string topic = getInput<std::string>("topic_name").value();
      publisher_ = node_.advertise<MessageT>(topic, 5, false);
    }

    MessageType msg;
    bool valid_msg = sendMessage(msg);
    if (!valid_msg)
    {
      return NodeStatus::FAILURE;
    }
    publisher_.publish(msg);
    return NodeStatus::SUCCESS;
  }
};


/// Method to register the service into a factory.
template <class DerivedT>
static void RegisterRosPublisher(BT::BehaviorTreeFactory& factory,
  const std::string& registration_ID,
  ros::NodeHandle& node_handle)
{
  NodeBuilder builder = [&node_handle](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(node_handle, name, config);
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = RosPublisherNode<typename DerivedT::MessageType>::providedPorts();
  manifest.ports.insert(basic_ports.begin(), basic_ports.end());

  factory.registerBuilder(manifest, builder);
}

} // end namespace BT

#endif // BEHAVIOR_TREE_BT_TOPIC_NODE_H
