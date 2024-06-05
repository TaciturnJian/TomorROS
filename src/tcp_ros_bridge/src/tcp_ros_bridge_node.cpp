#include <LangYa/Network.hpp>
#include <LangYa/Messages.hpp>
#include <LangYa/CodeHelper.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace LangYa;

int main(int argc, char** argv)
{
    const std::string app_name{ "trb" };

    if (!InitializeGlobalLogger(app_name)) return -1;

    // 初始化 tcp bridge
    using tcp_bridge_flow_type = SyncCommunicationPlanner<TCPSocketMaker, NaviCommandMessage, NaviControlMessage>;
    tcp_bridge_flow_type tcp_bridge_flow{};
    {
        const auto maker_handle = std::make_shared<TCPSocketMaker>();
        maker_handle->Parameters.RemoteEndpoints.emplace_back(
            boost::asio::ip::tcp::endpoint{ boost::asio::ip::address_v4::from_string("127.0.0.1"), 8989 });
        tcp_bridge_flow.Acceptor = maker_handle;
        tcp_bridge_flow.ReaderDestination = std::make_shared<EmptyDestinationNode<NaviCommandMessage>>();
    }

    if (!tcp_bridge_flow.Check()) return -1;

    // 初始化 ros
    ros::init(argc, argv, "tcp_ros_bridge_node");
    ros::NodeHandle node_handle{};
    ros::Publisher publisher = node_handle.advertise<std_msgs::String>("test_topic", 10);

    std::atomic_bool ros_done{ false };

    tf::Taskflow taskflow;

    tcp_bridge_flow.Plan(taskflow);
    taskflow.emplace(
        [&publisher, &ros_done]
        {
            std_msgs::String msg{};
            ros::Rate publisher_loop_rate{ 10 };
            std::uint64_t i = 0;

            while (ros::ok())
            {
                std::ostringstream stream{};
                stream << "hello" << i++ << '\n';
                msg.data = stream.str();
                publisher.publish(msg);
                ros::spinOnce();
                publisher_loop_rate.sleep();
            }

            ros_done = true;
        }).name("Publisher");

    taskflow.emplace([&ros_done, &tcp_bridge_flow]
        {
            while (!ros_done && !tcp_bridge_flow.Done) std::this_thread::sleep_for(std::chrono::seconds{ 2 });
        }).name("Observer");

    tf::Executor{}.run(taskflow).wait();

    std::ostringstream stream{};
    taskflow.dump(stream);
    spdlog::info("MainDump> {}", stream.str());

    return 0;
}
