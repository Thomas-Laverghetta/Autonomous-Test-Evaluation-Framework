#ifndef ATEF_BASE_H
#define ATEF_BASE_H

#include <string>
#include <vector>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "atef_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

class ATEF_BaseNode;
class SerialObject;
class Topic;
typedef void (ATEF_BaseNode::*NodeFuncPtr)(void);


// IMPORTANT!! - Make sure there is a definition for CreateApplicationATEF_BaseNode() in derivative applications
// --- This function should return a new instance of the class deriving from ATEF_BaseNode.
// --- This function is used when creating the ATEF_BaseNode singleton.
extern ATEF_BaseNode* CreateApplicationNode(); 


class ATEF_BaseNode
{
 protected:
	ATEF_BaseNode();
	~ATEF_BaseNode();
	static ATEF_BaseNode* instance;

private:
	friend class Topic;

	bool terminate;
	std::string _nodeName;

	std::vector<NodeFuncPtr> initFunctions;
	std::map<std::string, NodeFuncPtr> inputFunctions;
	std::vector<NodeFuncPtr> coreFunctions;
	std::vector<NodeFuncPtr> exitFunctions;
	std::vector<Topic*> subscriptions;
	std::vector<Topic*> publishers;

	void CallInputFunction(std::string topicName);


	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr loop_pub;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr loop_sub;
	void Loop(const std_msgs::msg::Bool::SharedPtr msg);
	std_msgs::msg::Bool loop_msg;
public:
	static ATEF_BaseNode* Get();
	void Run(int argc, char** argv);
	void Terminate();

protected:
	virtual void Setup(int argc, char** argv);	

	std::string GetNodeName();
	void Subscribe(std::string topicName, SerialObject* object);
	void Publish(std::string topicName, SerialObject* object);

	void RegisterInitFunction(NodeFuncPtr f);
	void RegisterInputFunction(std::string topicName, NodeFuncPtr f);
	void RegisterCoreFunction(NodeFuncPtr f);
	void RegisterExitFunction(NodeFuncPtr f);

	std::string FindTopicName(std::string parameterName);
};

#endif