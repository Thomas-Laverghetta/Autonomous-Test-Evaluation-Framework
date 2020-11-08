#include "ATEF_Node.h"
#include "SerialObject.h"
#include "Keyboard.h"
#include "int_serial_msg/msg/int_multi_array.hpp"
#include <chrono>

// --------- ROS Topic Implementation ----------
using namespace std;
using namespace int_serial_msg::msg;
using namespace std_msgs::msg;
using std::placeholders::_1;

class Topic
{
	string name;
	SerialObject* object;

	int* sendBuffer;
	int* recvBuffer;
	IntMultiArray sendMsg;

	rclcpp::Publisher<IntMultiArray>::SharedPtr pub;
	rclcpp::Subscription<IntMultiArray>::SharedPtr sub;

public:
	Topic(string topicName, SerialObject* topicObject);

	string TopicName() { return(name); }
	SerialObject& TopicObject() { return(*object); }

	void Publish();
    void Callback(const IntMultiArray::SharedPtr msg);

	rclcpp::Publisher<IntMultiArray>::SharedPtr& Publisher() { return(pub); }
	rclcpp::Subscription<IntMultiArray>::SharedPtr& Subscriber() { return(sub); }
};

Topic::Topic(std::string topicName, SerialObject* topicObject)
{
	name = topicName;
	object = topicObject;
	sendBuffer = new int[topicObject->GetObjectSize()];	// allocate buffer space
	recvBuffer = new int[topicObject->GetObjectSize()];	// allocate buffer space
}

void Topic::Publish()
{
	object->Serialize(sendBuffer);  // obtain serialized data buffer	
	object->flagged = false;		// resetting publish flag
	
	sendMsg.data.clear();								// clear existing data
	for (int i = 0; i < object->GetObjectSize(); i++)	// fill message buffer
		sendMsg.data.push_back(sendBuffer[i]);

	pub->publish(sendMsg); 			// publishing message
}

void Topic::Callback(const IntMultiArray::SharedPtr msg)
{
    for(int i = 0; i < object->GetObjectSize(); i++)	// Convert msg vector to char array
	  recvBuffer[i] = (char)msg->data[i];

    object->Deserialize(recvBuffer);		// Deserialize data
	
	Node::Get()->CallInputFunction(name); 	// Notify input function    

}

// ---------------------------------------------

// used to sense if key for saving state was pressed
bool KEY_PRESSED = false;

void Node::KeyEventListener(){
	rclcpp::Rate loop_rate(20);
	while(rclcpp::ok() && !terminate){		
		Keyboard_Update(0, 1000);	// checking for keyboard input
		KEY_PRESSED = (Keyboard_GetLastKey() == 's');
		Keyboard_Cleanup();
		loop_rate.sleep();
	}
}

// --------- ROS Node Implementation ----------
shared_ptr<rclcpp::Node> NodeHandle;
Node* Node::instance = NULL;


Node* Node::Get()
{
	if (instance == NULL)
	{
		instance = CreateApplicationNode();
		return(instance);
	}
	else
	{
		return(instance);
	}
}


Node::Node()
{
	terminate = false;
}


Node::~Node()
{
	// delete NodeHandle;
}

string Node::GetNodeName(){ return NodeHandle->get_name(); }

void Node::Setup(int argc, char** argv) {}

void Node::Run(int argc, char** argv)
{	
	rclcpp::init(argc, argv);				// 
	
	NodeHandle = std::make_shared<rclcpp::Node>("Node");

	// creating loop
	string topicName = string(NodeHandle->get_name()) + "Loop";
	loop_sub = NodeHandle->create_subscription<Bool>(topicName, 5, bind(&Node::Loop, this, _1));
	loop_pub = NodeHandle->create_publisher<Bool>(topicName, 5);

	// init loop msg
	loop_msg.data = true;

	Setup(argc,argv);		// call to setup application-specific initialization

	for (auto func = initFunctions.begin(); func != initFunctions.end(); func++)	// call init functions from this class
	  (Node::Get()->*(*func)) ();

	bool StateLoad;
	NodeHandle->declare_parameter<bool>("STATE_LOAD", false);
  	NodeHandle->get_parameter("STATE_LOAD", StateLoad);
	
	// loading saved state information
	if (StateLoad){
		SaveStateLoad(string(NodeHandle->get_name()));
	}

	// starting key listener thread
	key_listener_t = thread(&Node::KeyEventListener, this);

	// initating control loop
	loop_pub->publish(loop_msg);

	// starting ROS2 spinner
	rclcpp::spin(NodeHandle);
}

// control loop
void Node::Loop(const std_msgs::msg::Bool::SharedPtr msg)
{
	(void)msg;
	for (auto func = coreFunctions.begin(); func != coreFunctions.end(); func++)	// call all core functions
	  (Node::Get()->*(*func)) ();

	for (auto& topic : publishers) {												// call all publishers with flagged data
		if (topic->TopicObject().GetFlagged())
		{
			// (*topic)->TopicObject().SetFlagged(false);
			topic->Publish();
		}
	}

	if (terminate){
		for (auto func = exitFunctions.begin(); func != exitFunctions.end(); func++)	// call all exit functions before control loop exits
			(Node::Get()->*(*func)) ();
		
		exit(0);
	}
	
	// save state key event flagged
	if (KEY_PRESSED){
		KEY_PRESSED = false;
		SaveStateSave(GetNodeName());
	}

	// next loop
	loop_pub->publish(loop_msg);
}


void Node::Terminate()
{
	terminate = true;
}

void Node::Subscribe(std::string topicName, SerialObject* object)
{
	Topic* t = new Topic(topicName, object);
	t->Subscriber() = NodeHandle->create_subscription<IntMultiArray>(topicName, 1000, bind(&Topic::Callback, t, _1));
	subscriptions.push_back(t);
}

void Node::Publish(std::string topicName, SerialObject* object)
{
	Topic* t = new Topic(topicName, object);
	t->Publisher() = NodeHandle->create_publisher<IntMultiArray>(topicName, 1000);
	publishers.push_back(t);
}


void Node::RegisterInitFunction(NodeFuncPtr f)
{
	initFunctions.push_back(f);
}

void Node::RegisterInputFunction(std::string topicName, NodeFuncPtr f)
{
	inputFunctions.insert(std::make_pair(topicName, f));		// add input function if NOT registered
}

void Node::RegisterCoreFunction(NodeFuncPtr f)
{
	coreFunctions.push_back(f);
}

void Node::RegisterExitFunction(NodeFuncPtr f)
{
	exitFunctions.push_back(f);
}

void Node::CallInputFunction(std::string topicName)
{
	if(inputFunctions.find(topicName) != inputFunctions.end()) // call input function if registered
	  (Node::Get()->*(inputFunctions[topicName])) ();
}


std::string Node::FindTopicName(std::string parameterName)
{
	std::string topicName;


	NodeHandle->declare_parameter<string>("~" + parameterName, "");
  	NodeHandle->get_parameter("~" + parameterName, topicName);

	return(topicName);
}

// ---------------------------------------------

