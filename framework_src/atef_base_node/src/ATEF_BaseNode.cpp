#include "ATEF_BaseNode.h"
#include "SerialObject.h"
#include "Keyboard.h"
#include <chrono>

// --------- ROS Topic Implementation ----------

using namespace std;
using namespace atef_msgs::msg;
using namespace std_msgs::msg;
using std::placeholders::_1;

class Topic
{
	string name;
	SerialObject* object;

	char* sendBuffer;
	char* recvBuffer;
	ByteMultiArray sendMsg;

	rclcpp::Publisher<ByteMultiArray>::SharedPtr pub;
	rclcpp::Subscription<ByteMultiArray>::SharedPtr sub;

public:
	Topic(string topicName, SerialObject* topicObject);

	string TopicName() { return(name); }
	SerialObject& TopicObject() { return(*object); }

	void Publish();
    void Callback(const ByteMultiArray::SharedPtr msg);

	rclcpp::Publisher<ByteMultiArray>::SharedPtr& Publisher() { return(pub); }
	rclcpp::Subscription<ByteMultiArray>::SharedPtr& Subscriber() { return(sub); }
};

Topic::Topic(std::string topicName, SerialObject* topicObject)
{
	name = topicName;
	object = topicObject;
	sendBuffer = new char[topicObject->GetObjectSize()];	// allocate buffer space
	recvBuffer = new char[topicObject->GetObjectSize()];	// allocate buffer space
}

void Topic::Publish()
{
	object->Serialize(sendBuffer);  // obtain serialized data buffer	

	sendMsg.data.clear();								// clear existing data
	for (int i = 0; i < object->GetObjectSize(); i++)	// fill message buffer
		sendMsg.data.push_back(sendBuffer[i]);

	pub->publish(sendMsg); 			// publishing message

}

void Topic::Callback(const ByteMultiArray::SharedPtr msg)
{
    for(int i = 0; i < object->GetObjectSize(); i++)	// Convert msg vector to char array
	  recvBuffer[i] = (char)msg->data[i];

    object->Deserialize(recvBuffer);		// Deserialize data
	
	ATEF_BaseNode::Get()->CallInputFunction(name); 	// Notify input function    

}

// ---------------------------------------------

// used to sense if key for saving state was pressed
bool KEY_PRESSED = false;

void ATEF_BaseNode::KeyEventListener(){
	rclcpp::Rate loop_rate(20);
	while(rclcpp::ok() && !terminate){		
		Keyboard_Update(0, 1000);	// checking for keyboard input
		KEY_PRESSED = (Keyboard_GetLastKey() == 's');
		Keyboard_Cleanup();
		loop_rate.sleep();
	}
}

// --------- ROS ATEF_BaseNode Implementation ----------
shared_ptr<rclcpp::Node> NodeHandle;
ATEF_BaseNode* ATEF_BaseNode::instance = NULL;


ATEF_BaseNode* ATEF_BaseNode::Get()
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


ATEF_BaseNode::ATEF_BaseNode()
{
	terminate = false;
}


ATEF_BaseNode::~ATEF_BaseNode()
{
	// delete NodeHandle;
}

string ATEF_BaseNode::GetNodeName(){ return NodeHandle->get_name(); }

void ATEF_BaseNode::Setup(int argc, char** argv) {}

void ATEF_BaseNode::Run(int argc, char** argv)
{	
	rclcpp::init(argc, argv);				// 
	
	NodeHandle = std::make_shared<rclcpp::Node>("node");

	string topicName = string(NodeHandle->get_name()) + "Loop";
	loop_sub = NodeHandle->create_subscription<Bool>(topicName, 5, bind(&ATEF_BaseNode::Loop, this, _1));
	loop_pub = NodeHandle->create_publisher<Bool>(topicName, 5);

	loop_msg.data = true;

	Setup(argc,argv);		// call to setup application-specific initialization

	for (auto func = initFunctions.begin(); func != initFunctions.end(); func++)	// call init functions from this class
	  (ATEF_BaseNode::Get()->*(*func)) ();

	bool StateLoad;
	NodeHandle->declare_parameter<bool>("STATE_LOAD", false);
  	NodeHandle->get_parameter("STATE_LOAD", StateLoad);
	
	// loading saved state information
	if (StateLoad){
		SaveStateLoad(string(NodeHandle->get_name()));
	}

	// starting key listener thread
	key_listener_t = thread(&ATEF_BaseNode::KeyEventListener, this);

	// initating control loop
	loop_pub->publish(loop_msg);

	// starting ROS2 spinner
	rclcpp::spin(NodeHandle);
}

// control loop
void ATEF_BaseNode::Loop(const std_msgs::msg::Bool::SharedPtr msg)
{
	(void)msg;
	for (auto func = coreFunctions.begin(); func != coreFunctions.end(); func++)	// call all core functions
	  (ATEF_BaseNode::Get()->*(*func)) ();

	for (auto topic = publishers.begin(); topic != publishers.end(); topic++)	// call all publishers with flagged data
	{
		if ((*topic)->TopicObject().GetFlagged())
		{
			(*topic)->TopicObject().SetFlagged(false);
			(*topic)->Publish();
		}
	}

	if (terminate){
		for (auto func = exitFunctions.begin(); func != exitFunctions.end(); func++)	// call all exit functions before control loop exits
			(ATEF_BaseNode::Get()->*(*func)) ();
		
		exit(0);
	}
	
	if (KEY_PRESSED){
		KEY_PRESSED = false;
		SaveStateSave(GetNodeName());
	}

	loop_pub->publish(loop_msg);
}


void ATEF_BaseNode::Terminate()
{
	terminate = true;
}

void ATEF_BaseNode::Subscribe(std::string topicName, SerialObject* object)
{
	Topic* t = new Topic(topicName, object);
	t->Subscriber() = NodeHandle->create_subscription<ByteMultiArray>(topicName, 1000, bind(&Topic::Callback, t, _1));
	subscriptions.push_back(t);
}

void ATEF_BaseNode::Publish(std::string topicName, SerialObject* object)
{
	Topic* t = new Topic(topicName, object);
	t->Publisher() = NodeHandle->create_publisher<ByteMultiArray>(topicName, 1000);
	publishers.push_back(t);
}


void ATEF_BaseNode::RegisterInitFunction(NodeFuncPtr f)
{
	initFunctions.push_back(f);
}

void ATEF_BaseNode::RegisterInputFunction(std::string topicName, NodeFuncPtr f)
{
	inputFunctions.insert(std::make_pair(topicName, f));		// add input function if NOT registered
}

void ATEF_BaseNode::RegisterCoreFunction(NodeFuncPtr f)
{
	coreFunctions.push_back(f);
}

void ATEF_BaseNode::RegisterExitFunction(NodeFuncPtr f)
{
	exitFunctions.push_back(f);
}

void ATEF_BaseNode::CallInputFunction(std::string topicName)
{
	if(inputFunctions.find(topicName) != inputFunctions.end()) // call input function if registered
	  (ATEF_BaseNode::Get()->*(inputFunctions[topicName])) ();
}


std::string ATEF_BaseNode::FindTopicName(std::string parameterName)
{
	std::string topicName;


	NodeHandle->declare_parameter<string>("~" + parameterName, "");
  	NodeHandle->get_parameter("~" + parameterName, topicName);

	return(topicName);
}

// ---------------------------------------------

