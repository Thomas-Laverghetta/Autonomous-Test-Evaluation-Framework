#include "ATEF_Base.h"
#include "SerialObject.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte.hpp"

// --------- ROS Topic Implementation ----------

using namespace std;
using namespace std_msgs::msg;
using std::placeholders::_1;

class Topic
{
	string name;
	SerialObject* object;

	char* sendBuffer;
	char* recvBuffer;
	Byte sendMsg[];

	rclcpp::Publisher<Byte[]>::SharedPtr pub;
	rclcpp::Subscription<Byte[]>::SharedPtr sub;

public:
	Topic(string topicName, SerialObject* topicObject);

	string TopicName() { return(name); }
	SerialObject& TopicObject() { return(*object); }

	void Publish();
    void Callback(const Byte[]::SharedPtr msg);

	rclcpp::Publisher<Byte[]>::SharedPtr& Publisher() { return(pub); }
	rclcpp::Subscription<Byte[]>::SharedPtr& Subscriber() { return(sub); }
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

void Topic::Callback(const Byte[]::SharedPtr msg)
{
    for(int i = 0; i < object->GetObjectSize(); i++)	// Convert msg vector to char array
	  recvBuffer[i] = (char)msg->data[i];

    object->Deserialize(recvBuffer);		// Deserialize data
	
	ATEF_BaseNode::Get()->CallInputFunction(name); 	// Notify input function    

}

// ---------------------------------------------



// --------- ROS ATEF_BaseNode Implementation ----------
shared_ptr<rclcpp::ATEF_BaseNode> ATEF_BaseNodeHandle;
ATEF_BaseNode* ATEF_BaseNode::instance = NULL;

ATEF_BaseNode* ATEF_BaseNode::Get()
{
	if (instance == NULL)
	{
		instance = CreateApplicationATEF_BaseNode();
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
	delete ATEF_BaseNodeHandle;
}


void ATEF_BaseNode::Setup(int argc, char** argv) {}


void ATEF_BaseNode::SetATEF_BaseNodeName(int argc, char** argv, std::string& ATEF_BaseNodeName)
{
	ATEF_BaseNodeName = ("Unnamed");		// Default ATEF_BaseNode Name
}

void ATEF_BaseNode::Init(int argc, char** argv)
{	
	SetATEF_BaseNodeName(argc, argv, _ATEF_BaseNodeName);

	rclcpp::init(argc, argv);				// 
	
	ATEF_BaseNodeHandle = std::make_shared<rclcpp::ATEF_BaseNode>(_ATEF_BaseNodeName);

	Setup(argc,argv);		// call to setup application-specific initialization

	for (auto func = initFunctions.begin(); func != initFunctions.end(); func++)	// call init functions from this class
	  (ATEF_BaseNode::Get()->*(*func)) ();
}

void ATEF_BaseNode::Loop()
{
	while (terminate == false)	// Loop endlessly until terminated
	{
		ros::spinOnce();		// query ROS to process callbacks

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
	}

	for (auto func = exitFunctions.begin(); func != exitFunctions.end(); func++)	// call all exit functions before control loop exits
		(ATEF_BaseNode::Get()->*(*func)) ();
}


void ATEF_BaseNode::Terminate()
{
	terminate = true;
}

void ATEF_BaseNode::Subscribe(std::string topicName, SerialObject* object)
{
	Topic* t = new Topic(topicName, object);
	t->Subscriber() = ATEF_BaseNodeHandle->subscribe(topicName, 1000, bind(&Topic::Callback, t, _1));
	subscriptions.push_back(t);
}

void ATEF_BaseNode::Publish(std::string topicName, SerialObject* object)
{
	Topic* t = new Topic(topicName, object);
	t->Publisher() = ATEF_BaseNodeHandle->create_publisher<Byte[]>(topicName, 1000);
	publishers.push_back(t);
}


void ATEF_BaseNode::RegisterInitFunction(ATEF_BaseNodeFuncPtr f)
{
	initFunctions.push_back(f);
}

void ATEF_BaseNode::RegisterInputFunction(std::string topicName, ATEF_BaseNodeFuncPtr f)
{
	inputFunctions.insert(std::make_pair(topicName, f));		// add input function if NOT registered
}

void ATEF_BaseNode::RegisterCoreFunction(ATEF_BaseNodeFuncPtr f)
{
	coreFunctions.push_back(f);
}

void ATEF_BaseNode::RegisterExitFunction(ATEF_BaseNodeFuncPtr f)
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

	if(ros::param::has("~" + parameterName))
		ros::param::get("~" + parameterName, topicName);
	else
		topicName = "";

	return(topicName);
}

// ---------------------------------------------

