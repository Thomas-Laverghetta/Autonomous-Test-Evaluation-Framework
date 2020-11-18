#include "BasicNode.h"

#include <cmath>
#include <cstdio>
#include <signal.h>
#include <unistd.h>

// using namespace ATEF;

// ------------------------------------------
// IMPORTANT!! - Make sure there is a definition for CreateApplicationATEF_BaseNode()
Node* CreateApplicationNode()
{
	return new BaseNode();        // Make sure to change this to correct Node class type
}
// ------------------------------------------


void termination_handler (int signum)
{
  Node::Get()->Terminate(); // Example call to terminate the application with OS control signal
}


void BaseNode::Setup(int argc, char** argv)
{
    std::string input_topic = FindTopicName("TopicInput1");
    std::string output_topic = FindTopicName("TopicOutput1");

    // Example of subscribing to certain topic and connecting to object "input".    
	Subscribe(input_topic, &input);

    // Also example registing "OnReceiveInput" to be an input function called to handle received data from topic
	RegisterInputFunction(input_topic,static_cast<NodeFuncPtr>(&BaseNode::OnReceiveInput));	
	
    // Example of publishing to certain topic and connecting to object "output"
	Publish(output_topic, &output);

    // Example of registering "AppInit" to be an initialization function processed once after Setup()
	RegisterInitFunction(static_cast<NodeFuncPtr>(&BaseNode::AppInit));

    // Example of registering "Process" to be a core function processed continuously
	RegisterCoreFunction(static_cast<NodeFuncPtr>(&BaseNode::Process));

    // Example of registering "OnExit" to be an exit function called before application exit
    RegisterExitFunction(static_cast<NodeFuncPtr>(&BaseNode::OnExit));
}

void BaseNode::AppInit()
{	
    // Example application specific initialization
    input.data = 1;
    output.data = 1; 
    recv_input = false;
}


void BaseNode::OnReceiveInput()
{
	// Example handling of receiving data from "INPUT_TOPIC"
    recv_input = true;
}

void BaseNode::Process()
{
 	// Example handle termination signal CTRL-C --- Call "termination_handler"
  	if (signal (SIGINT, termination_handler) == SIG_IGN)
    	signal (SIGINT, SIG_IGN);	

    // Example continuous "polling" until all input(s) received
    
        // Example of modifying output value and flagging data for publishing
        output.data = input.data * 2.0f;
        output.publish();

        //recv_input = false; // reset flag for polling
        sleep(1);

    printf("Input=%f : Output=%f\n", input.data, output.data); fflush(stdout);
}


void BaseNode::OnExit()
{
    // Example handling application exit....
    printf("Node Finished.......\n");
}




