#include "ATEF_BaseNode.h"

#include <cmath>
#include <cstdio>
#include <signal.h>
#include <unistd.h>

// ------------------------------------------
// IMPORTANT!! - Make sure there is a definition for CreateApplicationATEF_BaseNode()
ATEF_BaseNode* CreateApplicationATEF_BaseNode()
{
	return new BasicATEF_BaseNode();        // Make sure to change this to correct ATEF_BaseNode class type
}
// ------------------------------------------


void termination_handler (int signum)
{
  ATEF_BaseNode::Get()->Terminate(); // Example call to terminate the application with OS control signal
}


void BasicATEF_BaseNode::Setup(int argc, char** argv)
{
    std::string input_topic = FindTopicName("input1");
    std::string output_topic = FindTopicName("output1");

    // Example of subscribing to certain topic and connecting to object "input".    
	Subscribe(input_topic, &input);

    // Also example registing "OnReceiveInput" to be an input function called to handle received data from topic
	RegisterInputFunction(input_topic,static_cast<ATEF_BaseNodeFuncPtr>(&BasicATEF_BaseNode::OnReceiveInput));	
	
    // Example of publishing to certain topic and connecting to object "output"
	Publish(output_topic, &output);

    // Example of registering "AppInit" to be an initialization function processed once after Setup()
	RegisterInitFunction(static_cast<ATEF_BaseNodeFuncPtr>(&BasicATEF_BaseNode::AppInit));

    // Example of registering "Process" to be a core function processed continuously
	RegisterCoreFunction(static_cast<ATEF_BaseNodeFuncPtr>(&BasicATEF_BaseNode::Process));

    // Example of registering "OnExit" to be an exit function called before application exit
    RegisterExitFunction(static_cast<ATEF_BaseNodeFuncPtr>(&BasicATEF_BaseNode::OnExit));
}

void BasicATEF_BaseNode::SetATEF_BaseNodeName(int argc, char** argv, std::string& ATEF_BaseNodeName)
{
	ATEF_BaseNodeName = "BasicATEF_BaseNode";
}

void BasicATEF_BaseNode::AppInit()
{	
    // Example application specific initialization 

    recv_input = false;
}


void BasicATEF_BaseNode::OnReceiveInput()
{
	// Example handling of receiving data from "INPUT_TOPIC"

    recv_input = true;
}

void BasicATEF_BaseNode::Process()
{
 	// Example handle termination signal CTRL-C --- Call "termination_handler"
  	if (signal (SIGINT, termination_handler) == SIG_IGN)
    	signal (SIGINT, SIG_IGN);	

    // Example continuous "polling" until all input(s) received
    if(recv_input)
    {
        // Example of modifying output value and flagging data for publishing
        output.SetValue(input.GetValue() * 2.0f);
        output.SetFlagged(true);

        recv_input = false; // reset flag for polling
    }

}


void BasicATEF_BaseNode::OnExit()
{
    // Example handling application exit....
    printf("ATEF_BaseNode Finished.......\n");
}




