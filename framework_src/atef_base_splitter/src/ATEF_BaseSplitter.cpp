#include "ATEF_BaseSplitter.h"
#include "SerialObject.h"
#include <cstdio>
#include <signal.h>
#include <unistd.h>

// termination_handler
// system termination signal callback.  Makes call to ATEF_BaseNode::Terminate() to halt ATEF_BaseNode's main loop and exit application.
void termination_handler (int signum)
{
  ATEF_BaseNode::Get()->Terminate();
}


void ATEF_BaseSplitter::Setup(int argc, char** argv)
{
	CreateObjects(argc, argv, _physicalObject, _virtualObject, _inputObject);
	SetTopicNames(argc, argv, _physicalName, _virtualName, _inputName);
	SetMode(argc, argv, _mode);

	Subscribe(_inputName, _inputObject);
	Publish(_physicalName, _physicalObject);
	Publish(_virtualName, _virtualObject);	

	RegisterInputFunction(_inputName, static_cast<NodeFuncPtr>(&ATEF_BaseSplitter::OnReceiveInput));		
	RegisterCoreFunction(static_cast<NodeFuncPtr>(&ATEF_BaseSplitter::Process));
}


int ATEF_BaseSplitter::GetMode()
{
	return _mode;
}


void ATEF_BaseSplitter::OnReceiveInput()
{
	if(_mode == 0)
	{
		Split(_physicalObject, _virtualObject, _inputObject);
		_physicalObject->SetFlagged(true);
	}
	else if(_mode == 1)
	{
		Split(_physicalObject, _virtualObject, _inputObject);
		_virtualObject->SetFlagged(true);	
	}
	else if(_mode == 2)
	{
		Split(_physicalObject, _virtualObject, _inputObject);
		_physicalObject->SetFlagged(true);
		_virtualObject->SetFlagged(true);	
	}

}


// Process
// --- Note: Standard system termination signal (CTRL-C on Unix/Linux) is checked to in order to terminate ATEF_BaseSplitter ATEF_BaseNode
void ATEF_BaseSplitter::Process()
{	
 	// ---- Termination Signal ------ //
  	if (signal (SIGINT, termination_handler) == SIG_IGN)
    	signal (SIGINT, SIG_IGN);


}