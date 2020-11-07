#include "ATEF_BaseCombiner.h"
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


void ATEF_BaseCombiner::Setup(int argc, char** argv)
{
	CreateObjects(argc, argv, _physicalObject, _virtualObject, _outputObject);
	SetTopicNames(argc, argv, _physicalName, _virtualName, _outputName);
	SetMode(argc, argv, _mode);

	Subscribe(_physicalName, _physicalObject);
	Subscribe(_virtualName, _virtualObject);
	Publish(_outputName, _outputObject);

	RegisterInputFunction(_physicalName, static_cast<NodeFuncPtr>(&ATEF_BaseCombiner::OnReceivePhysical));
	RegisterInputFunction(_virtualName, static_cast<NodeFuncPtr>(&ATEF_BaseCombiner::OnReceiveVirtual));
	RegisterCoreFunction(static_cast<NodeFuncPtr>(&ATEF_BaseCombiner::Process));

	recv_physical = false;
	recv_virtual = false;
}

int ATEF_BaseCombiner::GetMode()
{
	return _mode;
}

void ATEF_BaseCombiner::OnReceivePhysical()
{
	recv_physical = true;
}

void ATEF_BaseCombiner::OnReceiveVirtual()
{
	recv_virtual = true;
}


// Process
// --- Waits until approapriate data has been received before calling Combine() and flagging output data for publishing.
// --- Note: Combine will be called once all data is received based on the current mode of operation.
// --- (i.e. if mode is set to 0, Combine will be called once physical data has been received.)
// --- (if mode is set to 2, Combine will be called once both physical and virtual data have been received.)
// --- Note: Standard system termination signal (CTRL-C on Unix/Linux) is checked to in order to terminate ATEF_BaseCombiner ATEF_BaseNode
void ATEF_BaseCombiner::Process()
{	
 	// ---- Termination Signal ------ //
  	if (signal (SIGINT, termination_handler) == SIG_IGN)
    	signal (SIGINT, SIG_IGN);

	if(_mode == 0 && recv_physical)
	{
		Combine(_physicalObject, _virtualObject, _outputObject);
		_outputObject->SetFlagged(true);
		recv_physical = false;
	}
	else if(_mode == 1 && recv_virtual)
	{
		Combine(_physicalObject, _virtualObject, _outputObject);
		_outputObject->SetFlagged(true);
		recv_virtual = false;
	}
	else if (_mode == 2 && recv_physical && recv_virtual)
	{
		Combine(_physicalObject, _virtualObject, _outputObject);
		_outputObject->SetFlagged(true);
		recv_physical = false;
		recv_virtual = false;
	}
}