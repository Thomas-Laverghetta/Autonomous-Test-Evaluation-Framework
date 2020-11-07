#pragma once


#include "ATEF_BaseNode.h"
#include "FloatObject.h"

class BasicATEF_BaseNode : public ATEF_BaseNode
{
private:
	FloatObject	input;
	FloatObject output;	
	bool recv_input;
protected:

	// Setup -- REQUIRED
	// Sets up Subscriptions and Publishing for the ATEF_BaseNode. Registers member functions for execution.
	void Setup(int argc, char** argv);

private:
	// AppInit -- 
	// Example Initialization function.  Called after Setup().  Used for application specific initialization
	void AppInit();	

	// OnReceiveInput --
	// Example Input function. Called on notification of received data from subscription topic. Used to handle receiving new data.
	void OnReceiveInput();	

	// Process --
	// Example Core function. Called every iteration of control loop. Used to do any continuous process the ATEF_BaseNode might require.
	void Process();

	// OnExit -- 
	// Example Exit function. Called right before control loop exits; before the application closes. Used to handle any clean up
	void OnExit();
};