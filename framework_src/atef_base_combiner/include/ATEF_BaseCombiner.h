#pragma once

#include <string>
#include <vector>

#include "ATEF_BaseNode.h"

class ATEF_BaseCombiner : public ATEF_BaseNode
{
private:
	SerialObject* _physicalObject;
	SerialObject* _virtualObject;
	SerialObject* _outputObject;
	bool recv_physical;
	bool recv_virtual;
	std::string _physicalName;
	std::string _virtualName;
	std::string _outputName;
	int _mode;
public:	
	// GetMode
	// --- Returns the current operational mode of the ATEF_BaseCombiner
	// --- Return value:  integer indicating mode (0 - physical, 1 - virtual, 2 - augmented)
	int GetMode();
protected:
	// Setup
	// --- Registers necessary topics, input functions, and core functions for the ATEF_BaseCombiner to work.
	// --- Params: 	(in) argc - number of program arguments
	// ---			(in) argv - program arguments 
	void Setup(int argc, char** argv);

	// CreateObjects
	// --- Virtual function for creating the appropriate physical, virtual, and output objects for the ATEF_BaseCombiner. Called during Setup()
	// --- Params: 	(in) argc - number of program arguments
	// --- 			(in) argv - program arguments if needed
	// ---			(out) physicalObject - reference to allocate object subscribed to physical data
	// ---			(out) virtualObject - reference to allocate object subscribed to virtual data
	// ---			(out) physicalObject - reference to allocate object published from this ATEF_BaseCombiner
	//
	virtual void CreateObjects(int argc, char** argv, SerialObject*& physicalObject, SerialObject*& virtualObject, SerialObject*& outputObject) = 0;

	// SetTopicNames
	// --- Virtual function for specifying the appropriate topics for subscribing to physical and virtual data and publishing ATEF_BaseCombiner output. Called during Setup()
	// --- Params: 	(in) argc - number of program arguments
	// --- 			(in) argv - program arguments if needed
	// ---			(out) physicalName - reference to topic name for subscribing to physical data
	// ---			(out) virtualName - reference to topic name subscribing to virtual data
	// ---			(out) outputName - reference to topic name for publishing from this ATEF_BaseCombiner
	//
	virtual void SetTopicNames(int argc, char** argv, std::string& physicalName, std::string& virtualName, std::string& outputName) = 0;

	// SetMode
	// --- Virtual function for specifying mode of operation for the ATEF_BaseCombiner.
	// --- Params: 	(in) argc - number of program arguments
	// --- 			(in) argv - program arguments if needed
	// ---			(out) mode - reference to mode variable
	//
	virtual void SetMode(int argc, char**argv, int& mode) = 0;


	// SetTopicNames
	// --- Virtual function for performing the appropriate operation based on mode and incoming data.
	// --- Note: Combine will be called once all data is received based on the current mode of operation.
	// --- (i.e. if mode is set to 0, Combine will be called once physical data has been received.)
	// --- (if mode is set to 2, Combine will be called once both physical and virtual data have been received.)
	// --- Params: 	(in) physicalObject - reference to received physical object data. 
	// ---			(in) virtualObject - reference to received virtual object data. 
	// ---			(out) outputObject - reference to output object data to publish.
	//
	virtual void Combine(SerialObject* physicalObject, SerialObject* virtualObject, SerialObject* outputObject) = 0;	
private:
	// OnReceivePhysical
	// --- Input function handling receiving data from physical topic
	void OnReceivePhysical();

	// OnReceiveVirtual
	// --- Input function handling receiving data from virtual topic.
	void OnReceiveVirtual();

	// Process
	// --- ATEF_BaseCombiner Core function. Waits until approapriate data has been received before calling Combine() and flagging output data for publishing.
	void Process();
};

