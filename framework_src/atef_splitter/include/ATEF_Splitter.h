#ifndef ATEF_SPLITTER_H
#define ATEF_SPLITTER_H

#include <string>
#include <vector>

#include "ATEF_Node.h"

// namespace ATEF{
	class Splitter : public Node
	{
	private:
		SerialObject* _inputObject;
		SerialObject* _physicalObject;
		SerialObject* _virtualObject;	
		
		std::string _inputName;
		std::string _physicalName;
		std::string _virtualName;	

		int _mode;
	public:	

		int GetMode();
	protected:

		void Setup(int argc, char** argv);

		virtual void CreateObjects(int argc, char** argv, SerialObject*& physicalObject, SerialObject*& virtualObject, SerialObject*& inputObject) = 0;

		virtual void SetTopicNames(int argc, char** argv, std::string& physicalName, std::string& virtualName, std::string& inputName) = 0;

		virtual void SetMode(int argc, char**argv, int& mode) = 0;

		virtual void Split(SerialObject* physicalObject, SerialObject* virtualObject, SerialObject* inputObject) = 0;	
	private:

		void OnReceiveInput();	

		void Process();	
	};
// }
#endif