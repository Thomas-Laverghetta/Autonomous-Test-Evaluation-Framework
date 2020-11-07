#ifndef ATEF_BASE_H
#define ATEF_BASE_H

#include <string>
#include <vector>
#include <map>

class ATEF_BaseNode;
class SerialObject;
class Topic;
typedef void (ATEF_BaseNode::*ATEF_BaseNodeFuncPtr)(void);


// IMPORTANT!! - Make sure there is a definition for CreateApplicationATEF_BaseNode() in derivative applications
// --- This function should return a new instance of the class deriving from ATEF_BaseNode.
// --- This function is used when creating the ATEF_BaseNode singleton.
extern ATEF_BaseNode* CreateApplicationATEF_BaseNode(); 


class ATEF_BaseNode
{
 protected:
	ATEF_BaseNode();
	~ATEF_BaseNode();
	static ATEF_BaseNode* instance;

private:
	friend class Topic;

	bool terminate;
	std::string _ATEF_BaseNodeName;

	std::vector<ATEF_BaseNodeFuncPtr> initFunctions;
	std::map<std::string, ATEF_BaseNodeFuncPtr> inputFunctions;
	std::vector<ATEF_BaseNodeFuncPtr> coreFunctions;
	std::vector<ATEF_BaseNodeFuncPtr> exitFunctions;
	std::vector<Topic*> subscriptions;
	std::vector<Topic*> publishers;

	void CallInputFunction(std::string topicName);

public:
	static ATEF_BaseNode* Get();
	void Init(int argc, char** argv);
	void Loop();
	void Terminate();

	
protected:
	virtual void Setup(int argc, char** argv);	
	virtual void SetATEF_BaseNodeName(int argc, char** argv, std::string& ATEF_BaseNodeName);

	void Subscribe(std::string topicName, SerialObject* object);
	void Publish(std::string topicName, SerialObject* object);

	void RegisterInitFunction(ATEF_BaseNodeFuncPtr f);
	void RegisterInputFunction(std::string topicName, ATEF_BaseNodeFuncPtr f);
	void RegisterCoreFunction(ATEF_BaseNodeFuncPtr f);
	void RegisterExitFunction(ATEF_BaseNodeFuncPtr f);

	std::string FindTopicName(std::string parameterName);
};

#endif