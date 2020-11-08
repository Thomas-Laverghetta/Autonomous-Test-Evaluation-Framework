#ifndef SERIAL_OBJ_H
#define SERIAL_OBJ_H

#include "ATEF_Node.h"

class SerialObject
{
private:
	friend class ATEF::Topic;
	bool flagged;
public:
	virtual void Serialize(char* outBuffer) = 0;
	virtual void Deserialize(const char* inBuffer) = 0;
	virtual int GetObjectSize() = 0;

	bool GetFlagged() { return(flagged); }
	void Publish() {flagged = true;}
};

#endif