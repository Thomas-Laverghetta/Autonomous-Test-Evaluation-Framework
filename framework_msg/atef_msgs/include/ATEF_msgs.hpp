#ifndef ATEF_MSGS_H
#define ATEF_MSGS_H

#include "SerialObject.h"
namespace ATEF::msgs {

template <class T>
class Auto : public SerialObject
{
	virtual void Serialize(int * outBuffer)
	{
		int index = 0;
		int * dataRef;

		AddToBuffer(outBuffer, dataRef, index, data);
	}

	virtual void Deserialize(const int * inBuffer)
	{
		int index = 0;
		int *dataRef;

		TakeFromBuffer(inBuffer, dataRef, index, data);
	}

	virtual int GetObjectSize()
	{	
		return((sizeof(data))/sizeof(int));
	}
public:
	T data;
	Auto() {}
};
}
#endif