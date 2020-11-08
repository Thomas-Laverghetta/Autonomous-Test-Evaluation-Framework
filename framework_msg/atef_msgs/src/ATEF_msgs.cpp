#include "ATEF_msgs.h"

#include <cstdio>

using namespace ATEF::msgs;

void Auto::Serialize(int * outBuffer)
{
	int index = 0;
	int * dataRef;

	AddToBuffer(outBuffer, dataRef, index, data);
}

void Auto::Deserialize(const int * inBuffer)
{
	int index = 0;
	int *dataRef;

	TakeFromBuffer(inBuffer, dataRef, index, data);
}

int Auto::GetObjectSize()
{	
	return((sizeof(data))/sizeof(int));
}
