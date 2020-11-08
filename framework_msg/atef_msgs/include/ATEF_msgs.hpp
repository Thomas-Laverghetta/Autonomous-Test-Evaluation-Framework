#ifndef ATEF_MSGS_H
#define ATEF_MSGS_H

#include "SerialObject.h"
namespace ATEF::msgs {

template <class T>
class Auto : public SerialObject
{
	virtual void Serialize(int* outBuffer);
	virtual void Deserialize(const int* inBuffer);
	virtual int GetObjectSize();
public:
	T data;
	Auto() {}
};
}
#endif