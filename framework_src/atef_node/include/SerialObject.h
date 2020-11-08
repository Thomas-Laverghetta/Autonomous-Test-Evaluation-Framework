#ifndef SERIAL_OBJ_H
#define SERIAL_OBJ_H

// #include "ATEF_Node.h"

class SerialObject
{
private:
	friend class Topic;
	bool flagged;
protected:
	virtual void Serialize(int* outBuffer) = 0;
	virtual void Deserialize(const int* inBuffer) = 0;
	virtual int GetObjectSize() = 0;

	template <class T>
	void AddToBuffer(int* dataBuffer, int* dataRef, int& index, T obj)
	{
		for (int i = 0; i < sizeof(T) / sizeof(int); i++) {
			dataBuffer[index++] = dataRef[i];
		}
	}

	template <class T>
	void TakeFromBuffer(const int* dataBuffer, int* dataRef, int& index, T obj)
	{
		for (int i = 0; i < sizeof(T) / sizeof(int); i++) {
			dataRef[i] = dataBuffer[index++];
		}
	}
public:
	bool GetFlagged() { return(flagged); }
	void publish() {flagged = true;}
};

#endif