#ifndef SAVE_STATE_H
#define SAVE_STATE_H
/*
* Creator: Thomas J Laverghetta (tlave002@odu.edu)
*/
#include <fstream>
#include <string>
#include <vector>


// Save State Class
class SaveState {
public:
	/* Constuctor for SaveState objects.*/
	SaveState();

	/* Constuctor for SaveState when loading dynamic object.*/
	SaveState(unsigned int id);

	/* Serialize state-save child object to save-file.*/
	virtual void Save(std::ofstream& saveFile) = 0;

	/* Deserialize state-save child object from load-file.*/
	virtual void Load(std::ifstream& loadFile) = 0;

	/* Repoints SaveState objects to points using state object array.*/
	virtual void Repoint(const std::vector<SaveState*>& SaveStateList) = 0;
	
	/* Returns class id associated with this object.*/
	virtual unsigned int GetSaveStateClassId() = 0;

	/* Gets State Save Object ID.*/
	unsigned int GetSaveStateId() { return _id; }

	/* Unregisters with save state manager.*/
	~SaveState();

	static unsigned int _nextSaveStateClassId;
private:
	// State Save Identifier
	unsigned int _id;
	static unsigned int _nextId;
};

// Unique Save State ID
#define UNIQUE_SAVE_STATE_ID \
public: \
	static unsigned int GetSaveStateClassID(){ static unsigned int ID = SaveState::_nextSaveStateClassId++; return ID; }\
	unsigned int GetSaveStateClassId() { return GetSaveStateClassID(); };

// Function pointer to static New Function that each class must create
typedef SaveState* (*NewFunctor)(unsigned int);

///// Functions for SaveStateManager accessors

/* Registers class with Save-State Manager*/
void SaveStateClassRegister(unsigned int classId, NewFunctor newFunctor);

/* Loads system states from load binary file*/
void SaveStateLoad(std::string loadFile);

/* Loads system states from load binary file*/
void SaveStateSave(std::string saveFile);

// returns a new class Id to identify class
unsigned int GetUniqueClassId();

#endif