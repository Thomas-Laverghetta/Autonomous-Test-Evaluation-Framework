#pragma once

/*
* Creator: Thomas J Laverghetta (tlave002@odu.edu)
*/


#include "SaveState.h"
#include <vector>
#include <map>
#include <string>

using namespace std;
using namespace ATEF;
class SaveStateManager
{
public:
	/* Registers SaveState with manager.*/
	static void Register(SaveState* ss);

	/* Remove object from save list.*/
	static void Unregister(SaveState* ss);

	/* Saves all objects to file.*/
	static void SaveAll(string& saveFile);

	/* Loads all objects from file.*/
	static void LoadAll(string& loadFile);

	/* Registers Class for class mapping.
	All SaveState Classes must create static */
	static void RegisterClass(unsigned int classId, NewFunctor newFunctor);

	/* Gets the save-state list.*/
	static vector<SaveState*> GetSaveStateList();
private:
	static vector<SaveState*> _SaveStateList;
	static map<unsigned int, NewFunctor> _classMap;
};
