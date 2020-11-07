#ifndef KEYBOARD_H
#define KEYBOARD_H

void Keyboard_Init();

void Keyboard_Update(unsigned int update_seconds, unsigned int update_microseconds);

char Keyboard_GetLastKey();

void Keyboard_Cleanup();
#endif