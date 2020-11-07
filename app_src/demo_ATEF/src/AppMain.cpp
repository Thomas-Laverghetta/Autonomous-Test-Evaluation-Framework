#include "ATEF_BaseNode.h"
#include <cstdio>

int main(int argc, char** argv)
{    
    ATEF_BaseNode::Get()->Init(argc, argv);
	ATEF_BaseNode::Get()->Loop();

	return 0;
}
