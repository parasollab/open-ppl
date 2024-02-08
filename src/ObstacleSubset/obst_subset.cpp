#include <iostream>
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"

int main(int argc, char const *argv[])
{
    CollisionDetectionValidity cdv = CollisionDetectionValidity();
    std::cout << cdv.GetEnvironment() << std::endl;
    return 0;
}



// This needs to test how a dynamic roadmap changes as obstacles move
// It needs to make a dynamic roadmap
// Move some obstacles
// Check what in the roadmap is now valid/invalid
// Maybe do that a few times
