
#include <iostream>

#include "can_data_handler.h"

using namespace std;

int CANVelopera::init()
{

    lib_init_velopera(&funcs);

    state = funcs.alloc_state();
    return 0;
}
