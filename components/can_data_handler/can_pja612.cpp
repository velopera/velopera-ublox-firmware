#include <iostream>

#include "can_data_handler.h"

using namespace std;

int CANPJA612::init()
{
    lib_init_pja612(&funcs);

    state = funcs.alloc_state();
    return 0;
}
