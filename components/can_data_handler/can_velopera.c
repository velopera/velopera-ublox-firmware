
// #include <iostream>

// #include "can_data_handler.h"

// using namespace std;

// int
// CANVelopera::init()
// {

//   lib_init_velopera(&funcs);

//   state = funcs.alloc_state();
//   return 0;
// }

#include "can_data_handler.h"

void
CANVelopera_init(CANVelopera* velopera)
{
  CANDataHandler_init(&velopera->base);

  lib_init_velopera(&velopera->base.funcs);
  velopera->base.state = velopera->base.funcs.alloc_state();
}