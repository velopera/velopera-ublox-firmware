// #pragma once
// // #include <string>
// #include "parser_library.h"

// class CANDataHandler
// {
// protected:
//   void* state;

// public:
//   CANDataHandler();
//   ~CANDataHandler();
//   library_funcs funcs;

//   virtual int init() = 0;

//   void handleCANData(unsigned long id, uint64_t data);
// };

// class CANVelopera : public CANDataHandler
// {
// public:
//   CANDataHandler::CANDataHandler;
//   int init();
// };

#pragma once

#include "parser_library.h"
#include <stdint.h>
#include <stdlib.h>

typedef struct
{
  void* state;
  library_funcs funcs;
} CANDataHandler;

void
CANDataHandler_init(CANDataHandler* handler);
void
CANDataHandler_cleanup(CANDataHandler* handler);
void
CANDataHandler_handleCANData(CANDataHandler* handler,
                             unsigned long id,
                             uint64_t data);

typedef struct
{
  CANDataHandler base;
} CANVelopera;

void
CANVelopera_init(CANVelopera* velopera);