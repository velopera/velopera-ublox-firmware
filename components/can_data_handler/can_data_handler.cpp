
#include <iostream>

#include "can_data_handler.h"

using namespace std;

CANDataHandler::CANDataHandler() {}

CANDataHandler::~CANDataHandler()
{
  if (state)
  {
    free(state);
  }
}

void CANDataHandler::handleCANData(unsigned long id, uint64_t data)
{
  // char topic[256];
  // char output[2048];
  dbcc_time_stamp_t time_stamp = 0;

  // funcs.unpack(state, id, data, 8, time_stamp);

  // funcs.print(state, id, output, sizeof(output));

  funcs.handle_message(&funcs, state, id, data, 8, time_stamp);
}
