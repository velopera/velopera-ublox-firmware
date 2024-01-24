#pragma once
#include <string>
#include "parser_library.h"

class CANDataHandler
{
protected:
  void *state;

public:
  CANDataHandler();
  ~CANDataHandler();
  library_funcs funcs;

  virtual int init() = 0;

  void handleCANData(unsigned long id, uint64_t data);
};

class CANVelopera : public CANDataHandler
{
public:
  CANDataHandler::CANDataHandler;
  int init();
};

