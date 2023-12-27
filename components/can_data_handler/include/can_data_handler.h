#pragma once
#include <string>
#include "parser_library.h"

class CANDataHandler
{
protected:
  library_funcs funcs;
  void *state;

public:
  CANDataHandler();
  ~CANDataHandler();

  virtual int init() = 0;

  void handleCANData(unsigned long id, uint64_t data);
};

class CANVelopera : public CANDataHandler
{
public:
  CANDataHandler::CANDataHandler;
  int init();
};

class CANPJA612 : public CANDataHandler
{
public:
  CANDataHandler::CANDataHandler;
  int init();
};
