#pragma once
#include "rom.h"

struct msg_part
{
  uint16_t id;
  uint16_t length;
};

struct msg_num : msg_part
{
  static constexpr uint16_t ID = 0x0000;
  uint32_t num;
};

struct msg_temp : msg_part
{
  static constexpr uint16_t ID = 0x0001;
  rom_t rom;
  int16_t temp;
};

struct msg_power : msg_part
{
  static constexpr uint16_t ID = 0x0002;
  uint8_t voltage; //0 = 0, 255 >= 3.3V
  uint8_t reserved;
};

struct message
{
  static constexpr uint16_t EYE = 0xf0e0;
  uint16_t eye;
  uint16_t total_length;
};

struct message_rftemp1 : message
{
  msg_num num;
  msg_temp temp;
  msg_power power;
};

