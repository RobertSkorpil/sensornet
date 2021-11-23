#pragma once

struct rom_t
{
  uint8_t bytes[8];
  constexpr rom_t()
    : bytes() { }
};
