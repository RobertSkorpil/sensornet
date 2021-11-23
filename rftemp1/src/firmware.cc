#include <string.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/crc16.h>
#include <../../common/include/message.h>

namespace {
template<int REG, uint8_t VALUE>
struct hw_bit
{
  constexpr volatile uint8_t *reg() const
  {
    return reinterpret_cast<volatile uint8_t *>(REG);
  }
  __attribute__((always_inline)) inline void set() const
  {
    *reg() |= VALUE;
  }

  inline void reset() const
  {
    *reg() &= ~VALUE;
  }

  inline bool value() const
  {
    return *reg() & VALUE;
  }
};
}

#define REG_TO_NUM(reg) (reinterpret_cast<int>(&reg))
constexpr hw_bit<REG_TO_NUM(DDRB), 1 << PB3> rf_enable;
constexpr hw_bit<REG_TO_NUM(PORTB), 1 << PB3> rf_tx;
constexpr hw_bit<REG_TO_NUM(DDRB), 1 << PB4> one_wire_out;
constexpr hw_bit<REG_TO_NUM(PORTB), 1 << PB4> one_wire_tx;
constexpr hw_bit<REG_TO_NUM(PINB), 1 << PB4> one_wire_rx;

// return:
//   false: no presence signal
//   true:  presense signal detected
static bool one_wire_reset()
{
  one_wire_tx.reset();
  one_wire_out.set();
  _delay_us(480);
  one_wire_out.reset();
  one_wire_tx.set();
  _delay_us(30);
  bool present = !one_wire_rx.value();
  _delay_us(480);
  return present;
}

static void one_wire_write_0()
{
  one_wire_tx.reset();
  one_wire_out.set();
  _delay_us(60);
  one_wire_out.reset();
  one_wire_tx.set();
  _delay_us(10);
}

static void one_wire_write_1()
{
  one_wire_tx.reset();
  one_wire_out.set();
  _delay_us(1);
  one_wire_out.reset();
  one_wire_tx.set();
  _delay_us(30);
}

static bool one_wire_read_bit()
{
  one_wire_tx.reset();
  one_wire_out.set();
  _delay_us(2);
  one_wire_out.reset();
  one_wire_tx.set();
  _delay_us(8);
  bool value = one_wire_rx.value();
  _delay_us(50);
  return value;
}

static uint8_t one_wire_read()
{
  uint8_t value = 0;
  for(uint8_t b = 0; b < 8; ++b)
    value = (value >> 1) | (one_wire_read_bit() * 0x80);

  return value;
}

static void one_wire_write(uint8_t value)
{
  for(uint8_t b = 0; b < 8; ++b)
  {
    if(value & 1)
      one_wire_write_1();
    else
      one_wire_write_0();
    value >>= 1;
  }
}

static void rf_tx_0()
{
  rf_tx.set();
  _delay_us(900);
  rf_tx.reset();
  _delay_us(200);
}

static void rf_tx_1()
{
  rf_tx.set();
  _delay_us(450);
  rf_tx.reset();
  _delay_us(200);
}

static void rf_send(uint8_t value)
{
  for(uint8_t b = 0; b < 8; ++b)
  {
    if(value & 0x80)
      rf_tx_1();
    else
      rf_tx_0();
    value <<= 1;
  }
}

static rom_t read_rom()
{
  one_wire_reset();
  one_wire_write(0x33);
  rom_t rom;
  for(uint8_t b = 0; b < 8; ++b)
    rom.bytes[b] = one_wire_read();
  return rom;
}

static void write_ds18b20_config(uint8_t value)
{
  one_wire_reset();
  one_wire_write(0x4e); //Write scratchpad
  one_wire_write(0xff);
  one_wire_write(0x7f);  //R0 = 1, R1 = 1, conversion time 750ms, 12bit resolution
}

static uint16_t read_ds18b20_temp()
{
  one_wire_reset();
  one_wire_write(0xcc); //Skip ROM
  one_wire_write(0x44);
  _delay_ms(750);

  one_wire_reset();
  one_wire_write(0xcc); //Skip ROM
  one_wire_write(0xbe); //Read scratchpad
  //TODO check CRC
  uint8_t lo = one_wire_read();
  uint8_t hi = one_wire_read();
  one_wire_reset();
  return hi * 256 + lo;
} 

static void rf_ring()
{          
  rf_tx.set();
  _delay_us(1500);
  rf_tx.reset();
  _delay_us(1000);
}

static void rf_send_message(const message_rftemp1 &msg)
{
  const uint8_t *begin = reinterpret_cast<const uint8_t *>(&msg);
  const uint8_t *end = begin + sizeof(msg);

  uint16_t crc = 0xffff;

  rf_ring();
  while(begin != end)
  {
    crc = _crc16_update(crc, *begin);
    rf_send(*begin++);
  }
  rf_send(crc);
  rf_send(crc >> 8);
  rf_send(0xa0);
  rf_send(0xb0);

  rf_tx.reset();
}

int __attribute__((noreturn)) main()
{    
  rf_enable.set();
  message_rftemp1 msg;
  msg.eye = message::EYE;
  msg.total_length = sizeof(msg);
  msg.num.id = msg_num::ID;
  msg.num.length = sizeof(msg.num);
  msg.num.num = 0;
  msg.temp.id = msg_temp::ID;
  msg.temp.length = sizeof(msg.temp);

  for(;;)
  {
    msg.temp.rom = read_rom();
    msg.temp.temp = read_ds18b20_temp();

    for(int i = 0; i < 3; ++i)
    {
      rf_send_message(msg);
      rf_send_message(msg);
      rf_send_message(msg);
      _delay_ms(1000);
    }
    _delay_ms(60000);
    ++msg.num.num;
  }
}
