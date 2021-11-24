#include <string.h>
#include <stdlib.h>
#include <avr/builtins.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/sleep.h>
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
  for(int i = 0; i < 100; ++i)
  {
    rf_tx.reset();
    _delay_us(200);
    rf_tx.set();
    _delay_us(200);
  }
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

  rf_tx.reset();
}

static uint8_t measure_voltage()
{
  ADMUX = (1 << REFS1) | // Internal 1.1V Reference
          (1 << ADLAR) | //Left Adjust result
          (1 << MUX0); //Measure ADC1 (PB2)

  DIDR0 = (1 << ADC1D); //Disable digital input PB2

  ADCSRA = (1 << ADEN) | //Enable ADC
           (1 << ADSC) | //Start conversion
           (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //1:128 ADC clock division
  do
  {
/*    MCUCR = (1 << SE) | //Sleep enable
            (1 << SM0); // ADC Noise Reduction + Start ADC conversion
    __builtin_avr_sleep();*/
  }while(ADCSRA & (1 << ADSC));

  uint8_t value = ADCH;

  ADCSRA = 0; //Disable ADC

  return value;
}

ISR(WDT_vect)
{}

void sleep(int delay)
{
  delay = (delay + 7) / 8;
  for(int i = 0; i < (delay + 7 / 8); ++i)
  {
    WDTCR = (1 << WDIE) | //Watchdog timeout interrupt enable
            (1 << WDP3) | (1 << WDP0); //8s timeout
    wdt_reset();
    sei();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();
  }
}

int __attribute__((noreturn)) main()
{    
  sei();
  rf_enable.set();
  message_rftemp1 msg;
  msg.eye = message::EYE;
  msg.total_length = sizeof(msg);
  msg.num.id = msg_num::ID;
  msg.num.length = sizeof(msg.num);
  msg.num.num = 0;
  msg.temp.id = msg_temp::ID;
  msg.temp.length = sizeof(msg.temp);
  msg.power.id = msg_power::ID;
  msg.power.length = sizeof(msg.power);
  msg.power.reserved = 0;

  bool rand_seeded = false;
  for(;;)
  {
    msg.power.voltage = measure_voltage();
  
    msg.temp.rom = read_rom();
    if(!rand_seeded)
    {
      srand(*reinterpret_cast<uint32_t *>(msg.temp.rom.bytes));
      rand_seeded = true;
    }

    msg.temp.temp = read_ds18b20_temp();

    rf_send_message(msg);
    _delay_ms(500);
    rf_send_message(msg);
    _delay_ms(1000);
    rf_send_message(msg);
    int delay = 30 + rand() / 1024;
    sleep(delay);

    ++msg.num.num;
  }
}
