#include <cstdio>
#include <iostream>
#include <atomic>
#include <array>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/queue.h"
#include "../../common/include/message.h"

constexpr uint GPIO_RF = 6;
constexpr uint GPIO_LED = 25;
constexpr uint32_t EDGE_LO = 4;
constexpr uint32_t EDGE_HI = 8;

void print_message(const message *msg)
{
  printf("Total length: %d bytes\n", msg->total_length);
  const uint8_t *curr_ptr = reinterpret_cast<const uint8_t *>(msg + 1);
  int rem = msg->total_length - sizeof(message);
  for(int p = 0; rem >= sizeof(msg_part); ++p)
  {
    const msg_part *part = reinterpret_cast<const msg_part *>(curr_ptr);
    printf("Part: %d\n", p + 1);
    printf("Length: %d\n", part->length);
    printf("Type ID: %d\n", part->id);
    curr_ptr += part->length;
    rem -= part->length;

    if(part->id == 0)
    {
      const msg_num *num = reinterpret_cast<const msg_num *>(part);
      printf("Sample #: %d\n", num->num);
    }
    else if(part->id == 1)
    {
      const msg_temp *temp = reinterpret_cast<const msg_temp *>(part);
      printf("Temperature: %lf°C\n", temp->temp / 16.0);
    }
  }
}

uint16_t crc16_update(uint16_t crc, uint8_t a)
{
  int i;
  crc ^= a;
  for (i = 0; i < 8; ++i)
  {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc = (crc >> 1);
  }
  return crc;
}

struct rf_event
{
  uint32_t event_type;
  int32_t time_diff;

};

void gpio_irq_callback(uint gpio, uint32_t events)
{
  constexpr auto min_1_time = 70;
  constexpr auto max_1_time = 400;
  constexpr auto min_0_time = 550;
  constexpr auto max_0_time = 1200;

  static uint16_t crc_comp = 0xffff;
  static uint8_t word = 0;
  static uint8_t bits = 0;

  static std::array<uint8_t, 256> recv_buffer;
  static uint8_t *recv_next = recv_buffer.begin();
  const message *msg = reinterpret_cast<const message *>(recv_buffer.data());

  static absolute_time_t time_last = 0;

  absolute_time_t time_now = get_absolute_time();

  if(events & EDGE_HI)
  {
    time_last = time_now;
    return;
  }
    
  int32_t time_diff = absolute_time_diff_us(time_last, time_now);

  static auto reset_word = [&]()
  {
    word = 0;
    bits = 0;
  };

  static auto reset = [&]()
  {
    reset_word();
    
    auto count = recv_next - recv_buffer.begin();

    recv_next = recv_buffer.begin();
    crc_comp = 0xffff;
  };

  if(time_diff < 100)    
    ;
  else if(time_diff >= min_1_time && time_diff <= max_1_time)
  {
    word <<= 1;
    word |= 0x1;
    ++bits;
  }
  else if(time_diff >= min_0_time && time_diff <= max_0_time)
  {
    word <<= 1;
    ++bits;
  }
  else
    reset();

  if(bits == 8)
  {
    *recv_next++ = word;
    auto count = recv_next - recv_buffer.begin();

    if(count < 4 || count <= msg->total_length)
      crc_comp = crc16_update(crc_comp, word);

    if(count == 2 && msg->eye != message::EYE)
      reset();
    else if(count >= 4)
    {
      if(msg->total_length + 2 > recv_buffer.size())
      {
        printf("Message too long");
        reset();
      }
      else if(count == msg->total_length + 2)
      {
        uint16_t crc_recv = *reinterpret_cast<const uint16_t*>(recv_next - 2);
        printf("CRC RECV %04d, COMP %04d\n", crc_recv, crc_comp);
        if(crc_recv == crc_comp)
          print_message(msg);
        reset();
      }
      else if(count > msg->total_length + 2)
      {
        printf("Recieve overflow");
        reset();
      }
    }

    reset_word();
  }

  time_last = time_now;
}

int main()
{
  stdio_usb_init();
  std::ios::sync_with_stdio(false);
  gpio_init(GPIO_RF);
  gpio_init(GPIO_LED);
  gpio_set_dir(GPIO_LED, GPIO_OUT);

  gpio_set_irq_enabled_with_callback(GPIO_RF, EDGE_LO | EDGE_HI, true, gpio_irq_callback);

  for(;;);

  return 0;
}
