#include <cstdio>
#include <iostream>
#include <atomic>
#include <array>
#include <variant>
#include <vector>
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "hardware/gpio.h"
#include "../../common/include/message.h"

//#define DEBUG
//#define DEBUG2

constexpr uint GPIO_RF = 6;
constexpr uint GPIO_LED = 25;
constexpr uint GPIO_LED2 = 15;
constexpr uint GPIO_LED3 = 16;
constexpr uint32_t EDGE_LO = 4;
constexpr uint32_t EDGE_HI = 8;

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

static queue_t event_q;
struct reset_t {};
struct listen_t {};
using rf_event_t = std::variant<uint8_t, reset_t, listen_t>;

void gpio_irq_callback(uint gpio, uint32_t events)
{
  constexpr auto min_1_time = 250;
  constexpr auto max_1_time = 500;
  constexpr auto min_0_time = 650;
  constexpr auto max_0_time = 950;
  constexpr auto min_silence_time = 150;
  constexpr auto max_silence_time = 500;

  static uint8_t word = 0;
  static uint8_t bits = 0;

  static absolute_time_t time_last = 0;
  enum class reset_reason_t
  {
    SILENCE_TOO_LONG,
    SILENCE_TOO_SHORT,
    BAD_PULSE_LENGTH,
    OTHER,
  };

  enum class state_t
  {
    ONE_0,
    ONE_1,
    ONE_2,
    ONE_3,
    ZERO_1_0,
    ZERO_1_1,
    ZERO_1_2,
    ZERO_1_3,
    LISTEN,
  };

  static state_t state = state_t::ONE_0;
  auto zero_expected = []()
  {
    return state < state_t::ONE_0 || (state >= state_t::ZERO_1_0 && state <= state_t::ZERO_1_3);
  };

  static auto reset_word = [&]()
  {
    word = 0;
    bits = 0;
  };
  static auto reset = [&](reset_reason_t reason)
  {
    reset_word();
    if(state == state_t::LISTEN)
    {
      auto event = rf_event_t { reset_t{} };
      queue_try_add(&event_q, &event);
#ifdef DEBUG
      printf("DEBUG;RESET=%d;\n", reason);
#endif
    }
    state = state_t::ONE_0;
  };

  absolute_time_t time_now = get_absolute_time();
  int32_t time_diff = absolute_time_diff_us(time_last, time_now);
  time_last = time_now;

#ifdef DEBUG2
    printf("DEBUG;T=%d;\n", time_diff);
#endif

  if(events & EDGE_HI)
  {

    if(time_diff > max_silence_time)
      reset(reset_reason_t::SILENCE_TOO_LONG);
    else if(time_diff < min_silence_time)
      reset(reset_reason_t::SILENCE_TOO_SHORT);

    return;
  }

  if(time_diff >= min_1_time && time_diff <= max_1_time)
  {
    if(state == state_t::LISTEN)
    {
      word <<= 1;
      word |= 0x1;
      ++bits;
    }
    else if(!zero_expected())    
      state = static_cast<state_t>(static_cast<int>(state) + 1);
    else
      reset(reset_reason_t::OTHER);
  }
  else if(time_diff >= min_0_time && time_diff <= max_0_time)
  {
    if(state == state_t::LISTEN)
    {
      word <<= 1;
      ++bits;
    }
    else if(zero_expected())
    {
      auto prev_state = state;
      if(state != state_t::LISTEN)
        state = static_cast<state_t>(static_cast<int>(state) + 1);
      if(state == state_t::LISTEN && prev_state != state_t::LISTEN)
      {
        auto event = rf_event_t { listen_t{} };
        queue_try_add(&event_q, &event);
        event = rf_event_t { 0xe0 };
        queue_try_add(&event_q, &event);
        event = rf_event_t { 0xf0 };
        queue_try_add(&event_q, &event);
      }
    }
    else
    {
      reset(reset_reason_t::OTHER);
    }
  }
  else
    reset(reset_reason_t::BAD_PULSE_LENGTH);

  if(state == state_t::LISTEN)
  {
    if(bits == 8)
    {
      auto event = rf_event_t { word };
      queue_try_add(&event_q, &event);
      reset_word();
    }
  }
}

bool check_message(const message *msg)
{
  const uint8_t *curr_ptr = reinterpret_cast<const uint8_t *>(msg + 1);
  int rem = msg->total_length - sizeof(message);
  for(int p = 0; rem >= int(sizeof(msg_part)); ++p)
  {
    const msg_part *part = reinterpret_cast<const msg_part *>(curr_ptr);
    if(part->length < sizeof(msg_part) || part->length > rem)
    {
      printf("ERROR;TYPE:MALFORMED;\n");
      return false;
    }

    curr_ptr += part->length;
    rem -= part->length;
  }

  if(rem)
  {
    printf("ERROR;TYPE:MALFORMED;\n");
    return false;
  }
  return true;
}

void print_message(const message *msg)
{
  printf("MSG;");
  const uint8_t *curr_ptr = reinterpret_cast<const uint8_t *>(msg + 1);
  size_t rem = msg->total_length - sizeof(message);
  for(int p = 0; rem >= sizeof(msg_part); ++p)
  {
    const msg_part *part = reinterpret_cast<const msg_part *>(curr_ptr);
    curr_ptr += part->length;
    rem -= part->length;

    if(part->id == 0)
    {
      const msg_num *num = reinterpret_cast<const msg_num *>(part);
      printf("SAMPLE:%d;", num->num);
    }
    else if(part->id == 1)
    {
      const msg_temp *temp = reinterpret_cast<const msg_temp *>(part);
      printf("DALLASID:%016llX;", *reinterpret_cast<const uint64_t *>(temp->rom.bytes));
      printf("TEMP:%.2lf;", temp->temp / 16.0);
    }
    else if(part->id == 2)
    {
      const msg_power *power = reinterpret_cast<const msg_power *>(part);
      printf("VOLT:%.2lf;", power->voltage / 256.0 * 3.3);
    }
  }
  printf("\n");
}

int main()
{
  stdio_usb_init();
  std::ios::sync_with_stdio(false);

  queue_init(&event_q, sizeof(rf_event_t), 128);

  gpio_init(GPIO_RF);
  gpio_init(GPIO_LED);
  gpio_init(GPIO_LED2);
  gpio_init(GPIO_LED3);
  gpio_set_dir(GPIO_LED, GPIO_OUT);  
  gpio_set_dir(GPIO_LED2, GPIO_OUT);  
  gpio_set_dir(GPIO_LED3, GPIO_OUT);  
  gpio_set_irq_enabled_with_callback(GPIO_RF, EDGE_LO | EDGE_HI, true, gpio_irq_callback);

  static std::array<uint8_t, 256> recv_buffer;
  static uint8_t *recv_next;
  static uint16_t crc_comp;
  const message *msg = reinterpret_cast<const message *>(recv_buffer.data());

  auto reset = [&](){
#ifdef DEBUG    
    if(recv_next != recv_buffer.begin())
    {
      printf("DEBUG;RESET;DATA:");
      for(auto i = recv_buffer.begin(); i < recv_next; ++i)
        printf("%02X ", *i);
      printf(";\n");
    }
#endif
    recv_next = recv_buffer.begin();
    crc_comp = 0xffff;
  };
  reset();

  for(;;)
  {
    rf_event_t event;
    queue_remove_blocking(&event_q, &event);

    std::visit(
        [&](auto &&arg)
        {
          using T = std::decay_t<decltype(arg)>;
          if constexpr (std::is_same_v<T, reset_t>)
          {
            reset();
            gpio_put(GPIO_LED, false);
          }
          else if constexpr (std::is_same_v<T, listen_t>)
          {
            gpio_put(GPIO_LED, true);
          }
          else if constexpr (std::is_same_v<T, uint8_t>)
          {
#ifdef DEBUG          
            printf("DEBUG;DATA:%02X;\n", arg);
#endif            
            *recv_next++ = arg;
            auto length = recv_next - recv_buffer.begin();
            if(length < 4 || length <= msg->total_length)
              crc_comp = crc16_update(crc_comp, arg);

            if(length == 2 && msg->eye != message::EYE)
              reset();
            else if(length >= 4)
            {
              if(size_t(msg->total_length + 2) > recv_buffer.size())
              {
                printf("ERROR;TYPE:BUFSMALL;\n");
                reset();
              }
              else if(length == msg->total_length + 2)
              {
                uint16_t crc_recv = *reinterpret_cast<const uint16_t*>(recv_next - 2);
                if(crc_recv == crc_comp)
                {
                  gpio_put(GPIO_LED2, true);
                  if(check_message(msg))
                  {
                    gpio_put(GPIO_LED3, true);
                    print_message(msg);
                  }
                  gpio_put(GPIO_LED2, false);
                  gpio_put(GPIO_LED3, false);
                }
                else
                  printf("ERROR;TYPE:CRC;RECV:%04X;COMP:%04X;\n", crc_recv, crc_comp);
                  
                reset();
              }
              else if(length > msg->total_length + 2)
              {
                printf("ERROR;TYPE:TOOLONG;\n");
                reset();
              }
            }
          }
        },  
        event);
  }        

  return 0;
}
