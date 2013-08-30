/*
 * This program turns on the 4 leds of the stm32f4 discovery board
 * one after another.
 * It defines shortcut definitions for the led pins and
 * stores the order of the leds in an array which is being
 * iterated in a loop.
 *
 * This program is free human culture like poetry, mathematics
 * and science. You may use it as such.
 */

#include <math.h>

#include <stm32f4_discovery.h>


/*
  I/O pins used to control the motor driver.
  PB3 and PB4 default to debug/jtag stuff, so use PB0-2 and PB5-7.

  Each of the three channels need an enable ENn and an input INn.
*/
#define CHAN_EN1 GPIO_Pin_0  /* Connect to pin 3 on L6234 DIP20 */
#define CHAN_IN1 GPIO_Pin_1             /* pin 2 */
#define CHAN_EN2 GPIO_Pin_2             /* pin 18 */
#define CHAN_IN2 GPIO_Pin_5             /* pin 19 */
#define CHAN_EN3 GPIO_Pin_6             /* pin 8 */
#define CHAN_IN3 GPIO_Pin_7             /* pin 9 */


/* This is apparently needed for libc/libm (eg. powf()). */
int __errno;


static void delay(__IO uint32_t nCount)
{
    while(nCount--)
        __asm("nop"); // do nothing
}


static void setup_serial(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* enable peripheral clock for USART2 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* GPIOA Configuration:  USART2 TX on PA2 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect USART2 pins to AF2 */
  // TX = PA2
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);

  USART_Cmd(USART2, ENABLE); // enable USART2
}


static void
setup_gpio(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /*
    Let's set up PB0-5 as output pins.
    PB0,4,6 is enable for channel 1,2,3.
    PB1,5,7 is IN for channel 1,2,3.
  */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin =
    CHAN_EN1|CHAN_IN1|CHAN_EN2|CHAN_IN2|CHAN_EN3|CHAN_IN3;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}


static void
setup_led(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
}


static void
led_on(void)
{
  GPIO_SetBits(GPIOG, GPIO_Pin_15);
}


static void
led_off(void)
{
  GPIO_ResetBits(GPIOG, GPIO_Pin_15);
}


/* Set a channel to given level (-1, 0, or 1). */
static void
set_channel(unsigned channel, int level)
{
  unsigned in, enable;
  switch (channel)
  {
  case 1:
    enable = CHAN_EN1;
    in = CHAN_IN1;
    break;
  case 2:
    enable = CHAN_EN2;
    in = CHAN_IN2;
    break;
  case 3:
    enable = CHAN_EN3;
    in = CHAN_IN3;
    break;
  default:
    return;
  }

  if (level < 0)
  {
    GPIO_SetBits(GPIOB, enable);
    GPIO_ResetBits(GPIOB, in);
  }
  else if (level > 0)
  {
    GPIO_SetBits(GPIOB, enable);
    GPIO_SetBits(GPIOB, in);
  }
  else
    GPIO_ResetBits(GPIOB, enable);
}


static void
serial_putchar(USART_TypeDef* USARTx, uint32_t c)
{
  while(!(USARTx->SR & USART_FLAG_TC));
  USART_SendData(USARTx, c);
}


static void
serial_puts(USART_TypeDef *usart, const char *s)
{
  while (*s)
    serial_putchar(usart, (uint8_t)*s++);
}


static void
serial_output_hexdig(USART_TypeDef* USARTx, uint32_t dig)
{
  serial_putchar(USARTx, (dig >= 10 ? 'A' - 10 + dig : '0' + dig));
}


__attribute__ ((unused))
static void
serial_output_hexbyte(USART_TypeDef* USARTx, uint8_t byte)
{
  serial_output_hexdig(USARTx, byte >> 4);
  serial_output_hexdig(USARTx, byte & 0xf);
}


__attribute__ ((unused))
static void
println_uint32(USART_TypeDef* usart, uint32_t val)
{
  char buf[13];
  char *p = buf;
  uint32_t l, d;

  l = 1000000000UL;
  while (l > val && l > 1)
    l /= 10;

  do
  {
    d = val / l;
    *p++ = '0' + d;
    val -= d*l;
    l /= 10;
  } while (l > 0);

  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  serial_puts(usart, buf);
}


__attribute__ ((unused))
static void
println_int32(USART_TypeDef* usart, int32_t val)
{
  if (val < 0)
  {
    serial_putchar(usart, '-');
    println_uint32(usart, (uint32_t)0 - (uint32_t)val);
  }
  else
    println_uint32(usart, val);
}


static void
float_to_str(char *buf, float f, uint32_t dig_before, uint32_t dig_after)
{
  float a;
  uint32_t d;
  uint8_t leading_zero;

  if (f == 0.0f)
  {
    buf[0] = '0';
    buf[1] = '\0';
    return;
  }
  if (f < 0)
  {
    *buf++ = '-';
    f = -f;
  }
  a =  powf(10.0f, (float)dig_before);
  if (f >= a)
  {
    buf[0] = '#';
    buf[1] = '\0';
    return;
  }
  leading_zero = 1;
  while (dig_before)
  {
    a /= 10.0f;
    d = (uint32_t)(f / a);
    if (leading_zero && d == 0 && a >= 10.0f)
      *buf++ = ' ';
    else
    {
      leading_zero = 0;
      *buf++ = '0' + d;
      f -= d*a;
    }
    --dig_before;
  }
  if (!dig_after)
  {
    *buf++ = '\0';
    return;
  }
  *buf++ = '.';
  do
  {
    f *= 10.0f;
    d = (uint32_t)f;
    *buf++ = '0' + d;
    f -= (float)d;
    --dig_after;
  } while (dig_after);
  *buf++ = '\0';
}


__attribute__ ((unused))
static void
println_float(USART_TypeDef* usart, float f,
              uint32_t dig_before, uint32_t dig_after)
{
  char buf[21];
  char *p = buf;

  float_to_str(p, f, dig_before, dig_after);
  while (*p)
    ++p;
  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  serial_puts(usart, buf);
}


int main(void)
{
  static const uint32_t m_delay = 10000000;

  delay(2000000);
  setup_serial();
  setup_gpio();
  setup_led();
  serial_puts(USART2, "Initialising...\r\n");
  set_channel(1, 0);
  set_channel(2, 0);
  set_channel(3, 0);
  delay(2000000);

  serial_puts(USART2, "Hello world, ready to blink!\r\n");

  while (1)
  {
    set_channel(1, 0);
    set_channel(2, 1);
    set_channel(3, -1);
    serial_puts(USART2, "a");
    led_off();
    delay(m_delay);

    set_channel(1, -1);
    set_channel(2, 1);
    set_channel(3, 0);
    serial_puts(USART2, "b");
    led_on();
    delay(m_delay);

    set_channel(1, -1);
    set_channel(2, 0);
    set_channel(3, 1);
    serial_puts(USART2, "c");
    led_off();
    delay(m_delay);

    set_channel(1, 0);
    set_channel(2, -1);
    set_channel(3, 1);
    serial_puts(USART2, "d");
    led_on();
    delay(m_delay);

    set_channel(1, 1);
    set_channel(2, -1);
    set_channel(3, 0);
    serial_puts(USART2, "e");
    led_off();
    delay(m_delay);

    set_channel(1, 1);
    set_channel(2, 0);
    set_channel(3, -1);
    serial_puts(USART2, "f");
    led_on();
    delay(m_delay);
  }

  return 0;
}
