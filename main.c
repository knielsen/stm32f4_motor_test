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
  EN use channel 1/2/3 of TIM4 on PA0/1/2.
  IN 1/2/3 use PA4/PA5/PA8
*/
#define CHAN_EN1 GPIO_Pin_0  /* Connect to pin 3 on L6234 DIP20 */
#define CHAN_IN1 GPIO_Pin_4             /* pin 2 */
#define CHAN_EN2 GPIO_Pin_1             /* pin 18 */
#define CHAN_IN2 GPIO_Pin_5             /* pin 19 */
#define CHAN_EN3 GPIO_Pin_2             /* pin 8 */
#define CHAN_IN3 GPIO_Pin_8             /* pin 9 */


/* This is apparently needed for libc/libm (eg. powf()). */
int __errno;

static volatile uint32_t pwm_pulse_width = 0;
static volatile int32_t pwm_pulse_width_1 = 0;
static volatile int32_t pwm_pulse_width_2 = 0;
static volatile int32_t pwm_pulse_width_3 = 0;


static void delay(__IO uint32_t nCount)
{
    while(nCount--)
        __asm("nop"); // do nothing
}


static void setup_serial(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* enable peripheral clock for USART1 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* GPIOA Configuration:  USART1 TX on PA9 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect USART1 pins to AF2 */
  // TX = PA9
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  USART_Cmd(USART1, ENABLE); // enable USART1
}


static void
setup_gpio(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = CHAN_IN1|CHAN_IN2|CHAN_IN3;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}


static const uint32_t pwm_period = 84000000/50000;

static void
setup_timer(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 |
    GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_6 | GPIO_Pin_7 |
    GPIO_Pin_8;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);

  TIM_TimeBaseStructure.TIM_Period = pwm_period-1;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1Init(TIM5, &TIM_OCInitStructure);
  TIM_OC2Init(TIM5, &TIM_OCInitStructure);
  TIM_OC3Init(TIM5, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_ARRPreloadConfig(TIM5, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
  TIM_Cmd(TIM5, ENABLE);
}


static void
setup_timer_interrupt(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
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


/*
  Set a channel to given level (-1, 0, or 1).

  LEVEL set to 0 turns off the channel (EN=0).
  LEVEL set to -1 or 1 turns on the channel (EN=1) and sets IN=0 or 1.

  DUTY controls the PWM and polarity.
  The absolute value of DUTY sets the duty cycle for EN (0->0%, 1->100%).

  The sign sets the polarity (positive enables at the start of the PWM period,
  negative at the nd of the period).
*/
static void
set_channel(unsigned channel, int level, float duty)
{
  unsigned in;
  volatile int32_t *enable;
  int32_t period;

  switch (channel)
  {
  case 1:
    enable = &pwm_pulse_width_1;
    in = CHAN_IN1;
    break;
  case 2:
    enable = &pwm_pulse_width_2;
    in = CHAN_IN2;
    break;
  case 3:
    enable = &pwm_pulse_width_3;
    in = CHAN_IN3;
    break;
  default:
    return;
  }

  period = (int32_t)(duty*pwm_period);
  if (level < 0)
  {
    *enable = period;
    GPIO_ResetBits(GPIOA, in);
  }
  else if (level > 0)
  {
    *enable = period;
    GPIO_SetBits(GPIOA, in);
  }
  else
    *enable = 0;
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


void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM3->CCR1 = pwm_pulse_width;
  }
}


void TIM4_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    TIM4->CCR1 = pwm_pulse_width;
  }
}


void TIM5_IRQHandler(void)
{
  int32_t tmp;

  if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

    tmp = pwm_pulse_width_1;
    if (tmp < 0)
    {
      TIM5->CCR1 = pwm_period + (uint32_t)tmp;
      TIM_OC1PolarityConfig(TIM5, TIM_OCPolarity_Low);
    }
    else
    {
      TIM5->CCR1 = (uint32_t)tmp;
      TIM_OC1PolarityConfig(TIM5, TIM_OCPolarity_High);
    }

    tmp = pwm_pulse_width_2;
    if (tmp < 0)
    {
      TIM5->CCR2 = pwm_period +  (uint32_t)tmp;
      TIM_OC2PolarityConfig(TIM5, TIM_OCPolarity_Low);
    }
    else
    {
      TIM5->CCR2 = (uint32_t)tmp;
      TIM_OC2PolarityConfig(TIM5, TIM_OCPolarity_High);
    }

    tmp = pwm_pulse_width_3;
    if (tmp < 0)
    {
      TIM5->CCR3 = pwm_period + (uint32_t)tmp;
      TIM_OC3PolarityConfig(TIM5, TIM_OCPolarity_Low);
    }
    else
    {
      TIM5->CCR3 = (uint32_t)tmp;
      TIM_OC3PolarityConfig(TIM5, TIM_OCPolarity_High);
    }
  }
}


int main(void)
{
  static const uint32_t m_delay = 10000000;

  delay(2000000);
  setup_serial();
  setup_gpio();
  setup_led();
  setup_timer();
  setup_timer_interrupt();
  serial_puts(USART1, "Initialising...\r\n");
  set_channel(1, 0, 1);
  set_channel(2, 0, 1);
  set_channel(3, 0, 1);
  delay(2000000);

  serial_puts(USART1, "Hello world, ready to blink!\r\n");

  while (1)
  {
/*
    set_channel(1, 0);
    set_channel(2, 1);
    set_channel(3, -1);
    serial_puts(USART1, "a");
    led_off();
    pwm_pulse_width = pwm_period/6;
    delay(m_delay);

    set_channel(1, -1);
    set_channel(2, 1);
    set_channel(3, 0);
    serial_puts(USART1, "b");
    led_on();
    pwm_pulse_width = 2*pwm_period/6;
    delay(m_delay);
*/
    set_channel(1, -1, 1);
    set_channel(2, 0, -1);
    set_channel(3, 1, 1);
    serial_puts(USART1, "c");
    led_off();
    pwm_pulse_width = 3*pwm_period/6;
    delay(m_delay);

    set_channel(1, -1, 0.8);
    set_channel(2, -1, -0.2);
    set_channel(3, 1, 1);
    serial_puts(USART1, "/");
    led_off();
    pwm_pulse_width = 3*pwm_period/6;
    delay(m_delay);

    set_channel(1, -1, 0.60);
    set_channel(2, -1, -0.40);
    set_channel(3, 1, 1);
    serial_puts(USART1, "/");
    led_off();
    pwm_pulse_width = 3*pwm_period/6;
    delay(m_delay);

    set_channel(1, -1, 0.4);
    set_channel(2, -1, -0.6);
    set_channel(3, 1, 1);
    serial_puts(USART1, "/");
    led_off();
    pwm_pulse_width = 3*pwm_period/6;
    delay(m_delay);

    set_channel(1, -1, 0.2);
    set_channel(2, -1, -0.8);
    set_channel(3, 1, 1);
    serial_puts(USART1, "!");
    led_off();
    pwm_pulse_width = 3*pwm_period/6;
    delay(m_delay);

    set_channel(1, 0, 1);
    set_channel(2, -1, -1);
    set_channel(3, 1, 1);
    serial_puts(USART1, "d");
    led_on();
    pwm_pulse_width = 4*pwm_period/6;
    delay(m_delay);
/*
    set_channel(1, 1);
    set_channel(2, -1);
    set_channel(3, 0);
    serial_puts(USART1, "e");
    led_off();
    pwm_pulse_width = 5*pwm_period/6;
    delay(m_delay);

    set_channel(1, 1);
    set_channel(2, 0);
    set_channel(3, -1);
    serial_puts(USART1, "f");
    led_on();
    pwm_pulse_width = pwm_period;
    delay(m_delay);
*/
  }

  return 0;
}
