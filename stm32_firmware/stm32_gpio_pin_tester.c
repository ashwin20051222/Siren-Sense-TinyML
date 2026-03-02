/**
 * ============================================================================
 *  STM32 GPIO Pin Tester — Bare-Metal (NUCLEO-F103RB)
 *  Lights up ONE pin at a time, tells you which pin it is via Serial.
 *  Use this to verify your physical wiring matches the pin map!
 * ============================================================================
 *
 *  HOW TO USE:
 *    1. Flash this file to your NUCLEO-F103RB
 *    2. Open Serial Monitor at 115200 baud (ST-Link VCP)
 *    3. Watch which LED/wire lights up for each announced pin
 *    4. Write down what each pin is actually connected to
 *    5. Compare with WIRING.md to find mismatches
 *
 *  The program cycles through all traffic + pedestrian pins:
 *    - Each pin is ON for 3 seconds, all others OFF
 *    - Serial prints: ">>> NOW TESTING: PA8 = North GREEN <<<"
 *    - You look at your board and note which physical LED lit up
 *
 *  BUILD:
 *    arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O2 -DSTM32F103xB \
 *      -T STM32F103RBTx_FLASH.ld -nostartfiles \
 *      stm32_gpio_pin_tester.c -o pin_tester.elf
 * ============================================================================
 */

#include <stdint.h>

/* === Base addresses === */
#define PERIPH_BASE 0x40000000UL
#define APB1_BASE PERIPH_BASE
#define APB2_BASE (PERIPH_BASE + 0x10000UL)
#define AHB_BASE (PERIPH_BASE + 0x20000UL)

#define RCC_BASE (AHB_BASE + 0x1000UL)
#define GPIOA_BASE (APB2_BASE + 0x0800UL)
#define GPIOB_BASE (APB2_BASE + 0x0C00UL)
#define GPIOC_BASE (APB2_BASE + 0x1000UL)
#define GPIOD_BASE (APB2_BASE + 0x1400UL)
#define USART2_BASE (APB1_BASE + 0x4400UL)
#define AFIO_BASE (APB2_BASE + 0x0000UL)

/* SysTick */
#define STK_CTRL (*(volatile uint32_t *)0xE000E010UL)
#define STK_LOAD (*(volatile uint32_t *)0xE000E014UL)
#define STK_VAL (*(volatile uint32_t *)0xE000E018UL)

/* === Register structures === */
typedef struct {
  volatile uint32_t CR, CFGR, CIR, APB2RSTR, APB1RSTR;
  volatile uint32_t AHBENR, APB2ENR, APB1ENR;
  volatile uint32_t BDCR, CSR;
} RCC_TypeDef;
#define RCC ((RCC_TypeDef *)RCC_BASE)

typedef struct {
  volatile uint32_t CRL, CRH, IDR, ODR, BSRR, BRR, LCKR;
} GPIO_TypeDef;
#define GPIOA ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD ((GPIO_TypeDef *)GPIOD_BASE)

typedef struct {
  volatile uint32_t SR, DR, BRR_, CR1, CR2, CR3, GTPR;
} USART_TypeDef;
#define USART2 ((USART_TypeDef *)USART2_BASE)

typedef struct {
  volatile uint32_t EVCR, MAPR, EXTICR[4], RESERVED, MAPR2;
} AFIO_TypeDef;
#define AFIO ((AFIO_TypeDef *)AFIO_BASE)

#define RCC_APB2ENR_IOPAEN (1U << 2)
#define RCC_APB2ENR_IOPBEN (1U << 3)
#define RCC_APB2ENR_IOPCEN (1U << 4)
#define RCC_APB2ENR_IOPDEN (1U << 5)
#define RCC_APB2ENR_AFIOEN (1U << 0)
#define RCC_APB1ENR_USART2EN (1U << 17)

/* ===================== GLOBALS ===================== */

static volatile uint32_t msTicks = 0;

void SysTick_Handler(void) { msTicks++; }

static void delay_ms(uint32_t ms) {
  uint32_t start = msTicks;
  while ((msTicks - start) < ms)
    ;
}

/* ===================== GPIO HELPERS ===================== */

static void gpio_set_mode(GPIO_TypeDef *port, uint8_t pin, uint8_t mode_cnf) {
  if (pin < 8) {
    uint32_t shift = pin * 4;
    port->CRL &= ~(0xFUL << shift);
    port->CRL |= ((uint32_t)mode_cnf << shift);
  } else {
    uint32_t shift = (pin - 8) * 4;
    port->CRH &= ~(0xFUL << shift);
    port->CRH |= ((uint32_t)mode_cnf << shift);
  }
}

static void gpio_write(GPIO_TypeDef *port, uint8_t pin, uint8_t val) {
  if (val)
    port->BSRR = (1UL << pin);
  else
    port->BRR = (1UL << pin);
}

/* ===================== CLOCK SETUP ===================== */

static void SystemClock_Config(void) {
  RCC->CR |= (1U << 16);
  while (!(RCC->CR & (1U << 17)))
    ;
  *(volatile uint32_t *)0x40022000 |= 0x02;
  RCC->CFGR |= (1U << 16);
  RCC->CFGR |= (7U << 18);
  RCC->CFGR |= (4U << 8);
  RCC->CR |= (1U << 24);
  while (!(RCC->CR & (1U << 25)))
    ;
  RCC->CFGR |= (2U << 0);
  while (((RCC->CFGR >> 2) & 3U) != 2U)
    ;
  STK_LOAD = 72000 - 1;
  STK_VAL = 0;
  STK_CTRL = 0x07;
}

/* ===================== USART2 ===================== */

static void USART2_Init(void) {
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  gpio_set_mode(GPIOA, 2, 0x0A);
  gpio_set_mode(GPIOA, 3, 0x04);
  USART2->BRR_ = 0x139;
  USART2->CR1 = (1U << 13) | (1U << 3);
}

static void uart_putc(char c) {
  while (!(USART2->SR & (1U << 7)))
    ;
  USART2->DR = (uint8_t)c;
}

static void uart_puts(const char *s) {
  while (*s)
    uart_putc(*s++);
}

static void uart_put_num(uint32_t n) {
  char buf[12];
  int i = 0;
  if (n == 0) {
    uart_putc('0');
    return;
  }
  while (n > 0) {
    buf[i++] = '0' + (n % 10);
    n /= 10;
  }
  while (i-- > 0)
    uart_putc(buf[i]);
}

/* ===================== PIN TEST TABLE ===================== */

typedef struct {
  GPIO_TypeDef *port;
  uint8_t pin;
  const char *port_name;   /* e.g. "PA0" */
  const char *expected_fn; /* e.g. "North RED traffic light" */
} PinTest_t;

/*
 * NUCLEO-F103RB Physical Pin Locations (Morpho headers):
 *
 *   PIN      MORPHO HEADER   POSITION     ARDUINO HEADER
 *   ----     -------------   --------     --------------
 *   PA0      CN7  pin 28     Left side    A0 (CN8)
 *   PA1      CN7  pin 30     Left side    A1 (CN8)
 *   PA8      CN10 pin 23     Right side   D7 (CN5)
 *   PA9      CN10 pin 21     Right side   D8 (CN9)
 *   PA10     CN10 pin 33     Right side   D2 (CN5)
 *   PA11     CN10 pin 14     Right side   --
 *   PA12     CN10 pin 12     Right side   --
 *   PB4      CN10 pin 27     Right side   D5 (CN5)
 *   PB5      CN10 pin 29     Right side   D4 (CN5)
 *   PB6      CN10 pin 17     Right side   D10 (CN9)
 *   PB7      CN7  pin 21     Left side    --
 *   PB8      CN10 pin 3      Right side   D15 (CN9)
 *   PB9      CN10 pin 5      Right side   D14 (CN9)
 *   PB12     CN10 pin 16     Right side   --
 *   PB13     CN10 pin 30     Right side   --
 *   PB14     CN10 pin 28     Right side   --
 *   PB15     CN10 pin 26     Right side   --
 *   PC0      CN7  pin 38     Left side    A5 (CN8)
 *   PC1      CN7  pin 36     Left side    A4 (CN8)
 *   PC2      CN7  pin 35     Left side    --
 *   PC4      CN10 pin 34     Right side   --
 *   PC5      CN10 pin 6      Right side   --
 *   PC6      CN10 pin 4      Right side   --
 *   PC7      CN10 pin 19     Right side   D9 (CN9)
 *   PC8      CN10 pin 2      Right side   --
 *   PC9      CN10 pin 1      Right side   --
 *   PC10     CN7  pin 1      Left side    --
 *   PC13     CN7  pin 23     Left side    -- (User LED)
 */

static const PinTest_t test_pins[] = {
    /* ===== TRAFFIC LIGHTS ===== */
    /* North */
    {GPIOA, 0, "PA0", "North RED traffic LED     (CN7-28 / A0)"},
    {GPIOA, 1, "PA1", "North YELLOW traffic LED  (CN7-30 / A1)"},
    {GPIOA, 8, "PA8", "North GREEN traffic LED   (CN10-23 / D7)"},

    /* East */
    {GPIOA, 9, "PA9", "East RED traffic LED      (CN10-21 / D8)"},
    {GPIOA, 10, "PA10", "East YELLOW traffic LED   (CN10-33 / D2)"},
    {GPIOA, 11, "PA11", "East GREEN traffic LED    (CN10-14)"},

    /* South */
    {GPIOB, 13, "PB13", "South RED traffic LED     (CN10-30)"},
    {GPIOB, 14, "PB14", "South YELLOW traffic LED  (CN10-28)"},
    {GPIOB, 15, "PB15", "South GREEN traffic LED   (CN10-26)"},

    /* West */
    {GPIOC, 8, "PC8", "West RED traffic LED      (CN10-2)"},
    {GPIOC, 9, "PC9", "West YELLOW traffic LED   (CN10-1)"},
    {GPIOC, 10, "PC10", "West GREEN traffic LED    (CN7-1)"},

    /* ===== PEDESTRIAN LEDs ===== */
    {GPIOB, 9, "PB9", "North Ped RED LED         (CN10-5 / D14)"},
    {GPIOB, 4, "PB4", "North Ped GREEN LED       (CN10-27 / D5)"},
    {GPIOC, 2, "PC2", "East Ped RED LED          (CN7-35)"},
    {GPIOB, 12, "PB12", "East Ped GREEN LED        (CN10-16)"},
    {GPIOC, 0, "PC0", "South Ped RED LED         (CN7-38 / A5)"},
    {GPIOC, 1, "PC1", "South Ped GREEN LED       (CN7-36 / A4)"},
    {GPIOC, 4, "PC4", "West Ped RED LED          (CN10-34)"},
    {GPIOC, 5, "PC5", "West Ped GREEN LED        (CN10-6)"},
};

#define NUM_PINS (sizeof(test_pins) / sizeof(test_pins[0]))

/* ===================== GPIO INIT ===================== */

static void GPIO_Init(void) {
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN |
                  RCC_APB2ENR_IOPDEN | RCC_APB2ENR_AFIOEN;

  /* Disable JTAG to free PB4 (keep SWD) */
  AFIO->MAPR = (AFIO->MAPR & ~(7UL << 24)) | (2UL << 24);

  /* Set all test pins as push-pull outputs */
  for (uint32_t i = 0; i < NUM_PINS; i++) {
    gpio_set_mode(test_pins[i].port, test_pins[i].pin, 0x02);
    gpio_write(test_pins[i].port, test_pins[i].pin, 0); /* OFF */
  }

  /* PC13 user LED */
  gpio_set_mode(GPIOC, 13, 0x02);
}

/* Turn all test pins OFF */
static void all_off(void) {
  for (uint32_t i = 0; i < NUM_PINS; i++) {
    gpio_write(test_pins[i].port, test_pins[i].pin, 0);
  }
}

/* ===================== MAIN ===================== */

int main(void) {
  SystemClock_Config();
  GPIO_Init();
  USART2_Init();

  uart_puts("\r\n");
  uart_puts("########################################################\r\n");
  uart_puts("#                                                      #\r\n");
  uart_puts("#    STM32 GPIO PIN TESTER                             #\r\n");
  uart_puts("#    Tests each LED pin one at a time                  #\r\n");
  uart_puts("#                                                      #\r\n");
  uart_puts("#    Watch which LED lights up for each pin!           #\r\n");
  uart_puts("#    Write down any mismatches.                        #\r\n");
  uart_puts("#                                                      #\r\n");
  uart_puts("#    Each pin stays ON for 3 seconds.                  #\r\n");
  uart_puts("#    Total pins to test: ");
  uart_put_num(NUM_PINS);
  uart_puts("                        #\r\n");
  uart_puts("#                                                      #\r\n");
  uart_puts("########################################################\r\n\r\n");

  uart_puts("Starting in 2 seconds...\r\n\r\n");
  delay_ms(2000);

  uint32_t round = 0;

  while (1) {
    round++;
    uart_puts("========================================\r\n");
    uart_puts("  ROUND ");
    uart_put_num(round);
    uart_puts("\r\n");
    uart_puts("========================================\r\n\r\n");

    for (uint32_t i = 0; i < NUM_PINS; i++) {
      /* Turn everything OFF */
      all_off();

      /* Announce on serial */
      uart_puts(">>> PIN ");
      uart_put_num(i + 1);
      uart_puts("/");
      uart_put_num(NUM_PINS);
      uart_puts(": ");
      uart_puts(test_pins[i].port_name);
      uart_puts(" = ");
      uart_puts(test_pins[i].expected_fn);
      uart_puts(" <<<\r\n");

      /* Turn ON this one pin */
      gpio_write(test_pins[i].port, test_pins[i].pin, 1);

      /* Blink PC13 to show alive */
      gpio_write(GPIOC, 13, 0); /* ON (inverted) */

      /* Wait 3 seconds */
      delay_ms(3000);

      /* Turn it OFF */
      gpio_write(test_pins[i].port, test_pins[i].pin, 0);
      gpio_write(GPIOC, 13, 1); /* OFF */

      /* Brief gap */
      delay_ms(500);
    }

    uart_puts("\r\n");
    uart_puts("*** ROUND COMPLETE! ***\r\n");
    uart_puts("All pins tested. Restarting in 5 seconds...\r\n");
    uart_puts("Check your notes against WIRING.md!\r\n\r\n");

    /* Quick flash all pins together */
    for (int flash = 0; flash < 3; flash++) {
      for (uint32_t i = 0; i < NUM_PINS; i++)
        gpio_write(test_pins[i].port, test_pins[i].pin, 1);
      delay_ms(300);
      all_off();
      delay_ms(300);
    }

    delay_ms(4000);
  }

  return 0;
}

/* ===================== VECTOR TABLE ===================== */

extern uint32_t _estack;

__attribute__((section(".isr_vector"))) void (*const vectors[])(void) = {
    (void (*)(void))&_estack,
    (void (*)(void))main,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    (void (*)(void))SysTick_Handler,
};
