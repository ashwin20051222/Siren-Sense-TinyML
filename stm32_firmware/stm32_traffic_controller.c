/**
 * ============================================================================
 *  STM32 Traffic Light Controller — Bare-Metal Firmware
 *  Board: NUCLEO-F103RB (STM32F103RBT6, 72 MHz, 128KB Flash, 20KB RAM)
 * ============================================================================
 *
 *  FEATURES:
 *    - 4-direction traffic lights (N/E/S/W) with R/Y/G LEDs
 *    - 4 pedestrian buttons with Red/Green walk LEDs
 *    - USART3 (PB10/PB11) receives "AMB:<lane>\n" from ESP32 receiver
 *    - USART2 (PA2/PA3) for debug serial output (ST-Link VCP)
 *    - 16x2 I2C LCD (PCF8574 backpack, bit-banged on PC6/PC7)
 *    - Emergency vehicle override: highest priority
 *    - SysTick 1ms timing
 *
 *  BUILD:
 *    arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O2 -DSTM32F103xB \
 *      -T STM32F103RBTx_FLASH.ld -nostartfiles \
 *      stm32_traffic_controller.c -o traffic.elf
 *
 *    Or paste into STM32CubeIDE main.c (between USER CODE BEGIN/END blocks)
 *
 *  PIN MAP:
 *    Traffic LEDs: PA0,PA1,PA8 (N) | PA9,PA10,PA11 (E) |
 *                  PB13,PB14,PB15 (S) | PC8,PC9,PC10 (W)
 *    Ped Buttons:  PB5(N), PB6(E), PB7(S), PB8(W)
 *    Ped LEDs:     PB9/PB4 (N R/G) | PB3/PB12 (E R/G) |
 *                  PC0/PC1 (S R/G) | PC4/PC5 (W R/G)
 *    ESP32 UART:   PB10(TX3), PB11(RX3) — USART3 @ 115200
 *    Debug UART:   PA2(TX2), PA3(RX2) — USART2 @ 115200
 *    LCD I2C:      PC6(SDA), PC7(SCL) — bit-banged
 * ============================================================================
 */

/* ---- STM32F103 Register Definitions (minimal, no HAL) ---- */

#include <stdint.h>
#include <string.h>

/* Base addresses */
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
#define USART3_BASE (APB1_BASE + 0x4800UL)
#define AFIO_BASE (APB2_BASE + 0x0000UL)

/* SysTick */
#define STK_CTRL (*(volatile uint32_t *)0xE000E010UL)
#define STK_LOAD (*(volatile uint32_t *)0xE000E014UL)
#define STK_VAL (*(volatile uint32_t *)0xE000E018UL)

/* RCC registers */
typedef struct {
  volatile uint32_t CR, CFGR, CIR, APB2RSTR, APB1RSTR;
  volatile uint32_t AHBENR, APB2ENR, APB1ENR;
  volatile uint32_t BDCR, CSR;
} RCC_TypeDef;
#define RCC ((RCC_TypeDef *)RCC_BASE)

/* GPIO registers */
typedef struct {
  volatile uint32_t CRL, CRH, IDR, ODR, BSRR, BRR, LCKR;
} GPIO_TypeDef;
#define GPIOA ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD ((GPIO_TypeDef *)GPIOD_BASE)

/* USART registers */
typedef struct {
  volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
} USART_TypeDef;
#define USART2 ((USART_TypeDef *)USART2_BASE)
#define USART3 ((USART_TypeDef *)USART3_BASE)

/* AFIO */
typedef struct {
  volatile uint32_t EVCR, MAPR, EXTICR[4], RESERVED, MAPR2;
} AFIO_TypeDef;
#define AFIO ((AFIO_TypeDef *)AFIO_BASE)

/* Bit definitions */
#define RCC_APB2ENR_IOPAEN (1U << 2)
#define RCC_APB2ENR_IOPBEN (1U << 3)
#define RCC_APB2ENR_IOPCEN (1U << 4)
#define RCC_APB2ENR_IOPDEN (1U << 5)
#define RCC_APB2ENR_AFIOEN (1U << 0)
#define RCC_APB1ENR_USART2EN (1U << 17)
#define RCC_APB1ENR_USART3EN (1U << 18)

#define USART_CR1_UE (1U << 13)
#define USART_CR1_TE (1U << 3)
#define USART_CR1_RE (1U << 2)
#define USART_CR1_RXNEIE (1U << 5)
#define USART_SR_TXE (1U << 7)
#define USART_SR_RXNE (1U << 5)
#define USART_SR_TC (1U << 6)

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 1: CONFIGURATION                                       ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

/* Timing (milliseconds) */
#define GREEN_TIME_MS 10000
#define YELLOW_TIME_MS 3000
#define ALL_RED_GAP_MS 1000
#define PED_WALK_TIME_MS 7000
#define PED_FLASH_TIME_MS 3000
#define EMERGENCY_CLEAR_MS 2000  /* time to clear current lane before override \
                                  */
#define EMERGENCY_GREEN_MS 15000 /* emergency lane green duration */

/* LCD I2C */
#define LCD_I2C_ADDR 0x27 /* PCF8574 address (try 0x3F if blank) */
#define LCD_BL 0x08
#define LCD_EN 0x04
#define LCD_RS 0x01

/* Lane indices */
#define LANE_NORTH 0
#define LANE_EAST 1
#define LANE_SOUTH 2
#define LANE_WEST 3
#define NUM_LANES 4

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 2: GLOBAL STATE                                        ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

static volatile uint32_t msTicks = 0;

/* Traffic FSM */
typedef enum {
  STATE_GREEN,
  STATE_YELLOW,
  STATE_ALL_RED,
  STATE_PED_WALK,
  STATE_PED_FLASH,
  STATE_EMERGENCY_CLEAR,
  STATE_EMERGENCY_GREEN
} TrafficState_t;

static TrafficState_t currentState = STATE_GREEN;
static uint8_t activeLane = LANE_NORTH;
static uint32_t stateStartMs = 0;

/* Pedestrian */
static uint8_t pedRequested[NUM_LANES] = {0, 0, 0, 0};
static uint8_t pedActiveForLane = 0;

/* Emergency */
static volatile uint8_t emergencyRequested = 0;
static volatile uint8_t emergencyLane = 0;
static uint8_t emergencyActive = 0;

/* USART3 receive buffer */
#define UART_BUF_SIZE 32
static volatile char uartBuf[UART_BUF_SIZE];
static volatile uint8_t uartIdx = 0;

/* LCD state */
static uint32_t lastLcdUpdate = 0;

/* Lane names */
static const char *LANE_NAMES[] = {"NORTH", "EAST ", "SOUTH", "WEST "};
static const char *LANE_SHORT[] = {"N", "E", "S", "W"};

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 3: LOW-LEVEL FUNCTIONS                                  ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

void SysTick_Handler(void) { msTicks++; }

static void delay_ms(uint32_t ms) {
  uint32_t start = msTicks;
  while ((msTicks - start) < ms)
    ;
}

static uint32_t millis(void) { return msTicks; }

/* GPIO configuration helper */
static void gpio_set_mode(GPIO_TypeDef *port, uint8_t pin, uint8_t mode_cnf) {
  /* mode_cnf: 4 bits (CNFx[1:0] | MODEx[1:0]) */
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

static uint8_t gpio_read(GPIO_TypeDef *port, uint8_t pin) {
  return (port->IDR >> pin) & 1U;
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 4: CLOCK SETUP (72 MHz from 8 MHz HSE)                 ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

static void SystemClock_Config(void) {
  /* Enable HSE (8 MHz on NUCLEO from ST-Link) */
  RCC->CR |= (1U << 16); /* HSEON */
  while (!(RCC->CR & (1U << 17)))
    ; /* Wait HSE ready */

  /* Flash: 2 wait states for 72 MHz */
  *(volatile uint32_t *)0x40022000 |= 0x02;

  /* PLL: HSE / 1 * 9 = 72 MHz */
  RCC->CFGR |= (1U << 16); /* PLLSRC = HSE */
  RCC->CFGR |= (7U << 18); /* PLLMUL = 9 */
  RCC->CFGR |= (4U << 8);  /* APB1 prescaler = /2 (36 MHz max) */

  /* Enable PLL */
  RCC->CR |= (1U << 24); /* PLLON */
  while (!(RCC->CR & (1U << 25)))
    ; /* Wait PLL ready */

  /* Switch to PLL */
  RCC->CFGR |= (2U << 0); /* SW = PLL */
  while (((RCC->CFGR >> 2) & 3U) != 2U)
    ; /* Wait SWS = PLL */

  /* SysTick: 72 MHz / 72000 = 1 kHz (1ms) */
  STK_LOAD = 72000 - 1;
  STK_VAL = 0;
  STK_CTRL = 0x07; /* Enable, AHB clock, interrupt */
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 5: GPIO INITIALIZATION                                  ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

static void GPIO_Init(void) {
  /* Enable clocks: GPIOA, GPIOB, GPIOC, GPIOD, AFIO */
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN |
                  RCC_APB2ENR_IOPDEN | RCC_APB2ENR_AFIOEN;

  /* Disable JTAG to free PB3, PB4 (keep SWD) */
  AFIO->MAPR =
      (AFIO->MAPR & ~(7UL << 24)) | (2UL << 24); /* SWJ-DP without JTAG */

  /* --- Traffic Light LEDs (Push-Pull Output, 2 MHz) --- */
  /* Mode: 0b0010 = Output 2MHz, Push-Pull */
  /* North: PA0(R), PA1(Y), PA8(G) */
  gpio_set_mode(GPIOA, 0, 0x02);
  gpio_set_mode(GPIOA, 1, 0x02);
  gpio_set_mode(GPIOA, 8, 0x02);
  /* East: PA9(R), PA10(Y), PA11(G) */
  gpio_set_mode(GPIOA, 9, 0x02);
  gpio_set_mode(GPIOA, 10, 0x02);
  gpio_set_mode(GPIOA, 11, 0x02);
  /* South: PB13(R), PB14(Y), PB15(G) */
  gpio_set_mode(GPIOB, 13, 0x02);
  gpio_set_mode(GPIOB, 14, 0x02);
  gpio_set_mode(GPIOB, 15, 0x02);
  /* West: PC8(R), PC9(Y), PC10(G) */
  gpio_set_mode(GPIOC, 8, 0x02);
  gpio_set_mode(GPIOC, 9, 0x02);
  gpio_set_mode(GPIOC, 10, 0x02);

  /* --- Pedestrian LEDs (Push-Pull Output, 2 MHz) --- */
  /* North Ped: PB9(R), PB4(G) */
  gpio_set_mode(GPIOB, 9, 0x02);
  gpio_set_mode(GPIOB, 4, 0x02);
  /* East Ped: PB3(R), PB12(G) */
  gpio_set_mode(GPIOB, 3, 0x02);
  gpio_set_mode(GPIOB, 12, 0x02);
  /* South Ped: PC0(R), PC1(G) */
  gpio_set_mode(GPIOC, 0, 0x02);
  gpio_set_mode(GPIOC, 1, 0x02);
  /* West Ped: PC4(R), PC5(G) */
  gpio_set_mode(GPIOC, 4, 0x02);
  gpio_set_mode(GPIOC, 5, 0x02);

  /* --- Pedestrian Buttons (Input Pull-Up) --- */
  /* Mode: 0b1000 = Input with Pull-up/Pull-down, then set ODR for pull-up */
  /* PB5(N), PB6(E), PB7(S), PB8(W) */
  gpio_set_mode(GPIOB, 5, 0x08);
  gpio_set_mode(GPIOB, 6, 0x08);
  gpio_set_mode(GPIOB, 7, 0x08);
  gpio_set_mode(GPIOB, 8, 0x08);
  GPIOB->ODR |= (1U << 5) | (1U << 6) | (1U << 7) | (1U << 8); /* Pull-up */

  /* --- User LED PC13 (Push-Pull Output, inverted on NUCLEO) --- */
  gpio_set_mode(GPIOC, 13, 0x02);

  /* --- LCD I2C: PC6(SDA), PC7(SCL) — Open-Drain Output for bit-bang --- */
  gpio_set_mode(GPIOC, 6, 0x06); /* Output 2MHz, Open-Drain */
  gpio_set_mode(GPIOC, 7, 0x06);
  gpio_write(GPIOC, 6, 1); /* Idle high */
  gpio_write(GPIOC, 7, 1);
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 6: USART2 (Debug Serial — ST-Link VCP)                 ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

static void USART2_Init(void) {
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

  /* PA2 = USART2 TX: AF Push-Pull, 2 MHz */
  gpio_set_mode(GPIOA, 2, 0x0A);
  /* PA3 = USART2 RX: Input Floating */
  gpio_set_mode(GPIOA, 3, 0x04);

  /* Baud = 115200, APB1 = 36 MHz → BRR = 36000000 / 115200 ≈ 312.5 → 0x139 */
  USART2->BRR = 0x139;
  USART2->CR1 = USART_CR1_UE | USART_CR1_TE;
}

static void uart2_putc(char c) {
  while (!(USART2->SR & USART_SR_TXE))
    ;
  USART2->DR = (uint8_t)c;
}

static void uart2_puts(const char *s) {
  while (*s)
    uart2_putc(*s++);
}

static void uart2_print_num(uint32_t n) {
  char buf[12];
  int i = 0;
  if (n == 0) {
    uart2_putc('0');
    return;
  }
  while (n > 0) {
    buf[i++] = '0' + (n % 10);
    n /= 10;
  }
  while (i-- > 0)
    uart2_putc(buf[i]);
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 7: USART3 (ESP32 Receiver — PB10 TX / PB11 RX)        ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

static void USART3_Init(void) {
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

  /* PB10 = USART3 TX: AF Push-Pull, 2 MHz */
  gpio_set_mode(GPIOB, 10, 0x0A);
  /* PB11 = USART3 RX: Input Floating */
  gpio_set_mode(GPIOB, 11, 0x04);

  /* Baud = 115200, APB1 = 36 MHz */
  USART3->BRR = 0x139;
  USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

  /* Enable USART3 interrupt in NVIC (IRQ 39) */
  *(volatile uint32_t *)(0xE000E100 + 4 * (39 / 32)) |= (1UL << (39 % 32));
}

/* USART3 interrupt handler — receives "AMB:<lane>\n" from ESP32 */
void USART3_IRQHandler(void) {
  if (USART3->SR & USART_SR_RXNE) {
    char c = (char)(USART3->DR & 0xFF);

    if (c == '\n' || c == '\r') {
      if (uartIdx > 0) {
        uartBuf[uartIdx] = '\0';
        /* Parse "AMB:<lane>" */
        if (uartIdx >= 5 && uartBuf[0] == 'A' && uartBuf[1] == 'M' &&
            uartBuf[2] == 'B' && uartBuf[3] == ':') {
          int lane = uartBuf[4] - '0';
          if (lane >= 0 && lane < NUM_LANES) {
            emergencyLane = (uint8_t)lane;
            emergencyRequested = 1;
          }
        }
        uartIdx = 0;
      }
    } else {
      if (uartIdx < UART_BUF_SIZE - 1)
        uartBuf[uartIdx++] = c;
      else
        uartIdx = 0;
    }
  }
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 8: BIT-BANGED I2C + LCD DRIVER (PC6=SDA, PC7=SCL)     ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

/* Microsecond-ish delay (approximate at 72 MHz) */
static void i2c_delay(void) {
  volatile uint32_t n = 36; /* ~5µs at 72 MHz */
  while (n--)
    ;
}

#define SDA_HIGH() gpio_write(GPIOC, 6, 1)
#define SDA_LOW() gpio_write(GPIOC, 6, 0)
#define SCL_HIGH() gpio_write(GPIOC, 7, 1)
#define SCL_LOW() gpio_write(GPIOC, 7, 0)

static void i2c_start(void) {
  SDA_HIGH();
  i2c_delay();
  SCL_HIGH();
  i2c_delay();
  SDA_LOW();
  i2c_delay();
  SCL_LOW();
  i2c_delay();
}

static void i2c_stop(void) {
  SDA_LOW();
  i2c_delay();
  SCL_HIGH();
  i2c_delay();
  SDA_HIGH();
  i2c_delay();
}

static void i2c_write_byte(uint8_t data) {
  for (int i = 7; i >= 0; i--) {
    if (data & (1 << i))
      SDA_HIGH();
    else
      SDA_LOW();
    i2c_delay();
    SCL_HIGH();
    i2c_delay();
    SCL_LOW();
    i2c_delay();
  }
  /* ACK bit (ignore) */
  SDA_HIGH();
  i2c_delay();
  SCL_HIGH();
  i2c_delay();
  SCL_LOW();
  i2c_delay();
}

static void lcd_i2c_send(uint8_t data) {
  i2c_start();
  i2c_write_byte(LCD_I2C_ADDR << 1); /* Address + Write */
  i2c_write_byte(data);
  i2c_stop();
}

static void lcd_pulse(uint8_t data) {
  lcd_i2c_send(data | LCD_EN);
  lcd_i2c_send(data & ~LCD_EN);
}

static void lcd_send_nibble(uint8_t nibble, uint8_t mode) {
  lcd_pulse((nibble & 0xF0) | mode | LCD_BL);
}

static void lcd_send_byte(uint8_t val, uint8_t mode) {
  lcd_send_nibble(val & 0xF0, mode);
  lcd_send_nibble((val << 4) & 0xF0, mode);
}

static void lcd_cmd(uint8_t cmd) {
  lcd_send_byte(cmd, 0);
  if (cmd <= 0x02)
    delay_ms(2);
}

static void lcd_data(uint8_t ch) { lcd_send_byte(ch, LCD_RS); }

static void lcd_init(void) {
  delay_ms(50);
  lcd_send_nibble(0x30, 0);
  delay_ms(5);
  lcd_send_nibble(0x30, 0);
  delay_ms(5);
  lcd_send_nibble(0x30, 0);
  delay_ms(1);
  lcd_send_nibble(0x20, 0);
  delay_ms(1);
  lcd_cmd(0x28); /* 4-bit, 2 lines, 5x8 */
  lcd_cmd(0x0C); /* Display on, cursor off */
  lcd_cmd(0x01); /* Clear */
  lcd_cmd(0x06); /* Entry mode: increment */
}

static void lcd_set_cursor(uint8_t row, uint8_t col) {
  lcd_cmd(0x80 | ((row ? 0x40 : 0x00) + col));
}

static void lcd_print(const char *s) {
  while (*s)
    lcd_data((uint8_t)*s++);
}

static void lcd_print_padded(const char *s, uint8_t width) {
  uint8_t i = 0;
  while (*s && i < width) {
    lcd_data((uint8_t)*s++);
    i++;
  }
  while (i < width) {
    lcd_data(' ');
    i++;
  }
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 9: TRAFFIC LIGHT CONTROL                               ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

/* Traffic light pin structures */
typedef struct {
  GPIO_TypeDef *rPort;
  uint8_t rPin;
  GPIO_TypeDef *yPort;
  uint8_t yPin;
  GPIO_TypeDef *gPort;
  uint8_t gPin;
} TrafficLight_t;

typedef struct {
  GPIO_TypeDef *rPort;
  uint8_t rPin;
  GPIO_TypeDef *gPort;
  uint8_t gPin;
  GPIO_TypeDef *btnPort;
  uint8_t btnPin;
} PedestrianLight_t;

static const TrafficLight_t TL[NUM_LANES] = {
    {GPIOA, 0, GPIOA, 1, GPIOA, 8},    /* North: PA0(R), PA1(Y), PA8(G) */
    {GPIOA, 9, GPIOA, 10, GPIOA, 11},  /* East:  PA9(R), PA10(Y), PA11(G) */
    {GPIOB, 13, GPIOB, 14, GPIOB, 15}, /* South: PB13(R), PB14(Y), PB15(G) */
    {GPIOC, 8, GPIOC, 9, GPIOC, 10},   /* West:  PC8(R), PC9(Y), PC10(G) */
};

static const PedestrianLight_t PL[NUM_LANES] = {
    {GPIOB, 9, GPIOB, 4, GPIOB, 5},  /* North: PB9(R), PB4(G), PB5(Btn) */
    {GPIOB, 3, GPIOB, 12, GPIOB, 6}, /* East:  PB3(R), PB12(G), PB6(Btn) */
    {GPIOC, 0, GPIOC, 1, GPIOB, 7},  /* South: PC0(R), PC1(G), PB7(Btn) */
    {GPIOC, 4, GPIOC, 5, GPIOB, 8},  /* West:  PC4(R), PC5(G), PB8(Btn) */
};

/* Set a single traffic light color */
static void tl_set(uint8_t lane, uint8_t r, uint8_t y, uint8_t g) {
  gpio_write(TL[lane].rPort, TL[lane].rPin, r);
  gpio_write(TL[lane].yPort, TL[lane].yPin, y);
  gpio_write(TL[lane].gPort, TL[lane].gPin, g);
}

/* Set pedestrian LEDs */
static void ped_set(uint8_t lane, uint8_t red, uint8_t green) {
  gpio_write(PL[lane].rPort, PL[lane].rPin, red);
  gpio_write(PL[lane].gPort, PL[lane].gPin, green);
}

/* All lanes RED */
static void all_red(void) {
  for (int i = 0; i < NUM_LANES; i++) {
    tl_set(i, 1, 0, 0);
    ped_set(i, 1, 0); /* Pedestrian: DON'T WALK */
  }
}

/* Set active lane GREEN, others RED */
static void set_green(uint8_t lane) {
  for (int i = 0; i < NUM_LANES; i++) {
    if (i == lane) {
      tl_set(i, 0, 0, 1); /* GREEN */
    } else {
      tl_set(i, 1, 0, 0); /* RED */
    }
    ped_set(i, 1, 0); /* All peds: DON'T WALK */
  }
}

/* Set active lane YELLOW, others RED */
static void set_yellow(uint8_t lane) {
  for (int i = 0; i < NUM_LANES; i++) {
    if (i == lane) {
      tl_set(i, 0, 1, 0); /* YELLOW */
    } else {
      tl_set(i, 1, 0, 0); /* RED */
    }
    ped_set(i, 1, 0);
  }
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 10: PEDESTRIAN BUTTON SCANNING                         ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

static uint32_t lastBtnScan = 0;

static void scan_pedestrian_buttons(void) {
  uint32_t now = millis();
  if ((now - lastBtnScan) < 50)
    return; /* Debounce: 50ms */
  lastBtnScan = now;

  for (int i = 0; i < NUM_LANES; i++) {
    if (!gpio_read(PL[i].btnPort,
                   PL[i].btnPin)) { /* Button pressed (active LOW) */
      if (!pedRequested[i]) {
        pedRequested[i] = 1;
        uart2_puts("[PED] Button pressed: ");
        uart2_puts(LANE_NAMES[i]);
        uart2_puts("\r\n");
      }
    }
  }
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 11: TRAFFIC STATE MACHINE                              ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

static void enter_state(TrafficState_t state) {
  currentState = state;
  stateStartMs = millis();
}

static void next_lane(void) { activeLane = (activeLane + 1) % NUM_LANES; }

static void traffic_fsm_update(void) {
  uint32_t elapsed = millis() - stateStartMs;

  /* Check emergency request (highest priority) */
  if (emergencyRequested && currentState != STATE_EMERGENCY_CLEAR &&
      currentState != STATE_EMERGENCY_GREEN) {
    emergencyRequested = 0;
    emergencyActive = 1;
    uart2_puts("\r\n*** EMERGENCY VEHICLE! Lane ");
    uart2_putc('0' + emergencyLane);
    uart2_puts(" (");
    uart2_puts(LANE_NAMES[emergencyLane]);
    uart2_puts(") ***\r\n");

    /* If we're already on the emergency lane and green, just hold it */
    if (activeLane == emergencyLane && currentState == STATE_GREEN) {
      enter_state(STATE_EMERGENCY_GREEN);
      set_green(emergencyLane);
      return;
    }

    /* Otherwise, quickly clear current traffic */
    set_yellow(activeLane);
    enter_state(STATE_EMERGENCY_CLEAR);
    return;
  }

  switch (currentState) {

  case STATE_GREEN:
    if (elapsed == 0) {
      set_green(activeLane);
      uart2_puts("[TL] GREEN -> ");
      uart2_puts(LANE_NAMES[activeLane]);
      uart2_puts("\r\n");
    }

    if (elapsed >= GREEN_TIME_MS) {
      /* Check if pedestrian requested for this lane */
      if (pedRequested[activeLane]) {
        pedRequested[activeLane] = 0;
        pedActiveForLane = activeLane;
        enter_state(STATE_PED_WALK);
        /* Keep current lane green, activate ped green */
        ped_set(activeLane, 0, 1); /* WALK */
        uart2_puts("[PED] Walk phase -> ");
        uart2_puts(LANE_NAMES[activeLane]);
        uart2_puts("\r\n");
      } else {
        set_yellow(activeLane);
        enter_state(STATE_YELLOW);
      }
    }
    break;

  case STATE_PED_WALK:
    if (elapsed >= PED_WALK_TIME_MS) {
      enter_state(STATE_PED_FLASH);
    }
    break;

  case STATE_PED_FLASH:
    /* Flash pedestrian green LED */
    if (((elapsed / 250) % 2) == 0) {
      ped_set(pedActiveForLane, 0, 1); /* On */
    } else {
      ped_set(pedActiveForLane, 0, 0); /* Off */
    }

    if (elapsed >= PED_FLASH_TIME_MS) {
      ped_set(pedActiveForLane, 1, 0); /* Back to DON'T WALK */
      set_yellow(activeLane);
      enter_state(STATE_YELLOW);
    }
    break;

  case STATE_YELLOW:
    if (elapsed >= YELLOW_TIME_MS) {
      all_red();
      enter_state(STATE_ALL_RED);
    }
    break;

  case STATE_ALL_RED:
    if (elapsed >= ALL_RED_GAP_MS) {
      next_lane();
      enter_state(STATE_GREEN);
    }
    break;

  case STATE_EMERGENCY_CLEAR:
    /* Yellow for 2 seconds, then switch to emergency lane */
    if (elapsed >= EMERGENCY_CLEAR_MS) {
      all_red();
      delay_ms(500); /* Brief all-red gap */
      activeLane = emergencyLane;
      set_green(emergencyLane);
      enter_state(STATE_EMERGENCY_GREEN);
      uart2_puts("[EMG] Emergency lane GREEN -> ");
      uart2_puts(LANE_NAMES[emergencyLane]);
      uart2_puts("\r\n");
    }
    break;

  case STATE_EMERGENCY_GREEN:
    if (elapsed >= EMERGENCY_GREEN_MS) {
      emergencyActive = 0;
      uart2_puts("[EMG] Emergency override ended.\r\n");
      /* Resume normal cycle from next lane */
      set_yellow(activeLane);
      enter_state(STATE_YELLOW);
    }
    break;
  }
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 12: LCD DISPLAY UPDATE                                  ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

static void lcd_update(void) {
  uint32_t now = millis();
  if ((now - lastLcdUpdate) < 500)
    return;
  lastLcdUpdate = now;

  uint32_t elapsed = now - stateStartMs;
  char row0[17], row1[17];

  if (currentState == STATE_EMERGENCY_CLEAR ||
      currentState == STATE_EMERGENCY_GREEN) {
    /* === EMERGENCY MODE === */
    uint32_t totalMs;
    if (currentState == STATE_EMERGENCY_CLEAR)
      totalMs = EMERGENCY_CLEAR_MS;
    else
      totalMs = EMERGENCY_GREEN_MS;

    uint32_t remMs = (elapsed < totalMs) ? totalMs - elapsed : 0;
    uint32_t remSec = (remMs + 999) / 1000;

    /* Row 0: "!! EMERGENCY !!" */
    memcpy(row0, "!! EMERGENCY !!", 16);
    row0[15] = '\0';

    /* Row 1: "DIR:NORTH  14s" */
    int len = 0;
    row1[len++] = 'D';
    row1[len++] = 'I';
    row1[len++] = 'R';
    row1[len++] = ':';
    const char *ln = LANE_NAMES[emergencyLane];
    while (*ln && len < 9)
      row1[len++] = *ln++;
    while (len < 11)
      row1[len++] = ' ';
    if (remSec >= 10) {
      row1[len++] = '0' + (remSec / 10);
    } else {
      row1[len++] = ' ';
    }
    row1[len++] = '0' + (remSec % 10);
    row1[len++] = 's';
    while (len < 16)
      row1[len++] = ' ';
    row1[len] = '\0';

  } else if (currentState == STATE_PED_WALK ||
             currentState == STATE_PED_FLASH) {
    /* === PEDESTRIAN MODE === */
    uint32_t totalMs = PED_WALK_TIME_MS + PED_FLASH_TIME_MS;
    uint32_t pedElapsed = elapsed;
    if (currentState == STATE_PED_FLASH)
      pedElapsed += PED_WALK_TIME_MS; /* approximate */
    uint32_t remMs =
        (elapsed < (currentState == STATE_PED_WALK ? PED_WALK_TIME_MS
                                                   : PED_FLASH_TIME_MS))
            ? (currentState == STATE_PED_WALK ? PED_WALK_TIME_MS
                                              : PED_FLASH_TIME_MS) -
                  elapsed
            : 0;
    uint32_t remSec = (remMs + 999) / 1000;

    memcpy(row0, "PED CROSSING    ", 16);
    row0[16] = '\0';

    int len = 0;
    const char *ln = LANE_NAMES[pedActiveForLane];
    while (*ln && len < 5)
      row1[len++] = *ln++;
    while (len < 6)
      row1[len++] = ' ';
    row1[len++] = 'W';
    row1[len++] = 'A';
    row1[len++] = 'L';
    row1[len++] = 'K';
    row1[len++] = ' ';
    if (remSec >= 10)
      row1[len++] = '0' + (remSec / 10);
    else
      row1[len++] = ' ';
    row1[len++] = '0' + (remSec % 10);
    row1[len++] = 's';
    while (len < 16)
      row1[len++] = ' ';
    row1[len] = '\0';

  } else {
    /* === NORMAL MODE === */
    uint32_t dur = 0;
    switch (currentState) {
    case STATE_GREEN:
      dur = GREEN_TIME_MS;
      break;
    case STATE_YELLOW:
      dur = YELLOW_TIME_MS;
      break;
    case STATE_ALL_RED:
      dur = ALL_RED_GAP_MS;
      break;
    default:
      break;
    }
    uint32_t remMs = (elapsed < dur) ? dur - elapsed : 0;
    uint32_t remSec = (remMs + 999) / 1000;
    if (remSec > 99)
      remSec = 99;

    /* Row 0: "N:10E:--S:--W:--" */
    int len = 0;
    for (int i = 0; i < NUM_LANES; i++) {
      row0[len++] = *LANE_SHORT[i];
      row0[len++] = ':';
      if (i == activeLane && currentState != STATE_ALL_RED) {
        if (remSec >= 10)
          row0[len++] = '0' + (remSec / 10);
        else
          row0[len++] = '0' + (remSec / 10);
        row0[len++] = '0' + (remSec % 10);
      } else {
        row0[len++] = '-';
        row0[len++] = '-';
      }
    }
    row0[len] = '\0';

    /* Row 1: "Active: NORTH" or "YELLOW > NORTH" or "ALL RED  WAIT" */
    if (currentState == STATE_GREEN) {
      len = 0;
      const char *pre = "Active: ";
      while (*pre)
        row1[len++] = *pre++;
      const char *ln = LANE_NAMES[activeLane];
      while (*ln && len < 16)
        row1[len++] = *ln++;
      while (len < 16)
        row1[len++] = ' ';
      row1[len] = '\0';
    } else if (currentState == STATE_YELLOW) {
      len = 0;
      const char *pre = "YELLOW> ";
      while (*pre)
        row1[len++] = *pre++;
      const char *ln = LANE_NAMES[activeLane];
      while (*ln && len < 16)
        row1[len++] = *ln++;
      while (len < 16)
        row1[len++] = ' ';
      row1[len] = '\0';
    } else {
      memcpy(row1, "ALL RED  WAIT   ", 16);
      row1[16] = '\0';
    }
  }

  lcd_set_cursor(0, 0);
  lcd_print_padded(row0, 16);
  lcd_set_cursor(1, 0);
  lcd_print_padded(row1, 16);
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 13: MAIN                                                ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

int main(void) {
  SystemClock_Config();
  GPIO_Init();
  USART2_Init();
  USART3_Init();

  uart2_puts("\r\n================================================\r\n");
  uart2_puts("  STM32 Traffic Controller v4.0\r\n");
  uart2_puts("  4-Way + Pedestrian + Emergency (ESP-NOW)\r\n");
  uart2_puts("  USART2=Debug, USART3=ESP32 Receiver\r\n");
  uart2_puts("================================================\r\n");

  /* Initialize LCD */
  lcd_init();
  lcd_set_cursor(0, 0);
  lcd_print("Traffic Control ");
  lcd_set_cursor(1, 0);
  lcd_print("  Booting...    ");
  delay_ms(1000);

  /* Start traffic lights: all red, then begin cycle */
  all_red();
  delay_ms(500);
  enter_state(STATE_GREEN);

  uart2_puts("[Ready] Traffic controller running.\r\n");
  uart2_puts("[UART3] Waiting for ESP32 on PB11 (RX)...\r\n\r\n");

  lcd_set_cursor(0, 0);
  lcd_print("System Ready!   ");
  lcd_set_cursor(1, 0);
  lcd_print("Lane: NORTH     ");
  delay_ms(1500);

  /* Blink user LED to show alive */
  gpio_write(GPIOC, 13, 0); /* ON (inverted) */

  /* Main loop */
  while (1) {
    scan_pedestrian_buttons();
    traffic_fsm_update();
    lcd_update();

    /* Heartbeat: toggle PC13 every 500ms */
    if ((millis() / 500) % 2)
      gpio_write(GPIOC, 13, 0); /* ON */
    else
      gpio_write(GPIOC, 13, 1); /* OFF */
  }

  return 0; /* Never reached */
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 14: STARTUP (Minimal Vector Table)                      ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

/*
 * If compiling standalone (not in CubeIDE), you need a startup file.
 * The Nucleo board package includes one. If using CubeIDE, paste
 * the code above into the USER CODE sections of main.c.
 *
 * For CubeIDE integration:
 *   - Copy Sections 1-12 into main.c between USER CODE BEGIN/END
 *   - The SystemClock_Config, GPIO_Init etc. should be called
 *     from the CubeIDE-generated main() function
 *   - Skip Section 14 (CubeIDE provides its own startup)
 *
 * Minimum vector table for standalone compilation:
 */

extern uint32_t _estack; /* Defined in linker script */

__attribute__((section(".isr_vector"))) void (*const vectors[])(void) = {
    (void (*)(void))&_estack, /* Initial stack pointer */
    (void (*)(void))main,     /* Reset handler (simplified) */
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0, /* NMI, HardFault, etc. (stubs) */
    0,
    0,
    0,
    0,
    0,
    (void (*)(void))SysTick_Handler,   /* SysTick (position 15) */
    [16 ... 38] = 0,                   /* IRQ 0-22 */
    (void (*)(void))USART3_IRQHandler, /* IRQ 39 = USART3 */
};
