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
 *    Ped LEDs:     PB9/PB4 (N R/G) | PC2/PB12 (E R/G) |
 *                  PC0/PC1 (S R/G) | PC4/PC5 (W R/G)
 *    IR Sensors:   PC11(N), PC12(E), PD2(S), PA12(W) — GPIO_Input Pull-Up
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

/* Interrupt control (Cortex-M3 CPSID/CPSIE) */
#define __disable_irq() __asm volatile("cpsid i" ::: "memory")
#define __enable_irq() __asm volatile("cpsie i" ::: "memory")

/* IR Sensor pins (Input Pull-Up, active-low) */
#define IR_NORTH_PORT GPIOC
#define IR_NORTH_PIN 11
#define IR_EAST_PORT GPIOC
#define IR_EAST_PIN 12
#define IR_SOUTH_PORT GPIOD
#define IR_SOUTH_PIN 2
#define IR_WEST_PORT GPIOA
#define IR_WEST_PIN 12
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
#define EMERGENCY_CLEAR_MS                                                     \
  2000                           /* time to clear current lane before override \
                                  */
#define EMERGENCY_GREEN_MS 15000 /* emergency lane green duration */
#define IR_EXTEND_MAX_MS 5000    /* max IR extension beyond GREEN_TIME_MS */
#define EMERGENCY_RED_GAP_MS 500 /* all-red gap after emergency yellow */

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
static uint8_t stateFirstRun =
    1; /* Flag to ensure LEDs are set on state entry */

/* Pedestrian */
static uint8_t pedRequested[NUM_LANES] = {0, 0, 0, 0};
static uint8_t pedActiveForLane = 0;
static uint8_t btnPrevState[NUM_LANES] = {1, 1, 1,
                                          1}; /* 1 = released (pull-up) */
static uint8_t pedPendingAfterYellow = 0; /* route ped request through yellow */
static uint8_t savedLaneBeforeEmergency = 0; /* resume cycle after emergency */

/* Emergency */
static volatile uint8_t emergencyRequested = 0;
static volatile uint8_t emergencyLane = 0;
static uint8_t emergencyActive = 0;
static uint8_t emergencyRedGapDone =
    0; /* ensures all_red() fires once in clear gap */

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

/* Safe on Cortex-M3: 32-bit aligned load is atomic (single LDR instruction).
   Would need __disable_irq() on architectures without this guarantee. */
static uint32_t millis(void) { return msTicks; }

/* GPIO configuration helper
   WARNING: This RMW is NOT interrupt-safe. If any ISR modifies the same
   GPIO port's CRL/CRH, wrap calls in __disable_irq/__enable_irq. */
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

/* Forward declaration — defined in Section 14 (startup). Called here if
   HSE/PLL fails, to enter safe all-red fault state. */
void Default_Handler(void);

static void SystemClock_Config(void) {
  /* Enable HSE (8 MHz on NUCLEO from ST-Link) */
  RCC->CR |= (1U << 16); /* HSEON */
  {
    uint32_t timeout = 500000U; /* ~50ms at default HSI 8MHz */
    while (!(RCC->CR & (1U << 17)) && --timeout)
      ;
    if (!timeout) {
      /* HSE failed — UART baud rates depend on 36 MHz APB1 clock.
         Limping at 8 MHz would garble all ESP32 comms. Halt safely. */
      Default_Handler();
    }
  }

  /* Flash ACR: clear LATENCY[2:0] then set 2 wait states for 72 MHz */
  volatile uint32_t *FLASH_ACR = (volatile uint32_t *)0x40022000;
  *FLASH_ACR = (*FLASH_ACR & ~0x07U) | 0x02U;

  /* PLL: HSE / 1 * 9 = 72 MHz */
  RCC->CFGR |= (1U << 16); /* PLLSRC = HSE */
  RCC->CFGR |= (7U << 18); /* PLLMUL = 9 */
  RCC->CFGR |= (4U << 8);  /* APB1 prescaler = /2 (36 MHz max) */

  /* Enable PLL */
  RCC->CR |= (1U << 24); /* PLLON */
  {
    uint32_t timeout = 500000U;
    while (!(RCC->CR & (1U << 25)) && --timeout)
      ;
    if (!timeout) {
      /* PLL failed — UART baud rates will be wrong. Halt safely. */
      Default_Handler();
    }
  }

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

  /* Disable JTAG to free PB4 (keep SWD) — PB3 is no longer used */
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
  /* East Ped: PC2(R), PB12(G) */
  gpio_set_mode(GPIOC, 2, 0x02);
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

  /* --- User LED PA5 (Push-Pull Output, active-high on NUCLEO-F103RB) --- */
  /* NOTE: PC13 is the User Button on Nucleo — setting it as push-pull output
     risks a VDD-to-GND short if the button is pressed. PA5 = LD2 (green). */
  gpio_set_mode(GPIOA, 5, 0x02);

  /* --- IR Sensors (Input Pull-Up, active-low) --- */
  /* PC11(N), PC12(E), PD2(S), PA12(W) */
  gpio_set_mode(IR_NORTH_PORT, IR_NORTH_PIN, 0x08);
  gpio_set_mode(IR_EAST_PORT, IR_EAST_PIN, 0x08);
  gpio_set_mode(IR_SOUTH_PORT, IR_SOUTH_PIN, 0x08);
  gpio_set_mode(IR_WEST_PORT, IR_WEST_PIN, 0x08);
  /* Enable pull-ups for all IR sensor pins */
  IR_NORTH_PORT->ODR |= (1U << IR_NORTH_PIN);
  IR_EAST_PORT->ODR |= (1U << IR_EAST_PIN);
  IR_SOUTH_PORT->ODR |= (1U << IR_SOUTH_PIN);
  IR_WEST_PORT->ODR |= (1U << IR_WEST_PIN);

  /* --- LCD I2C: PC6(SDA), PC7(SCL) — Open-Drain Output for bit-bang --- */
  gpio_set_mode(GPIOC, 6, 0x06); /* Output 2MHz, Open-Drain */
  gpio_set_mode(GPIOC, 7, 0x06);
  gpio_write(GPIOC, 6, 1); /* Idle high */
  gpio_write(GPIOC, 7, 1);
}

/* Read IR sensor — returns 1 if vehicle detected (active-low: 0 = triggered) */
static uint8_t ir_read(uint8_t lane) {
  switch (lane) {
  case LANE_NORTH:
    return !gpio_read(IR_NORTH_PORT, IR_NORTH_PIN);
  case LANE_EAST:
    return !gpio_read(IR_EAST_PORT, IR_EAST_PIN);
  case LANE_SOUTH:
    return !gpio_read(IR_SOUTH_PORT, IR_SOUTH_PIN);
  case LANE_WEST:
    return !gpio_read(IR_WEST_PORT, IR_WEST_PIN);
  default:
    return 0;
  }
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
  /* Clear error flags (ORE/NE/FE/PE) FIRST to prevent ISR re-entry loop.
     On STM32F103, reading SR then DR clears ORE (RM0008 §27.6.1). */
  uint32_t sr = USART3->SR;
  if (sr & 0x0FU) {   /* ORE|NE|FE|PE bits [3:0] */
    (void)USART3->DR; /* Read DR to clear error flags */
    return;           /* Discard corrupted byte */
  }

  if (sr & USART_SR_RXNE) {
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
      /* If buffer full, silently drop chars until next \n flushes */
    }
  }
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 8: BIT-BANGED I2C + LCD DRIVER (PC6=SDA, PC7=SCL)     ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

/* Microsecond-ish delay (approximate at 72 MHz)
   Uses NOP instructions for deterministic timing regardless of
   compiler optimization level or Flash wait-state configuration. */
static void i2c_delay(void) {
  /* ~5µs at 72 MHz ≈ 360 cycles. Each NOP = 1 cycle on Cortex-M3.
     Loop overhead adds ~4 cycles/iteration, so 36 iterations ≈ 180 cycles.
     Two calls per I2C half-clock gives adequate setup/hold time. */
  for (volatile uint32_t n = 36; n > 0; n--) {
    __asm volatile("nop");
  }
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
    if (data & (1U << i)) /* MISRA-C 12.2: unsigned shift operand */
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
  i2c_delay(); /* HD44780 needs EN pulse width >= 450ns */
  i2c_delay();
  lcd_i2c_send(data & ~LCD_EN);
  i2c_delay();
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
  delay_ms(50); /* Power-on delay (HD44780 needs >40ms) */
  lcd_send_nibble(0x30, 0);
  delay_ms(5); /* Wait >4.1ms */
  lcd_send_nibble(0x30, 0);
  delay_ms(2); /* Wait >100us */
  lcd_send_nibble(0x30, 0);
  delay_ms(2);
  lcd_send_nibble(0x20, 0); /* Switch to 4-bit mode */
  delay_ms(2);              /* Wait for mode switch to complete */
  lcd_cmd(0x28);            /* 4-bit, 2 lines, 5x8 */
  delay_ms(1);
  lcd_cmd(0x0C); /* Display on, cursor off */
  delay_ms(1);
  lcd_cmd(0x01); /* Clear */
  delay_ms(3);   /* Clear command needs >1.52ms */
  lcd_cmd(0x06); /* Entry mode: increment */
  delay_ms(1);
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
    {GPIOC, 2, GPIOB, 12, GPIOB, 6}, /* East:  PC2(R), PB12(G), PB6(Btn) */
    {GPIOC, 0, GPIOC, 1, GPIOB, 7},  /* South: PC0(R), PC1(G), PB7(Btn) */
    {GPIOC, 4, GPIOC, 5, GPIOB, 8},  /* West:  PC4(R), PC5(G), PB8(Btn) */
};

/* Set a single traffic light color */
static void tl_set(uint8_t lane, uint8_t r, uint8_t y, uint8_t g) {
  if (lane >= NUM_LANES)
    return; /* MISRA-C 18.1: bounds guard */
  gpio_write(TL[lane].rPort, TL[lane].rPin, r);
  gpio_write(TL[lane].yPort, TL[lane].yPin, y);
  gpio_write(TL[lane].gPort, TL[lane].gPin, g);
}

/* Set pedestrian LEDs */
static void ped_set(uint8_t lane, uint8_t red, uint8_t green) {
  if (lane >= NUM_LANES)
    return; /* MISRA-C 18.1: bounds guard */
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
    uint8_t btnNow = gpio_read(PL[i].btnPort, PL[i].btnPin);
    if (!btnNow && btnPrevState[i]) { /* Falling edge (1->0): button press */
      if (!pedRequested[i]) {
        pedRequested[i] = 1;
        uart2_puts("[PED] Button pressed: ");
        uart2_puts(LANE_NAMES[i]);
        uart2_puts("\r\n");
      }
    }
    btnPrevState[i] = btnNow;
  }
}

/* ╔═══════════════════════════════════════════════════════════════════╗
 * ║  SECTION 11: TRAFFIC STATE MACHINE                              ║
 * ╚═══════════════════════════════════════════════════════════════════╝ */

static void enter_state(TrafficState_t state) {
  currentState = state;
  stateStartMs = millis();
  stateFirstRun = 1; /* LED/output update on first FSM tick */
}

static void next_lane(void) { activeLane = (activeLane + 1) % NUM_LANES; }

static void traffic_fsm_update(void) {
  /* NOTE: unsigned subtraction handles uint32_t rollover correctly (~49.7
     days). Do NOT change these to signed types. */
  uint32_t elapsed = millis() - stateStartMs;

  /* Atomically read + clear emergency request with PRIMASK preservation.
     Saves/restores interrupt state for safe nested critical sections. */
  uint32_t primask;
  __asm volatile("mrs %0, primask" : "=r"(primask)); /* save IRQ state */
  __disable_irq();
  uint8_t emReq = emergencyRequested;
  uint8_t emLane = emergencyLane;
  emergencyRequested = 0;
  if (primask == 0U) {
    __enable_irq(); /* only re-enable if IRQs were enabled before */
  }

  /* Check emergency request (highest priority) */
  if (emReq) {
    if (emergencyActive && currentState == STATE_EMERGENCY_GREEN &&
        activeLane == emLane) {
      /* Same lane already green for emergency — extend timer */
      stateStartMs = millis();
      uart2_puts("[EMG] Extended emergency on lane ");
      uart2_print_num(emLane);
      uart2_puts("\r\n");
      return;
    }

    /* Only save cycle position on FIRST emergency entry — preserves
       original lane if back-to-back emergencies arrive. */
    if (!emergencyActive) {
      savedLaneBeforeEmergency = activeLane;
    }
    emergencyActive = 1;
    emergencyLane = emLane; /* update global for LCD */
    uart2_puts("\r\n*** EMERGENCY VEHICLE! Lane ");
    uart2_print_num(emLane);
    uart2_puts(" (");
    uart2_puts(LANE_NAMES[emLane]);
    uart2_puts(") ***\r\n");

    /* If we're already on the emergency lane and green, just hold it */
    if (activeLane == emLane && currentState == STATE_GREEN) {
      enter_state(STATE_EMERGENCY_GREEN);
      set_green(emLane);
      return;
    }

    /* Otherwise, quickly clear current traffic */
    /* If interrupting a pedestrian phase, force DON'T WALK */
    if (currentState == STATE_PED_WALK || currentState == STATE_PED_FLASH) {
      ped_set(pedActiveForLane, 1, 0); /* DON'T WALK */
      pedPendingAfterYellow = 0;       /* cancel any pending ped */
    }

    /* Determine if vehicles are already stopped (non-GREEN state).
       If so, skip the yellow clearance — showing yellow to stopped cars
       is dangerous (signals them to prepare to move as ambulance arrives). */
    uint8_t wasGreen =
        (currentState == STATE_GREEN || currentState == STATE_EMERGENCY_GREEN);
    enter_state(STATE_EMERGENCY_CLEAR);
    emergencyRedGapDone = 0;
    if (!wasGreen) {
      stateFirstRun = 0;       /* prevent set_yellow() in stateFirstRun */
      all_red();               /* assert red immediately (executed once) */
      emergencyRedGapDone = 1; /* skip redundant all_red in gap phase */
      stateStartMs -= EMERGENCY_CLEAR_MS; /* fast-forward to red gap phase */
    }
    return;
  }

  switch (currentState) {

  case STATE_GREEN:
    if (stateFirstRun) {
      stateFirstRun = 0;
      set_green(activeLane);
      uart2_puts("[TL] GREEN -> ");
      uart2_puts(LANE_NAMES[activeLane]);
      uart2_puts("\r\n");
    }

    if (elapsed >= GREEN_TIME_MS) {
      /* Pedestrian check takes priority over IR extension */
      if (pedRequested[activeLane]) {
        pedRequested[activeLane] = 0;
        pedActiveForLane = activeLane;
        pedPendingAfterYellow = 1;
        set_yellow(activeLane);
        enter_state(STATE_YELLOW);
      } else if (ir_read(activeLane) &&
                 elapsed < GREEN_TIME_MS + IR_EXTEND_MAX_MS) {
        break; /* extend green for vehicles */
      } else {
        set_yellow(activeLane);
        enter_state(STATE_YELLOW);
      }
    }
    break;

  case STATE_PED_WALK:
    if (stateFirstRun) {
      stateFirstRun = 0;
      ped_set(pedActiveForLane, 0, 1); /* WALK */
      uart2_puts("[PED] Walk phase -> ");
      uart2_puts(LANE_NAMES[pedActiveForLane]);
      uart2_puts("\r\n");
    }
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
      next_lane();
      enter_state(STATE_GREEN);
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
      if (pedPendingAfterYellow) {
        pedPendingAfterYellow = 0;
        enter_state(STATE_PED_WALK);
      } else {
        next_lane();
        enter_state(STATE_GREEN);
      }
    }
    break;

  case STATE_EMERGENCY_CLEAR:
    if (stateFirstRun) {
      stateFirstRun = 0;
      set_yellow(activeLane);
      uart2_puts("[EMG] Clearing for emergency...\r\n");
    }
    if (elapsed >= EMERGENCY_CLEAR_MS &&
        elapsed < EMERGENCY_CLEAR_MS + EMERGENCY_RED_GAP_MS) {
      if (!emergencyRedGapDone) {
        emergencyRedGapDone = 1;
        all_red(); /* Execute exactly once on entering the red gap */
      }
    } else if (elapsed >= EMERGENCY_CLEAR_MS + EMERGENCY_RED_GAP_MS) {
      emergencyRedGapDone = 0;
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
      /* Proper clearance: emergency lane gets YELLOW → ALL_RED before
         cross-traffic resumes. Set activeLane to emergency lane so
         set_yellow/all_red act on it, then ALL_RED will restore
         the saved lane via next_lane() logic. */
      /* activeLane is already emergencyLane from EMERGENCY_CLEAR entry */
      set_yellow(activeLane);
      /* Pre-set activeLane so after YELLOW→ALL_RED, STATE_GREEN
         resumes on the saved lane (not next after emergency lane) */
      activeLane = savedLaneBeforeEmergency;
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

    /* Row 0: "!! EMERGENCY !!" (15 chars, padded to 16 by lcd_print_padded) */
    memcpy(row0, "!! EMERGENCY !!", 16);
    row0[16] = '\0';

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
      pedElapsed += PED_WALK_TIME_MS;
    uint32_t remMs = (pedElapsed < totalMs) ? totalMs - pedElapsed : 0;
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
      dur = ir_read(activeLane) ? GREEN_TIME_MS + IR_EXTEND_MAX_MS
                                : GREEN_TIME_MS;
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
          row0[len++] = ' ';
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

  /* Initialize button prev-state from hardware to avoid false edge on boot */
  for (int i = 0; i < NUM_LANES; i++)
    btnPrevState[i] = gpio_read(PL[i].btnPort, PL[i].btnPin);

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
  gpio_write(GPIOA, 5, 1); /* ON (active-high on Nucleo LD2) */

  /* Main loop */
  while (1) {
    scan_pedestrian_buttons();
    traffic_fsm_update();
    lcd_update();

    /* Heartbeat: toggle PA5 (LD2) every 500ms */
    if ((millis() / 500) % 2)
      gpio_write(GPIOA, 5, 1); /* ON */
    else
      gpio_write(GPIOA, 5, 0); /* OFF */
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
 * WARNING: This minimal vector table does NOT include a startup routine
 * that copies .data from Flash to RAM or zeroes .bss. When using
 * standalone compilation, your linker script + startup must handle this,
 * or all initialized globals (stateFirstRun, btnPrevState, etc.) may be
 * wrong on cold boot. CubeIDE provides this automatically.
 *
 * Minimum vector table for standalone compilation:
 */

/* CubeIDE defines USE_HAL_DRIVER and provides its own startup file
   (startup_stm32f103rbtx.s) with Default_Handler, Reset_Handler, and
   the full vector table. Skip this section when building inside CubeIDE
   to avoid linker "multiple definition" errors. */
#ifndef USE_HAL_DRIVER

extern uint32_t _estack; /* Defined in linker script */
extern uint32_t _sidata; /* Start of .data init values in Flash */
extern uint32_t _sdata;  /* Start of .data section in RAM */
extern uint32_t _edata;  /* End of .data section in RAM */
extern uint32_t _sbss;   /* Start of .bss section in RAM */
extern uint32_t _ebss;   /* End of .bss section in RAM */

/* Default fault handler — fail-safe: set all lights RED, signal fault */
void Default_Handler(void) {
  /* Force all lanes RED via direct register writes (safe even if RAM is
   * corrupt) */
  /* North: PA0=RED ON */
  GPIOA->BSRR = (1UL << 0);
  /* East: PA9=RED ON */
  GPIOA->BSRR = (1UL << 9);
  /* South: PB13=RED ON */
  GPIOB->BSRR = (1UL << 13);
  /* West: PC8=RED ON */
  GPIOC->BSRR = (1UL << 8);
  /* Fast-blink PA5 (LD2) to indicate fault */
  while (1) {
    GPIOA->ODR ^= (1UL << 5);
    for (volatile uint32_t i = 0; i < 200000; i++)
      ;
  }
}

/* Proper Reset Handler: initializes .data and .bss before calling main().
   Without this, initialized globals (stateFirstRun=1, btnPrevState={1,1,1,1})
   contain unpredictable RAM garbage on cold boot. The standard CRT startup
   (crt0) normally does this, but our minimal vector table bypasses it. */
void Reset_Handler(void) {
  /* Copy .data section from Flash to RAM */
  uint32_t *src = &_sidata;
  uint32_t *dst = &_sdata;
  while (dst < &_edata) {
    *dst++ = *src++;
  }

  /* Zero .bss section */
  dst = &_sbss;
  while (dst < &_ebss) {
    *dst++ = 0;
  }

  /* Call main */
  main();

  /* If main() ever returns, enter fault handler */
  Default_Handler();
}

__attribute__((section(".isr_vector"))) void (*const vectors[])(void) = {
    (void (*)(void))&_estack, /* Initial stack pointer */
    Reset_Handler,            /* Reset handler (proper CRT init) */
    Default_Handler,          /* NMI */
    Default_Handler,          /* HardFault */
    Default_Handler,          /* MemManage */
    Default_Handler,          /* BusFault */
    Default_Handler,          /* UsageFault */
    0,
    0,
    0,
    0, /* Reserved */
    0, /* SVCall */
    0,
    0,                                 /* Reserved */
    0,                                 /* PendSV */
    (void (*)(void))SysTick_Handler,   /* SysTick (position 15) */
    [16 ... 54] = Default_Handler,     /* All unused IRQs → safe trap */
    (void (*)(void))USART3_IRQHandler, /* Index 55 = IRQ39 = USART3 */
};

#else /* USE_HAL_DRIVER defined (CubeIDE build) */

/* Override CubeIDE's default HardFault handler with our safety handler.
   CubeIDE's startup defines these as __weak, so our strong version wins. */
void HardFault_Handler(void) {
  /* Force all lanes RED */
  GPIOA->BSRR = (1UL << 0);  /* North RED */
  GPIOA->BSRR = (1UL << 9);  /* East RED */
  GPIOB->BSRR = (1UL << 13); /* South RED */
  GPIOC->BSRR = (1UL << 8);  /* West RED */
  /* Fast-blink PA5 (LD2) to indicate fault */
  while (1) {
    GPIOA->ODR ^= (1UL << 5);
    for (volatile uint32_t i = 0; i < 200000; i++)
      ;
  }
}

#endif /* USE_HAL_DRIVER */
