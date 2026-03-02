/**
 * ============================================================================
 *  STM32 LCD Test — Bare-Metal (NUCLEO-F103RB)
 *  Tests the 16x2 I2C LCD (PCF8574 backpack) on PC6(SDA) / PC7(SCL)
 * ============================================================================
 *
 *  PURPOSE:
 *    Minimal test to verify the LCD is working. Cycles through test patterns:
 *      1. "Hello World!" on first boot
 *      2. Scrolling counter every second
 *      3. All characters fill test
 *    Also blinks PC13 (user LED) and prints debug on USART2 (ST-Link VCP).
 *
 *  BUILD:
 *    arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O2 -DSTM32F103xB \
 *      -T STM32F103RBTx_FLASH.ld -nostartfiles \
 *      stm32_lcd_test.c -o lcd_test.elf
 *
 *  WIRING:
 *    LCD SDA  → PC6
 *    LCD SCL  → PC7
 *    LCD VCC  → 5V (from NUCLEO 5V pin)
 *    LCD GND  → GND
 *
 *  LCD I2C ADDRESS:
 *    Default: 0x27 — if LCD is blank, try changing to 0x3F below
 * ============================================================================
 */

#include <stdint.h>
#include <string.h>

/* === Base addresses === */
#define PERIPH_BASE 0x40000000UL
#define APB1_BASE PERIPH_BASE
#define APB2_BASE (PERIPH_BASE + 0x10000UL)
#define AHB_BASE (PERIPH_BASE + 0x20000UL)

#define RCC_BASE (AHB_BASE + 0x1000UL)
#define GPIOA_BASE (APB2_BASE + 0x0800UL)
#define GPIOB_BASE (APB2_BASE + 0x0C00UL)
#define GPIOC_BASE (APB2_BASE + 0x1000UL)
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

typedef struct {
  volatile uint32_t SR, DR, BRR_, CR1, CR2, CR3, GTPR;
} USART_TypeDef;
#define USART2 ((USART_TypeDef *)USART2_BASE)

typedef struct {
  volatile uint32_t EVCR, MAPR, EXTICR[4], RESERVED, MAPR2;
} AFIO_TypeDef;
#define AFIO ((AFIO_TypeDef *)AFIO_BASE)

/* Bit defines */
#define RCC_APB2ENR_IOPAEN (1U << 2)
#define RCC_APB2ENR_IOPBEN (1U << 3)
#define RCC_APB2ENR_IOPCEN (1U << 4)
#define RCC_APB2ENR_AFIOEN (1U << 0)
#define RCC_APB1ENR_USART2EN (1U << 17)

/* ===================== CONFIGURATION ===================== */

/*
 * *** CHANGE THIS IF YOUR LCD SHOWS NOTHING ***
 * Common addresses: 0x27 or 0x3F
 * If 0x27 doesn't work, try 0x3F
 */
#define LCD_I2C_ADDR 0x27

#define LCD_BL 0x08 /* Backlight bit */
#define LCD_EN 0x04 /* Enable bit */
#define LCD_RS 0x01 /* Register Select bit */

/* ===================== GLOBALS ===================== */

static volatile uint32_t msTicks = 0;

void SysTick_Handler(void) { msTicks++; }

static void delay_ms(uint32_t ms) {
  uint32_t start = msTicks;
  while ((msTicks - start) < ms)
    ;
}

static uint32_t millis(void) { return msTicks; }

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

/* ===================== CLOCK SETUP (72 MHz) ===================== */

static void SystemClock_Config(void) {
  RCC->CR |= (1U << 16); /* HSEON */
  while (!(RCC->CR & (1U << 17)))
    ; /* Wait HSE ready */

  *(volatile uint32_t *)0x40022000 |= 0x02; /* Flash 2 wait states */

  RCC->CFGR |= (1U << 16); /* PLLSRC = HSE */
  RCC->CFGR |= (7U << 18); /* PLLMUL = 9 */
  RCC->CFGR |= (4U << 8);  /* APB1 prescaler = /2 */

  RCC->CR |= (1U << 24); /* PLLON */
  while (!(RCC->CR & (1U << 25)))
    ; /* Wait PLL ready */

  RCC->CFGR |= (2U << 0); /* SW = PLL */
  while (((RCC->CFGR >> 2) & 3U) != 2U)
    ; /* Wait SWS = PLL */

  STK_LOAD = 72000 - 1; /* SysTick 1ms */
  STK_VAL = 0;
  STK_CTRL = 0x07;
}

/* ===================== USART2 (Debug Serial) ===================== */

static void USART2_Init(void) {
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  gpio_set_mode(GPIOA, 2, 0x0A);        /* PA2 TX: AF Push-Pull */
  gpio_set_mode(GPIOA, 3, 0x04);        /* PA3 RX: Input Floating */
  USART2->BRR_ = 0x139;                 /* 115200 baud at 36 MHz APB1 */
  USART2->CR1 = (1U << 13) | (1U << 3); /* UE + TE */
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

static void uart_put_hex(uint8_t val) {
  const char hex[] = "0123456789ABCDEF";
  uart_putc(hex[(val >> 4) & 0x0F]);
  uart_putc(hex[val & 0x0F]);
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

/* ===================== BIT-BANGED I2C (PC6=SDA, PC7=SCL) =====================
 */

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

static uint8_t i2c_write_byte(uint8_t data) {
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
  /* Read ACK */
  SDA_HIGH();
  i2c_delay();
  /* Switch SDA to input to read ACK */
  gpio_set_mode(GPIOC, 6, 0x08); /* Input pull-up */
  GPIOC->ODR |= (1U << 6);       /* Pull-up */
  SCL_HIGH();
  i2c_delay();
  uint8_t ack = !((GPIOC->IDR >> 6) & 1U); /* ACK = SDA low */
  SCL_LOW();
  i2c_delay();
  /* Switch SDA back to open-drain output */
  gpio_set_mode(GPIOC, 6, 0x06);
  return ack;
}

static void lcd_i2c_send(uint8_t data) {
  i2c_start();
  i2c_write_byte(LCD_I2C_ADDR << 1);
  i2c_write_byte(data);
  i2c_stop();
}

static void lcd_pulse(uint8_t data) {
  lcd_i2c_send(data | LCD_EN);
  i2c_delay();
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
  delay_ms(50);
  lcd_send_nibble(0x30, 0);
  delay_ms(5);
  lcd_send_nibble(0x30, 0);
  delay_ms(2);
  lcd_send_nibble(0x30, 0);
  delay_ms(2);
  lcd_send_nibble(0x20, 0);
  delay_ms(2);   /* 4-bit mode */
  lcd_cmd(0x28); /* 4-bit, 2 lines, 5x8 */
  delay_ms(1);
  lcd_cmd(0x0C); /* Display on, cursor off */
  delay_ms(1);
  lcd_cmd(0x01); /* Clear display */
  delay_ms(3);
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

static void lcd_clear(void) {
  lcd_cmd(0x01);
  delay_ms(3);
}

/* ===================== I2C SCANNER ===================== */

static uint8_t i2c_scan_address(uint8_t addr) {
  i2c_start();
  uint8_t ack = i2c_write_byte(addr << 1);
  i2c_stop();
  delay_ms(1);
  return ack;
}

static void i2c_scan(void) {
  uart_puts("\r\n=== I2C Address Scanner ===\r\n");
  uart_puts("Scanning addresses 0x01 to 0x7F...\r\n");

  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 128; addr++) {
    if (i2c_scan_address(addr)) {
      uart_puts("  Found device at 0x");
      uart_put_hex(addr);
      uart_puts("\r\n");
      found++;
    }
  }

  if (found == 0) {
    uart_puts("  *** NO DEVICES FOUND! ***\r\n");
    uart_puts("  Check wiring:\r\n");
    uart_puts("    LCD SDA -> PC6\r\n");
    uart_puts("    LCD SCL -> PC7\r\n");
    uart_puts("    LCD VCC -> 5V\r\n");
    uart_puts("    LCD GND -> GND\r\n");
  } else {
    uart_puts("  Found ");
    uart_put_num(found);
    uart_puts(" device(s)\r\n");
  }
  uart_puts("===========================\r\n\r\n");
}

/* ===================== GPIO INIT ===================== */

static void GPIO_Init(void) {
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN |
                  RCC_APB2ENR_AFIOEN;

  /* PC13 = User LED (push-pull output) */
  gpio_set_mode(GPIOC, 13, 0x02);

  /* PC6 = SDA, PC7 = SCL — Open-Drain Output for bit-bang I2C */
  gpio_set_mode(GPIOC, 6, 0x06);
  gpio_set_mode(GPIOC, 7, 0x06);
  gpio_write(GPIOC, 6, 1); /* Idle high */
  gpio_write(GPIOC, 7, 1);
}

/* ===================== MAIN ===================== */

int main(void) {
  SystemClock_Config();
  GPIO_Init();
  USART2_Init();

  uart_puts("\r\n");
  uart_puts("============================================\r\n");
  uart_puts("  STM32 LCD Test v1.0\r\n");
  uart_puts("  LCD: 16x2 I2C (PCF8574) on PC6/PC7\r\n");
  uart_puts("  LCD_I2C_ADDR: 0x");
  uart_put_hex(LCD_I2C_ADDR);
  uart_puts("\r\n");
  uart_puts("============================================\r\n");

  /* ---- Step 1: Scan I2C bus ---- */
  uart_puts("\r\n[Step 1] Scanning I2C bus...\r\n");
  i2c_scan();

  /* ---- Step 2: Initialize LCD ---- */
  uart_puts("[Step 2] Initializing LCD...\r\n");
  lcd_init();
  uart_puts("  LCD init done.\r\n");

  /* ---- Step 3: Test backlight ---- */
  uart_puts("[Step 3] Testing backlight (on/off/on)...\r\n");
  lcd_i2c_send(0x00); /* Backlight OFF */
  delay_ms(500);
  lcd_i2c_send(LCD_BL); /* Backlight ON */
  delay_ms(500);
  uart_puts("  Backlight test done.\r\n");

  /* ---- Step 4: Display "Hello World!" ---- */
  uart_puts("[Step 4] Writing 'Hello World!' to LCD...\r\n");
  lcd_clear();
  lcd_set_cursor(0, 0);
  lcd_print("Hello World!    ");
  lcd_set_cursor(1, 0);
  lcd_print("LCD Test v1.0   ");
  uart_puts("  Text written. You should see it on the LCD now.\r\n");
  delay_ms(3000);

  /* ---- Step 5: Display address info ---- */
  uart_puts("[Step 5] Showing I2C address on LCD...\r\n");
  lcd_clear();
  lcd_set_cursor(0, 0);
  lcd_print("I2C Addr: 0x");
  {
    const char hex[] = "0123456789ABCDEF";
    lcd_data(hex[(LCD_I2C_ADDR >> 4) & 0x0F]);
    lcd_data(hex[LCD_I2C_ADDR & 0x0F]);
  }
  lcd_set_cursor(1, 0);
  lcd_print("SDA=PC6 SCL=PC7 ");
  delay_ms(3000);

  /* ---- Step 6: Running counter ---- */
  uart_puts("[Step 6] Starting counter loop...\r\n");
  uart_puts("  LCD should show a counting number.\r\n");
  uart_puts("  PC13 LED blinks every 500ms.\r\n\r\n");

  uint32_t counter = 0;
  uint32_t lastUpdate = 0;
  uint32_t lastBlink = 0;
  uint8_t ledState = 0;

  while (1) {
    uint32_t now = millis();

    /* Update counter on LCD every 1 second */
    if ((now - lastUpdate) >= 1000) {
      lastUpdate = now;
      counter++;

      /* Row 0: counter */
      lcd_set_cursor(0, 0);
      lcd_print("Count: ");
      {
        char buf[10];
        int i = 0;
        uint32_t n = counter;
        if (n == 0) {
          buf[i++] = '0';
        } else {
          while (n > 0) {
            buf[i++] = '0' + (n % 10);
            n /= 10;
          }
        }
        /* Pad to 8 chars */
        while (i < 8)
          buf[i++] = ' ';
        /* Print reversed */
        for (int j = 7; j >= 0; j--)
          lcd_data(buf[j]);
      }

      /* Row 1: uptime */
      lcd_set_cursor(1, 0);
      {
        uint32_t sec = counter;
        uint32_t min = sec / 60;
        sec %= 60;
        lcd_print("Up: ");
        lcd_data('0' + (min / 10));
        lcd_data('0' + (min % 10));
        lcd_data(':');
        lcd_data('0' + (sec / 10));
        lcd_data('0' + (sec % 10));
        lcd_print("       ");
      }

      /* Debug serial */
      uart_puts("[LCD] Counter = ");
      uart_put_num(counter);
      uart_puts("\r\n");
    }

    /* Blink PC13 every 500ms */
    if ((now - lastBlink) >= 500) {
      lastBlink = now;
      ledState = !ledState;
      gpio_write(GPIOC, 13, ledState ? 0 : 1); /* Inverted on NUCLEO */
    }
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
