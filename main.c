//*****************************************************************************
//
// Anti-Theft Backpack Guardian (CC3200)
//
// Base:    Teni's file (OLED menus, AWS/TLS, IR decoder, state machine)
// Sensors: Our working code (GPS/UART1, BH1750 light, BMA222 accel, buzzer)
//
// Pin assignments:
//   PIN_01 / PIN_02  : I2C SCL/SDA  -> BMA222 accel (0x18) + BH1750 (0x23)
//   PIN_15 / GPIOA2 0x40 : Buzzer (active-LOW)
//   PIN_18 / GPIOA3 0x10 : IR receiver
//   PIN_55 / PIN_57  : UART0 TX/RX  (debug, 115200)
//   PIN_58 / PIN_59  : UART1 TX/RX  (GPS, 9600)
//
//*****************************************************************************

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

// SimpleLink
#include "simplelink.h"

// Driverlib
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"
#include "spi.h"
#include "pin.h"
#include "systick.h"

// Common interface
#include "pinmux.h"
#include "gpio_if.h"
#include "common.h"
#include "uart_if.h"
#include "i2c_if.h"

// OLED
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"

// Network (Teni's working WiFi/TLS helpers)
#include "utils/network_utils.h"

//*****************************************************************************
// AWS / WiFi credentials  -- UPDATE THESE
//*****************************************************************************
#define SERVER_NAME     "a1yd6cishud0ro-ats.iot.us-east-1.amazonaws.com"
#define GOOGLE_DST_PORT  8443

#define POSTHEADER "POST /things/MahiandTeni_CC3200/shadow HTTP/1.1\r\n"
#define HOSTHEADER "Host: a1yd6cishud0ro-ats.iot.us-east-1.amazonaws.com\r\n"
#define CHEADER    "Connection: Keep-Alive\r\n"
#define CTHEADER   "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1  "Content-Length: "
#define CLHEADER2  "\r\n\r\n"

// TLS date/time for certificate validation
#define DATE    7
#define MONTH   3
#define YEAR    2026
#define HOUR    2
#define MINUTE  49
#define SECOND  0

//*****************************************************************************
// Vector table (needed for SimpleLink ISR registration)
//*****************************************************************************
#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
// OLED / SPI
//*****************************************************************************
#define SPI_BITRATE   100000

// Colors
#define BLACK    0x0000
#define BLUE     0x001F
#define GREEN    0x07E0
#define CYAN     0x07FF
#define RED      0xF800
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF
#define CLOUD    0x77F7

#define SCREEN_HEIGHT 128
#define SCREEN_WIDTH  128

//*****************************************************************************
// IR decoder -- Teni's working decoder (TV remote code 1017)
// TIMERA0 free-running up-counter @ 80MHz, GPIO GPIOA3/PIN_18
//*****************************************************************************
#define TIMER_TICKS_PER_SEC       80000000UL
#define MULTITAP_TIMEOUT_MS       800UL
#define MULTITAP_TIMEOUT_TICKS    ((TIMER_TICKS_PER_SEC / 1000UL) * MULTITAP_TIMEOUT_MS)
#define MAX_MSG_LEN               6

#define MEASURED_DATA_TICKS           ((uint32_t)(0.070 * TIMER_TICKS_PER_SEC))
#define MEASURED_NEW_DATA_WAIT_TICKS  ((uint32_t)(0.033 * TIMER_TICKS_PER_SEC))
#define MEASURED_0_TICKS_LOWER        60000
#define MEASURED_0_TICKS_UPPER        68000
#define MEASURED_1_TICKS_LOWER        190000
#define MEASURED_1_TICKS_UPPER        200000
#define MEASURED_OUTER_TICKS_LOWER    70000
#define MEASURED_OUTER_TICKS_UPPER    74000

#define IR_GPIO_PORT_BASE   GPIOA3_BASE
#define IR_GPIO_PIN         0x10

//*****************************************************************************
// Buzzer -- PIN_15 / GPIOA2 bit 0x40, active-LOW module
//*****************************************************************************
#define BUZZER_PORT               GPIOA2_BASE
#define BUZZER_PIN                0x40
#define BUZZER_TRIGGER_ACTIVE_LOW 1
#define BUZZER_FORCE_OFF          0

//*****************************************************************************
// Accelerometer (BMA222 at I2C 0x18)
//*****************************************************************************
#define ACCEL_I2C_ADDR        0x18
#define ACCEL_REG_X           0x05
#define ACCEL_REG_Y           0x03
#define MOTION_ABS_THRESHOLD  7
#define MOTION_DELTA_THRESHOLD 4
#define MOTION_HITS_REQUIRED  4
#define MOTION_SAMPLE_MS      200

//*****************************************************************************
// BH1750 light sensor (I2C 0x23, ADDR pin floating)
// Shares I2C bus with accelerometer -- no conflict
//*****************************************************************************
#define BH1750_I2C_ADDR           0x23
#define BH1750_CMD_POWER_ON       0x01
#define BH1750_CMD_RESET          0x07
#define BH1750_CMD_CONT_HRES      0x10
#define LIGHT_LUX_DELTA_THRESHOLD 50
#define LIGHT_HITS_REQUIRED       3

//*****************************************************************************
// GPS (UART1) -- PIN_58=TX->GPS_RXD, PIN_59=RX<-GPS_TXD, 9600 baud
//*****************************************************************************
#define GPS_UART_BAUD           9600
#define GPS_CONNECT_TIMEOUT_MS  180000
#define GPS_COORD_PERIOD_MS     5000
#define GPS_DIAG_ECHO_LINES     8

//*****************************************************************************
// SysTick -- 40ms tick for sensor timing
//*****************************************************************************
#define SYSTICK_RELOAD_VAL  3200000UL   // ~40ms @ 80MHz

//*****************************************************************************
// Teni's IR key / state enums -- kept exactly as-is
//*****************************************************************************
typedef enum {
  KEY_NONE = 0,
  KEY_0, KEY_1, KEY_2, KEY_3, KEY_4,
  KEY_5, KEY_6, KEY_7, KEY_8, KEY_9,
  KEY_ENTER, KEY_DELETE,
  KEY_VOLUP, KEY_VOLDOWN,
  KEY_PGUP, KEY_PGDOWN
} IRKey;

typedef enum {
  None = 0,
  Disarm,
  Arm,
  Alert,
  Reset,
  Settings,
  PasswordSet
} LockState;

//*****************************************************************************
// Globals
//*****************************************************************************

// IR decoder (volatile - written in ISR)
static volatile uint32_t g_ulIRData      = 0;
static volatile uint32_t g_ulPreviousTick = 0;
static volatile uint32_t g_ulFirstTick   = 0;
static volatile uint32_t g_ulEndTick     = 0;
static volatile uint32_t g_ulDelta       = 0;
static volatile int      g_iEdgeType     = 0;
static volatile int      g_bDataReady    = 0;
static volatile int      g_bStartDetected = 0;
static volatile int      g_bNewDataWord  = 1;
static volatile int      g_bDataCooldown = 0;
static volatile int      g_bTotalEdges   = 0;

// Multi-tap compose buffer (Teni's feature)
static char     g_composeBuf[MAX_MSG_LEN + 1] = {0};
static uint32_t g_composeLen  = 0;

static volatile uint32_t g_incomingLen = 0;
char            g_password[MAX_MSG_LEN + 1] = {0};
char            g_passwordPrompt[MAX_MSG_LEN + 1] = {0};
static volatile bool g_bDisplayRefreshPending = false;
static volatile bool updated      = false;
static volatile bool passwordSet  = false;
static bool hasPasswordSet = false;
volatile uint32_t lastPress = 0;

// State machine
static LockState currentState = None;
static LockState lastState    = None;
static IRKey     g_lastTapKey  = KEY_NONE;
static uint32_t  g_lastTapTick = 0;
static uint32_t  g_lastTapCycle = 0;
volatile int     currentOption  = 0;
bool             connected      = false;
bool             checkPassword = false;

// SysTick 40ms counter (used by sensor timing)
static volatile uint32_t g_tick40ms = 0;

// Accelerometer / motion
static bool     g_i2c_available   = false;
static int      g_prev_x          = 0;
static int      g_prev_y          = 0;
static uint8_t  g_motion_hits     = 0;
static uint32_t g_motion_next_sample = 0;

// BH1750 light sensor
static bool     g_bh1750_ready       = false;
static uint16_t g_light_baseline_lux = 0;
static uint8_t  g_light_hits         = 0;
static bool     g_last_trigger_light = false;
static bool     g_alert_posted       = false;  // true once http_post fired for this alert

// GPS
typedef struct {
  bool  valid;
  float lat;
  float lon;
} gps_fix_t;

static gps_fix_t g_fix               = { false, 0.0f, 0.0f };
static bool      g_gps_lock_reported = false;
static bool      g_gps_fail_reported = false;
static uint32_t  g_gps_connect_deadline = 0;
static bool      g_gps_rx_seen       = false;
static uint32_t  g_gps_diag_lines    = 0;
static uint32_t  g_gps_coord_next    = 0;

//*****************************************************************************
// SysTick handler
//*****************************************************************************
static void AppSysTickHandler(void)
{
  g_tick40ms++;
}

static void SysTickInitApp(void)
{
  MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);
  MAP_SysTickIntRegister(AppSysTickHandler);
  MAP_SysTickIntEnable();
  MAP_SysTickEnable();
}

static uint32_t ms_to_ticks40(uint32_t ms)
{
  return (ms + 39) / 40;
}

//*****************************************************************************
// Buzzer -- our working code
//*****************************************************************************
static void buzzer_on(void)
{
#if BUZZER_FORCE_OFF
  MAP_GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, BUZZER_PIN);
  return;
#endif
#if BUZZER_TRIGGER_ACTIVE_LOW
  MAP_GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, 0);
#else
  MAP_GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, BUZZER_PIN);
#endif
}

static void buzzer_off(void)
{
#if BUZZER_TRIGGER_ACTIVE_LOW
  MAP_GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, BUZZER_PIN);
#else
  MAP_GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, 0);
#endif
}

//*****************************************************************************
// I2C pin config -- our working code
//*****************************************************************************
static void ConfigureI2CPins(void)
{
  PinTypeI2C(PIN_01, PIN_MODE_1); // SCL
  PinTypeI2C(PIN_02, PIN_MODE_1); // SDA
}

//*****************************************************************************
// Accelerometer (BMA222 at 0x18) -- our working code
//*****************************************************************************
static int accel_read_i8(uint8_t reg, int *out)
{
  uint8_t u8 = 0;
  if (!g_i2c_available) return -1;
  if (I2C_IF_Write(ACCEL_I2C_ADDR, &reg, 1, 0) != SUCCESS) return -1;
  if (I2C_IF_Read(ACCEL_I2C_ADDR,  &u8,  1)    != SUCCESS) return -1;
  *out = (int)((int8_t)u8);
  return SUCCESS;
}

static bool motion_detect_sample(void)
{
  int x = 0, y = 0, ax, ay, dx, dy;
  bool moved = false;

  if (accel_read_i8(ACCEL_REG_X, &x) != SUCCESS) return false;
  if (accel_read_i8(ACCEL_REG_Y, &y) != SUCCESS) return false;

  ax = (x < 0) ? -x : x;
  ay = (y < 0) ? -y : y;
  dx = x - g_prev_x; if (dx < 0) dx = -dx;
  dy = y - g_prev_y; if (dy < 0) dy = -dy;
  g_prev_x = x;
  g_prev_y = y;

  if (ax >= MOTION_ABS_THRESHOLD  || ay >= MOTION_ABS_THRESHOLD)  moved = true;
  if (dx >= MOTION_DELTA_THRESHOLD || dy >= MOTION_DELTA_THRESHOLD) moved = true;

  if (moved) { if (g_motion_hits < 255) g_motion_hits++; }
  else       { if (g_motion_hits > 0)   g_motion_hits--; }

  return (g_motion_hits >= MOTION_HITS_REQUIRED);
}

//*****************************************************************************
// BH1750 light sensor -- our working code
//*****************************************************************************
static bool bh1750_write_cmd(uint8_t cmd)
{
  if (!g_i2c_available) return false;
  return (I2C_IF_Write(BH1750_I2C_ADDR, &cmd, 1, 1) == SUCCESS);
}

static bool bh1750_init(void)
{
  if (!bh1750_write_cmd(BH1750_CMD_POWER_ON)) return false;
  if (!bh1750_write_cmd(BH1750_CMD_RESET))    return false;
  if (!bh1750_write_cmd(BH1750_CMD_CONT_HRES)) return false;
  g_bh1750_ready = true;
  Report("Light: BH1750 init OK at 0x23\r\n");
  return true;
}

static uint16_t bh1750_read_lux(void)
{
  uint8_t buf[2];
  if (!g_bh1750_ready || !g_i2c_available) return 0;
  if (I2C_IF_Read(BH1750_I2C_ADDR, buf, 2) != SUCCESS) return 0;
  uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
  return (uint16_t)((uint32_t)raw * 10 / 12); // divide by 1.2
}

static void light_sensor_arm_baseline(void)
{
  g_light_baseline_lux = bh1750_read_lux();
  g_light_hits = 0;
  Report("Light baseline lux: %u\r\n", (unsigned int)g_light_baseline_lux);
}

static bool light_sensor_open_sample(void)
{
  uint16_t now   = bh1750_read_lux();
  uint16_t delta = (now > g_light_baseline_lux) ? (now - g_light_baseline_lux) : 0;

  if (delta >= LIGHT_LUX_DELTA_THRESHOLD) {
      if (g_light_hits < 255) g_light_hits++;
      Report("Light: lux=%u base=%u delta=%u hits=%u\r\n",
             (unsigned int)now, (unsigned int)g_light_baseline_lux,
             (unsigned int)delta, (unsigned int)g_light_hits);
  } else {
      if (g_light_hits > 0) g_light_hits--;
  }
  return (g_light_hits >= LIGHT_HITS_REQUIRED);
}

//*****************************************************************************
// GPS (UART1 polled) -- our working code
//*****************************************************************************
static void ConfigureGpsPins(void)
{
  PinTypeUART(PIN_58, PIN_MODE_6); // UART1_TX -> GPS RXD
  PinTypeUART(PIN_59, PIN_MODE_6); // UART1_RX <- GPS TXD
}

static void GPSUartInit(void)
{
  MAP_UARTDisable(UARTA1_BASE);
  MAP_UARTConfigSetExpClk(UARTA1_BASE,
                          MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
                          GPS_UART_BAUD,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
  MAP_UARTEnable(UARTA1_BASE);
}

static char *csv_next(char **p)
{
  char *s, *c;
  if (!p || !*p) return 0;
  s = *p;
  c = strchr(s, ',');
  if (c) { *c = '\0'; *p = c + 1; }
  else   { *p = 0; }
  return s;
}

static float nmea_to_decimal(const char *val, char hemi)
{
  float v, dec;
  int   deg;
  float min;
  if (!val || !val[0]) return 0.0f;
  v   = (float)atof(val);
  deg = (int)(v / 100.0f);
  min = v - ((float)deg * 100.0f);
  dec = (float)deg + (min / 60.0f);
  if (hemi == 'S' || hemi == 'W') dec = -dec;
  return dec;
}

static void gps_try_parse_rmc(char *line)
{
  char *p, *hdr, *time_f, *status, *lat, *latH, *lon, *lonH;
  bool was_valid;

  if (!line) return;
  if (strncmp(line, "$GPRMC", 6) != 0 && strncmp(line, "$GNRMC", 6) != 0) return;

  p         = line;
  was_valid = g_fix.valid;
  hdr    = csv_next(&p); (void)hdr;
  time_f = csv_next(&p); (void)time_f;
  status = csv_next(&p);
  lat    = csv_next(&p);
  latH   = csv_next(&p);
  lon    = csv_next(&p);
  lonH   = csv_next(&p);

  if (!status || status[0] != 'A') {
      g_fix.valid = false;
      if (was_valid && g_gps_lock_reported) {
          Report("GPS: lock lost\r\n");
          g_gps_lock_reported = false;
      }
      return;
  }
  if (!lat || !latH || !lon || !lonH) { g_fix.valid = false; return; }

  g_fix.lat   = nmea_to_decimal(lat, latH[0]);
  g_fix.lon   = nmea_to_decimal(lon, lonH[0]);
  g_fix.valid = true;
  if (!g_gps_lock_reported) {
      Report("GPS: fix acquired lat=%.5f lon=%.5f\r\n",
             (double)g_fix.lat, (double)g_fix.lon);
      g_gps_lock_reported = true;
      g_gps_fail_reported = false;
  }
}

static void GPSPoll(void)
{
  static char     buf[128];
  static uint32_t idx = 0;
  int c;

  while (MAP_UARTCharsAvail(UARTA1_BASE)) {
      c = MAP_UARTCharGetNonBlocking(UARTA1_BASE);
      if (c < 0) break;
      if (c == '\r') continue;
      if (c == '\n') {
          buf[idx] = '\0';
          if (idx > 0) {
              char tmp[128];
              strncpy(tmp, buf, sizeof(tmp) - 1);
              tmp[sizeof(tmp) - 1] = '\0';
              g_gps_rx_seen = true;
              if (g_gps_diag_lines < GPS_DIAG_ECHO_LINES) {
                  Report("GPS: %s\r\n", tmp);
                  g_gps_diag_lines++;
              }
              gps_try_parse_rmc(tmp);
          }
          idx = 0;
      } else {
          if (idx < sizeof(buf) - 1) buf[idx++] = (char)c;
          else idx = 0;
      }
  }

  if (!g_fix.valid && !g_gps_fail_reported &&
      ((int32_t)(g_tick40ms - g_gps_connect_deadline) >= 0)) {
      Report(g_gps_rx_seen ?
             "GPS: no fix (indoors?), continuing\r\n" :
             "GPS: no UART data -- check PIN_59 wiring, baud=9600\r\n");
      g_gps_fail_reported = true;
  }
}

//*****************************************************************************
// BoardInit -- Teni's version (compatible with network_utils.h)
//*****************************************************************************
static void BoardInit(void)
{
#ifndef USE_TIRTOS
#if defined(ccs)
  MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
  MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
  MAP_IntMasterEnable();
  MAP_IntEnable(FAULT_SYSTICK);
  PRCMCC3200MCUInit();
}

//*****************************************************************************
// set_time -- Teni's version (uses g_time from network_utils)
//*****************************************************************************
static int set_time(void)
{
  long retVal;
  g_time.tm_day  = DATE;
  g_time.tm_mon  = MONTH;
  g_time.tm_year = YEAR;
  g_time.tm_sec  = SECOND;
  g_time.tm_min  = MINUTE;
  g_time.tm_hour = HOUR;
  retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                     SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                     sizeof(SlDateTime), (unsigned char *)(&g_time));
  ASSERT_ON_ERROR(retVal);
  return SUCCESS;
}

//*****************************************************************************
// IR decoder helpers -- Teni's working functions, kept exactly
//*****************************************************************************
static inline uint32_t GetTick(void)
{
  return MAP_TimerValueGet(TIMERA0_BASE, TIMER_A);
}

static inline bool HasElapsed(uint32_t now, uint32_t then, uint32_t interval)
{
  return (uint32_t)(now - then) >= interval;
}

static void GPIOA3IntHandler(void)
{
  uint32_t status = MAP_GPIOIntStatus(IR_GPIO_PORT_BASE, true);
  MAP_GPIOIntClear(IR_GPIO_PORT_BASE, status);

  uint32_t ulCurrentTick = GetTick();
  g_ulDelta        = ulCurrentTick - g_ulPreviousTick;
  g_ulPreviousTick = ulCurrentTick;

  if (g_bDataCooldown && HasElapsed(ulCurrentTick, g_ulEndTick, MEASURED_NEW_DATA_WAIT_TICKS)){
      g_bDataCooldown = 0;
  }

  if (g_bNewDataWord && !g_bDataCooldown) {
      g_bNewDataWord  = 0;
      g_ulFirstTick   = ulCurrentTick;
  }

  if ((ulCurrentTick - g_ulFirstTick) >= MEASURED_DATA_TICKS && !g_bDataCooldown) {
      g_bNewDataWord  = 1;
      g_ulEndTick     = ulCurrentTick;
      g_bDataCooldown = 1;
      g_bDataReady    = 1;
      g_bStartDetected = 0;
  }

  if (g_bTotalEdges == 3){
      g_bStartDetected = 1;
  }

  if (g_bStartDetected && g_iEdgeType) {
      int currentBit = (g_bTotalEdges - 3) / 2;
      if (MEASURED_OUTER_TICKS_LOWER < g_ulDelta && g_ulDelta < MEASURED_OUTER_TICKS_UPPER)
          g_iEdgeType = !g_iEdgeType;
      if (MEASURED_1_TICKS_LOWER < g_ulDelta && g_ulDelta < MEASURED_1_TICKS_UPPER)
          g_ulIRData |= (1UL << currentBit);
  }

  g_bTotalEdges++;
  g_iEdgeType = !g_iEdgeType;
}

static void InitTimer0FreeRunning(void)
{
  MAP_PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
  MAP_TimerConfigure(TIMERA0_BASE, TIMER_CFG_PERIODIC_UP);
  MAP_TimerEnable(TIMERA0_BASE, TIMER_A);
}

static void InitIRGPIO(void)
{
  MAP_GPIOIntRegister(IR_GPIO_PORT_BASE, GPIOA3IntHandler);
  MAP_GPIOIntTypeSet(IR_GPIO_PORT_BASE, IR_GPIO_PIN, GPIO_BOTH_EDGES);
  MAP_GPIOIntClear(IR_GPIO_PORT_BASE, IR_GPIO_PIN);
  MAP_GPIOIntEnable(IR_GPIO_PORT_BASE, IR_GPIO_PIN);
}

static IRKey DecodeIRToKey(uint32_t ulData)
{
  switch (ulData) {
      case 0x8167E9: return KEY_1;
      case 0x796869: return KEY_2;
      case 0x7568A9: return KEY_3;
      case 0x7168E9: return KEY_4;
      case 0x6D6929: return KEY_5;
      case 0x696969: return KEY_6;
      case 0x6569A9: return KEY_7;
      case 0x6169E9: return KEY_8;
      case 0x5D6A29: return KEY_9;
      case 0x596A69: return KEY_0;
      case 0x316CE9: return KEY_ENTER;
      case 0xC563A9: return KEY_DELETE;
      case 0x3D6C29: return KEY_VOLUP;
      case 0x396C69: return KEY_VOLDOWN;
      case 0xBD6429: return KEY_PGUP;
      case 0xDD6229: return KEY_PGDOWN;
      default:       return KEY_NONE;
  }
}

void OLED_RemoveGraphic(void)
{
  fillRect(1, 8, 126, 108, BLACK);
}

static bool passwordCheck(LockState state){
   fillRect(0, 12, 128, 36, CLOUD);
   fillRect(0, 32, 128, 12, BLACK);

   setCursor(8, 16);
   setTextSize(1);
   setTextColor(BLACK, CLOUD);

   Outstr("Enter current pin");
   drawRect(0, 32, 128, 12, WHITE);

   checkPassword = true;
   CommitCurrentCharacter();
   bool randomButtonControl = false;
   while(checkPassword){
       if (g_bDataReady && randomButtonControl){
           ButtonPressControl();
      } else if(!randomButtonControl){
          g_ulIRData    = 0;
          g_bDataReady  = 0;
          g_bTotalEdges = 0;
          randomButtonControl = true;
      }
   }

   if((strcmp(g_password, g_passwordPrompt) != 0)){
       Report("Incorrect password\n\r");
       buzzer_on();
       MAP_UtilsDelay(2666666);
       buzzer_off();
   }

    if(state == Alert){
           OLED_DrawCurrentState(state);
       } else {
           OLED_PrintMenuPage(state);
           updated = 1;
       }

   return (strcmp(g_password, g_passwordPrompt) == 0);
}

// Teni's state-machine key handler -- kept exactly
static void StateKeyCode(IRKey key)
{
  lastState = currentState;

  switch (key) {
      case KEY_0:
          currentState = (currentState == Settings) ? Disarm :
                         (currentState == Disarm)   ? Settings : currentState;
          if (currentState != lastState) updated = 0;
          return;

      case KEY_1:
          if(currentState == Disarm){
              currentState = Arm;
          } else if (currentState == Arm){
              if(hasPasswordSet){
                  if(!passwordCheck(currentState)){
                     return;
                 }
              }
              currentState = Reset;
          } else if (currentState == Settings ){
              if(hasPasswordSet){
                  if(!passwordCheck(currentState)){
                      return;
                  }
             }
              currentState = PasswordSet;
          }
          if(currentState != lastState){
              updated = 0;
          }
          return;

      case KEY_2:
          if(currentState == Arm){
              currentState = Alert;
          } else if (currentState == Alert){
              if(hasPasswordSet){
                  if(!passwordCheck(currentState)){return;}
              }
              currentState = Arm;
          } else if (currentState == Settings){
              if(hasPasswordSet){
                  if(!passwordCheck(currentState)){return;}
                  strcpy(g_password, "");
                  hasPasswordSet = false;
                  Report("Password has been removed");
              } else {
                  setCursor(8,92);
                  setTextSize(1);
                  setTextColor(WHITE, BLACK);
                  Outstr("  No password to");
                  setCursor(8,102);
                  Outstr("      remove");
                  MAP_UtilsDelay(1000000);
                  fillRect(8,92,110,20,BLACK);
              }
          }
          if(currentState != lastState){
              updated = 0;
          }
          return;

      case KEY_PGUP:
          currentOption = (currentOption + 1) % 3;
          updated = 0;
          return;

      case KEY_PGDOWN:
          currentOption--;
          if (currentOption < 0) currentOption = 0;
          updated = 0;
          return;

      default:
          Report("no state registered");
          return;
  }
}

static const char *NumberKeyCode(IRKey key)
{
  lastState = currentState;

  switch (key) {
      case KEY_1: return "1";
      case KEY_2: return "2";
      case KEY_3: return "3";
      case KEY_4: return "4";
      case KEY_5: return "5";
      case KEY_6: return "6";
      case KEY_7: return "7";
      case KEY_8: return "8";
      case KEY_9: return "9";
      case KEY_0: return "0";

      default: return "";
  }
}

static void CommitCurrentCharacter(void)
{
  g_lastTapKey   = KEY_NONE;
  g_lastTapCycle = 0;
  lastPress = GetTick();
}

void OLED_DrawText(char *text) {
   drawChar(6 + (((g_composeLen-1)*20)), 32, text[g_composeLen-1], 0xFFFF, 0x0000, 2);
}

void OLED_DrawFullText(char *text) {
   int i = 0;
   while(i < strlen(text)){
       drawChar(6 + (((i-1)*20)), 32, text[i], 0xFFFF, 0x0000, 2);
       i++;
   }
}

void OLED_DrawMenuBackground(void)
{
  drawRect(0, 0, SCREEN_WIDTH, 1,   WHITE);
  drawRect(0, 0, SCREEN_WIDTH, 8,   WHITE);
  drawRect(0, 0, SCREEN_WIDTH, 120, WHITE);
  drawRect(0, 0, SCREEN_WIDTH, 127, WHITE);
}

void OLED_PrintMenuPage(LockState state)
{
  OLED_RemoveGraphic();
  Report("current State: %d", state);
  switch (state) {
      case Arm:
          fillScreen(BLACK);
          setCursor(4, 32);
          setTextSize(4);
          setTextColor(WHITE, YELLOW);
          fillRect(0, 32, 128, 24, YELLOW);
          Outstr("ARMED");
          setCursor(16, 16);
          setTextSize(1);
          setTextColor(WHITE, BLACK);
          Outstr("Press 1 to Disarm");
          setCursor(0, 72);
          setTextSize(1);
          setTextColor(WHITE, BLACK);
          Outstr("Press 2 to Test Alert");
          return;

      case Disarm:
          setCursor(24, 0);
          setTextSize(1);
          setTextColor(WHITE, BLUE);
          fillRect(0, 0, 128, 8, BLUE);
          Outstr("DISARMED MENU");
          OLED_DrawMenuBackground();
          setCursor(2, 20);
          setTextSize(1);
          setTextColor(WHITE, BLACK);
          Outstr("Press 1 to Arm");
          setCursor(2, 30);
          setTextSize(1);
          setTextColor(WHITE, BLACK);
          Outstr("Press 0 for Settings");
          return;
      case Settings:
          fillScreen(BLACK);
          setCursor(36, 0);
          setTextSize(1);
          setTextColor(WHITE, RED);
          OLED_DrawMenuBackground();
          fillRect(0, 0, 128, 8, RED);
          Outstr("SETTINGS");
          setCursor(2, 20);
          setTextSize(1);
          setTextColor(WHITE, BLACK);
          Outstr("Press 1 to Set New");
          setCursor(2, 30);
          Outstr("Password");
          setCursor(2, 40);
          Outstr("Press 2 to Remove");
          setCursor(2, 50);
          Outstr("Password");
          setCursor(2, 70);
          Outstr("Press 0 to go Back");
          return;
      default:
          return;
  }
}

static void RefreshDisplay(void)
{
  if ((lastState != currentState && !updated)) {
      OLED_PrintMenuPage(currentState);
      updated = 1;
  }
}

static void MultiTapHandleKey(IRKey key)
{
  uint32_t now = GetTick();

  if(currentState == PasswordSet || checkPassword){
      if (key == KEY_DELETE) {
                 CommitCurrentCharacter();
                 if (g_composeLen > 0) {
                     Report("Removed num: %c\n\r", g_composeBuf[g_composeLen]);
                     OLED_DrawText("      ");
                     fillRect(0, 32, 128, 14, BLACK);
                     drawRect(0, 32, 128, 12, WHITE);
                     OLED_DrawFullText(g_composeBuf);
                     g_composeLen--;
                     g_composeBuf[g_composeLen] = '\0';

                 }
                 RefreshDisplay();
                 return;
             }

             if (key == KEY_ENTER) {
                 CommitCurrentCharacter();
                 if (g_composeLen == 6 && !checkPassword) {
                       fillScreen(BLACK);
                       strncpy(g_password, g_composeBuf, MAX_MSG_LEN);
                       g_password[MAX_MSG_LEN] = '\0';
                       while(g_composeLen){
                           g_composeLen--;
                           g_composeBuf[g_composeLen] = '\0';
                       }
                       setCursor(16, 72);
                       setTextSize(1);
                       setTextColor(GREEN, BLACK);
                       Outstr("PASSWORD SET");
                       Report("Your new password is ");
                       Report(g_password);
                       Report("\n\r");
                       hasPasswordSet = true;
                       currentState = Settings;
                 } else if(checkPassword){
                     checkPassword = false;
                     strncpy(g_passwordPrompt, g_composeBuf, MAX_MSG_LEN);
                     g_passwordPrompt[MAX_MSG_LEN] = '\0';
                     while(g_composeLen){
                         g_composeLen--;
                         g_composeBuf[g_composeLen] = '\0';
                     }
                 }
                 return;
             }
  }

  if (key < KEY_0 || key > KEY_9) return;

  const char *chars = NumberKeyCode(key);
  uint32_t charsetLen = (uint32_t)strlen(chars);
  if (charsetLen == 0 || g_composeLen >= MAX_MSG_LEN) return;

  bool withinThreshold = !HasElapsed(now, g_lastTapTick, MULTITAP_TIMEOUT_TICKS);
  bool statechanged = false;
   if(currentState != PasswordSet && !checkPassword){
             Report("lastPress = %d, now = %d, difference = %d, withinThreshold = %d\n\r", lastPress, now, now - lastPress, withinThreshold);
             StateKeyCode(key);

         } else if (g_composeLen < MAX_MSG_LEN) {
             g_lastTapCycle = 0;
             g_composeBuf[g_composeLen++] = chars[g_lastTapCycle];
             g_composeBuf[g_composeLen]   = '\0';
             Report("Current num: %c\n\r", g_composeBuf[g_composeLen-1]);
             OLED_DrawText(g_composeBuf);
         }

  g_lastTapKey  = key;
  g_lastTapTick = now;
}

static void ButtonPressControl(void)
{
  IRKey key = DecodeIRToKey(g_ulIRData);
  if (key != KEY_NONE){
      MultiTapHandleKey(key);
  }
  g_ulIRData    = 0;
  g_bDataReady  = 0;
  g_bTotalEdges = 0;
}

//*****************************************************************************
// OLED graphics -- Teni's functions, kept exactly
//*****************************************************************************
void OLED_DrawBackgroundGuardianGraphic(void)
{
  drawCircle(64, 64, 60, YELLOW);
  fillRoundRect(36, 48, 56, 16, 8, WHITE);
  fillRoundRect(36, 72, 56, 16, 8, WHITE);
  fillRect(48, 0, 32, 128, WHITE);
  fillRect(0, 0, 128, 24, RED);
  unsigned char *title = "  BACKPACK       GUARDIAN   ";
  int i = 0;
  while (i < 27) {
      drawChar(17 + ((i * 8) % 120), 1 + 11 * ((int)(i / 15)), title[i], WHITE, RED, 1);
      i++;
  }
  MAP_UtilsDelay(5000000);
}

void OLED_DrawLoadingScreen(void)
{
  fillRect(0, 52, 128, 24, BLACK);
  drawCircle(64, 64, 24, WHITE);
  drawCircle(64, 64, 25, WHITE);
  drawCircle(64, 64, 26, WHITE);
  connected ? drawCircle(64, 64, 60, YELLOW) : drawCircle(64, 64, 60, RED);
  fillRect(0, 52, 128, 24, BLACK);
  char *connection = connected ? "CONNECTED!" : "CONNECTING";
  int i = 0;
  while (i < 10) {
      drawChar(28 + 8 * i, 60, connection[i], YELLOW, 0x0000, 1);
      i++;
  }
  MAP_UtilsDelay(5000000);
}

void OLED_StatusUpdate(unsigned char *message, int currentLength)
{
  int currentChar = 0;
  while (currentChar < currentLength) {
      drawChar(6 + ((currentChar * 8) % 120), 96, message[currentChar], CLOUD, 0x0000, 1);
      currentChar++;
  }
}

void OLED_DrawCurrentState(LockState state)
{
  unsigned char *statusReport = "";
  int currentLength = 8;

  switch (state) {
      case Arm:
          OLED_RemoveGraphic();
          statusReport  = " STATUS: ARMED";
          currentLength += 6;
          OLED_StatusUpdate(statusReport, currentLength);
          fillRoundRect(36, 48, 56, 16, 8, WHITE);
          fillRoundRect(36, 72, 56, 16, 8, WHITE);
          fillRoundRect(44, 24, 40,  8, 8, WHITE);
          fillRoundRect(46, 28,  8, 16, 1, WHITE);
          fillRoundRect(74, 28,  8, 16, 1, WHITE);
          MAP_UtilsDelay(1400000);
          return;
      case Disarm:
          OLED_RemoveGraphic();
          Report("Disarmed\n\r");
          statusReport  = "STATUS:DISARMED";
          currentLength += 7;
          OLED_StatusUpdate(statusReport, currentLength);
          fillRoundRect(36, 48, 56, 16, 8, WHITE);
          fillRoundRect(36, 72, 56, 16, 8, WHITE);
          fillRoundRect(44, 20, 40,  8, 8, WHITE);
          fillRoundRect(46, 24,  8, 16, 1, WHITE);
          fillRoundRect(74, 22,  8, 12, 1, WHITE);
          MAP_UtilsDelay(1400000);
          return;
      case Settings:
          fillScreen(BLACK);
          Report("Settings\n\r");
          statusReport  = "    SETTINGS";
          currentLength += 4;
          OLED_StatusUpdate(statusReport, currentLength);
          return;
      case Alert:
          fillScreen(BLACK);
          setCursor(0, 0);
          setTextSize(2);
          setTextColor(RED, BLACK);
          Outstr("ALERT\n");
          setTextSize(1);
          setTextColor(WHITE, BLACK);
          Outstr("Movement\n");
          return;

      case Reset:
          fillScreen(BLACK);
          statusReport  = " STATUS: RESET";
          currentLength += 6;
          OLED_StatusUpdate(statusReport, currentLength);
          OLED_RemoveGraphic();
          MAP_UtilsDelay(1400000);
          return;

      case PasswordSet:
          fillScreen(BLACK);
          setCursor(8, 16);
          setTextSize(1);
          setTextColor(WHITE, BLACK);
          Outstr("Enter 6-digit pin");
          drawRect(0, 32, 128, 12, WHITE);
          OLED_DrawText(g_composeBuf);
          return;

      default:
          Report("Error");
          return;
  }
}

//*****************************************************************************
// http_post -- Fixed String formatting/JSON
//*****************************************************************************
#define DATA1 "{" \
                "\"state\": {\r\n"          \
                    "\"desired\" : {\r\n"   \
                        "\"Alert!\" :\""
// DUMMYMESSAGE replaced by dynamic alertMsg built at runtime
#define DATA2        "\"\r\n"  \
                "}"          \
            "}"               \
        "}\r\n\r\n"

static int http_post(int iTLSSockID)
{
  char  acSendBuff[512];
  char  acRecvbuff[1460];
  char  cCLLength[200];
  char *pcBufHeaders;
  int   lRetVal = 0;

  pcBufHeaders = acSendBuff;
  strcpy(pcBufHeaders, POSTHEADER);       pcBufHeaders += strlen(POSTHEADER);
  strcpy(pcBufHeaders, HOSTHEADER);       pcBufHeaders += strlen(HOSTHEADER);
  strcpy(pcBufHeaders, CHEADER);          pcBufHeaders += strlen(CHEADER);

  // Safely inject GPS coordinates into the JSON string without memory bugs
  char alertMsg[256];
  if (g_fix.valid) {
      sprintf(alertMsg, "ALERT! Backpack moved. Lat:%.6f Lon:%.6f", (double)g_fix.lat, (double)g_fix.lon);
  } else {
      sprintf(alertMsg, "ALERT! Backpack moved. No GPS fix available.");
  }

  int dataLength = strlen(DATA1) + strlen(alertMsg) + strlen(DATA2);

  strcpy(pcBufHeaders, CTHEADER);         pcBufHeaders += strlen(CTHEADER);
  strcpy(pcBufHeaders, CLHEADER1);        pcBufHeaders += strlen(CLHEADER1);
  sprintf(cCLLength, "%d", dataLength);
  strcpy(pcBufHeaders, cCLLength);        pcBufHeaders += strlen(cCLLength);
  strcpy(pcBufHeaders, CLHEADER2);        pcBufHeaders += strlen(CLHEADER2);
  strcpy(pcBufHeaders, DATA1);            pcBufHeaders += strlen(DATA1);
  strcpy(pcBufHeaders, alertMsg);         pcBufHeaders += strlen(alertMsg);
  strcpy(pcBufHeaders, DATA2);            pcBufHeaders += strlen(DATA2);

  UART_PRINT(acSendBuff);
  UART_PRINT("Message Sent!\r\n");

  lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
  if (lRetVal < 0) {
      UART_PRINT("POST failed. Error: %i\n\r", lRetVal);
      sl_Close(iTLSSockID);
      GPIO_IF_LedOn(MCU_RED_LED_GPIO);
      return lRetVal;
  }
  lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
  if (lRetVal < 0) {
      UART_PRINT("Recv failed. Error: %i\n\r", lRetVal);
      GPIO_IF_LedOn(MCU_RED_LED_GPIO);
      return lRetVal;
  } else {
      acRecvbuff[lRetVal] = '\0';
      UART_PRINT(acRecvbuff);
      UART_PRINT("\n\r\n\r");
  }
  return 0;
}

//*****************************************************************************
// main
//*****************************************************************************
void main(void)
{
  long lRetVal = -1;

  BoardInit();
  PinMuxConfig();

  // Buzzer: drive HIGH immediately (active-LOW, prevents boot buzz)
  MAP_GPIODirModeSet(BUZZER_PORT, BUZZER_PIN, GPIO_DIR_MODE_OUT);
  MAP_GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, BUZZER_PIN);

  InitTerm();
  ClearTerm();
  UART_PRINT("=== Backpack Guardian Debugger ===\r\n");

  // SysTick for sensor timing (40ms tick)
  SysTickInitApp();

  // IR timer + GPIO
  InitTimer0FreeRunning();
  InitIRGPIO();

  // I2C: accelerometer (0x18) + BH1750 (0x23)
  ConfigureI2CPins();
  I2C_IF_Open(I2C_MASTER_MODE_FST);
  g_i2c_available = true;
  Report("I2C: init OK\r\n");

  // BH1750 light sensor
  if (!bh1750_init()) {
      Report("Light: BH1750 not found -- light detection disabled\r\n");
      g_bh1750_ready = false;
  }

  // GPS on UART1 at 9600 baud (polled, non-blocking)
  MAP_PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);
  ConfigureGpsPins();
  GPSUartInit();
  g_gps_connect_deadline = g_tick40ms + ms_to_ticks40(GPS_CONNECT_TIMEOUT_MS);
  Report("GPS: UART1 init 9600 baud\r\n");

  // OLED SPI init
  UART_PRINT("OLED Setup\r\n");
  MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
  MAP_PRCMPeripheralReset(PRCM_GSPI);
  MAP_SPIReset(GSPI_BASE);
  MAP_SPIConfigSetExpClk(
      GSPI_BASE,
      MAP_PRCMPeripheralClockGet(PRCM_GSPI),
      SPI_BITRATE,
      SPI_MODE_MASTER,
      SPI_SUB_MODE_0,
      SPI_SW_CTRL_CS | SPI_4PIN_MODE | SPI_TURBO_OFF | SPI_CS_ACTIVELOW | SPI_WL_8
  );
  MAP_SPIEnable(GSPI_BASE);
  Adafruit_Init();
  fillScreen(BLACK);

  // Startup graphic
  OLED_DrawBackgroundGuardianGraphic();
  fillScreen(BLACK);

  // WiFi + TLS (Teni's working sequence)
  UART_PRINT("Connecting to WiFi...\r\n");
  OLED_DrawLoadingScreen();
  g_app_config.host = SERVER_NAME;
  g_app_config.port = GOOGLE_DST_PORT;

  lRetVal = connectToAccessPoint();
  if (lRetVal < 0) {
      UART_PRINT("Unable to connect to Access Point\r\n");
  }

  lRetVal = set_time();
  if (lRetVal < 0) {
      UART_PRINT("Unable to set time\r\n");
  }

  lRetVal = tls_connect();
  if (lRetVal < 0) {
      ERR_PRINT(lRetVal);
      fillScreen(RED);
      Outstr("Error Connecting to AWS\r\n");
  } else {
      UART_PRINT("Connected to AWS!\r\n");
      connected = true;
      OLED_DrawLoadingScreen();
  }

  fillScreen(BLACK);
  Report("Boot complete. Entering state machine.\r\n");

  g_gps_coord_next = g_tick40ms + ms_to_ticks40(GPS_COORD_PERIOD_MS);

  currentState = Disarm;

  while (1) {
      // --- Always poll GPS (non-blocking) ---
      GPSPoll();

      // --- Periodic GPS coordinate log ---
      if ((int32_t)(g_tick40ms - g_gps_coord_next) >= 0) {
          g_gps_coord_next += ms_to_ticks40(GPS_COORD_PERIOD_MS);
          if (g_fix.valid)
              Report("GPS: lat=%.6f lon=%.6f\r\n", (double)g_fix.lat, (double)g_fix.lon);
          else
              Report("GPS: no fix\r\n");
      }

      // =====================================================
      // DISARM state
      // =====================================================
      if (currentState == Disarm) {
          Report("entering current state: %d\r\n", currentState);
          updated = 0;
          CommitCurrentCharacter();
          OLED_DrawCurrentState(Disarm);
          Report("Disarm/Carry Mode On:\r\n");
          Report("(1) Arm  (0) Settings\r\n");

          while (currentState == Disarm ) {
              GPSPoll();
              RefreshDisplay();
              if (g_bDataReady){
                  ButtonPressControl();
          }

          }
      // =====================================================
      // ARM state -- with motion + light sensor detection
      // =====================================================
      } else if (currentState == Arm) {
          Report("entering current state: %d\r\n", currentState);
          updated = 0;
          CommitCurrentCharacter();
          OLED_DrawCurrentState(Arm);
          Report("Armed. Monitoring motion and light.\r\n");
          Report("(1) Disarm  (2) Test Alert\r\n");

          // Latch sensor baselines on arm entry
          accel_read_i8(ACCEL_REG_X, &g_prev_x);
          accel_read_i8(ACCEL_REG_Y, &g_prev_y);
          g_motion_hits = 0;
          g_light_hits  = 0;
          g_last_trigger_light = false;
          g_alert_posted = false;  // reset so next alert sends a fresh email
          light_sensor_arm_baseline();
          g_motion_next_sample = g_tick40ms + ms_to_ticks40(MOTION_SAMPLE_MS);

          CommitCurrentCharacter();
          RefreshDisplay();
          bool trigger = 1;

          while (currentState == Arm && trigger ){
              GPSPoll();

              if (g_bDataReady){
                  ButtonPressControl();
              }

              // Motion + light sensor sampling
              if ((int32_t)(g_tick40ms - g_motion_next_sample) >= 0) {
                  g_motion_next_sample = g_tick40ms + ms_to_ticks40(MOTION_SAMPLE_MS);

                  bool motion_alarm = motion_detect_sample();
                  bool light_alarm  = (g_bh1750_ready) ? light_sensor_open_sample() : false;

                  // FIXED MERGE BUG: Placed the state change OUTSIDE the "else" block
                  // so that a light_alarm actually triggers the siren system!
                  if (motion_alarm || light_alarm) {
                      g_last_trigger_light = light_alarm;

                      if (light_alarm){
                          Report("Alert trigger: bag-open (light change)\r\n");
                      }
                      else {
                          Report("Alert trigger: motion/lift detected\r\n");
                      }

                      lastState    = currentState;
                      currentState = Alert;
                      updated      = 0;
                      trigger      = 0;
                  }
              }
          }

      // =====================================================
      // SETTINGS state
      // =====================================================
      } else if (currentState == Settings) {
          Report("entering current state: %d\r\n", currentState);
          updated = 0;
          CommitCurrentCharacter();
          OLED_DrawCurrentState(Settings);
          Report("Settings Menu: (0) Back  (1) Set Password  (2) Remove Password\r\n");

          while (currentState == Settings ) {
              GPSPoll();
              RefreshDisplay();
              if (g_bDataReady){
                  ButtonPressControl();
              }
          }
          fillScreen(BLACK);

      // =====================================================
      // ALERT state -- buzzer on, http_post, wait for user
      // =====================================================
      } else if (currentState == Alert) {
          Report("entering current state: %d\r\n", currentState);
          updated = 0;
          CommitCurrentCharacter();
          OLED_DrawCurrentState(Alert);
          Report("ALERT! Trigger: %s\r\n", g_last_trigger_light ? "BAG_OPEN" : "MOTION");
          Report("(1) Reset  -- buzzer active\r\n");

          buzzer_on();
          if (!g_alert_posted && connected) {
              http_post(lRetVal);
              g_alert_posted = true;
          }

          int alertTicks = 0;
          bool buzzerToggle = true; // varies between on and off
          CommitCurrentCharacter();

          while (currentState == Alert ) {
              GPSPoll();
              // Keep logging GPS coords during alert so we get fresh fix in POST
              if ((int32_t)(g_tick40ms - g_gps_coord_next) >= 0) {
                  g_gps_coord_next += ms_to_ticks40(GPS_COORD_PERIOD_MS);
                  if (g_fix.valid)
                      Report("GPS: lat=%.6f lon=%.6f\r\n", (double)g_fix.lat, (double)g_fix.lon);
              }

              if(buzzerToggle){
                  buzzer_off();
              } else {
                  buzzer_on();
              }

              if(!(alertTicks % 1)){ // runs every 0.2s
                  buzzerToggle = !buzzerToggle;
              }

              if (g_bDataReady){
                  ButtonPressControl();
              }
              alertTicks++;
              MAP_UtilsDelay(2666666);  // ~100ms per iteration -> ~20s total buzz
          }

          buzzer_off();
          fillScreen(BLACK);
          currentState = Reset;

      // =====================================================
      // RESET state
      // =====================================================
      } else if (currentState == Reset) {
          Report("entering current state: %d\r\n", currentState);
          CommitCurrentCharacter();
          OLED_DrawCurrentState(Reset);
          Report("Reset: returning to Disarm\r\n");
          currentState = Disarm;

      // =====================================================
      // PASSWORD SET state
      // =====================================================
      } else if (currentState == PasswordSet) {
          Report("entering current state: %d\r\n", currentState);
          updated = 0;
          Report("PasswordSet: waiting for input (KEY_0 to cancel)\r\n");
          CommitCurrentCharacter();
          OLED_DrawCurrentState(PasswordSet);

          while (currentState == PasswordSet ) {
              GPSPoll();
              if (g_bDataReady){
                  ButtonPressControl();
              }
          }
      } else {
          Report("error: no current state: %d\r\n", currentState);
      }
  }
}

