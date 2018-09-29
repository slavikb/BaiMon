// BaiMon - Vaillant TurboTec family boiler monitor
//
//

#include "esp8266_peri.h"
#include "uart_register.h"

extern "C" {
#include "user_interface.h"
}

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiClient.h>

#include "local_config.h"

/////////////////////////////////////////////////////////////
// Defines

#define BAIMON_VERSION "1.3"

#define EBUS_SLAVE_ADDR(MASTER) ((MASTER)+5)

enum // EBus addresses
{
  EBUS_ADDR_HOST   = CONFIG_EBUS_ADDR_HOST,
  EBUS_ADDR_BOILER = EBUS_SLAVE_ADDR(CONFIG_EBUS_ADDR_BOILER),
};

/////////////////////////////////////////////////////////////
// History byffer sizes

// EBus data history buffer size
#define SERIAL_BUFFER_SIZE 0x1000
// Measurement parameter history size
#define MAX_PARM_HISTORY 64

// Web page parameters
// Show parameter measurements on page (<=MAX_PARM_HISTORY)
#define WEB_PARM_HISTORY 25
// Size of traffic data shown on diagnostics page (<=SERIAL_BUFFER_SIZE)
#define WEB_DIAG_DATA_SIZE 0x400

// Failed commands history size
#define FAILED_COMMAND_HISTORY_SIZE 4
// Max number of bytes to collect for failed command
#define FAILED_COMMAND_BYTES_MAX   128
// Number of bytes before command to display (<FAILED_COMMAND_BYTES_MAX)
#define FAILED_COMMAND_BYTES_BEFORE 16
// Time window to collect data for failed command history (ms)
#define FAILED_COMMAND_TIME_WINDOW 8000

/////////////////////////////////////////////////////////////
// WiFi

void SetupWifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(CONFIG_WIFI_SSID, CONFIG_WIFI_PASS);
}

/////////////////////////////////////////////////////////////
// EBus utilites

uint8 ICACHE_RAM_ATTR Crc8Byte(uint8 data, uint8 crc)
{
   for (unsigned int i = 0; i < 8; i++)
   {
      const uint8 polynom = (crc & 0x80) ? 0x9B : 0;

      crc <<= 1;
      if (data & 0x80)
         crc |= 0x01;

      crc ^= polynom;
      data <<= 1;
   }
   return crc;
}

uint8 ICACHE_RAM_ATTR Crc8Buf(const void *buf, uint32 len)
{
  uint8 crc = 0;
  const uint8 *p = (const uint8 *)buf;
  for (; len != 0; --len)
    crc = Crc8Byte(*p++, crc);
  return crc;
}

/////////////////////////////////////////////////////////////
// EBus processing

#define UART_EBUS UART0

enum // special receive statuses
{
  RecvTimeout = -1,  // timeout
  RecvError   = -2,  // framing error
  RecvUnknown = -3   // unexpected interrupt
};

enum
{
  // Max number of retries for failed command (bus collision or other error)
  CMD_MAX_RETRIES = 2,
  // Number of skipped SYN before retries
  // (randomly selected between MIN_SYNC_SKIP and MAX_SYN_SKIP)
  CMD_MIN_SYNC_SKIP = 1,
  CMD_MAX_SYNC_SKIP = 5
};

// EBus data is stored in ring buffer (for diagnostics)
uint8 g_serialBuffer[SERIAL_BUFFER_SIZE];
// Total byte count (if overflows, reset to 0x80000000)
uint32 g_serialByteCount = 0;

#define GET_SERIAL_BYTE_NO(NO) (g_serialBuffer[(NO) % SERIAL_BUFFER_SIZE])
#define PUT_SERIAL_BYTE(B) do { \
  g_serialBuffer[g_serialByteCount % SERIAL_BUFFER_SIZE] = (B); \
  if (++g_serialByteCount == 0) \
    g_serialByteCount = 0x80000000; \
} while(0)

// Total receive error counter
uint32 g_recvErrCnt = 0;

// Sync sense
volatile uint32 g_lastSyncTime = 0;  // last CLEAN sync time (with no data after)
volatile uint32 g_nonSyncData = 0;   // flag: non-sync data was received

#define INVALID_STATE_VALUE     0xFF
#define INVALID_TEMPERATURE_VALUE  ((short)0x8000)
#define INVALID_PRESSURE_VALUE  ((unsigned short)0xFFFF)

struct EBusCommand
{
  EBusCommand * m_next_cmd;
  uint32 m_byteIndex;      // start byte# (to find position in buffer)
  uint8  m_reqData[14];  // request raw bytes
  uint8  m_respData[12]; // response decoded data bytes
  uint8  m_reqSize;      // request size (raw bytes)
  uint8  m_respLen;      // expected response size (decoded bytes)
  uint8  m_count;        // byte counter (sync skip/request/response)
  uint8  m_state;        // CmdInit, etc
  uint8  m_respCrc;      // running CRC of reply bytes
  uint8  m_retryCount;      // retry number (0 - first)

  enum
  {
    CmdInit    = 0,  // init state
    ReqSend    = 1,  // sending request bytes (init state)
    ReqWaitAck = 2,  // waiting for ACK for request
    RspStart   = 3,  // waiting for response length
    RspData    = 4,  // receiving response data
    RspEsc     = 5,  // received 0xA9, waiting for escaped byte
    RspAck     = 6,  // crc ok, sent ACK on response, waiting for loopback
    CmdSuccess = 7,  // final success state
    RspNak     = 8,  // crc error, sent NAK on response, waiting for loopback
    CmdError   = 9,  // final error state
  };

  EBusCommand() : m_next_cmd(0)
  {
    Clear();
  }

  void Clear()
  {
    m_byteIndex = 0;
    m_state = CmdInit;
    m_reqSize = 0;
    m_respLen = 0;
    m_respCrc = 0;
    m_count = 0;
    m_retryCount = 0;
  }

  // put request byte
  void ReqPut(uint8 b)
  {
    if (m_reqSize > sizeof(m_reqData)-2)
      return;
    if (b == 0xAA || b == 0xA9)
    {
      m_reqData[m_reqSize++] = 0xA9;
      m_reqData[m_reqSize++] = (b == 0xA9) ? 0 : 1;
    }
    else
    {
      m_reqData[m_reqSize++] = b;
    }
  }

  void ReqFinish()
  {
    ReqPut(Crc8Buf(m_reqData,m_reqSize));
  }

  bool IsSuccess() const
  {
    return m_state == CmdSuccess;
  }

  // Prepare GetParm command (B5 09 03 0D ADDR_LO ADDR_HI)
  void PrepGetParm(unsigned int addrFrom, unsigned int addrTo, unsigned int parmNo, unsigned int respLen)
  {
    Clear();
    ReqPut(addrFrom);
    ReqPut(addrTo);
    ReqPut(0xB5);  // B5 09 GetData
    ReqPut(0x09);
    ReqPut(0x03);  // data len
    ReqPut(0x0D);  // GetParm
    ReqPut((uint8)parmNo);
    ReqPut((uint8)(parmNo >> 8));
    ReqFinish();
    m_respLen = respLen;
  }

  void PrepGetState(unsigned int addrFrom, unsigned int addrTo)
  {
    PrepGetParm(addrFrom, addrTo, 0xAB, 1);
  }

  unsigned int GetStateValue() const
  {
    if (! IsSuccess())
      return INVALID_STATE_VALUE;
    return m_respData[0];
  }

  void PrepGetTemperature(unsigned int addrFrom, unsigned int addrTo)
  {
    PrepGetParm(addrFrom, addrTo, 0x18, 3);
  }

  // returns temperature (in 1/16 C)
  short GetTemperatureValue() const
  {
    if (! IsSuccess())
      return INVALID_TEMPERATURE_VALUE;
    return (short)((unsigned int)m_respData[1] << 8 | m_respData[0]);
  }

  void PrepGetPressure(unsigned int addrFrom, unsigned int addrTo)
  {
    PrepGetParm(addrFrom, addrTo, 0x02, 3);
  }

  unsigned short GetPressureValue() const
  {
    if (! IsSuccess())
      return INVALID_PRESSURE_VALUE;
    return (unsigned short)((unsigned int)m_respData[1] << 8 | m_respData[0]);
  }
};

volatile EBusCommand * g_activeCommand = 0;

bool ICACHE_RAM_ATTR RetryOrFail(volatile EBusCommand *cmd)
{
  if (cmd->m_retryCount >= CMD_MAX_RETRIES)
  {
    cmd->m_state = EBusCommand::CmdError;
    g_activeCommand = cmd->m_next_cmd;
    return false;
  }
  // restart command at random interval
  uint32 r = RANDOM_REG32;
  
  cmd->m_state = EBusCommand::CmdInit;
  cmd->m_byteIndex = 0;
  cmd->m_respCrc = 0;
  cmd->m_retryCount++;
  cmd->m_count = CMD_MIN_SYNC_SKIP + (r % (CMD_MAX_SYNC_SKIP-CMD_MIN_SYNC_SKIP));
  return true;
}

// Send byte to EBus
void ICACHE_RAM_ATTR SendChar(uint8 ch)
{
  WRITE_PERI_REG(UART_FIFO(UART_EBUS), ch);
}

void ICACHE_RAM_ATTR ProcessReceive(int st)
{
  uint32 t = millis();

  volatile EBusCommand *cmd = g_activeCommand;
 
  if (st < 0) // recv error
  {
    g_recvErrCnt++;
    g_nonSyncData++;
    if (cmd != 0)
    {
      if (cmd->m_state != EBusCommand::CmdInit)
      {
        RetryOrFail(cmd);
      }
    }
    return;
  }
 
  uint8 ch = (uint8)st;
  PUT_SERIAL_BYTE(ch);
  if (ch == 0xAA)
  {
    if (g_nonSyncData == 0)
      g_lastSyncTime = t;
    g_nonSyncData = 0;
  }
  else
  {
    g_nonSyncData++;
  }
    
  if (cmd == 0)
    return;

  switch(cmd->m_state)
  {
  case EBusCommand::CmdInit:
    if (ch != 0xAA) // not a sync char, skip
      break;
    // sync char received
    if (cmd->m_count != 0)
    {
      // skip given (random) number of syncs before restart
      cmd->m_count--;
      break;
    }
    // start processing command
    cmd->m_byteIndex = g_serialByteCount;
    if (cmd->m_reqSize == 0)
    {
      RetryOrFail(cmd);
      break;
    }
    else
    {
      cmd->m_state = EBusCommand::ReqSend;
      cmd->m_count = 0;  // m_count is used as request byte counter
      SendChar(cmd->m_reqData[0]);
    }
    break;

  case EBusCommand::ReqSend:
    if (cmd->m_count >= cmd->m_reqSize)
    {
      RetryOrFail(cmd);
      break;
    }
    if (cmd->m_reqData[cmd->m_count] != ch)
    {
      RetryOrFail(cmd);
      break;
    }
    cmd->m_count++;
    if (cmd->m_count == cmd->m_reqSize)
      cmd->m_state = EBusCommand::ReqWaitAck;
    else
      SendChar(cmd->m_reqData[cmd->m_count]);
    break;

  case EBusCommand::ReqWaitAck:
    if (ch == 0) // ACK
    {
      cmd->m_state = EBusCommand::RspStart;
    }
    else
    {
      RetryOrFail(cmd);
    }
    break;

  case EBusCommand::RspStart: // receiving length (must match expected)
    if (ch == cmd->m_respLen)
    {
      cmd->m_respCrc = Crc8Byte(ch, 0);
      cmd->m_state = EBusCommand::RspData;
      cmd->m_count = 0;  // m_count used as response decoded byte counter
    }
    else
    {
      RetryOrFail(cmd);
    }
    break;

  case EBusCommand::RspData:
  case EBusCommand::RspEsc:
    if (cmd->m_count < cmd->m_respLen)  // excluding CRC byte(s)
      cmd->m_respCrc = Crc8Byte(ch, cmd->m_respCrc);

    if (cmd->m_state == EBusCommand::RspData)
    {
      if (ch == 0xA9)
      {
        cmd->m_state = EBusCommand::RspEsc;
        break;
      }
    }
    else
    {
      if (ch == 0x00)
      {
          ch = 0xA9;
      }
      else if (ch == 0x01)
      {
          ch = 0xAA;
      }
      else
      {
        RetryOrFail(cmd);
        break;
      }
      cmd->m_state = EBusCommand::RspData;
    }
    if (cmd->m_count == cmd->m_respLen)
    {
      // check CRC
      if (ch == cmd->m_respCrc)
      {
        cmd->m_state = EBusCommand::RspAck;
        SendChar(0x00); // send ACK
      }
      else
      {
        cmd->m_state = EBusCommand::RspNak;
        SendChar(0xFF); // send NAK
      }
      break;
    }
    cmd->m_respData[cmd->m_count] = ch;
    cmd->m_count++;
    break;

  case EBusCommand::RspAck:
    if (ch == 0x00)
    {
      g_activeCommand->m_state = EBusCommand::CmdSuccess;
      g_activeCommand = g_activeCommand->m_next_cmd;
      SendChar(0xAA); // send sync
      break;
    }
    // fall-through
  case EBusCommand::RspNak:
    RetryOrFail(cmd);
    break;

  default:
    g_activeCommand->m_state = EBusCommand::CmdError;
    break;
  }
}

/////////////////////////////////////////////////////////////
// Serial interrupt handler

#define CHECK_INT_STATUS(ST, MASK) (((ST) & (MASK)) == (MASK))

void ICACHE_RAM_ATTR uart_int_handler(void *va)
{
  for (;;)
  {
    const uint32 uartIntStatus = READ_PERI_REG(UART_INT_ST(UART_EBUS));
    if (uartIntStatus == 0)
      break;

    if (CHECK_INT_STATUS(uartIntStatus, UART_FRM_ERR_INT_ST))
    {
      ProcessReceive(RecvError);     
      WRITE_PERI_REG(UART_INT_CLR(UART_EBUS), UART_FRM_ERR_INT_CLR); 
    }
    else if (CHECK_INT_STATUS(uartIntStatus, UART_BRK_DET_INT_ST))
    {
      ProcessReceive(RecvError);     
      WRITE_PERI_REG(UART_INT_CLR(UART_EBUS), UART_BRK_DET_INT_CLR); 
    }
    else if (CHECK_INT_STATUS(uartIntStatus, UART_RXFIFO_FULL_INT_ST))
    {
      const uint32 fifoLen = (READ_PERI_REG(UART_STATUS(UART_EBUS)) >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT;
      for (uint32 i = 0; i < fifoLen; ++i)
      {
        const uint8 ch = READ_PERI_REG(UART_FIFO(UART_EBUS));
        ProcessReceive(ch);
      }
      WRITE_PERI_REG(UART_INT_CLR(UART_EBUS), UART_RXFIFO_FULL_INT_CLR); 
    }
    else if (CHECK_INT_STATUS(uartIntStatus, UART_RXFIFO_TOUT_INT_ST))
    {
      const uint32 fifoLen = (READ_PERI_REG(UART_STATUS(UART_EBUS)) >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT;
      for (uint32 i = 0; i < fifoLen; ++i)
      {
        const uint8 ch = READ_PERI_REG(UART_FIFO(UART_EBUS));
        ProcessReceive(ch);
      }
      ProcessReceive(RecvTimeout);
      WRITE_PERI_REG(UART_INT_CLR(UART_EBUS), UART_RXFIFO_TOUT_INT_CLR); 
    }
    else if (CHECK_INT_STATUS(uartIntStatus, UART_TXFIFO_EMPTY_INT_ST))
    {
      WRITE_PERI_REG(UART_INT_CLR(UART_EBUS), UART_TXFIFO_EMPTY_INT_CLR);
    }
    else
    {
      ProcessReceive(RecvUnknown);
    }
  }
}

void SetupEBus()
{
  Serial.begin(2400);

  // Purge receive buffer
  while(Serial.available())
    Serial.read();

  // UART CONF1 (other config bits are set to 0)
  const uint32 conf1 = (1 << UART_RXFIFO_FULL_THRHD_S); // | (0x78 << UART_RX_TOUT_THRHD_S) | UART_RX_TOUT_EN;

  WRITE_PERI_REG(UART_CONF1(UART0), conf1);
  ETS_UART_INTR_ATTACH(uart_int_handler, 0);

  WRITE_PERI_REG(UART_INT_ENA(UART0), UART_RXFIFO_FULL_INT_ENA|UART_FRM_ERR_INT_ENA|UART_BRK_DET_INT_ENA);
  ETS_UART_INTR_ENABLE();
}

/////////////////////////////////////////////////////////////
// Periodic monitoring procedure

struct ParmHistData
{
  uint32 m_timeStamp;
  uint32 m_byteIndex;
  short  m_temperature;
  unsigned short m_pressure;
  uint8  m_state;
  uint8  m_retries;
  uint16 m_savedBytes;   // saved byte count (for last failed commands)
};

//
// Indication state transition table:
//
// NO_SYNC -> (sync ok) -> FIRST_POLL -> (no sync) -> NO_SYNC
// FIRST_POLL -> (cmd ok) -> CMD_SUCC
// FIRST_POLL -> (cmd err) -> CMD_FAIL
// CMD_SUCC -> (cmd err) -> CMD_FAIL -> (cmd ok) -> CMD_SUCC
// CMD_FAIL -> (no sync) -> NO_SYNC
// 
enum
{
  STATE_NO_SYNC    = 0,
  STATE_FIRST_POLL = 1,
  STATE_CMD_SUCC   = 2,
  STATE_CMD_FAIL   = 3
};

enum
{
  COMMAND_TIMEOUT  = 1000,  // command timeout (time to wait before declaring command failed)
  SYNC_TIMEOUT     = 2000,  // sync timeout (time to wait before entering NO_SYNC state)

  FIRST_POLL_DELAY = CONFIG_FIRST_POLL_DELAY*1000,    // Delay before first EBus poll on startup
  NORM_POLL_PERIOD = CONFIG_POLL_INTERVAL_NORM*1000,  // EBus poll interval when last command succeeded
  FAIL_POLL_PERIOD = CONFIG_POLL_INTERVAL_FAIL*1000,  // EBus poll interval when last command failed 
};

ParmHistData g_parmHistory[MAX_PARM_HISTORY];
uint32 g_parmHistorySize = 0; // total size (must % MAX_PARM_HISTORY to get index)

ParmHistData g_lastFailedCommands[FAILED_COMMAND_HISTORY_SIZE];
uint32 g_failedCommandCount = 0; // number of last failed commands (must % FAILED_COMMAND_HISTORY_SIZE to get index)

// failed command buffer
uint8  g_failedCommandBytes[FAILED_COMMAND_BYTES_MAX * FAILED_COMMAND_HISTORY_SIZE];

// Appends received data to last failed command data buffer (up to FAILED_COMMAND_BYTES_MAX)
void SaveLastFailedCommandBytes()
{
  if (g_failedCommandCount == 0)
    return;

  uint32 cmdIdx = (g_failedCommandCount-1) % FAILED_COMMAND_HISTORY_SIZE;
  ParmHistData& cmd = g_lastFailedCommands[cmdIdx];
  if (cmd.m_savedBytes == FAILED_COMMAND_BYTES_MAX)
    return; // data buffer already filled

  if ((millis() - cmd.m_timeStamp) > FAILED_COMMAND_TIME_WINDOW)
    return;

  uint32 byteIndex = cmd.m_byteIndex;
  if (byteIndex >= FAILED_COMMAND_BYTES_BEFORE)
    byteIndex -= FAILED_COMMAND_BYTES_BEFORE;
  else
    byteIndex = 0;

  uint32 currentByteCount = g_serialByteCount;
  if (byteIndex > currentByteCount)
    return;  // currentByteCount overflows, this case is not supported for simplicity
  uint32 availBytes = currentByteCount - byteIndex;
  if (availBytes > SERIAL_BUFFER_SIZE/2) // data too old, buffer may be overwritten
    return;
  if (availBytes > FAILED_COMMAND_BYTES_MAX)
    availBytes = FAILED_COMMAND_BYTES_MAX;

  uint8 *cmdBytes = g_failedCommandBytes + (cmdIdx * FAILED_COMMAND_BYTES_MAX);
  while (cmd.m_savedBytes < availBytes)
  {
    cmdBytes[cmd.m_savedBytes] = g_serialBuffer[(byteIndex + cmd.m_savedBytes) % SERIAL_BUFFER_SIZE];
    cmd.m_savedBytes++;
  }
}

void SaveLastFailedCommand()
{
  if (g_parmHistorySize == 0)
    return;

  uint32 cmdIdx = g_failedCommandCount % FAILED_COMMAND_HISTORY_SIZE;
  // save last command to failed command list
  const ParmHistData& lastParmData = g_parmHistory[(g_parmHistorySize - 1) % MAX_PARM_HISTORY];
  g_lastFailedCommands[cmdIdx] = lastParmData;
  g_lastFailedCommands[cmdIdx].m_savedBytes = 0;
  g_failedCommandCount++;

  SaveLastFailedCommandBytes();
}

uint32 g_monitorState = STATE_NO_SYNC;

uint32 g_lastMonitorTime = 0;
uint32 g_lastSuccessTime = 0;
uint32 g_firstPollTime   = 0;      // first poll start time

uint32 g_monCmdActive = false;   // command active, wait MAX_CMD_DELAY after g_lastMonitorTime
uint32 g_syncOk = false;         // synchronization is OK
uint32 g_lastCmdSucceed = false; // last command succeeded

uint32 g_requestData = false;    // Flag: request data immediately

// EBus commands processed by interrupt handler
EBusCommand g_monGetState;
EBusCommand g_monGetTemp;
EBusCommand g_monGetPress;

// Narodmon.ru data upload
uint32 g_lastResultValid = false;
uint32 g_lastResultSent = false;
uint32 g_lastState = 0;
uint32 g_lastTempValue = 0;
uint32 g_lastPressValue = 0;

uint32 g_lastNarodMonTime = 0;

void SetupMonitor()
{
}

void ProcessMonitor()
{
  uint32 t = millis();
  bool needRequest = false;

  uint32 lastSyncTime = g_lastSyncTime;
  g_syncOk = (lastSyncTime != 0) && ((t - lastSyncTime) < SYNC_TIMEOUT);

  switch(g_monitorState)
  {
  case STATE_NO_SYNC:
    if (g_syncOk)
    {
      g_monitorState = STATE_FIRST_POLL;
      g_firstPollTime = t;
    }
    break;
  case STATE_FIRST_POLL:
  case STATE_CMD_FAIL:
    if (lastSyncTime != 0 && ! g_syncOk)
      g_monitorState = STATE_NO_SYNC;
    break;
  default:
    break;
  }

  if (g_monCmdActive)
  {
    if (g_activeCommand == 0 || t - g_lastMonitorTime > COMMAND_TIMEOUT)
    {
      // stop active monitor command, if any
      if (g_activeCommand == &g_monGetState || g_activeCommand == &g_monGetTemp || g_activeCommand == &g_monGetPress)
        g_activeCommand = 0;

      g_monCmdActive = false;

      // check last command result
      g_lastCmdSucceed = g_monGetState.IsSuccess() && g_monGetTemp.IsSuccess() && g_monGetPress.IsSuccess();

      ParmHistData& parmData = g_parmHistory[g_parmHistorySize % MAX_PARM_HISTORY];
      parmData.m_timeStamp = g_lastMonitorTime;
      parmData.m_byteIndex = g_monGetState.m_byteIndex;  // should use first command in chain
      parmData.m_temperature = g_monGetTemp.GetTemperatureValue();
      parmData.m_pressure = g_monGetPress.GetPressureValue();
      parmData.m_state = g_monGetState.GetStateValue();
      parmData.m_retries = g_monGetState.m_retryCount + g_monGetTemp.m_retryCount + g_monGetPress.m_retryCount;
      parmData.m_savedBytes = 0;
      g_parmHistorySize++;

      g_lastResultSent = false;
      g_lastState = parmData.m_state;

      if (parmData.m_temperature != INVALID_TEMPERATURE_VALUE && parmData.m_pressure != INVALID_PRESSURE_VALUE)
      {
        g_lastTempValue = parmData.m_temperature;
        g_lastPressValue = parmData.m_pressure;
        g_lastResultValid = true;
      }
      else
      {
        g_lastResultValid = false;
      }

      if (g_lastCmdSucceed)
      {
        g_lastSuccessTime = g_lastMonitorTime;
        g_monitorState = STATE_CMD_SUCC;
      }
      else
      {
        g_monitorState = g_syncOk ? STATE_CMD_FAIL : STATE_NO_SYNC;
        SaveLastFailedCommand();
      }
    }
  }
  else
  {
    switch(g_monitorState)
    {
    case STATE_FIRST_POLL:
      if (t - g_firstPollTime > FIRST_POLL_DELAY)
        needRequest = true;
      break;
    case STATE_CMD_SUCC:
      if (t - g_lastMonitorTime > NORM_POLL_PERIOD)
        needRequest = true;
      break;
    case STATE_CMD_FAIL:
      if (t - g_lastMonitorTime > FAIL_POLL_PERIOD)
        needRequest = true;
      break;
    default:
      break;
    }
  }

  if (g_requestData)
    needRequest = true;

  if (needRequest)
  {
    if (g_activeCommand == 0)
    {
      g_lastMonitorTime = t;
      g_monCmdActive = true;
      g_requestData = false;

      // Build EBus command chain
      g_monGetState.PrepGetState(EBUS_ADDR_HOST, EBUS_ADDR_BOILER);
      g_monGetTemp.PrepGetTemperature(EBUS_ADDR_HOST, EBUS_ADDR_BOILER);
      g_monGetPress.PrepGetPressure(EBUS_ADDR_HOST, EBUS_ADDR_BOILER);
      g_monGetState.m_next_cmd = &g_monGetTemp;
      g_monGetTemp.m_next_cmd = &g_monGetPress;
      // Enable command processing by interrupt handler
      g_activeCommand = &g_monGetState;
    }
  }

  SaveLastFailedCommandBytes();
}

/////////////////////////////////////////////////////////////
// Indication

uint32 g_ledState = 0; // bit mask

enum
{
  // Green LED [Power/WiFi]: On - connected, Blink - trying to connect
  GreenLed = 12,    // Wemos Mini: D6/MISO; Wemos D1: D12/MISO
  // Yellow LED [EBus sync]: On - EBus sync bytes (0xAA) detected, Off - not detected or mixed with garbage
  YellowLed = 13,   // Wemos Mini: D7/MOSI; Wemos D1: D11/MOSI
  // Red LED [Request/Error]: On - last request failed, Off - last request OK, Blink - request is being sent
  RedLed = 14       // Wemos Mini: D5/SCK;  Wemos D1: D13/SCK
};

enum
{
  WIFI_BLINK_PERIOD = 100,    // Green LED - blink when trying to connect WiFi
  COMMAND_BLINK_PERIOD = 50,  // Red LED - blink during EBus command processing
};

bool GetLed(int no)
{
  return (g_ledState & (1 << no)) != 0;
}

void SetLed(int no, bool onoff)
{
  const uint32 mask = 1 << no;
  if (onoff)
  {
    g_ledState |= mask;
    digitalWrite(no, LOW);
  }
  else
  {
    g_ledState &= ~mask;
    digitalWrite(no, HIGH);
  }
}

uint32 g_lastWifiBlinkTime = 0;

void SetWifiLedOn()
{
  if (! GetLed(GreenLed))
    SetLed(GreenLed, 1);
}

void SetWifiLedBlink()
{
    uint32 t = millis();
    uint32 dt = t - g_lastWifiBlinkTime;
    if (dt > WIFI_BLINK_PERIOD)
    {
      SetLed(GreenLed, !GetLed(GreenLed));
      g_lastWifiBlinkTime = t;
    }
}

uint32 g_lastErrBlinkTime = 0;

void SetErrLed(bool f)
{
  if (GetLed(RedLed) != f)
    SetLed(RedLed, f);
}

void SetErrLedBlink()
{
    uint32 t = millis();
    uint32 dt = t - g_lastErrBlinkTime;
    if (dt > COMMAND_BLINK_PERIOD)
    {
      SetLed(RedLed, !GetLed(RedLed));
      g_lastErrBlinkTime = t;
    }
}

void SetupIndication()
{
  g_ledState = 0;
  pinMode(GreenLed, OUTPUT);
  pinMode(YellowLed, OUTPUT);
  pinMode(RedLed, OUTPUT);
  // Blink all LEDs 3 times on startup (indication test)
  for (int i = 0; i < 3; ++i)
  {
    if (i != 0)
      delay(100);
    SetLed(GreenLed, true);
    SetLed(YellowLed, true);
    SetLed(RedLed, true);
    delay(100);
    SetLed(GreenLed, false);
    SetLed(YellowLed, false);
    SetLed(RedLed, false);
  }

  g_lastWifiBlinkTime = g_lastErrBlinkTime = millis();
}

void ProcessIndication()
{
  uint32 t = millis();

  if (WiFi.status() == WL_CONNECTED)
      SetWifiLedOn();
  else
      SetWifiLedBlink();
  
  SetLed(YellowLed, g_syncOk);

  if (g_activeCommand != 0)
    SetErrLedBlink();
  else
    SetErrLed(!g_lastCmdSucceed);
}

/////////////////////////////////////////////////////////////
// Web server

ESP8266WebServer g_webServer(80);

void webResponseBegin(String& resp, int refresh)
{
  resp.concat("<html><head>");
  if (refresh != 0)
  {
      resp.concat("<meta http-equiv=\'refresh\' content=\'");
      resp.concat(refresh);
      resp.concat("\'/>");
  }
  resp.concat("<title>BaiMon v " BAIMON_VERSION "</title>"
    "<style>"
      "body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }"
    "</style></head><body>");
}

void webResponseEnd(String& resp)
{
  resp.concat("</body></html>");
}

void webHandleRoot()
{
  uint32 t = millis();

  uint32 sec = t / 1000;
  uint32 min = sec / 60;
  uint32 hr = min / 60;  

  String resp;
  webResponseBegin(resp, 10);

  resp.concat("<h1>Boiler status</h1>");

  char tbuf[80];
  sprintf(tbuf, "%d days %02d:%02d:%02d", hr / 24, hr % 24, min % 60, sec % 60);
  resp.concat("<p>Uptime:");
  resp.concat(tbuf);

  resp.concat("<p>EBus sync: ");
  resp.concat(g_syncOk ? "<span style=\"color:green;\">OK</span>" : "<span style=\"color:red;\">FAIL</span>");

  resp.concat("<form action=\"/request-data\" method=\"POST\" enctype=\"application/x-www-form-urlencoded\">");
  resp.concat("<button type=\"submit\">Request data</button>");
  resp.concat("</form>");

  resp.concat("<hr>Measurement history:<br><table border=\"0\"><tr><th>Time</th><th>State</th><th>Temperature,C</th><th>Pressure,Bar</th><th>Retries</th></tr>");
  for (uint32 i = 0, cnt = (g_parmHistorySize > WEB_PARM_HISTORY) ? WEB_PARM_HISTORY : g_parmHistorySize; i != cnt; ++i)
  {
    const ParmHistData& parmData = g_parmHistory[(g_parmHistorySize - i - 1) % MAX_PARM_HISTORY];

    char stateBuf[20];
    const char *stateStr = stateBuf;
    if (parmData.m_state == INVALID_STATE_VALUE)
      stateStr = "<span style=\"color:red\">ERROR</span>";
    else
      sprintf(stateBuf, "S%02u", parmData.m_state);

    char tempBuf[20];
    const char *tempStr = tempBuf;
    if (parmData.m_temperature == INVALID_TEMPERATURE_VALUE)
      tempStr = "<span style=\"color:red\">ERROR</span>";
    else
      sprintf(tempBuf, "%d.%02u", parmData.m_temperature/16, (parmData.m_temperature & 0xF)*100/16);

    char pressBuf[20];
    const char *pressStr = pressBuf;
    if (parmData.m_pressure == INVALID_PRESSURE_VALUE)
      pressStr = "<span style=\"color:red\">ERROR</span>";
    else
      sprintf(pressBuf, "%d.%02u", parmData.m_pressure/1000, (parmData.m_pressure%1000)/10);

    uint32 dt = t - parmData.m_timeStamp;
    uint32 dtsec = dt / 1000;
    uint32 dtmin = dtsec / 60;
    uint32 dthr = dtmin / 60;  

    char buf[400];
    sprintf(buf, "<tr><td>-%02u:%02u:%02u</td><td>%s</td><td>%s</td><td>%s</td><td>%u</td></tr>",
      dthr, dtmin % 60, dtsec % 60, stateStr, tempStr, pressStr, parmData.m_retries);
    resp.concat(buf);
  }
  resp.concat("</table>");

  resp.concat("<hr><a href=\"/diag\">[Diagnostics]</a>");

  webResponseEnd(resp);

  g_webServer.send(200, "text/html", resp);
}

void webHandleRequestData()
{
  g_requestData = true;
  
  String resp;
  webResponseBegin(resp, 0);
  resp.concat("<h1>Requesting data</h1>");
  webResponseEnd(resp);

  g_webServer.sendHeader("Location", "/", false);
  g_webServer.send(302, "text/html", resp);
}

void webHandleDiag()
{
  uint32 byteCount = g_serialByteCount;
  uint32 byteStart = (byteCount < WEB_DIAG_DATA_SIZE) ? 0 : ((byteCount - WEB_DIAG_DATA_SIZE) & ~(0x20-1));

  uint32 t = millis();

  String resp;
  webResponseBegin(resp, 0);

  char tbuf[40];

  resp.concat("<h1>Diagnostics</h1>");
  resp.concat("MAC: ");
  resp.concat(WiFi.macAddress());
  resp.concat("<br>Total bytes received: ");
  sprintf(tbuf, "0x%08X", byteCount);
  resp.concat(tbuf);

  resp.concat(" errors: ");
  resp.concat(g_recvErrCnt);

  resp.concat("<hr>Last EBus data:<div style=\"font-family: monospace;\">");
  while (byteStart < byteCount)
  {
    uint32 portion = byteCount - byteStart;
    if (portion > 32)
      portion = 32;
    sprintf(tbuf, "%08X:", byteStart);
    resp.concat(tbuf);
    for (uint32 i = 0; i < portion; ++i)
    {
      uint8 ch = GET_SERIAL_BYTE_NO(byteStart+i);
      sprintf(tbuf, "%s%02X", ((i & 0x0F) == 0 ? "&nbsp;&nbsp;" : "&nbsp;"),  ch);
      resp.concat(tbuf);
    }
    byteStart += portion;
    resp.concat("<br>");
  }
  resp.concat("</div><hr><div>Failed command history:<br>");
  uint32 failedCmdCnt = FAILED_COMMAND_HISTORY_SIZE;
  if (g_failedCommandCount < failedCmdCnt)
    failedCmdCnt = g_failedCommandCount;

  for (uint32 n = 0; n < failedCmdCnt; ++n)
  {
    uint32 cmdIdx = (g_failedCommandCount - n - 1) % FAILED_COMMAND_HISTORY_SIZE;
    const ParmHistData& cmd = g_lastFailedCommands[cmdIdx];
    const uint8 *cmdBytes = g_failedCommandBytes + cmdIdx * FAILED_COMMAND_BYTES_MAX;

    uint32 dt = t - cmd.m_timeStamp;
    uint32 dtsec = dt / 1000;
    uint32 dtmin = dtsec / 60;
    uint32 dthr = dtmin / 60;  

    char buf[200];
    sprintf(buf, "<div>#%u: -%02u:%02u:%02u [%u bytes] [pre: %u]</div>", n+1, dthr, dtmin % 60, dtsec % 60, cmd.m_savedBytes, FAILED_COMMAND_BYTES_BEFORE);
    resp.concat(buf);
    resp.concat("<div style=\"font-family: monospace;\">");
    uint32 bc = 0;
    while (bc < cmd.m_savedBytes)
    {
      uint32 portion = cmd.m_savedBytes - bc;
      if (portion > 32)
        portion = 32;
      sprintf(tbuf, "%08X:", bc);
      resp.concat(tbuf);
      for (uint32 i = 0; i < portion; ++i)
      {
        uint8 ch = cmdBytes[bc + i];
        sprintf(tbuf, "%s%02X", ((i & 0x0F) == 0 ? "&nbsp;&nbsp;" : "&nbsp;"),  ch);
        resp.concat(tbuf);
      }
      bc += portion;
      resp.concat("<br>");
    }
    resp.concat("</div>");
  }

  resp.concat("</div><hr><a href=\"/\">[Main]</a>");

  webResponseEnd(resp);
  g_webServer.send(200, "text/html", resp);
}

void webHandleNotFound()
{
  String message = "404 Not Found\n\n";
  message += "URI: ";
  message += g_webServer.uri();
  message += "\nMethod: ";
  message += (g_webServer.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += g_webServer.args();
  message += "\n";
  for (uint8_t i=0; i<g_webServer.args(); i++){
    message += " " + g_webServer.argName(i) + ": " + g_webServer.arg(i) + "\n";
  }
  g_webServer.send(404, "text/plain", message);
}

void SetupWebServer()
{
  g_webServer.on("/", webHandleRoot);
  g_webServer.on("/request-data", webHandleRequestData);
  g_webServer.on("/diag", webHandleDiag);
  g_webServer.onNotFound(webHandleNotFound);
  g_webServer.begin();
}

void ProcessWebServer()
{
  g_webServer.handleClient();
}

/////////////////////////////////////////////////////////////
// Sending data to narodmon.ru

enum
{
  NarodMonMinDelay = CONFIG_NARODMON_START_DELAY*1000,   // Delay before first send after startup
  NarodMonInterval = CONFIG_NARODMON_SEND_INTERVAL*1000  // Regular data sending interval
};

void ProcessNarodmon()
{
#if CONFIG_NARODMON_ENABLE

  if (g_syncOk && g_lastResultSent)
    return;

  uint32 t = millis();

  // first time interval
  if (g_lastNarodMonTime == 0)
  {
    if (t < NarodMonMinDelay)
      return;
  }
  else
  {
    if ((t - g_lastNarodMonTime) < NarodMonInterval)
      return;
  }

  WiFiClient sock;
  if (! sock.connect("narodmon.ru", 8283))
    return;

  char databuf[200];
  if (g_syncOk && g_lastResultValid)
  {
    // send state, temperature and pressure
    sprintf(databuf, "#%s\n#S1#%d\n#T1#%d.%02u\n#P1#%d.%02u\n##\n",
      CONFIG_NARODMON_DEVICE_ID, g_lastState,
      g_lastTempValue/16, (g_lastTempValue & 0xF)*100/16,
      g_lastPressValue/1000, (g_lastPressValue % 1000)/10
    );
  }
  else
  {
    // send only state (255 if EBus is not available)
    const uint32 state = g_syncOk ? g_lastState : 255;
    sprintf(databuf, "#%s\n#S1#%d\n##\n",
      CONFIG_NARODMON_DEVICE_ID, state
    );
  }

  const size_t sz = strlen(databuf);
  sock.write((const uint8 *)databuf, sz);

  g_lastResultSent = true;
  g_lastNarodMonTime = t;

#endif // CONFIG_NARODMON_ENABLE
}

/////////////////////////////////////////////////////////////
// main

void setup()
{
  Serial.begin(2400);

  SetupIndication();
  SetupWifi();
  SetupEBus();
  SetupWebServer();
  SetupMonitor();
}

void loop()
{
  ProcessMonitor();
  ProcessIndication();
  ProcessNarodmon();
  ProcessWebServer();
  delay(10);
}
