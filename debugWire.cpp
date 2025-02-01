// debugWire.cpp 

// Based on https://github.com/dcwbrown/dwire-debug/tree/master
//  other dWire code https://github.com/dwtk/dwtk/blob/master/debugwire/flash.go  


// TODO: measure 0x55 bit times with ~7.5X baud rate:   see BaudTest.cpp
//   adjust to get 50% 0x80, 50% 0xC0 -- > 1 start bit + 6.5 data bits --> baud * 7.5 
//   16 MHz / 128 * 7.5 = 937500 baud
//  
// TODO: try finer baud rate settings
  // PL2303HX: 12 MHz * 32 / prescale / {1..255}  > 115200
     // https://elixir.bootlin.com/linux/v3.10/source/drivers/usb/serial/pl2303.c#L364
  // CP2104  24 MHz / N 
  // FTDI 3 MHz / (n + 0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875; n = 2..16384)
     // 3M, 2M, 24 MHz / N for N >= 16
     // Special cases n = 0 -> 3 MBaud; n = 1 -> 2 MBaud; Sub-integer divisors between 0 and 2 not allowed.
  // CP2102: 24 MHz / N >= 8 from 32 entry programmable table   1 MHz max?
	// CH340:  12 MHz / {1, 2, 8, 16, 64, 128, 512, 1024} / {2..256 or 9..256 when prescale = 1}
    // stop bit >= 2us
    // https://github.com/nospam2000/ch341-baudrate-calculation

// Use ATTiny85 as serial bridge; 96 MHz PCK timer to measure bit times

// TODO: cleanup

#define COM_PORT "COM22"

#define _CRT_SECURE_NO_WARNINGS

#include <Windows.h>
#include <cstdint>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <conio.h>
#include <time.h>

#define windows
#define FileHandle FILE*
#define Assert(a) if (!(a)) printf("Assert! %s", #a);
#define Fail(msg) printf(msg)

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint16_t u32;
typedef int64_t s64;
typedef uint64_t u64;

int F_CPU[] = { 24000000, 16000000, 18432000, /* 1000000 */}; // or 128 * freqOutDWire() --> counter
const int NumCpuHz = sizeof(F_CPU) / sizeof(F_CPU[0]);
int cpuHzIdx = 0;
int dWdivisor = 128; // default

const int Log2BaudDivisor = 7; // 7 default, less to speed up (3 min --> 3 MHz)

HANDLE hCom;

void openSerial() {
	hCom = CreateFileA("\\\\.\\" COM_PORT, GENERIC_WRITE | GENERIC_READ, 0, NULL, OPEN_EXISTING, NULL, NULL);
	if (hCom == INVALID_HANDLE_VALUE) exit(-2);

	DCB dcb = { 0 };
	dcb.DCBlength = sizeof(DCB);
	dcb.BaudRate = F_CPU[cpuHzIdx] / dWdivisor;      // TODO: increase by changing divisor
	dcb.ByteSize = 8;
  dcb.StopBits = 0; // +1) /2
	dcb.fBinary = TRUE;
	if (!SetCommState(hCom, &dcb)) exit(-3);

	COMMTIMEOUTS timeouts = { 0 };  // in ms
	timeouts.ReadIntervalTimeout = 1; // between characters
	timeouts.ReadTotalTimeoutMultiplier = 1; // * num requested chars
	timeouts.ReadTotalTimeoutConstant = 1; // + this = total timeout
	if (!SetCommTimeouts(hCom, &timeouts)) exit(-4);
}

int baudRate;

bool setBaudRate(int baud) {
  baudRate = baud;

  // TODO: try other serial chips
  switch (baud) {
    // CP2102 configured aliases for F_CPU: 1600000, 24000000, and 18432000 / 2^N; only guaranteed to work up to 1M baud
    case  1000000: baud = 1053258; break; 
		case  2880000: baud = 256000; break;

// > 3M baud don't work
    case 24000000: baud = 76800; break;
		case 12000000: baud = 64000; break;
    case  8000000: baud = 56000; break; 
		case  6000000: baud = 51200; break;
    case  4608000: baud = 28800; break;  // off 4.17% --> bad

		case  3000000: baud = 16000; break;

		case  2304000: baud = 14400; break;   // off 4.17% --> bad
		case  1500000: baud =  7200; break;
		case  1152000: baud =  4000; break;      
		case   750000: baud =  1800; break;
  }

  DCB dcb = { 0 };
	dcb.DCBlength = sizeof(DCB);
	dcb.BaudRate = baud;
	dcb.ByteSize = 8;
	dcb.fBinary = TRUE;
	return SetCommState(hCom, &dcb);
}

bool commError;

void commErrs(bool breakExpected) {
  while (1) {
    DWORD commErrors;
	  ClearCommError(hCom, &commErrors, NULL);
		commError |= commErrors != 0;
    switch (commErrors) {
      case 0 : return;
      case 0x08 : 
        printf("Frame\n"); 
        break;
      case 0x10 : 
      case 0x18 :
        if (breakExpected) 
					commError = false; 
        else 
          printf("Break\n");
        break;
      default : printf("CommErr %X ", commErrors); break;
    }
    Sleep(1);    
  }
}

int rxRdy() {
	COMSTAT cs;
	DWORD commErrors;
	if (!ClearCommError(hCom, &commErrors, &cs)) 
    return -1;
	return cs.cbInQue;
}

void SerialWrite(const u8 *bytes, int length) {
  WriteFile(hCom, bytes, length, NULL, NULL);
}

int SerialRead(u8 *buf, int len) {
  int totalRead = 0;
  do {
    DWORD lengthRead;
    ReadFile(hCom, buf + totalRead, len - totalRead, &lengthRead, NULL);
    if (lengthRead == 0) {
      printf("\nSerialRead expected %d bytes, got %d\n", len, totalRead); 
      if (totalRead) {
        printf(":\n  ");
        for (int i = 0; i < totalRead; i++)
          printf("%2X ", buf[i]);
      }
      break;
    }
    totalRead += lengthRead;
  } while(totalRead < len);
  return totalRead;
}

void rxFlush() {
	commErrs(true);  // TODO: break sometimes expected

  while (1) {  
    if (!rxRdy()) return;
    u8 byte;
    ReadFile(hCom, &byte, 1, NULL, NULL);
    printf("%X! ", byte); // extra
    Sleep(1);
  }
}

int SerialReadByte() {
  commErrs(false);
  u8 byte = 0;
  DWORD length;
  bool result = ReadFile(hCom, &byte, 1, &length, NULL);
  if (!result || length != 1) {
    printf("-");
    return 0xDE;
  }
  return byte;
}

#if 0
void SerialSendBytes(const u8 *out, int outlen) {
  for (int b = 0; b < outlen; ++b) {
    rxFlush(b);
    WriteFile(hCom, out + b, 1, NULL, NULL);
    u8 echoed = SerialReadByte();
    if (echoed != *(out + b)) {
      printf("\nEcho %2X != %2X sent  ", echoed, *(out+b));
    }
  }
}

#else

int echoErrors;

void SerialSendBytes(const u8 *out, int outlen) {
  rxFlush();
  WriteFile(hCom, out, outlen, NULL, NULL);
  FlushFileBuffers(hCom);
  Sleep(1);
  // Since TxD & RxD share the same wire, everything transmitted echos in the receive buffer (unless there is a collision). 
  // Drain echoed input and check that it is correct.
  u8 echoed[256];
  int totalRead = SerialRead(echoed, outlen);
  for (int i = 0; i < min(outlen, totalRead); i++) { 
    if (echoed[i] != out[i]) {
			printf("Echo: %2X != %2X sent\n", echoed[i], out[i]);
      if (++echoErrors > 10) {
        Sleep(5000);
        exit(3);
      }
    }
  }
}
#endif

u8   SerialOutBufBytes[256];
int  SerialOutBufLength = 0;

void txFlush() {
  if (SerialOutBufLength > 0) {
    SerialSendBytes(SerialOutBufBytes, SerialOutBufLength);
    SerialOutBufLength = 0;
  }
}

u8 syncBytes[256];
u8 syncIdx;

bool SerialSync() {  // after break, reset, single step, ...
  commErrs(true); 
  txFlush();
  Sleep(18);
	bool got55 = false;

  syncIdx = 0;
  while (1) {
    commErrs(true);
    DWORD gotBytes = 0;
    if (ReadFile(hCom, &syncBytes[syncIdx], 1, &gotBytes, NULL) && gotBytes) {
      got55 |= syncBytes[syncIdx] == 0x55;
      ++syncIdx;
    } else break;
  }
	return got55;
 }

bool SerialBreak() { // > 9 bit times at baud rate

#if 0
	int savedBaudRate = baudRate;
	setBaudRate(38400);  // lower baud rate for break
  u8 null = 0;
  WriteFile(hCom, &null, 1, NULL, NULL);
  ReadFile(hCom, &null, 1, NULL, NULL);
	setBaudRate(savedBaudRate); 
#else
  #ifdef windows
    SetCommBreak(hCom);
    // usleep(1000000 * (9 + 1) * 128 / F_CPU[cpuHzIdx]);
    ClearCommBreak(hCom);
  #else
    ioctl(port, TCFLSH, TCIOFLUSH);
    ioctl(port, TIOCSBRK);
    usleep(1000000 * (9 + 1) * 128 / F_CPU[cpuHzIdx]);
    ioctl(port, TIOCCBRK);
  #endif
#endif

  return SerialSync();
}


// Buffer accumulating debugWIRE data to be sent to the device to minimize
// the number of USB transactions used,

void SerialSend(const u8 *out, int outlen) {
#if 0
  SerialSendBytes(out, outlen); return;  // no buffer
#endif
  while (SerialOutBufLength + outlen >= sizeof(SerialOutBufBytes)) {
    // Total (buffered and passed here) meets or exceeds SerialOutBuf size.
    // Send buffered and new data until there remains between 0 and 127
    // bytes still to send in the buffer.
    int lenToCopy = sizeof(SerialOutBufBytes)-SerialOutBufLength;
    memcpy(SerialOutBufBytes+SerialOutBufLength, out, lenToCopy);
    SerialSendBytes(SerialOutBufBytes, sizeof(SerialOutBufBytes));
    SerialOutBufLength = 0;
    out += lenToCopy;
    outlen -= lenToCopy;
  }
  Assert(SerialOutBufLength + outlen <= sizeof(SerialOutBufBytes));
  memcpy(SerialOutBufBytes+SerialOutBufLength, out, outlen);
  SerialOutBufLength += outlen;
  // Remainder stays in buffer to be sent with next read request, or on a txFlush call.
}


int SerialReceive(u8 *in, int inlen) {
  txFlush(); // output
  SerialRead(in, inlen);
  return inlen;
}

int deviceType = 10; // TODO: search for signature

struct Characteristic {  // Device specific characteristics
  const char *name;
  int   signature;
  int   ioregSize;
  int   sramSize;
  int   eepromSize;
  int   flashSize;  // In bytes
  int   DWDR;       // DebugWIRE data register, aka MONDR - Monitor data register
  int   pageSize;   // In bytes
  int   boot;       // Lowest PC value giving boot section access
  int   bootflags;  // Where to find the boot sectore control flags, if any
  int   EECR;       // EEPROM control register index. EEDR and EEARL always follow directly.
  int   EEARH;      // EEPROM address high (doesn't exist on all devices)
} Characteristics[] = {
//  name             sig   io  sram eeprom flash  dwdr   pg  boot    bf eecr  eearh
  {"ATtiny13",    0x9007,  64,   64,   64,  1024, 0x2E,  32, 0x0000, 0, 0x1C, 0x00},
  {"ATtiny2313",  0x910a,  64,  128,  128,  2048, 0x1f,  32, 0x0000, 0, 0x1C, 0x00},

  {"ATtiny25",    0x9108,  64,  128,  128,  2048, 0x22,  32, 0x0000, 0, 0x1C, 0x1F},
  {"ATtiny24",    0x910B,  64,  128,  128,  2048, 0x27,  32, 0x0000, 0, 0x1C, 0x1F},

  {"ATmega48A",   0x9205, 224,  512,  256,  4096, 0x31,  64, 0x0000, 0, 0x1F, 0x22},
  {"ATtiny45",    0x9206,  64,  256,  256,  4096, 0x22,  64, 0x0000, 0, 0x1C, 0x1F},
  {"ATtiny44",    0x9207,  64,  256,  256,  4096, 0x27,  64, 0x0000, 0, 0x1C, 0x1F},
  {"ATmega48PA",  0x920A, 224,  512,  256,  4096, 0x31,  64, 0x0000, 0, 0x1F, 0x22},
  {"ATtiny441",   0x9215, 224,  256,  256,  4096, 0x27,  16, 0x0000, 0, 0x1C, 0x1F},

  {"ATmega88A",   0x930A, 224, 1024,  512,  8192, 0x31,  64, 0x0F80, 1, 0x1F, 0x22},
  {"ATtiny85",    0x930B,  64,  512,  512,  8192, 0x22,  64, 0x0000, 0, 0x1C, 0x1F},
  {"ATtiny84",    0x930C,  64,  512,  512,  8192, 0x27,  64, 0x0000, 0, 0x1C, 0x1F},
  {"ATmega88PA",  0x930F, 224, 1024,  512,  8192, 0x31,  64, 0x0F80, 1, 0x1F, 0x22},
  {"ATtiny841",   0x9315, 224,  512,  512,  8192, 0x27,  16, 0x0000, 0, 0x1C, 0x1F},
  {"ATmega8U2",   0x9389, 224,  512,  512,  8192, 0x31,  64, 0x0000, 0, 0x1F, 0x22},

  {"ATmega168A",  0x9406, 224, 1024,  512, 16384, 0x31, 128, 0x1F80, 1, 0x1F, 0x22},
  {"ATmega168PA", 0x940B, 224, 1024,  512, 16384, 0x31, 128, 0x1F80, 1, 0x1F, 0x22},
  {"ATmega16U2",  0x9489, 224,  512,  512, 16384, 0x31, 128, 0x0000, 0, 0x1F, 0x22},

  {"ATmega328P",  0x950F, 224, 2048, 1024, 32768, 0x31, 128, 0x3F00, 2, 0x1F, 0x22},
  {"ATmega328",   0x9514, 224, 2048, 1024, 32768, 0x31, 128, 0x3F00, 2, 0x1F, 0x22},
  {"ATmega32U2",  0x958A, 224, 1024, 1024, 32768, 0x31, 128, 0x0000, 0, 0x1F, 0x22},
  {0,                  0,   0,    0,    0,     0,    0,   0,      0, 0,    0}
};

struct Characteristic *CurrentCharacteristics() {
  if (deviceType < 0) printf("No device connected\n");
  return Characteristics + deviceType;
}
const char *Name(void) {return CurrentCharacteristics()->name;}
int  IoregSize(void)   {return CurrentCharacteristics()->ioregSize;}
int  SramSize(void)    {return CurrentCharacteristics()->sramSize;}
int  EepromSize(void)  {return CurrentCharacteristics()->eepromSize;}
int  FlashSize(void)   {return CurrentCharacteristics()->flashSize;}   // In bytes
int  PageSize(void)    {return CurrentCharacteristics()->pageSize;}    // In bytes
int  DWDRreg(void)     {return CurrentCharacteristics()->DWDR;}
int  DWDRaddr(void)    {return CurrentCharacteristics()->DWDR + 0x20;} // IO regs come after the 32 regs r0-r31
int  DataLimit(void)   {return 32 + IoregSize() + SramSize();}
int  BootSect(void)    {return CurrentCharacteristics()->boot;}
int  BootFlags(void)   {return CurrentCharacteristics()->bootflags;} // 1 = in ext fuse, 2 = in high fuse
int  EECR(void)        {return CurrentCharacteristics()->EECR;}
int  EEDR(void)        {return EECR()+1;}
int  EEARL(void)       {return EECR()+2;}
int  EEARH(void)       {return CurrentCharacteristics()->EEARH;}
u8  SPMCSR(void)       {return 0x37;} // SPMCSR is at the same address on all devices
u8  AddrFlag(void)     {return (FlashSize() < 8192) ? 0x10 : 0;} // Flag to include when setting PC or BP high byte

enum {MaxFlashPageSize = 128, MaxFlashSize = 32768, MaxSRamSize = 2048};

void DwSend(const u8 out) {
  SerialSend(&out, 1);
}

void DwSend(const u8 *out, int outlen) {
  SerialSend(out, outlen);
}

int  DwReceive(u8 *in, int inlen) {
  return SerialReceive(in, inlen);
}

void setBaud(int log2divisor) { // 3..7
	// Note that programming flash takes 9ms erase + 4.5ms for 32 words > 70us/byte
  //   so higher baud rate most useful for flash / SRAM readout check

  if (log2divisor < 4)
    DwSend(0xA3 - log2divisor);
  else DwSend(0x80 + log2divisor - 4); // set ATtiny baud
  dWdivisor = 1 << log2divisor;    // minimum 8 for FTDI (3 MHz max)
  txFlush(); // checks echo

  setBaudRate(F_CPU[cpuHzIdx] / dWdivisor);
  // DwSend(0x55);  // echoes FF or 0 - WHY??  TODO
  const u8 Sync = 0x55;
	WriteFile(hCom, &Sync, 1, NULL, NULL);
  if (!SerialSync())
    printf("sB: no 55\n");
  Sleep(1);
	commErrs(true);
}

void DwSync(void) {
  SerialSync();
}

int DwReadByte(void) {u8 byte = 0; DwReceive(&byte, 1); return byte;}
int DwReadWord(void) {u8 buf[2] = {0}; DwReceive(buf, 2); return (buf[0] << 8) | buf[1];}

u8 hi(int w) {return (w>>8) & 0xff;}
u8 lo(int w) {return (w   ) & 0xff;}


void DwSetPC(u16 pc) {
  u8 SetPC[] = {0xD0, hi(pc) | AddrFlag(), lo(pc)};
  DwSend(SetPC, sizeof SetPC);
}

u16 DwGetPC() {
  DwSend(0xF0);
  return DwReadWord();
}

void DwSetBP(u16 bp) {
  u8 SetBP[] = {0xD1, hi(bp) | AddrFlag(), lo(bp)};
  DwSend(SetBP, sizeof SetBP);
}

void DwInst(u16 inst) {
  u8 instr[] = {0x64, 0xD2, hi(inst), lo(inst), 0x23};
  DwSend(instr, sizeof instr);
}

void DwIn(u8 reg, u16 ioreg)  {DwInst(0xB000 | ((ioreg << 5) & 0x600) | ((reg << 4) & 0x01F0) | (ioreg & 0x000F));}
void DwOut(u16 ioreg, u8 reg) {DwInst(0xB800 | ((ioreg << 5) & 0x600) | ((reg << 4) & 0x01F0) | (ioreg & 0x000F));}

void DwGetRegs(int first, u8 *regs, int count) {
  if (count == 1) {
    DwOut(DWDRreg(), first);
  } else {
    DwSend(0x66);
    DwSetPC(first);
    DwSetBP(first + count);
    const u8 StartRegRead[] = {0xC2, 1, 0x20};
    DwSend(StartRegRead, sizeof StartRegRead);
  }
  DwReceive(regs, count);
}

void DwSetReg(int reg, u8 val) {DwIn(reg, DWDRreg()); DwSend(val);}

void DwSetRegs(int first, const u8 *regs, int count) {
  if (count <= 3) {
    DwSend(0x64); // Set single step loaded instruction mode
    while (count > 0) {DwSetReg(first, *regs); first++; regs++; count--;}
  } else {
    DwSend(0x66);
    DwSetPC(first);
    DwSetBP(first + count);
    const u8 StartRegWrite[] = {0xC2, 5, 0x20};
    DwSend(StartRegWrite, sizeof StartRegWrite);
    DwSend(regs, count);
  }
}

void DwSetZ(u16 z) {DwSetRegs(30, (u8*)&z, 2);}

enum {SPMEN=1, PGERS=3, PGWRT=5, BLBSET = 9, RWWSRE=0x11, SIGRD = 0x21};

u8 ReadSPMCSR(void) {
  u8 spmcsr;
  DwSend(0x64);        // Set up for single step mode
  DwIn(30, SPMCSR());         // in r30,SPMCSR
  DwGetRegs(30, &spmcsr, 1);  // spmcsr := r30
  //printf(" SPMCSR $"); Wx(spmcsr,2); Wsl(".");
  return spmcsr;
}


void RenableRWW(void) {
  if (BootSect()) {
    DwSetPC(BootSect());  // Set PC that allows access to all of flash
    DwSetReg(29, RWWSRE); // r29 := RWWSRE
    DwOut(SPMCSR(), 29);  // out SPMCSR,r29
    DwInst(0x95E8);       // spm
  }
}


void DwReadFlash(int addr, int len, u8 *buf) {
  int limit = addr + len;
  if (limit > FlashSize())
    printf("Attempted read beyond end of flash\n");
  while (addr < limit) {
    int length = min(limit-addr, 64);      // Read no more than 64 bytes at a time so PC remains in valid address space.
    DwSetZ(addr);                          // Z := First address to read
    DwSetPC(BootSect());                   // Set PC that allows access to all of flash
    DwSetBP(BootSect()+2*length);          // Set BP to load length bytes
    const u8 StartFlashRead[] = {0x66, 0xC2, 2, 0x20};
    DwSend(StartFlashRead, sizeof StartFlashRead);
    DwReceive(buf, length);
    addr += length;
    buf  += length;
  }
}

const u8 SpmBreak[] = {0xD2, 0x95, 0xE8, 0x33};

void EraseFlashPage(u16 a) { // a = byte address of first word of page
  Assert((a & (PageSize()-1)) == 0);

  u8 PageErase[] = {PGERS, lo(a), hi(a)};
  DwSetRegs(29, PageErase, sizeof PageErase); // r29 := op (erase page), Z = first byte address of page
  DwSetPC(BootSect());                       // Set PC that allows access to all of flash
  DwSend(0x64);                              // Set up for single step mode
  DwOut(SPMCSR(), 29);                       // out SPMCSR,r29 (select page erase)
  DwSend(SpmBreak, sizeof SpmBreak); // SPM
  DwSync();
}


void LoadPageBuffer(u16 a, const u8 *buf) {
  u8 WriteNextPageBufferWord[] = {SPMEN, lo(a), hi(a)};
  DwSetRegs(29, WriteNextPageBufferWord, sizeof WriteNextPageBufferWord); // r29 := op (write next page buffer word), Z = first byte address of page
  DwSend(0x64);                       // Set up for single step mode
  const u8 *limit = buf + PageSize();
  while (buf < limit) {
    DwSetRegs(0, buf, 2); buf += 2;  // r0 := low byte, r1 := high byte
    DwSetPC(BootSect());             // Set PC that allows access to all of flash
    DwOut(SPMCSR(), 29);             // out SPMCSR,r29 (write next page buffer word)
    DwInst(0x95E8);                  // spm
    DwInst(0x9632);                  // adiw Z,2
  }
}

void ProgramPage(u16 a) {
  DwSend(0x66);
  u8 PageWrite[] = {PGWRT, lo(a), hi(a)};
  DwSetRegs(29, PageWrite, sizeof PageWrite); // r29 = op (page write), Z = first byte address of page
  DwSetPC(BootSect());                       // Set PC that allows access to all of flash
  DwSend(0x64);                              // Set up for single step mode
  DwOut(SPMCSR(), 29);                       // out SPMCSR,r29 (PGWRT)
  if (BootSect()) {
    DwInst(0x95E8);                          // spm
    // while ((ReadSPMCSR() & 0x1F) != 0) printf("."); // Wait while programming busy
  } else {    
    DwSend(SpmBreak, sizeof SpmBreak);   // spm and break
  }
  DwSync();
}


void ShowPageStatus(u16 a, const char* msg) {
  printf("Page %4X %s  \r", a, msg);
}


void WriteFlashPage(u16 a, const u8 *buf) {
  // Uses r0, r1, r29, r30, r31

  u8 page[MaxFlashPageSize];
  Assert(PageSize() <= sizeof(page));

  RenableRWW();
  DwReadFlash(a, PageSize(), page);

  if (memcmp(buf, page, PageSize()) == 0) {
    ShowPageStatus(a, "unchanged");
    return;
  }

  bool erase = false;
  for (int i = 0; i < PageSize(); i++) 
    if (~page[i] & buf[i]) {
      erase= true; 
      break;
    }

  if (erase) {
    ShowPageStatus(a, "erasing");
    EraseFlashPage(a);
  }

  memset(page, 0xff, PageSize());
  if (memcmp(buf, page, PageSize()) == 0) 
		return; // blank page

  ShowPageStatus(a, "load buffer");
  LoadPageBuffer(a, buf);

  ShowPageStatus(a, "program");
  ProgramPage(a);
  ShowPageStatus(a, "programmed \n");

  RenableRWW();
}

u8 pageBuffer[MaxFlashPageSize];

void WriteFlash(u16 addr, const u8 *buf, int length) {
  Assert(addr + length <= FlashSize());
  Assert(length >= 0);
  if (length == 0) return;

  u8 R[32];
  DwGetRegs(0, R, 2); // Cache R0 and R1

  int pageOffsetMask = PageSize()-1;
  int pageBaseMask   = ~ pageOffsetMask;

  if (addr & pageOffsetMask) { // buf starts in the middle of a page
    int partBase   = addr & pageBaseMask;
    int partOffset = addr & pageOffsetMask;
    int partLength = min(PageSize()-partOffset, length);

    DwReadFlash(partBase, PageSize(), pageBuffer);
    memcpy(pageBuffer+partOffset, buf, partLength);
    WriteFlashPage(partBase, pageBuffer);

    addr   += partLength;
    buf    += partLength;
    length -= partLength;
  }

  Assert(length == 0  ||  ((addr & pageOffsetMask) == 0));
  while (length >= PageSize()) { // Write whole pages
    WriteFlashPage(addr, buf);
    addr   += PageSize();
    buf    += PageSize();
    length -= PageSize();
  }

  if (length) { // Write any remaining partial page
    Assert(length > 0);
    Assert(length < PageSize());
    Assert((addr & pageOffsetMask) == 0);
    DwReadFlash(addr, PageSize(), pageBuffer);
    memcpy(pageBuffer, buf, length);
    WriteFlashPage(addr, pageBuffer);
  }
 
  DwSetRegs(0, R, 2); // Restore cached registers R0 and R1
  printf("\n");
}

u16 getSignature() {  
  DwSend(0xF3);  // F3..FF = read signature;  (just start bit)
  txFlush();

  u16 sig = DwReadWord();
  if (sig != 0x930B) 
    printf("Sig %X\n", sig);
  return sig;
}

void reset() {
  DwSend(7); // reset 
  DwSend(0);
  DwSend(0x55);
  SerialSync();  // ? TODO baud??
}



u8 flashBuffer[MaxFlashSize];

void LoadBinary(HANDLE CurrentFile) {
  DWORD length;
  ReadFile(CurrentFile, flashBuffer, sizeof(flashBuffer), &length, NULL);
  if (length == 0) printf("File is empty.");

  printf("Loading %d bytes from binary file.", length);
  WriteFlash(0, flashBuffer, length);
}


FILE* hexFile;

u8 chkSum;
int pair;

u8 GetByte() { // convert hex to binary:  sed -e 's/://'  -e 's/../\\x&/g')"
  u8 data = 0;

  while (1) {
    int ch = fgetc(hexFile);
    if (ch == EOF) 
      exit(5);
    if (ch == ':')
      continue;
    if ((ch -= '0') < 0) 
      continue;
    if (ch > 9) 
      ch -= 'A' - '0' - 10; // hex to dec

    data <<= 4;
    data |= ch;
    if (!(++pair % 2)) {
      chkSum += data;
      return data;
    }
  }
}

int maxAddr;

bool loadHex() {
  memset(flashBuffer, 0xFF, sizeof flashBuffer);
  maxAddr = chkSum = pair = 0; 
  
  while (1) { // process records from .hex file,
    u8 len = GetByte();  
    u16 addr = ((u16)GetByte() << 8) + GetByte();
    if (addr + len > maxAddr)
      maxAddr = addr + len;
    u8 recType = GetByte();
    if (recType)  // 1 = end of file
      return true;

    u8* pBuf = flashBuffer + addr;
    while (len--) *pBuf++ = GetByte();  // data

    GetByte(); // checkSum - should sum back to 0
    if (chkSum) { 
      printf("Checksum error @ %X\n", addr);  
      return false;
    }            
  }   
}

void stepTo(u16 stopAt, int numSteps = 100) { 
  do {
    DwSend(0x5A); // single step context
		DwSend(0x31); // single step
    SerialSync();

    u16 PC = DwGetPC() - 1; // PC is ahead by 1
    DwSetPC(PC);
    printf("%X ", PC);
		if (PC == stopAt) 
			break;
	} while (--numSteps);
}

void step() {
  stepTo(0, 1);
}

bool checkSignature() {
  commError = false; 
  while (getSignature() != 0x930B) { //  || commError) {  // TODO: better baud rate test ********
		commError = false; 
    //++cpuHzIdx;
    //cpuHzIdx %= NumCpuHz;
    setBaudRate(F_CPU[cpuHzIdx] / dWdivisor);
		printf("CpuHz %d\n", cpuHzIdx);
    return true;
  }
  return false;
}

int main(int argc, char* argv[]) {
  if (argc > 1) {
		char hexPath[256];
		sprintf(hexPath, "C:/Users/Admin/Documents/Atmel Studio/7.0/%s/%s/Debug/%s.hex", argv[1], argv[1], argv[1]);
    hexFile = fopen(hexPath, "r");
    if (!hexFile) {
      printf("Can't open %s\n", hexPath);
      return 1;
    }
  } else hexFile = stdin;
  loadHex();

#if 1 // force page write for baud test
  time_t secs; time(&secs);
	maxAddr += 2 * PageSize(); // beyond executable code
  flashBuffer[maxAddr - (secs & (PageSize() - 1))] = 0xFF - (1 << ((secs >> 6) & 7));  // one bit low
#endif

  openSerial();
  rxFlush();

	SerialBreak(); // connect to target

  for (int i = 100; --i;)
    checkSignature();
     
	setBaud(Log2BaudDivisor); // TODO: fails flashing?  timing???
  if (getSignature() != 0x930B)
    _getch();

  txFlush();
  rxFlush();

  WriteFlash(0, flashBuffer, maxAddr);

  DwSetPC(0);
  DwSend(0x40); // Timers enabled, Breakpoint disabled
  DwSend(0x30); // Go
  txFlush();

	Sleep(8000);  // to see results

	return 0;
}
