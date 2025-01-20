// debugWire.cpp 

// Based on https://github.com/dcwbrown/dwire-debug/tree/master
//     other dWire code https://github.com/dwtk/dwtk/blob/master/debugwire/flash.go  

// TODO: fix GO
// TODO: cleanup

#define F_CPU (126000 * 128) // from freqOutDWire() -> counter;  or 18432000  
   // TODO: auto-adjust to 0x55 break response

#define COM_PORT "COM16" // CP2102 debugWire-1

#include <Windows.h>
#include <cstdint>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define windows
#define FileHandle FILE*
#define Assert(a) if (!(a)) printf("Assert! %s", a);
#define Fail(msg) printf(msg)

typedef uint8_t u8;
typedef uint16_t u16;
typedef int64_t s64;
typedef uint64_t u64;

HANDLE hCom;

void openSerial() {
	hCom = CreateFileA("\\\\.\\" COM_PORT, GENERIC_WRITE | GENERIC_READ, 0, NULL, OPEN_EXISTING, NULL, NULL);
	if (hCom == INVALID_HANDLE_VALUE) exit(-2);

	DCB dcb = { 0 };
	dcb.DCBlength = sizeof(DCB);
	dcb.BaudRate = F_CPU / 128;      // TODO: increase by changing divisor
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

bool setBaudRate(int baud) {
  DCB dcb = { 0 };
	dcb.DCBlength = sizeof(DCB);
	dcb.BaudRate = baud;
	dcb.ByteSize = 8;
	dcb.fBinary = TRUE;
	return SetCommState(hCom, &dcb);
}

void commErrs(bool breakExpected) {
  while (1) {
    DWORD commErrors;
	  ClearCommError(hCom, &commErrors, NULL);
    switch (commErrors) {
      case 0 : return;
      case 0x08 : printf("Frame\n"); break;
      case 0x10 : 
        if (!breakExpected) 
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

// replace with printfs:

#define Wflush()

void Wc(char c) { printf("%c", c);}
void Wd(s64 i, int w) { printf("%d", i);}
void Ws(const char *s) { printf("%s", s);}
void Wr(void) { Wc('\r');}
void Wl(void) { Ws("\r\n");}
void Wsl(const char *s) {Ws(s); Wl();}
void Wx(u64 i, int w) { printf("%X", i);}

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
      if (totalRead) {Ws(":\n  "); for (int i=0; i<totalRead; i++) {Wx(buf[i], 2); Ws(" ");}}
      break;
    }
    totalRead += lengthRead;
  } while(totalRead < len);
  return totalRead;
}

void rxFlush(int index) {
  commErrs(false);
  if (!rxRdy()) return;

  u8 byte = 0xDE;
  DWORD length = 0;
  bool extras = false;
  while (1) {
    ReadFile(hCom, &byte, 1, &length, NULL);
    if (!length) 
      return;
    printf("%X! ", byte); // extra
    if (!extras) {
      extras = true;
      printf("@ %d  ", index);
    }
    Sleep(1);
  }
}

int SerialReadByte() {
  commErrs(false);
  u8 byte = 0;
  DWORD length;
  bool result = ReadFile(hCom, &byte, 1, &length, NULL);
  if (!result || length != 1) {
    Ws("-");
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
  rxFlush(0);
  WriteFile(hCom, out, outlen, NULL, NULL);
  // Since TxD & RxD share the same wire, everything transmitted echos in the receive buffer (unless there is a collision). 
  // Drain echoed input and check that it is correct.
  u8 echoed[256];
  int totalRead = SerialRead(echoed, outlen);
  for (int i = 0; i < min(outlen, totalRead); i++) { 
    if (echoed[i] != out[i]) {
      Ws("Echo "); Wd(i+1,1); Ws(" of "); Wd(outlen,1);
      Ws(": Sent "); Wx(out[i],2); Ws(" got "); Wx(echoed[i],2); Wl();
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

void SerialSync() {  // after break, reset, single step, ...
  txFlush();

  int wait = 3;
  u8 byte = 0;
  while (1) {
		commErrs(true);
    switch (byte = SerialReadByte()) {
      case 0 :  // Eat 0x00 bytes generated by line at break (0v)
      case 0xFF : // Eat 0xFF bytes generated by line going high following break.
        continue;

      case 0x55 : 
        Sleep(16);
        rxFlush(0x55);
        return;

      case 0xDE : // nothing yet wait
        if (--wait) {
          Sleep(16);
          continue;
        } // else error -- fall thru to default
     
      default : 
        printf("Got %X vs. 55 expected\n", byte);
        rxFlush(-1);
        return;
    }
  }
}

void SerialBreak() { // 9 bit times at baud rate < 1ms

#ifdef windows
  SetCommBreak(hCom);
  // usleep(1000000 * (9 + 1) * 128 / F_CPU);
  ClearCommBreak(hCom);
#else
  ioctl(port, TCFLSH, TCIOFLUSH);
  ioctl(port, TIOCSBRK);
  usleep(1000000 * (9 + 1) * 128 / F_CPU);
  ioctl(port, TIOCCBRK);
#endif
  SerialSync();
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
  if (deviceType < 0) Fail("No device connected");
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
int  SPMCSR(void)      {return 0x37;} // SPMCSR is at the same address on all devices
int  AddrFlag(void)    {return (FlashSize() < 8192) ? 0x10 : 0;} // Flag to include when setting PC or BP high byte

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
  //Ws(" SPMCSR $"); Wx(spmcsr,2); Wsl(".");
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
  if (limit > FlashSize()) {Fail("Attempt to read beyond end of flash.");}
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


void EraseFlashPage(u16 a) { // a = byte address of first word of page
  Assert((a & (PageSize()-1)) == 0);

  u8 PageErase[] = {PGERS, lo(a), hi(a)};
  DwSetRegs(29, PageErase, sizeof PageErase); // r29 := op (erase page), Z = first byte address of page
  DwSetPC(BootSect());                       // Set PC that allows access to all of flash
  DwSend(0x64);                              // Set up for single step mode
  DwOut(SPMCSR(), 29);                       // out SPMCSR,r29 (select page erase)
  const u8 DoPageErase[] = {0xD2, 0x95, 0xE8, 0x33};
  DwSend(DoPageErase, sizeof DoPageErase); // SPM
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
  DwSend(0x64);                       // Set up for single step mode
  DwOut(SPMCSR(), 29);                       // out SPMCSR,r29 (PGWRT)
  if (BootSect()) {
    DwInst(0x95E8);                          // spm
    while ((ReadSPMCSR() & 0x1F) != 0) {Wc('.'); Wflush();} // Wait while programming busy
  } else {
    const u8 SpmBreak[] = {0xD2, 0x95, 0xE8, 0x33};
    DwSend(SpmBreak, sizeof SpmBreak);   // spm and break
    DwSync();
  }
}


void ShowPageStatus(u16 a, const char *msg) {
  Ws("$"); Wx(a,4); Ws(" - $"); Wx(a+PageSize()-1,4);
  Wc(' '); Ws(msg); Ws(".                "); Wr();
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

  int erase = 0;
  for (int i=0; i<PageSize(); i++) 
    if (~page[i] & buf[i]) {erase=1; break;}

  if (erase) {
    ShowPageStatus(a, "erasing");
    EraseFlashPage(a);
  }

  memset(page, 0xff, PageSize());
  if (memcmp(buf, page, PageSize()) == 0) 
    return;

  ShowPageStatus(a, "loading page buffer");
  LoadPageBuffer(a, buf);

  ShowPageStatus(a, "programming");
  ProgramPage(a);

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
  DwSend(0xF3);  // read signature
  u16 sig = DwReadWord();
  printf("Signature: %X\n", sig);
  return sig;
}

void reset() {
  DwSend(7); // reset 
  DwSend(0);
  DwSend(0x55);
  SerialSync();
}

void setBaud() { // OK
  DwSend(0x83); // set baud, expect 0x55
  setBaudRate(F_CPU / 128);  
  SerialSync();
}

void setFastBaud() {  // TODO: debug ************
#if 1
  DwSend(0xA0); // set baud, expect 0x55
  txFlush();
  setBaudRate(F_CPU / 8); // CP2102 table 2 MHz
#else
  DwSend(0xA1); // set baud, expect 0x55
  txFlush();
  setBaudRate(F_CPU / 4);
#endif
  // need finer control of timing to work right   TODO
  // SerialSync(); // too late to catch 0x55 **** 
	commErrs(true);
}

u8 FlashBuffer[MaxFlashSize];

void LoadBinary(HANDLE CurrentFile) {
  DWORD length;
  ReadFile(CurrentFile, FlashBuffer, sizeof(FlashBuffer), &length, NULL);
  if (length == 0) Ws("File is empty.");

  Ws("Loading "); Wd(length,1); Wsl(" flash bytes from binary image file.");
  WriteFlash(0, FlashBuffer, length);
}


u8 chkSum;
int pair;

u8 GetByte() { // convert hex to binary:  sed -e 's/://'  -e 's/../\\x&/g')"
  u8 data = 0;

  while (1) {
    int ch = getchar(); // from stdin
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
  memset(FlashBuffer, 0xFF, sizeof FlashBuffer);
  maxAddr = chkSum = pair = 0; 
  
  while (1) { // process records from .hex file,
    u8 len = GetByte();  
    u16 addr = ((u16)GetByte() << 8) + GetByte();
    if (addr + len > maxAddr)
      maxAddr = addr + len;
    u8 recType = GetByte();
    if (recType)  // 1 = end of file
      return true;

    u8* pBuf = FlashBuffer + addr;
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

int main(int argc, char* argv[]) {
  loadHex(); // from stdin

  openSerial();
  rxFlush(-2);

  SerialBreak(); // connect 

  setFastBaud();
  getSignature();

  WriteFlash(0, FlashBuffer, maxAddr);

  DwSetPC(0);
  DwSend(0x40); // Timers enabled, Breakpoint disabled
  DwSend(0x30); // Go
  txFlush();

	Sleep(5000);  // to see results

	return 0;
}



