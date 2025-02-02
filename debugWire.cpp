// debugWire routines

// Based on https://github.com/dcwbrown/dwire-debug/tree/master
//  other dWire code https://github.com/dwtk/dwtk/blob/master/debugwire/flash.go  

// TODO: too fast baud --> LoadPageBuffer echo errors (USB buffers??)


// Could calibrate OSCCAL and write value to EEPROM
//   FTDI 24MHz clock +/-0.17% or can use 12MHz xtal, FTProg set option
//   ?slow vs. Osccalibrate
// 
// Or could use ATTiny w/ xtal to accurately measure break response 0x55 bit times
//   Use ATTiny85 as serial bridge; 96 MHz PCK timer to measure 0x55 bit times?

#define _CRT_SECURE_NO_WARNINGS

#include <windows.h>
#include <stdio.h>
#include <conio.h>
#include <time.h>

typedef unsigned char u8;
typedef unsigned short u16;

#define	COM_PORT "COM14"  // FTDI

 // auto-detected by adjustToDWireBaud()
int F_CPU = 16000000; // default
int dWdivisor = 128;  // default

HANDLE hCom;

void openSerial() {
	hCom = CreateFileA("\\\\.\\" COM_PORT, GENERIC_WRITE | GENERIC_READ, 0, NULL, OPEN_EXISTING, NULL, NULL);
	if (hCom == INVALID_HANDLE_VALUE) exit(-2);

	DCB dcb = { 0 };
	dcb.DCBlength = sizeof(DCB);
	dcb.BaudRate = F_CPU / dWdivisor; 
	dcb.ByteSize = 8;
  dcb.StopBits = 0; // +1) /2
	dcb.fBinary = TRUE;
	if (!SetCommState(hCom, &dcb)) exit(-3);

	COMMTIMEOUTS timeouts = { 0 };  // in ms
	timeouts.ReadIntervalTimeout = 1; // between characters
	timeouts.ReadTotalTimeoutMultiplier = 1; // * num requested chars
	timeouts.ReadTotalTimeoutConstant = 1; // + this = total timeout
	if (!SetCommTimeouts(hCom, &timeouts)) exit(-4);

	if (!SetupComm(hCom, 1024, 1024)) printf("Can't SetupComm\n"); // Set size of I/O buffers (max 16384 on Win7)
}

const int SerialClockHz = 24000000; // for FTDI or CP2104
int currentBaud = F_CPU / dWdivisor;

bool setBaudRate(int baud) {
	// CP2104 24 MHz / N 
  // FTDI   3MHz, 2MHz, then 24 MHz / N for N >= 16
	// PL2303HX: 12 MHz * 32 / prescale / {1..255} > 115200
     // https://elixir.bootlin.com/linux/v3.10/source/drivers/usb/serial/pl2303.c#L364
  // CP2102: 24 MHz / N >= 8 from 32 entry programmable table   1 MHz max?
	// CH340:  12 MHz / {1, 2, 8, 16, 64, 128, 512, 1024} / {2..256 or 9..256 when prescale = 1}
    // https://github.com/nospam2000/ch341-baudrate-calculation
	currentBaud = baud;
  DCB dcb = { 0 };
	dcb.DCBlength = sizeof(DCB);
	dcb.BaudRate = baud;
	dcb.ByteSize = 8;
	dcb.fBinary = TRUE;
	return SetCommState(hCom, &dcb);
}

DWORD commErrs(bool breakExpected) {
	DWORD commErrors;
	ClearCommError(hCom, &commErrors, NULL);
	switch (commErrors) {
		case CE_FRAME: printf("Frame\n");	break;
		case CE_BREAK:
		case CE_BREAK | CE_FRAME:
			if (breakExpected)
				commErrors = 0;
			else
				printf("Break\n");
			break;
		case 0: return 0;
		default: printf("CommErr %X ", commErrors); break;
	}
	return commErrors;
}

int rxRdy() {
	COMSTAT cs;
	ClearCommError(hCom, NULL, &cs);
	return cs.cbInQue;
}

void rxFlush() {
  u8 sync[256];
	DWORD gotBytes;
  while (rxRdy()) 
		ReadFile(hCom, &sync, sizeof sync, &gotBytes, NULL);
}

void usleep(unsigned long usecs) {
    HANDLE timer = CreateWaitableTimer(NULL, TRUE, NULL);
    LARGE_INTEGER dueTime;
    dueTime.QuadPart = -((long long)usecs * 10); // 100 nanoseconds intervals

    SetWaitableTimer(timer, &dueTime, 0, NULL, NULL, FALSE);
    WaitForSingleObject(timer, INFINITE);
    CloseHandle(timer);
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
u8   AddrFlag(void)    {return (FlashSize() < 8192) ? 0x10 : 0;} // Flag to include when setting PC or BP high byte

enum {MaxFlashPageSize = 128, MaxFlashSize = 32768, MaxSRamSize = 2048};

#define hi(w) ((w) >> 8)
#define lo(w)  (w & 0xff)
#define littleEnd(w) lo(w), hi(w)
#define DwSetPC(pc) 0xD0, AddrFlag() | hi(pc), lo(pc)
#define DwSetBP(bp) 0xD1, AddrFlag() | lo(bp), lo(bp)
#define DwInst(inst) 0x64, 0xD2, hi(inst), lo(inst), 0x23
#define DwSlowInst(inst) 0xD2, hi(inst), lo(inst), 0x33
#define DwIn(reg, ioreg)  DwInst(0xB000 | ((ioreg << 5) & 0x600) | ((reg << 4) & 0x01F0) | (ioreg & 0x000F))
#define DwOut(ioreg, reg) DwInst(0xB800 | ((ioreg << 5) & 0x600) | ((reg << 4) & 0x01F0) | (ioreg & 0x000F))
#define DwSetReg(reg, val) DwIn(reg, DWDRreg()), (val)
#define DwGetReg(reg) DwOut(DWDRreg(), reg)
#define DwSetRegs(first, count)  0x66, DwSetPC(first), DwSetBP(first + count), 0xC2, 5, 0x20 // followed by count bytes of data
#define DwSetIoreg(reg, val) DwSetReg(0, val), DwOut(reg, 0)
#define DwSetZ(z) DwSetReg(30, lo(z)), DwSetReg(31, hi(z))   // Z :: r31:r30

bool DwSend(const u8* cmds, int len) {
  rxFlush();

	WriteFile(hCom, cmds, len, NULL, NULL);
	FlushFileBuffers(hCom);
	usleep(len * 12 * 1000000 / currentBaud);

	u8 echo[256];
	DWORD gotBytes;
	ReadFile(hCom, echo, len, &gotBytes, NULL);
	int echoErr = memcmp(cmds, echo, len);
	if (echoErr)
		printf("Echo! ");
	return !echoErr;
}

void DwSend(const u8 cmd) {
	DwSend(&cmd, 1);
}

const int NoRecv = -1;

int DwRecv() {
	u8 val;
	DWORD gotBytes;
	ReadFile(hCom, &val, 1, &gotBytes, NULL);
	if (gotBytes) return val;

	usleep(11 * 1000000 / currentBaud); 
  ReadFile(hCom, &val, 1, &gotBytes, NULL);
	if (gotBytes) return val;

	return NoRecv;
}

u16 DwReadWord(void) {return DwRecv() << 8 | DwRecv();}  // Big endian

bool DwSync() {
	int wait = 20;
	while (1) {
		commErrs(true);
		int recv = DwRecv();
		switch (recv) {
		  case -1:
			case 0:
			case 0xFF:
				break;
			case 0x55: return true;
			default:
				printf("%X ", recv);
				break;
		}
		Sleep(5);
		if (!wait--) {
			printf("No55! ");
			return false;
		}
	}
}

void dwSetPC(u16 pc) {
  u8 SetPC[] = {DwSetPC(pc)};
  DwSend(SetPC, sizeof SetPC);
}

u16 DwGetPC() {
  DwSend(0xF0);
  return DwReadWord();
}

void stepTo(u16 stopAt, int numSteps = 100) { 
  do {
    DwSend(0x5A); // single step context
		DwSend(0x31); // single step
    DwSync();

    u16 PC = DwGetPC() - 1; // PC is ahead by 1
    dwSetPC(PC);
    printf("%X ", PC);
		if (PC == stopAt) 
			break;
	} while (--numSteps);
}

void step() {
  stepTo(0, 1);
}

#define SPMCSR 0x37
#define SPM 0x95E8
enum {SPMEN=1, PGERS=3, PGWRT=5, BLBSET = 9, RWWSRE=0x11, SIGRD = 0x21};

u8 ReadSPMCSR(void) {
	rxFlush();
  u8 readSPMCSR[] = {
    DwIn(2, SPMCSR),    // in r2,SPMCSR
    DwGetReg(2),              
  };
  DwSend(readSPMCSR, sizeof readSPMCSR);
	return DwRecv();
}

void storeProgMemOp(u8 op, u16 addr) {
	u8 spmOperation[] = {
		DwSetPC(BootSect()),                    // Set PC that allows access to all of flash
    DwSetRegs(29, 3), op, littleEnd(addr),  // r29 := op, Z (r31:30) = first byte address of page
    DwOut(SPMCSR, 29),                      // out SPMCSR,r29 (op)
    DwInst(SPM),
  };
  DwSend(spmOperation, sizeof spmOperation);  
	while (ReadSPMCSR() & op) printf("."); // Wait while busy;   TODO: should take 5 ms??
}

void RenableRWW(void) {
  if (BootSect()) 
		storeProgMemOp(RWWSRE, 0); 
}

void DwReadFlash(int addr, int len, u8 *buf) {
  int limit = addr + len;
  if (limit > FlashSize()) printf("Read past end of flash\n");

  while (addr < limit) {
    int length = min(limit - addr, 64);      // Read no more than 64 bytes at a time so PC remains in valid address space.
    u8 readFlash64[] = {
      DwSetZ(addr),                          // Z := First address to read
      DwSetPC(BootSect()),                   // Set PC that allows access to all of flash
      DwSetBP(BootSect() + 2 * length),      // Set BP to load length bytes (PC increments by 2s?)
      0x66, 0xC2, 2, 0x20
    };
    DwSend(readFlash64, sizeof readFlash64);
		Sleep(16);
    ReadFile(hCom, buf, length, NULL, NULL);
    addr += length;
    buf  += length;
  }
}


void LoadPageBuffer(u16 addr, const u8* buf) {
  u8 setPageAddr[] = { DwSetPC(BootSect()), DwSetRegs(29, 3), SPMEN, littleEnd(addr), };  // r29 := op (write next page buffer word), Z = first byte address of page
  DwSend(setPageAddr, sizeof setPageAddr);

	int sent = 0;
  const u8 *limit = buf + PageSize();
  while (buf < limit) {
    u8 loadBufWord[] = {
      DwSetReg(0, *buf++), // r0 := low byte,
			DwSetReg(1, *buf++), // r1 := high byte
      DwOut(SPMCSR, 29),   // out SPMCSR,r29 = SPMEN (write next page buffer word)
      DwInst(SPM),                  
      DwInst(0x9632),      // adiw Z,2
    };

	  sent += sizeof loadBufWord;
    if (!DwSend(loadBufWord, sizeof loadBufWord))   // echo messed up - WHY??? FTDI buffer?   TODO *************   debugWire OK: 256 byte chunks
			printf("@%d\n", sent);
  }
}

void EraseFlashPage(u16 addr) { storeProgMemOp(PGERS, addr); }

void ProgramPage(u16 addr)    { storeProgMemOp(PGWRT, addr); }

void ShowPageStatus(u16 addr, const char* msg) {
  printf("Page %4X %s  \r", addr, msg);
}

void WriteFlashPage(u16 addr, const u8 *buf) {
  // Uses r0, r1, r29, r30, r31
  u8 page[MaxFlashPageSize];

  RenableRWW();
  DwReadFlash(addr, PageSize(), page);

  if (memcmp(buf, page, PageSize()) == 0) {
    ShowPageStatus(addr, "unchanged");
    return;
  }

  bool erase = false;
  for (int i = 0; i < PageSize(); i++) 
    if (~page[i] & buf[i]) {
      erase= true; 
      break;
    }

  if (erase) {
    ShowPageStatus(addr, "erasing");
    EraseFlashPage(addr);
  }

  memset(page, 0xff, PageSize());
  if (memcmp(buf, page, PageSize()) == 0) 
		return; // blank page

  ShowPageStatus(addr, "load buf");
  LoadPageBuffer(addr, buf);

  ShowPageStatus(addr, "program");
  ProgramPage(addr);
  ShowPageStatus(addr, "programmed \n");

  RenableRWW();
}

u8 pageBuffer[MaxFlashPageSize];

void WriteFlash(u16 addr, const u8 *buf, int length) {
  if (length == 0) return;

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

  while (length >= PageSize()) { // Write whole pages
    WriteFlashPage(addr, buf);
    addr   += PageSize();
    buf    += PageSize();
    length -= PageSize();
  }

  if (length) { // Write any remaining partial page
    DwReadFlash(addr, PageSize(), pageBuffer);
    memcpy(pageBuffer, buf, length);
    WriteFlashPage(addr, pageBuffer);
  }
 
  printf("\n");
}


u8 DwReadEEPROM(int addr) {
  #define EERE 1  // EEprom ReadEnable

	u8 readEEPROMCmd[] = {
  	DwSetRegs(29, 3), EERE, littleEnd(addr),  // r29 := EERE, r31:r30 := address
		DwOut(EEARL(), 30),   // out  EEARL,r30
		DwOut(EEARH(), 31),   // out  EEARH,r31
		DwOut(EECR(), 29),    // out  EECR,EERE
		DwIn(0, EEDR()),
		DwGetReg(0)
	};
	DwSend(readEEPROMCmd, sizeof readEEPROMCmd);	
	return DwRecv();
}

void DwWriteEEPROM(int addr, u8 val) {
  #define EEMPE 4  // EEprom Master Program Enable 
  #define EEPE  2  // EEprom Program Enable 

  u8 writeEEPROMCmd[] = {
		DwSetRegs(28, 4), EEMPE, EEPE, littleEnd(addr),  // r28 := EEMPE, r29 := EEPE, r31:r30 := address
		DwOut(EEARL(), 30),   // out  EEARL,r30
		DwOut(EEARH(), 31),   // out  EEARH,r31
		DwSetReg(0, val),     // r0 := val
		DwOut(EEDR(), 0),     // out  EEDR,r0
		DwOut(EECR(), 28),    // out  EECR,EEMPE
		DwOut(EECR(), 29),    // out  EECR,EEPE   // not within 4 clocks but works OK in single step mode
	};
	DwSend(writeEEPROMCmd, sizeof writeEEPROMCmd);
  Sleep(5); // Allow eeprom write to complete.
}

void setOsccal(u8 val) {
  #define OSCCAL 0x31
	u8 setOsccalCmd[] = { DwSetIoreg(OSCCAL, val) };
	DwSend(setOsccalCmd, sizeof setOsccalCmd);
	Sleep(1);  // let oscillator settle
}

void setCLKPR(u8 log2Prescale) {  // no effect !!!
  #define CLKPR  0x26
  #define CLKPCE 0x80  // Clock Prescaler Change Enable
	u8 setClkpr[] = {
		DwSetReg(0, CLKPCE),
		DwSetReg(1, log2Prescale),
		DwOut(CLKPR, 0),
		DwOut(CLKPR, 1)   // ? not within 4 clocks --> no effect
	};
	DwSend(setClkpr, sizeof setClkpr);
	FlushFileBuffers(hCom);
}

int setDebugWireMode(int divisor = SerialClockHz / 128, bool print = false) { // returns approximate baud rate
  if (print) printf("%3d: ", divisor);

  setBaudRate(SerialClockHz / divisor);
	rxFlush();
  ClearCommError(hCom, NULL, NULL);

	SetCommBreak(hCom);
	//usleep(2); // >= 30 24 MHz clocks >= 1.25us lo to initially trigger dWire mode -- or > 10 bit times
  usleep(12 * 1000000 * divisor / SerialClockHz); // break bit times
	ClearCommBreak(hCom);
	// response is hi 6.6us, lo 53.2us, high 58.8us, then 0x55 at (24000000) / 128

	// determine approximate average 0s and 1s run length from oversampled 0x55 at high baud rate
	// complicated by multiple stop bits not seen and assumed stop bits with 0s at end of byte
	int numRuns = 0, totalRunLen = 0;
	int bytePos = 0, syncByte = 0;

	while (1) {			
		int response = DwRecv();
		if (response == NoRecv) break;

		if (!bytePos++ || bytePos == 2 && !response) // first or another 0 from 60us low
			continue; 

		++syncByte;
		if (print) printf("0"); // start bit
		int runOf = 0;
	  int currentRunLen = 1;
		// average length of both ones and zeros runs

		bool firstRun = true;

		for (int bitPos = 8; bitPos > 0; --bitPos) {
			int bit = response & 1;
			if (print) printf(bit ? "1" : "0");
			response >>= 1;

			if (bit == runOf) 
				++currentRunLen;
			else {
				if (!firstRun || currentRunLen * numRuns >= totalRunLen) { // at least average run length
					totalRunLen += currentRunLen;
			  	++numRuns;
			  }
				currentRunLen = 1;
				runOf = bit;
				firstRun = false;
			}
		}

		if (print) printf(".1.");
		// at stop bit, currentRunLen is a minimum value, except last byte runOf = 1
		if (!numRuns) {
			++numRuns;
			totalRunLen += currentRunLen;
		}	else {
			if (runOf && !rxRdy()) // hi at end of last byte
				continue;
			
			if (currentRunLen * numRuns > totalRunLen) { // > average run length
				++numRuns;
				totalRunLen += currentRunLen;
			}
		}
	}

	double expectedRunLength = 128. / divisor;  // runs of 0s or 1s
	double avgRunLength = double(totalRunLen) / numRuns;

	if (print) printf(" %.2f %.2f %+3.0f%%\n", expectedRunLength, avgRunLength, 100 * (avgRunLength - expectedRunLength) / expectedRunLength) ;

	if (!numRuns) {
		printf("zR!");
		return SerialClockHz / dWdivisor - 1;
	}

	return SerialClockHz / avgRunLength / divisor;
}

bool chkSignature() {
	DwSend(0xF3); // GetSignature

	bool gotFirstSigByte = false;
	while (1) {
		int response = DwRecv();
		if (response == NoRecv) return false;
		switch (response) {
			case 0x93: gotFirstSigByte = true; break;
			case 0x0B: return gotFirstSigByte;
		}
	}
}

bool chkSigAtDivisor(int n) {
	setBaudRate(SerialClockHz / n);
	return chkSignature();
}

int getPreciseBaud(int roughBaud) {
	int n = (SerialClockHz  + roughBaud / 2) / roughBaud ;

	// expanding search for good signature response
	int search = 0;
	while (1) {
		if (chkSigAtDivisor(n + search)) break;

		if (search >= 0) search += max(1, search);
		search = -search;

		if (n + search < 8) {
			printf("gPB! ");
			return SerialClockHz / dWdivisor;
		}
	}
	n += search;

	// expanding searches to multiple edge fails
	// better binary searches for speed            TODO
	const int EdgeFails = 2;
	int fails = EdgeFails;
	int minN = n - 1;
	while (chkSigAtDivisor(minN) || fails--) --minN;

	fails = EdgeFails;
  int maxN = n + 1;
	while (chkSigAtDivisor(maxN) || fails--) ++maxN;

	if (maxN - minN < 2 * EdgeFails + 4)
		printf("not precise ");

	F_CPU = (long long)SerialClockHz * dWdivisor * 2 / (minN + maxN); // center

	return F_CPU / dWdivisor;  // dWire baud rate
}

int adjustToDWireBaud(bool unknown = false) {
	const int divisor = 43;
	int roughBaud = setDebugWireMode(divisor);
	if (unknown)
		printf("%8d ", roughBaud * 128);
	int baud = getPreciseBaud(roughBaud);
	printf("%8d MHz %+4.1f%% ", baud * 128, 100. * (baud - roughBaud) / baud);

	if (unknown) {
	  int baud2 = getPreciseBaud(baud);
	  printf("%8d MHz %+4.1f%% ",baud2 * 128, 100. * (baud2 - baud) / baud2); // final tolerance check
		baud = baud2;
  }
	setBaudRate(baud);
	return baud;
}

void go() {
	const u8 Go[] = {0x40, 0x30 };
	DwSend(Go, sizeof Go);
}


u8 flashBuffer[MaxFlashSize];
FILE* hexFile;
u8 chkSum;

u8 getByte() { // convert hex to binary:  sed -e 's/://'  -e 's/../\\x&/g')"
  u8 data = 0;
	int pair = 2;

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
    if (!--pair) {
      chkSum += data;
      return data;
    }
  }
}

int maxAddr;

bool loadHex() {
  memset(flashBuffer, 0xFF, sizeof flashBuffer);
  maxAddr = chkSum = 0; 
  
  while (1) { // process records from .hex file,
    u8 len = getByte();  
    u16 addr = ((u16)getByte() << 8) + getByte();
    if (addr + len > maxAddr)
      maxAddr = addr + len;
    u8 recType = getByte();
    if (recType)  // 1 = end of file
      return true;

    u8* pBuf = flashBuffer + addr;
    while (len--) *pBuf++ = getByte();  // data

    getByte(); // checkSum - should sum back to 0
    if (chkSum) { 
      printf("Checksum error @ %X\n", addr);  
      return false;
    }            
  }   
}

u16 getSignature() {  
  DwSend(0xF3);  // F3..FF = read signature;  (just start bit)

  u16 sig = DwReadWord();
  if (sig != 0x930B) 
    printf("Sig %X\n", sig);
  return sig;
}

void setBaudDivisor(int log2divisor) { // 3..7
	// Note that programming flash takes 9ms erase + 4.5ms for 32 words > 70us/byte
  //   so higher baud rate most useful for flash / SRAM readout check

	if (log2divisor < 3) {
		printf("Can't set FTDI baud > 3 MHz\n");
		return;
	}
	dWdivisor = 1 << log2divisor;    // minimum 8 for FTDI (3 MHz max)

  if (log2divisor < 4)
    DwSend(0xA3 - log2divisor);
  else DwSend(0x80 + log2divisor - 4); // set ATtiny baud

  setBaudRate(F_CPU / dWdivisor);

  // DwSend(0x55);  // echoes FF or 0 - WHY??  TODO
  const u8 Sync = 0x55;
	WriteFile(hCom, &Sync, 1, NULL, NULL);

	DwSync();
  Sleep(1);
	commErrs(true);
}


void baudTest() {
	while (1) {
		adjustToDWireBaud();

		for (int divisor = 8; divisor <= 128; ++divisor) {
			if (divisor == 9) divisor = 12;  // skip to 2 M baud
			if (divisor == 13) divisor = 16; // skip to 3 M / 2 = 1.5 M baud
			setDebugWireMode(divisor, true);
			setBaudRate(F_CPU / dWdivisor);

			if (!chkSignature())
				printf("sig! ");

			dwSetPC(0);

			go();
			Sleep(1); //  wait for OSCCAL load
			rxFlush();
		}
	}
}

void setRandomBaud() {
	LARGE_INTEGER count;
	QueryPerformanceCounter(&count);
	srand(count.LowPart);

	u8 CalOsccal = DwReadEEPROM(0);
	u8 osccal = rand() * 255 / RAND_MAX;
	setOsccal(osccal);
	adjustToDWireBaud(true);
}

int main(int argc, char* argv[]) {
	openSerial();  
	adjustToDWireBaud();

	// baudTest();
	// setRandomBaud();

  if (argc > 1) {
		char hexPath[256];
		sprintf(hexPath, "../../../Documents/Atmel Studio/7.0/%s/%s/Debug/%s.hex", argv[1], argv[1], argv[1]);
    hexFile = fopen(hexPath, "r");
		if (!hexFile)
			hexFile = fopen(argv[1]);
    if (!hexFile) {
      printf("\nCan't open .hex file %s\n", argv[1]);
      return 1;
    }
  } else hexFile = stdin;
  loadHex();

#if 1 // force page write for baud test
  time_t secs; time(&secs);
	maxAddr += 2 * PageSize(); // beyond executable code
  flashBuffer[maxAddr - (secs & (PageSize() - 1))] = 0xFF - (1 << ((secs >> 6) & 7));  // one bit low
#endif

	stepTo(0, 5);

	setBaudDivisor(5); // 5, 6 OK  4, 3 -> signature, LoadPageBuffer echo errors

	int sig = getSignature();
	printf("%X\n", sig); // set deviceType  TODO
	if (sig != 0x930B) exit(-5);

  WriteFlash(0, flashBuffer, maxAddr);

	dwSetPC(0); // restart will load preset  OSCCAL
  go();

	return 0;
}


#if 0  

// auto-baud rate accuracy tests
  8: 000000000.1.000000000.1.000000000.1.000000000.1. 16.00 9.00 -44%
 12: 011111111.1.000000000.1.011111111.1.000000000.1.011111111.1.000000000.1.011111111.1.000000000.1.011111111.1. 10.67 7.67 -28%
 16: 000000001.1.000000001.1.000000001.1.000000001.1.000000001.1. 8.00 8.00  +0%
 17: 000000001.1.000000001.1.000000001.1.000000001.1.000000001.1. 7.53 8.00  +6%
 18: 000000011.1.000000011.1.000000011.1.000000011.1.000000011.1. 7.11 7.00  -2%
 19: 000000011.1.000000011.1.000000011.1.000000011.1.000000011.1. 6.74 7.00  +4%
 20: 000000111.1.000000111.1.000000111.1.000000111.1.000000111.1. 6.40 6.00  -6%
 21: 000000111.1.000000111.1.000000111.1.000000111.1.000000111.1. 6.10 6.00  -2%
 22: 000000111.1.000000111.1.000000111.1.000000111.1.000000111.1. 5.82 6.00  +3%
 23: 000000111.1.000000111.1.000000111.1.000000111.1.000000111.1. 5.57 6.00  +8%
 24: 000001111.1.000001111.1.000001111.1.000001111.1.000001111.1. 5.33 5.00  -6%
 25: 000001111.1.000001111.1.000001111.1.000001111.1.000001111.1. 5.12 5.00  -2%
 26: 000001111.1.000001111.1.000001111.1.000001111.1.000001111.1. 4.92 5.00  +2%
 27: 000001111.1.000001111.1.000001111.1.000001111.1.000001111.1. 4.74 5.00  +5%
 28: 000001111.1.000011111.1.000011110.1.000011110.1.000111111.1. 4.57 4.33  -5%
 29: 000011111.1.000011110.1.000111100.1.001111100.1.001111111.1. 4.41 4.40  -0%
 30: 000011111.1.000111110.1.001111100.1.011111000.1. 4.27 4.80 +12%
 31: 000011110.1.000111100.1.001111000.1.000011111.1. 4.13 4.00  -3%
 32: 000011110.1.000111100.1.011110000.1.000011111.1. 4.00 4.00  +0%
 33: 000011110.1.001111000.1.000011110.1.001111111.1. 3.88 4.00  +3%
 34: 000011110.1.001111000.1.000011110.1.001111111.1. 3.76 4.00  +6%
 35: 000011100.1.011110000.1.000011100.1.011111111.1. 3.66 3.67  +0%
 36: 000011100.1.011110001.1.000011100.1.011111111.1. 3.56 3.50  -2%
 37: 000111100.1.011100001.1.000111100.1.011111111.1. 3.46 3.60  +4%
 38: 000111100.1.011100011.1.000111100.1.011111111.1. 3.37 3.40  +1%
 39: 000111100.1.000111100.1.000111111.1. 3.28 3.67 +12%
 40: 000111000.1.000111000.1.000111111.1. 3.20 3.00  -6%
 41: 000111000.1.000111000.1.000111111.1. 3.12 3.00  -4%
 42: 000111000.1.000111000.1.000111111.1. 3.05 3.00  -2%
 43: 000111000.1.000111000.1.000111111.1. 2.98 3.00  +1%
 44: 000111000.1.000111000.1.000111111.1. 2.91 3.00  +3%
 45: 000111000.1.000111000.1.000111111.1. 2.84 3.00  +5%
 46: 000111001.1.000111001.1.000111111.1. 2.78 2.71  -2%
 47: 000111001.1.000110001.1.000111111.1. 2.72 2.71  -0%
 48: 000110001.1.000110001.1.000111111.1. 2.67 2.71  +2%
 49: 000110001.1.000110001.1.000111111.1. 2.61 2.71  +4%
 50: 000110001.1.000110001.1.000111111.1. 2.56 2.71  +6%
 51: 000110001.1.000110001.1.000111111.1. 2.51 2.71  +8%
 52: 001110011.1.000110011.1.001111111.1. 2.46 2.33  -5%
 53: 001110011.1.001110011.1.001111111.1. 2.42 2.40  -1%
 54: 001110011.1.001110011.1.001111111.1. 2.37 2.40  +1%
 55: 001110011.1.001100011.1.001111111.1. 2.33 2.40  +3%
 56: 001110011.1.001100011.1.001111111.1. 2.29 2.40  +5%
 57: 001110011.1.001100110.1.011111111.1. 2.25 2.17  -4%
 58: 001100011.1.001100110.1.011111111.1. 2.21 2.17  -2%
 59: 001100011.1.011100110.1.011111111.1. 2.17 2.33  +8%
 60: 001100111.1.011001110.1. 2.13 2.29  +7%
 61: 001100110.1.011001100.1. 2.10 2.00  -5%
 62: 001100110.1.011001100.1. 2.06 2.00  -3%
 63: 001100110.1.011001100.1. 2.03 2.00  -2%
 64: 001100110.1.011001100.1. 2.00 2.00  +0%
 65: 001100110.1.001100111.1. 1.97 2.00  +2%
 66: 001100110.1.001100111.1. 1.94 2.00  +3%
 67: 001100110.1.001100111.1. 1.91 2.00  +5%
 68: 001100110.1.001100111.1. 1.88 2.00  +6%
 69: 001100100.1.001100111.1. 1.86 1.88  +1%
 70: 001100100.1.001101111.1. 1.83 1.75  -4%
 71: 001101100.1.001101111.1. 1.80 1.75  -3%
 72: 001101100.1.001101111.1. 1.78 1.75  -2%
 73: 001101100.1.001101111.1. 1.75 1.75  -0%
 74: 001001100.1.001001111.1. 1.73 1.75  +1%
 75: 001001100.1.001001111.1. 1.71 1.75  +3%
 76: 001001101.1.001001111.1. 1.68 1.62  -4%
 77: 001001101.1.001001111.1. 1.66 1.62  -2%
 78: 001001101.1.001001111.1. 1.64 1.62  -1%
 79: 001001001.1.001001111.1. 1.62 1.62  +0%
 80: 001001001.1.001001111.1. 1.60 1.62  +2%
 81: 001001001.1.001001111.1. 1.58 1.62  +3%
 82: 001001001.1.011001111.1. 1.56 1.71 +10%
 83: 001001001.1.011011111.1. 1.54 1.57  +2%
 84: 001001001.1.011011111.1. 1.52 1.57  +3%
 85: 001001001.1.011011111.1. 1.51 1.57  +4%
 86: 001011011.1.010011111.1. 1.49 1.50  +1%
 87: 011011011.1.010011111.1. 1.47 1.50  +2%
 88: 011011011.1.010011111.1. 1.45 1.50  +3%
 89: 011011011.1.010111111.1. 1.44 1.38  -4%
 90: 011011011.1.010111111.1. 1.42 1.38  -3%
 91: 011011010.1.011111111.1. 1.41 1.33  -5%
 92: 011011010.1.011111111.1. 1.39 1.33  -4%
 93: 011011010.1.011111111.1. 1.38 1.33  -3%
 94: 011010010.1.011111111.1. 1.36 1.33  -2%
 95: 011010010.1.011111111.1. 1.35 1.33  -1%
 96: 011010010.1.011111111.1. 1.33 1.33  +0%
 97: 011010010.1.011111111.1. 1.32 1.33  +1%
 98: 011010010.1.011111111.1. 1.31 1.33  +2%
 99: 011010110.1.011111111.1. 1.29 1.33  +3%
100: 011010110.1.011111111.1. 1.28 1.33  +4%
101: 011010110.1.011111111.1. 1.27 1.33  +5%
102: 011010110.1.011111111.1. 1.25 1.33  +6%
103: 011010110.1.011111111.1. 1.24 1.33  +7%
104: 010010100.1.011111111.1. 1.23 1.29  +4%
105: 010010100.1.011111111.1. 1.22 1.29  +5%
106: 010010100.1.011111111.1. 1.21 1.29  +6%
107: 010010101.1.011111111.1. 1.20 1.14  -4%
108: 010010101.1.011111111.1. 1.19 1.14  -4%
109: 010010101.1.011111111.1. 1.17 1.14  -3%
110: 010110101.1.011111111.1. 1.16 1.14  -2%
111: 010010101.1.011111111.1. 1.15 1.14  -1%
112: 010110101.1.011111111.1. 1.14 1.14  +0%
113: 010110101.1.011111111.1. 1.13 1.14  +1%
114: 010110101.1.011111111.1. 1.12 1.14  +2%
115: 010100101.1.011111111.1. 1.11 1.14  +3%
116: 010100101.1. 1.10 1.14  +4%
117: 010100101.1. 1.09 1.14  +4%
118: 010101101.1. 1.08 1.14  +5%
119: 010101001.1. 1.08 1.14  +6%
120: 010101011.1. 1.07 1.00  -6%
121: 010101010.1. 1.06 1.00  -5%
122: 010101010.1. 1.05 1.00  -5%
123: 010101010.1. 1.04 1.00  -4%
124: 010101010.1. 1.03 1.00  -3%
125: 010101010.1. 1.02 1.00  -2%
126: 010101010.1. 1.02 1.00  -2%
127: 010101010.1. 1.01 1.00  -1%
128: 010101010.1. 1.00 1.00  +0%
 
 8: 0 0 0 0 0 0 0        16X --> 1 + 15 lo bits
11: 0 EC/EE               buad off ??
14:                       9X -> 1 + 8 lo bits, no stop bit -> framing errors -> 0xFFs
15:                       should have stop bit??
16: 0 0 80 80 80 80 80    8X -> 1 + 7 lo bits
18: 0 0 C0 C0 C0 C0 C0    7X -> 1 + 6 lo bits
21:                       6X -> 1 + 5 lo bits
26:                       5X -> 1 + 4 lo bits
32: 0 78 3C  F F8         4X -> 0 0001 1110 ^0 0011 1100 0^0 1111 0000 ^0 0001 1111 1...  missing stop bits supplied

37: 0 3C 87 3C FF               0 0011 1100 ^0 1110 0001 110 0011 1100 ^0 1111 1111 1...
43:                       3X -> 0 0011 1000 111 0 0011 1000 111 0 0011 1111 1...
64:                       2X -> 0 0110 0110 ^0 1100 1100 1...   missing stop bit supplied
#endif
