/*
 *
 * SUMP Protocol Implementation for Arduino boards (compatible with PulseView)
 *
 * Copyright (c) 2011,2012,2013,2014,2015 Andrew Gillham
 * Copyright (c) 2021 Bernhard Nebel
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY ANDREW GILLHAM ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANDREW GILLHAM BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * This Arduino sketch implements the SUMP protocol compatible with PulseView for 
 * AVR MCUs. It is based on Andrew Gillham's implementation, but the acquisition part 
 * has been completely redesigned.
 *
 * V0.09 
 * -  switched the channels BACK to pins 8-13 for trigger reliability.
 *
 * V0.11
 * - With v0.11 you can now sample at 4MHz & 2MHz rates in addition to the 
 *     previous 1MHz and lower rates.  This is done via unrolled loops which
 *     makes the source code huge and the binary takes much more of the flash.
 *     v0.11 is just slightly to big for an ATmega168's flash.  You can comment
 *     out either captureInline2mhz() or captureInline4mhz() and it will fit.
 *     [ The code automatically skips the 2MHz code now, this isn't needed. ]
 *
 * V0.14 - Dec 16, 2015
 *  - released
 *
 * V0.21 - Oct 05, 2021
 * - took over the reversed sample dumping from dralisz82, corrected 
 *   the types of the index vars from unsigend to signed and introduced
 *   the compile-time constant REVERSED to be able to switch between the orders.
 * - filter out unknown SUMP protocol commands (0xCX), which confused the client
 *   since the data part could contain a 0x01, which started acquisition
 *   prematurely.
 * - redesigned acquisition to use inline assembler and Timer1. Now,
 *   triggering works up to 1 MHz. The unrolled loop for 2MHz was 
 *   replaced by an inline assembler loop, which does not maintain
 *   a buffer for the pre-trigger samples, though. The 4 MHz acquisition
 *   procedure was modified to work for 5 MHz. Support for ATmega168
 *   and ATmega32U4 was removed since it was only partial. 
 *
 * V0.22
 * - the "unknown" commands are actually trigger parameters for
 *   later stages. So, renamed them. Perhaps, at some point
 *   we can support them.
 * 
 */
#define FIRMWAREVERSION "0.22"

#define MAXMHZ 5 // 2 or 1 are also options, and they both save a lot of flash space because with 5, you get an unrolled loop.

#define REVERSED // PulseView wants to receive the sampled data in reverse order


/*
 * Arduino device profile:      ols.profile-agla.cfg
 * Arduino Mega device profile: ols.profile-aglam.cfg
 */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define CHANPIN PINA
#define CHAN0 22
#define CHAN1 23
#define CHAN2 24
#define CHAN3 25
#define CHAN4 26
#define CHAN5 27
#define CHAN6 28
#define CHAN7 29
#else
#define CHANPIN PINB
#define CHAN0 8
#define CHAN1 9
#define CHAN2 10
#define CHAN3 11
#define CHAN4 12
/* Comment out CHAN5 if you don't want to use the LED pin for an input */
#define CHAN5 13
#endif
#define ledPin 13

/* XON/XOFF are not supported. */
#define SUMP_RESET 0x00
#define SUMP_ARM   0x01
#define SUMP_QUERY 0x02
#define SUMP_XON   0x11
#define SUMP_XOFF  0x13

/* mask & values used, config ignored. only stage0 supported */
#define SUMP_TRIGGER_MASK 0xC0
#define SUMP_TRIGGER_VALUES 0xC1
#define SUMP_TRIGGER_CONFIG 0xC2

/* some commands that are used by PulseView, but are ignored in AGLA */
#define SUMP_TRIGGER_MASK2 0xC4
#define SUMP_TRIGGER_MASK3 0xC8
#define SUMP_TRIGGER_MASK4 0xCC
#define SUMP_TRIGGER_VALUES2 0xC5
#define SUMP_TRIGGER_VALUES3 0xC9
#define SUMP_TRIGGER_VALUES4 0xCD
#define SUMP_TRIGGER_CONFIG2 0xC6
#define SUMP_TRIGGER_CONFIG3 0xCA
#define SUMP_TRIGGER_CONFIG4 0xCE


/* Most flags (except RLE) are ignored. */
#define SUMP_SET_DIVIDER 0x80
#define SUMP_SET_READ_DELAY_COUNT 0x81
#define SUMP_SET_FLAGS 0x82
#define SUMP_SET_RLE 0x0100

/* extended commands -- self-test unsupported, but metadata is returned. */
#define SUMP_SELF_TEST 0x03
#define SUMP_GET_METADATA 0x04

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define MAX_CAPTURE_SIZE 7168
#elif defined(__AVR_ATmega328P__)
#define MAX_CAPTURE_SIZE 1536
#else
#error "MCU is not supported"
#endif

#define DEBUG_ENABLE DDRD = DDRD | B10000000
#define DEBUG_ON PORTD = B10000000
#define DEBUG_OFF PORTD = B00000000


/*
 * SUMP command from host (via serial)
 * SUMP commands are either 1 byte, or for the extended commands, 5 bytes.
 */
int cmdByte = 0;
byte cmdBytes[5];

byte logicdata[MAX_CAPTURE_SIZE];
// Removed 'unsigned' from the variables below
// so that one can output the data in reverse without problems
// should not cause any problem as long as we stay below a buffer size of
// of 32000.
int logicIndex = 0;
int readCount = MAX_CAPTURE_SIZE;
int delayCount = 0;
byte trigger = 0;
byte trigger_values = 0;
unsigned int delayTime = 0;
unsigned long divider = 0;
boolean rleEnabled = 0;

void setup()
{
  Serial.begin(115200);
  /*
   * set debug pin (digital pin 8) to output right away so it settles.
   * this gets toggled during sampling as a way to measure
   * the sample time.  this is used during development to
   * properly pad out the sampling routines.
   */
  DEBUG_ENABLE; /* debug measurement pin */

  pinMode(CHAN0, INPUT);
  pinMode(CHAN1, INPUT);
  pinMode(CHAN2, INPUT);
  pinMode(CHAN3, INPUT);
  pinMode(CHAN4, INPUT);
#ifdef CHAN5
  pinMode(CHAN5, INPUT);
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  pinMode(CHAN6, INPUT);
  pinMode(CHAN7, INPUT);
#else
#ifndef CHAN5
  pinMode(ledPin, OUTPUT);
#endif
#endif /* Mega */
}

void loop()
{
  int i;

  if (Serial.available() > 0) {
    cmdByte = Serial.read();
    switch (cmdByte) {
    case SUMP_RESET:
      /*
       * We don't do anything here as some unsupported extended commands have
       * zero bytes and are mistaken as resets.  This can trigger false resets
       * so we don't erase the data or do anything for a reset.
       */
      break;
    case SUMP_QUERY:
      /* return the expected bytes. */
      Serial.write('1');
      Serial.write('A');
      Serial.write('L');
      Serial.write('S');
      break;
    case SUMP_ARM:
      /*
       * Zero out any previous samples before arming.
       * Done here instead via reset due to spurious resets.
       */
      for (i = 0 ; i < MAX_CAPTURE_SIZE; i++) {
        logicdata[i] = 0;
      }
      acquire();
      break;
    case SUMP_TRIGGER_MASK:
      /*
       * the trigger mask byte has a '1' for each enabled trigger so
       * we can just use it directly as our trigger mask.
       */
      getCmd();
      trigger = cmdBytes[0];
      break;
    case SUMP_TRIGGER_VALUES:
      /*
       * trigger_values can be used directly as the value of each bit
       * defines whether we're looking for it to be high or low.
       */
      getCmd();
      trigger_values = cmdBytes[0];
      break;
    case SUMP_TRIGGER_CONFIG:
    case SUMP_TRIGGER_MASK2:
    case SUMP_TRIGGER_MASK3:
    case SUMP_TRIGGER_MASK4:
    case SUMP_TRIGGER_VALUES2:
    case SUMP_TRIGGER_VALUES3:
    case SUMP_TRIGGER_VALUES4:
    case SUMP_TRIGGER_CONFIG2:
    case SUMP_TRIGGER_CONFIG3:
    case SUMP_TRIGGER_CONFIG4:
      /* read the rest of the command bytes, but ignore them. */
      getCmd();
      break;
    case SUMP_SET_DIVIDER:
      /*
       * the shifting needs to be done on the 32bit unsigned long variable
       * so that << 16 doesn't end up as zero.
       */
      getCmd();
      divider = cmdBytes[2];
      divider = divider << 8;
      divider += cmdBytes[1];
      divider = divider << 8;
      divider += cmdBytes[0];
      break;
    case SUMP_SET_READ_DELAY_COUNT:
      /*
       * this just sets up how many samples there should be before
       * and after the trigger fires.  The readCount is total samples
       * to return and delayCount number of samples after the trigger.
       * this sets the buffer splits like 0/100, 25/75, 50/50
       * for example if readCount == delayCount then we should
       * return all samples starting from the trigger point.
       * if delayCount < readCount we return (readCount - delayCount) of
       * samples from before the trigger fired.
       */
      getCmd();
      readCount = 4 * (((cmdBytes[1] << 8) | cmdBytes[0]) + 1);
      if (readCount > MAX_CAPTURE_SIZE)
        readCount = MAX_CAPTURE_SIZE;
      delayCount = 4 * (((cmdBytes[3] << 8) | cmdBytes[2]) + 1);
      if (delayCount > MAX_CAPTURE_SIZE)
        delayCount = MAX_CAPTURE_SIZE;
      break;
    case SUMP_SET_FLAGS:
      /* read the rest of the command bytes and check if RLE is enabled. */
      getCmd();
      // rleEnabled = ((cmdBytes[1] & B1000000) != 0);
      break;
    case SUMP_GET_METADATA:
      send_metadata();
      break;
    case SUMP_SELF_TEST:
      /* ignored. */
      break;
    }
  }
}


/*
 * Extended SUMP commands are 5 bytes.  A command byte followed by 4 bytes
 * of options. We already read the command byte, this gets the remaining
 * 4 bytes of the command.
 */
void getCmd() {
  delay(10);
  cmdBytes[0] = Serial.read();
  cmdBytes[1] = Serial.read();
  cmdBytes[2] = Serial.read();
  cmdBytes[3] = Serial.read();
}

void acquire() {
  switch (divider) {
#if (MAXMHZ >= 5)
  case 19: acquire5MHz(); // this has an unrolled loop with only rudimentary trigger handling
    break;
#endif
#if (MAXMHZ >= 2)
  case 49: acquire2MHz(); // special purpose loop with only rudimentary trigger handling
    break;
#endif
  default: acquireSlow(); // all the 'slow' stuff up to 1 MHz
    break;
  }
}


/*
 * This function provides sampling with triggering and a circular trigger
 * buffer. Works up to 1 MHz.
 */
void acquireSlow() {
  int i = 0;
  unsigned long top;
  byte prescale = 1; // prescaler is 1
  byte * ptr = &logicdata[0];

  if (divider < 99) divider = 99; // means at most 1 MHz
  top = (divider+1)/100*16;
  if (top >= 65536) {
    top = top / 256;
    prescale = 4; // prescaler is 256
  }
  top--;
  
  // Configure Timer 1
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | prescale;
  TCCR1C = 0;
  OCR1A = top;
  TCNT1 = 0;
  
  // initialize index
  logicIndex = 0;

  cli();

  /* C source code is replaced by inline assembler below
  do {
    while (!(TIFR1 & _BV(OCF1A)));
    TIFR1 |= _BV(OCF1A);
    inp = CHANPIN;
    logicdata[logicIndex++] = inp;
    if (logicIndex >= readCount) {
      logicIndex = 0;
    }
  } while ((trigger_values ^ inp ) & trigger); 
  
  // keep sampling for delayCount after trigger
  for (i = 0 ; i < delayCount; i++) {
    while (!(TIFR1 & _BV(OCF1A)));
    TIFR1 |= _BV(OCF1A);
    logicdata[logicIndex++] = CHANPIN;
    if (logicIndex >= readCount) {
      logicIndex = 0;
    }
  }
  */
  
  asm volatile(
    "\n\t"
    "movw r22, %[LOGICDATAaddr]               ; save addr of logicdata\n\t"
    "movw r20, r22                            ; copy start addr to r22:r23\n\t"                    
    "add  r20, %A[READCOUNT]                  ; set stop addr in r20:r21\n\t"
    "adc r21, %B[READCOUNT]\n"
    "TRIGLOOP: sbis %[TIFR1addr], %[OCF1Abit] ; test for timer overflow (2 cycles)\n\t"
    "rjmp TRIGLOOP                            ; (no cycle!) \n\t"
    "sbi %[TIFR1addr], %[OCF1Abit]            ; clear overflow bit (2 cycles)\n\t"
    "in __tmp_reg__, %[CHANPINaddr]           ; read input data (1 cycle)\n\t"
    "st %a[LOGICDATAaddr]+, __tmp_reg__       ; store input to array and post increment (2 cycles)\n\t"
    "cp %A[LOGICDATAaddr], r20                ; compare lower byte of ptr with lower byte of stop addr (1 cycle)\n\t"
    "brne TRIGTEST                            ; OK, jump to test for trigger condition (worst case 1 cycle)\n\t"
    "cp %B[LOGICDATAaddr], r21                ; compare high bytes (1 cycle)  \n\t"
    "brne TRIGTEST                            ; OK, jump (2 cycles), otherwise execute next instr (1 cycle)\n\t"
    "movw %A[LOGICDATAaddr], r22              ; restore original value (1 cycle)\n"
    "TRIGTEST: eor __tmp_reg__, %[TRIGVAL]    ; inp = inp XOR trigger_value., i.e., zero if trigger value present (1 cycle)\n\t"
    "and __tmp_reg__, %[TRIGMASK]             ; inp = inp AND trigger, i.e., one as long as trig cond not satisfied (1 cycle)\n\t"
    "brne TRIGLOOP                            ; wait for trigger (worst case 2 cycles), sum: 16 cycles\n\t"
    ";Now the sampling loop:\n\t"
    "SAMPLOOP: sbis %[TIFR1addr], %[OCF1Abit] ; test for timer overflow (2 cycles)\n\t"
    "rjmp SAMPLOOP                            ; (no cycle!) \n\t"
    "sbi %[TIFR1addr], %[OCF1Abit]            ; clear overflow bit (2 cycles)\n\t"
    //    "sbi %[PORTDaddr], 7\n\t"
    //     "cbi %[PORTDaddr], 7\n\t"
    "in __tmp_reg__, %[CHANPINaddr]           ; read input data (1 cycle)\n\t"
    "st %a[LOGICDATAaddr]+, __tmp_reg__       ; store input to array and post increment (2 cycles)\n\t"
    "cp %A[LOGICDATAaddr], r20                ; compare lower byte of ptr with lower byte of stop addr (1 cycle)\n\t"
    "brne ENDTEST                             ; OK, jump to test for trigger condition (worst case 1 cycle)\n\t"
    "cp %B[LOGICDATAaddr], r21                ; compare high bytes (1 cycle)  \n\t"
    "brne ENDTEST                             ; OK, jump (2 cycles), otherwise execute next instr (1 cycle)\n\t"
    "movw %A[LOGICDATAaddr], r22              ; restore original value (1 cycle)\n"
    "ENDTEST: sbiw %A[DELAYCOUNT], 1          ; decrement delayCount (2 cycles)\n\t"
    "brne SAMPLOOP                            ; continue until delayCount bytes have been sampled, sum: 16 cycles\n\t"
    "sub %A[LOGICDATAaddr], r22               ; subtract from ptr the base address\n\t"
    "sbc %B[LOGICDATAaddr], r23               ; high byte subtraction\n\t"
    "movw %A[LOGICINDEX], %A[LOGICDATAaddr]   ; move value to output variable\n"
      
    : [LOGICINDEX] "=r" (logicIndex)
    : [LOGICDATAaddr] "e" (ptr),	
      [OCF1Abit] "I" (OCF1A),
      [CHANPINaddr] "I" (_SFR_IO_ADDR(CHANPIN)),
      [TIFR1addr] "I" (_SFR_IO_ADDR(TIFR1)),
      [READCOUNT] "r" (readCount),
      [DELAYCOUNT] "e" (delayCount), 
      [TRIGMASK] "r" (trigger),
      [TRIGVAL] "r" (trigger_values),
      [PORTDaddr] "I" (_SFR_IO_ADDR(PORTD))
     : "r20", "r21", "r22", "r23" );

  sei();

  /* Reversing dumping to be compatible with PulseView 0.4.1 */
#ifdef REVERSED
  logicIndex--;
  for (i = 0 ; i < readCount; i++) {
    if (logicIndex < 0 ) {
      logicIndex = readCount-1;
    }
    Serial.write(logicdata[logicIndex--]);
  }
#else
  for (i = 0 ; i < readCount; i++) {
    if (logicIndex >= readCount) {
      logicIndex = 0;
    }
    Serial.write(logicdata[logicIndex++]);
  }
#endif
}

/*
 * This function provides sampling with triggering and a circular trigger
 * buffer for 2 MHz. Timing is done by counting intruction cycles. Triggering
 * is only rudimentary.
 */
void acquire2MHz() {
  int i = 0;
  byte * ptr = &logicdata[0];

  delayCount = readCount - 1;
  /*
   * disable interrupts during capture to maintain precision.
   */
  cli();

  /* C source code is replaced by inline assembler below
  do {
    inp = CHANPIN;
  } while ((trigger_values ^ inp ) & trigger); 
  logicdata[0] = inp;
  
  // keep sampling for delayCount after trigger
  for (i = 0 ; i < delayCount; i++) {
    logicdata[logicIndex++] = CHANPIN;
  }
  */

  asm volatile(
    "\n\t"
    "TRIGLOOP2: in __tmp_reg__, %[CHANPINaddr]; read input data (1 cycle)\n\t"
    "st %a[LOGICDATAaddr], __tmp_reg__        ; store input to array (2 cycles)\n\t"
    "eor __tmp_reg__, %[TRIGVAL]              ; inp = inp XOR trigger_value., i.e., zero if trigger value present (1 cycle)\n\t"
    "and __tmp_reg__, %[TRIGMASK]             ; inp = inp AND trigger, i.e., one as long as trig cond not satisfied (1 cycle)\n\t"
    "brne TRIGLOOP2                           ; wait for trigger (1 cycle if false)\n\t"
    "adiw %A[LOGICDATAaddr], 1                ; increment pointer, (2 cycles). This makes it 8 cycles!\n\t"
    ";Now the sampling loop:\n\t"
    "SAMPLOOP2: in __tmp_reg__, %[CHANPINaddr]; read input data (1 cycle)\n\t"
    "nop                                      ; cycle padding (1 cycle)\n\t"
    "st %a[LOGICDATAaddr]+, __tmp_reg__       ; store input to array and post increment (2 cycles)\n\t"
    "sbiw %A[DELAYCOUNT], 1                   ; decrement delayCount (2 cycles)\n\t"
    "brne SAMPLOOP2                           ; continue until delayCount bytes have been sampled (2 cycles), sum: 8 cycles\n\t"
      
    : 
    : [LOGICDATAaddr] "e" (ptr),	
      [CHANPINaddr] "I" (_SFR_IO_ADDR(CHANPIN)),
      [DELAYCOUNT] "e" (delayCount), 
      [TRIGMASK] "r" (trigger),
      [TRIGVAL] "r" (trigger_values) 
     : );

  sei();

  /* Reversing dumping to be compatible with PulseView 0.4.1 */
#ifdef REVERSED
  for (i = readCount-1; i >= 0; i--) {
#else
  for (i = 0 ; i < readCount; i++) {
#endif
#ifdef USE_PORTD
  Serial.write(logicdata[i] >> 2);
#else
  Serial.write(logicdata[i]);
#endif
  }
}


/*
 * This function returns the metadata about our capabilities.  It is sent in
 * response to the  OpenBench Logic Sniffer extended get metadata command.
 *
 */
void send_metadata() {
  /* device name */
  Serial.write((uint8_t)0x01);
  Serial.print(F("AGLA"));
#if defined(REVERSED)
  Serial.print(F("rev"));
#endif
  Serial.print(F("V1"));
  Serial.write((uint8_t)0x00);

  /* firmware version */
  Serial.write((uint8_t)0x02);
  Serial.print(F(FIRMWAREVERSION));
  Serial.write((uint8_t)0x00);

  /* sample memory */
  Serial.write((uint8_t)0x21);
  Serial.write((uint8_t)0x00);
  Serial.write((uint8_t)0x00);
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  /* 7168 bytes */
  Serial.write((uint8_t)0x1C);
  Serial.write((uint8_t)0x00);
#elif defined(__AVR_ATmega328P__)
  /* 1536 bytes */
  Serial.write((uint8_t)0x06);
  Serial.write((uint8_t)0x00);
#else
  #error "MCU not supported"
#endif /* Mega */

  /* sample rate (1MHz) */
  Serial.write((uint8_t)0x23);
#if (MAXMHZ == 1)
  Serial.write((uint8_t)0x00);
  Serial.write((uint8_t)0x0F);
  Serial.write((uint8_t)0x42);
  Serial.write((uint8_t)0x40);
#elif (MAXMHZ == 2)
  Serial.write((uint8_t)0x00);
  Serial.write((uint8_t)0x1E);
  Serial.write((uint8_t)0x84);
  Serial.write((uint8_t)0x80);
#elif (MAXMHZ == 5)
  Serial.write((uint8_t)0x00);
  Serial.write((uint8_t)0x4C);
  Serial.write((uint8_t)0x4B);
  Serial.write((uint8_t)0x40);
#else
  #error "Wrong maximal frequency. Can only be 1, 2, or 5 MHz."
#endif
  /* number of probes (6 by default on Arduino UNO, 8 on Mega) */
  Serial.write((uint8_t)0x40);
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  Serial.write((uint8_t)0x08);
#else
#ifdef CHAN5
  Serial.write((uint8_t)0x06);
#else
  Serial.write((uint8_t)0x05);
#endif /* CHAN5 */
#endif /* Mega */

  /* protocol version (2) */
  Serial.write((uint8_t)0x41);
  Serial.write((uint8_t)0x02);

  /* end of data */
  Serial.write((uint8_t)0x00);
}

