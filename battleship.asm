;*******************************************************************************
;                                                                              *
;    Microchip licenses this software to you solely for use with Microchip     *
;    products. The software is owned by Microchip and/or its licensors, and is *
;    protected under applicable copyright laws.  All rights reserved.          *
;                                                                              *
;    This software and any accompanying information is for suggestion only.    *
;    It shall not be deemed to modify Microchip?s standard warranty for its    *
;    products.  It is your responsibility to ensure that this software meets   *
;    your requirements.                                                        *
;                                                                              *
;    SOFTWARE IS PROVIDED "AS IS".  MICROCHIP AND ITS LICENSORS EXPRESSLY      *
;    DISCLAIM ANY WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING  *
;    BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS    *
;    FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL          *
;    MICROCHIP OR ITS LICENSORS BE LIABLE FOR ANY INCIDENTAL, SPECIAL,         *
;    INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, HARM TO     *
;    YOUR EQUIPMENT, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR    *
;    SERVICES, ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY   *
;    DEFENSE THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER      *
;    SIMILAR COSTS.                                                            *
;                                                                              *
;    To the fullest extend allowed by law, Microchip and its licensors         *
;    liability shall not exceed the amount of fee, if any, that you have paid  *
;    directly to Microchip to use this software.                               *
;                                                                              *
;    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF    *
;    THESE TERMS.                                                              *
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Filename:                                                                 *
;    Date:                                                                     *
;    File Version:                                                             *
;    Author:                                                                   *
;    Company:                                                                  *
;    Description:                                                              *
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Notes: In the MPLAB X Help, refer to the MPASM Assembler documentation    *
;    for information on assembly instructions.                                 *
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Known Issues: This template is designed for relocatable code.  As such,   *
;    build errors such as "Directive only allowed when generating an object    *
;    file" will result when the 'Build in Absolute Mode' checkbox is selected  *
;    in the project properties.  Designing code in absolute mode is            *
;    antiquated - use relocatable mode.                                        *
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Revision History:                                                         *
;                                                                              *
;*******************************************************************************



;*******************************************************************************
; Processor Inclusion
;
; TODO Step #1 Open the task list under Window > Tasks.  Include your
; device .inc file - e.g. #include <device_name>.inc.  Available
; include files are in C:\Program Files\Microchip\MPLABX\mpasmx
; assuming the default installation path for MPLAB X.  You may manually find
; the appropriate include file for your device here and include it, or
; simply copy the include generated by the configuration bits
; generator (see Step #2).
;
;*******************************************************************************

; TODO INSERT INCLUDE CODE HERE
#include <p18f24k40.inc>
    
;*******************************************************************************
;
; TODO Step #2 - Configuration Word Setup
;
; The 'CONFIG' directive is used to embed the configuration word within the
; .asm file. MPLAB X requires users to embed their configuration words
; into source code.  See the device datasheet for additional information
; on configuration word settings.  Device configuration bits descriptions
; are in C:\Program Files\Microchip\MPLABX\mpasmx\P<device_name>.inc
; (may change depending on your MPLAB X installation directory).
;
; MPLAB X has a feature which generates configuration bits source code.  Go to
; Window > PIC Memory Views > Configuration Bits.  Configure each field as
; needed and select 'Generate Source Code to Output'.  The resulting code which
; appears in the 'Output Window' > 'Config Bits Source' tab may be copied
; below.
;
;*******************************************************************************

; TODO INSERT CONFIG HERE
; CONFIG1L
  CONFIG  FEXTOSC = OFF         ; External Oscillator mode Selection bits (EC (external clock) above 8 MHz; PFM set to high power)
  CONFIG  RSTOSC = HFINTOSC_64MHZ; Power-up default value for COSC bits (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

; CONFIG1H
  CONFIG  CLKOUTEN = ON        ; Clock Out Enable bit (CLKOUT function is disabled)
  CONFIG  CSWEN = ON            ; Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
  CONFIG  FCMEN = ON            ; Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

; CONFIG2L
  CONFIG  MCLRE = EXTMCLR       ; Master Clear Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
  CONFIG  PWRTE = OFF           ; Power-up Timer Enable bit (Power up timer disabled)
  CONFIG  LPBOREN = OFF         ; Low-power BOR enable bit (ULPBOR disabled)
  CONFIG  BOREN = SBORDIS       ; Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

; CONFIG2H
  CONFIG  BORV = VBOR_2P45      ; Brown Out Reset Voltage selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
  CONFIG  ZCD = OFF             ; ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
  CONFIG  PPS1WAY = ON          ; PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
  CONFIG  STVREN = ON           ; Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
  CONFIG  DEBUG = ON           ; Debugger Enable bit (Background debugger disabled)
  CONFIG  XINST = OFF           ; Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

; CONFIG3L
  CONFIG  WDTCPS = WDTCPS_31    ; WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
  CONFIG  WDTE = OFF             ; WDT operating mode (WDT enabled regardless of sleep)

; CONFIG3H
  CONFIG  WDTCWS = WDTCWS_7     ; WDT Window Select bits (window always open (100%); software control; keyed access not required)
  CONFIG  WDTCCS = SC           ; WDT input clock selector (Software Control)

; CONFIG4L
  CONFIG  WRT0 = OFF            ; Write Protection Block 0 (Block 0 (000800-001FFFh) not write-protected)
  CONFIG  WRT1 = OFF            ; Write Protection Block 1 (Block 1 (002000-003FFFh) not write-protected)

; CONFIG4H
  CONFIG  WRTC = OFF            ; Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
  CONFIG  WRTB = OFF            ; Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
  CONFIG  WRTD = OFF            ; Data EEPROM Write Protection bit (Data EEPROM not write-protected)
  CONFIG  SCANE = ON            ; Scanner Enable bit (Scanner module is available for use, SCANMD bit can control the module)
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

; CONFIG5L
  CONFIG  CP = OFF              ; UserNVM Program Memory Code Protection bit (UserNVM code protection disabled)
  CONFIG  CPD = OFF             ; DataNVM Memory Code Protection bit (DataNVM code protection disabled)

; CONFIG5H

; CONFIG6L
  CONFIG  EBTR0 = OFF           ; Table Read Protection Block 0 (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
  CONFIG  EBTR1 = OFF           ; Table Read Protection Block 1 (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)

; CONFIG6H
  CONFIG  EBTRB = OFF           ; Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

;*******************************************************************************
;
; TODO Step #3 - Variable Definitions
;
; Refer to datasheet for available data memory (RAM) organization assuming
; relocatible code organization (which is an option in project
; properties > mpasm (Global Options)).  Absolute mode generally should
; be used sparingly.
;
; Example of using GPR Uninitialized Data
;
;   GPR_VAR        UDATA
;   MYVAR1         RES        1      ; User variable linker places
;   MYVAR2         RES        1      ; User variable linker places
;   MYVAR3         RES        1      ; User variable linker places
;
;   ; Example of using Access Uninitialized Data Section (when available)
;   ; The variables for the context saving in the device datasheet may need
;   ; memory reserved here.
;   INT_VAR        UDATA_ACS
;   W_TEMP         RES        1      ; w register for context saving (ACCESS)
;   STATUS_TEMP    RES        1      ; status used for context saving
;   BSR_TEMP       RES        1      ; bank select used for ISR context saving
;
;*******************************************************************************

; TODO PLACE VARIABLE DEFINITIONS GO HERE
  
  NUMCHARS equ 0x09
  TMR1THRESH equ 0xC3 ;195 cycles
  UPDN equ 0
  LFTRT equ 1
  DISPFRMTHRESH equ 0xEF
 RANDOMCOMP equ 0x04
 TRACKING equ 0x0A
bank1 udata
xdir res 1
xpos res 1
 ydir res 1
 ypos res 1
parity res 1
count res 1
count1 res 1
count2 res 1
count5 res 1
linemod res 1
index res 1
 mdata res 1
 mdata1 res 1
 state res 1
 spitcount res 1
 framecount res 1
 shipsize res 1
 direction res 1
 byte1 res 1
 byte2 res 1
 byte3 res 1
 cursorx res 1
 cursory res 1
 cell res 1
 mdone res 1
 plyrmoveflag res 1
 dispplyrships res 1
 dispshipfrmcnt res 1
 rand res 1
 randmod res 1
 x res 1
 y res 1
 clroffsetcnt res 1
 udoffsetcnt res 1
 mbutton res 1
 messagepointer res 1
 tmp res 1
 dutycycle res 1
 amplitude res 1
 compmoveflag res 1
 compdelaycnt res 1
 compdelay res 1
 xypointer res 1
offsettable res 0x64 

 bank2 udata
 compships res 0x64 ;holds computer ships
 playerships res 0x64 ;holds player ships
 arrpointer res 1 ;points to maximum index in array1 and array2
 compstate res 1
 
 bank3 udata
 xystack res 0x64 ;holds xy for ship tracking
 hitcountscomp res 0x05
 hitcountsplayer res 0x05
 shipdataplayer res 0x0A
 shipdatacomp res 0x0A
 shipindex res 1
 x1 res 1
 y1 res 1
 hitcountp res 1
 hitcountc res 1
 gamedonec res 1
 gamedonep res 1
;*******************************************************************************
; Reset Vector
;*******************************************************************************

RES_VECT  CODE    0x0000            ; processor reset vector
GOTO START    

;*******************************************************************************
; TODO Step #4 - Interrupt Service Routines
;
; There are a few different ways to structure interrupt routines in the 8
; bit device families.  On PIC18's the high priority and low priority
; interrupts are located at 0x0008 and 0x0018, respectively.  On PIC16's and
; lower the interrupt is at 0x0004.  Between device families there is subtle
; variation in the both the hardware supporting the ISR (for restoring
; interrupt context) as well as the software used to restore the context
; (without corrupting the STATUS bits).
;
; General formats are shown below in relocatible format.
;
;------------------------------PIC16's and below--------------------------------
;
; ISR       CODE    0x0004           ; interrupt vector location
;
;     <Search the device datasheet for 'context' and copy interrupt
;     context saving code here.  Older devices need context saving code,
;     but newer devices like the 16F#### don't need context saving code.>
;
;     RETFIE
;
;----------------------------------PIC18's--------------------------------------
;
; ISRHV     CODE    0x0008
;     GOTO    HIGH_ISR
; ISRLV     CODE    0x0018
;     GOTO    LOW_ISR
;
; ISRH      CODE                     ; let linker place high ISR routine
; HIGH_ISR
;     <Insert High Priority ISR Here - no SW context saving>
;     RETFIE  FAST
;
; ISRL      CODE                     ; let linker place low ISR routine
; LOW_ISR
;       <Search the device datasheet for 'context' and copy interrupt
;       context saving code here>
;     RETFIE
;
;*******************************************************************************
  
ISRHV CODE 0x0008
 GOTO HIGH_ISR
 
ISRH CODE 0x000E
HIGH_ISR
 call ISR
 retfie FAST
;*******************************************************************************
; MAIN PROGRAM
;*******************************************************************************

MAIN_PROG CODE 0x0020       ; let linker place main program
  
getcount:
    banksel state
    movf state,w,BANKED
    banksel WREG
    rlncf WREG,w,BANKED
    movff PCL,tmp
    addwf PCL,1,BANKED
    retlw 0xA6 ;game process
    retlw 0x08 ;draw message
    ;retlw 0x03 ;3 lines of gap
    retlw 0x50 ;draw grid
    retlw 0x03 ;horiz equalization
    retlw 0x03 ;vertical equalization
    retlw 0x03 ;horiz equalization
    
branchisr:
    movff PCL,tmp
    addwf PCL,1,BANKED ;1 cycle
 bra gameprocess ; 2 cycles
 bra drawmessage
 ;bra gap
 bra drawimage
 bra horizequal
 bra vsync
 bra horizequal

getimage:
    movff PCL,tmp
    addwf PCL,1,BANKED
    
    ;character 0, empty
    retlw 0x00 ;00000000
    retlw 0x7E ;01111110
    retlw 0x7E ;01111110
    retlw 0x7E ;01111110
    retlw 0x7E ;01111110
    retlw 0x7E ;01111110
    retlw 0x7E ;01111110
    retlw 0x00 ;00000000
    
    ;character 1, miss
    retlw 0x00 ;00000000
    retlw 0x3C ;00111100
    retlw 0x5A ;01011010
    retlw 0x66 ;01100110
    retlw 0x66 ;01100110
    retlw 0x5A ;01011010
    retlw 0x3C ;00111100
    retlw 0x00 ;00000000
    
    ;character2, solid box, used as hit indicator and ship indicator for player ships
    retlw 0x00 ;00000000
    retlw 0x7E ;01111110
    retlw 0x42 ;01000010
    retlw 0x42 ;01000010
    retlw 0x42 ;01000010
    retlw 0x42 ;01000010
    retlw 0x7E ;01111110
    retlw 0x00 ;00000000
    
    ;character 3, fill completely, used as sink indicator
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    
    ;character 4, explosion
    retlw b'01010100'
    retlw b'00101010'
    retlw b'01010100'
    retlw b'00101010'
    retlw b'01010100'
    retlw b'00101010'
    retlw b'01010100'
    retlw b'00101010'
    
START
    ;initialization
    banksel ANSELA
    clrf ANSELA, BANKED        ;pins are digital
    clrf ANSELB,BANKED
    clrf ANSELC, BANKED
    bsf ANSELC,3,BANKED ;allow adc input at rc3
    setf WPUA, BANKED          ;weak pull ups
    setf WPUB, BANKED
    setf WPUC, BANKED
    bcf WPUA,1, BANKED ;disable pull ups for mdata and mclock
    bcf WPUA,2, BANKED
    
    
    
    setf SLRCONA, BANKED
    setf SLRCONB, BANKED
    setf SLRCONC, BANKED
    
    ;initialize table pointer, upper,high,low
    
    ;clear plyrmoveflag
    banksel plyrmoveflag
    clrf plyrmoveflag,BANKED
    
    ;clear cell
    banksel cell
    clrf cell,BANKED
    
    ;clear duty cycle
    banksel dutycycle
    clrf dutycycle,BANKED
    
    ;set plyr ships flag
    banksel dispplyrships
    setf dispplyrships,BANKED
    
    ;clear dispshipfrmcnt
    banksel dispshipfrmcnt
    clrf dispshipfrmcnt,BANKED
    
    ;set amplitude to 0x0A
    banksel amplitude
    movlw 0x0A
    movwf amplitude,BANKED
    
    ;initialize offsettable
    lfsr FSR0,offsettable
    banksel POSTINC0
    movlw 0x00
    movwf POSTINC0,BANKED
    movlw 0x63 ;decimal 99
    banksel count
    movwf count, BANKED
init:
    banksel POSTINC0
    movlw 0
    movwf POSTINC0, BANKED
    banksel count
    decfsz count, 1, BANKED
    bra init
    
    ;setup timer 0 with a 1:4 prescaler, 8 bits 
    movlw 0x42
    banksel T0CON1
    movwf T0CON1, BANKED
    setf TMR0H, BANKED ;timer0 period value
    bsf T0CON0,T0EN, BANKED
    
    ;setup timer 1 to fosc/4
    banksel TMR1CLK
    bsf TMR1CLK,0,BANKED
    
    ;set negative edge detect on IOCAN 1,2
    banksel IOCAN
    bsf IOCAN,1,BANKED
    bsf IOCAN,2,BANKED
    
    ;setup GPIO
    ;pin5 is sync, RA3
    ;pin11 is video, RC0
    ;RB0 is the LED
    ;pin3 mclock is RA1
    ;pin4 mdata is RA2
    ;pin12 audio is RC1
    banksel TRISA
    bcf TRISA,3, BANKED
    bcf TRISC,0, BANKED
    bcf TRISC,1,BANKED ;audio tris
    bcf TRISC,3, BANKED ;SPI clock
    bcf TRISB,0, BANKED
    bsf LATC,0, BANKED
    bsf LATA,3, BANKED
    bcf LATB,0, BANKED
    movlw 0x0E ;setup PPS module for SPI data at RC0
    banksel RC0PPS
    movwf RC0PPS, BANKED
    movlw 0x0D ;setup SPI clock at RC3
    banksel RC3PPS
    movwf RC3PPS, BANKED 
    movlw 0x07
    banksel RC1PPS
    movwf RC1PPS,BANKED ;audio, pwm3
    ;setup SPI
    
    ;setup PWM3 and timer2
    banksel T2CON
    bsf T2CLKCON,0,BANKED
    movlw b'00000110'
    movwf T2RST,BANKED
    bsf T2CON,T2ON,BANKED
    movlw 0x40 ;64 into period reg
    banksel PR2
    movwf PR2,BANKED
    banksel PWM3DCL
    clrf PWM3DCL
    movlw 0x20
    movwf PWM3DCH,BANKED
    bsf PWM3CON,EN
    
    
    
    
    ;setup adc
    banksel ADCON0
    bsf ADCON0,ADFM, BANKED ;formatting
    movlw 0x3D ;select temp sensor
    movwf ADPCH,BANKED
    setf ADCLK, BANKED ;fosc/128
    movlw 0x20
    movwf ADACQ, BANKED
    bsf ADCON0,ADON
    banksel FVRCON
    bsf FVRCON,TSEN,BANKED ;enable the temperature sensor
    
    ;initialize mouse
    ;banksel parity
    ;clrf parity
    movlw 0x3D
    call delay ;delay for 1 second to allow reset, works
    movlw 0xF4 ;enable data reporting
    banksel mdata
    movwf mdata, BANKED
    
    call sendcommand
    call readbyte ;sends 0xF4 ONLY!!! no 0xFA
    
    ;initialize player ships and clear interrupt flags
    banksel PIR4
    bcf PIR4,TMR1IF
    banksel PIR0
    bcf PIR0,TMR0IF
    call seed
    call clearplayer
    call clearcomp
    call inithitcounts
    call initplayer
    call initcomp
    
  
    
    ;initialize computer ships
    ;call initcomp
    
    ;initialize board with player ships
    ;call initboard
    
;cont:
 ;   call readbyte
 ;   call readbyte
 ;   call readbyte
    ;test bit 1 of mdata1
    ;left mouse button is bit 0
    ;right mouse button is bit 1
    ;x sign is bit 4
    ;y sign is bit 5
    ;x delta is byte 1
    ;y delta is byte 2
  ;  banksel LATB
  ;  bcf LATB,0,BANKED
  ;  banksel mdata1
  ;  btfss mdata1,0,BANKED
  ;  bra cont1
  ;  banksel LATB
  ;  bsf LATB,0,BANKED
;cont1:    
 ;   bra cont
    
    ;call readbyte
    ;call readbyte
 ;   movlw 0xEA ;enable stream mode
  ;  banksel mdata
   ; movwf mdata
   ; call sendcommand ;sends wreg data to mouse
   ; call readbyte ;reads a byte from mouse and puts in wreg
    ;movlw 0xF4 ;enable reporting in stream mode
   ; banksel mdata
   ; movwf mdata
   ; call sendcommand
   ; call readbyte
    ;for debug, count clock pulses, are we getting an ack bit? Yes we are. 13 bits
    
    ;first read is FF
    ;second read is FD
    ;third read is 11010101, D5 
    ;banksel mdata1
    ;btfss mdata1,0
    ;bra mainloop
    ;banksel LATB
    ;bsf LATB,0
   
    banksel xpos
    clrf xpos,BANKED
    banksel state
    clrf state,BANKED
    banksel count1
    movlw 0xA6
    movwf count1,BANKED
    banksel index
    clrf index,BANKED
    banksel linemod
    clrf linemod,BANKED
    banksel spitcount
    clrf spitcount,BANKED
    banksel ypos
    clrf ypos,BANKED
    banksel framecount
    clrf framecount,BANKED
    banksel cursorx
    clrf cursorx,BANKED
    banksel cursory
    clrf cursory,BANKED
    banksel count
    clrf count,BANKED
    banksel compstate
    movlw low RANDOMCOMP
    movwf compstate,BANKED
    banksel xypointer
    clrf xypointer,BANKED
    banksel hitcountp
    clrf hitcountp,BANKED
    banksel hitcountc
    clrf hitcountc,BANKED
    banksel gamedonec
    clrf gamedonec,BANKED
    clrf gamedonep,BANKED
    
    lfsr FSR2,xystack ;set stack at FSR2
    ;enable interrupts
    banksel PIE0
 ;   bsf PIE0,TMR0IE
    banksel PIE4
 ;   bsf PIE4,TMR1IE
    banksel INTCON
 ;   bsf INTCON,GIE
 ;   bsf INTCON,PEIE
    
    
    
    ;mouse loop
    banksel mdone
    clrf mdone,BANKED
    
mouseloop:
    call readbyte
    movff mdata1,byte1
    call readbyte
    movff mdata1,byte2
    call readbyte
    movff mdata1,byte3  
    banksel mdone
    setf mdone,BANKED 
    
    ;test bit0 of byte1
    banksel byte1
    btfsc byte1,0,BANKED
    nop
    bra mouseloop    
    
   
    
    ;horizontal sync routine, 9 cycles in front porch
horizline:
    banksel PIR0 ;1 cycle
    ;btfss PIR0,TMR0IF ;1 cycle
    ;bra horizline
    
    ;wait for front porch, 7 cycles
    bcf PIR0,TMR0IF ;1 cycle
    banksel count ;1 cycle
    movlw 7;1 cycle
    movwf count ;1 cycle
    
    nop ;1 cycle
    nop ;1 cycle
    banksel LATA ; 1 cycle
    bcf LATA,3 ;clear sync, 1 cycle
    
    ;wait for back porch, 75 cycles
    banksel count ;1 cycle
    movlw  0x18;1 cycle
    movwf count ;1 cycle
backporch:
    decfsz count ;1 cycle
    bra backporch ;2 cycles
    
    banksel LATA
    bsf LATA,3 ;set sync
    
    return
    
    
    ;vertical sync routine, 9 cycles for front porch
vertline:
    banksel PIR0 ;1 cycle
    ;btfss PIR0,TMR0IF ;1 cycle
    ;bra vertline
    
    ;wait for front porch, 7 cycles
    bcf PIR0,TMR0IF ;1 cycle
    banksel count ;1 cycle
    movlw 7;1 cycle
    movwf count ;1 cycle
    nop ;1  cycle
    nop ;1 cycle
    banksel LATA ;1 cycle
    bsf LATA,3 ;set sync, 1 cycle
    
    ;wait for back porch, 75 cycles
    banksel count ;1 cycle
    movlw  0x18;1 cycle
    movwf count ;1 cycle
backporchvert:
    decfsz count ;1 cycle
    bra backporchvert ;2 cycles
    
    banksel LATA
    bcf LATA,3 ;clear sync
    
    return    
    
;delay for ~1 second
delay:
    banksel T0CON0
    bsf T0CON0,4, BANKED ;temporarily make timer0 a 16 bit timer
delaystart:
    banksel TMR0H
    clrf TMR0H, BANKED
    clrf TMR0L, BANKED
waitovf:
    banksel PIR0
    btfss PIR0,TMR0IF, BANKED
    bra waitovf
    bcf PIR0,TMR0IF, BANKED
    banksel WREG
    decfsz WREG,1, BANKED
    bra delaystart
    
    ;turn tmr0 back to 8 bit
    banksel T0CON0
    bcf T0CON0,4, BANKED
    setf TMR0H, BANKED
    clrf TMR0L, BANKED
    banksel PIR0
    bcf PIR0,TMR0IF, BANKED
    return
    
;send a command in wreg to mouse
sendcommand:
      
    banksel LATA
    ;bring clock low for 100us
    bcf LATA,1, BANKED
    bcf TRISA,1, BANKED
    
     ;bring data low
    banksel LATA
    bcf LATA,2, BANKED
    bcf TRISA,2, BANKED
    
    ;delay for 3200 cycles
    movlw 0xC8 ;200
    banksel count
    movwf count,BANKED
delay1:
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    decfsz count,1,BANKED
    bra delay1
    
    ;release clock
    banksel TRISA
    bsf TRISA,1, BANKED
    
    ;wait for clock to be pulled low
waitlow1:
    btfsc PORTA,1, BANKED
    bra waitlow1
    
    ;wait for clock high
waithigh10:
    btfss PORTA,1,BANKED
    bra waithigh10
    
    ;wait for clock low
waitlow10:
    btfsc PORTA,1,BANKED
    bra waitlow10
    
    ;loop 8x to send data
    movlw 0x08
    banksel count
    movwf count, BANKED
senddata:
    banksel mdata
    btfsc mdata,0, BANKED
    bra setdata
    banksel LATA
    bcf LATA,2, BANKED
    
finishsetdata:
   
    banksel mdata
    rrncf mdata,1,1
    
    ;wait for clock to be high
waithigh2:
    banksel PORTA
    btfss PORTA,1, BANKED
    bra waithigh2
    
    ;wait for clock to be low
waitlow2:
    
    ;read the ra1 bit into the adc
    ;banksel ANSELA
    ;bsf ANSELA,1
    ;bsf ADCON0,ADON ;enable the adc
    ;bsf ADCON0,ADGO
;readanalog:
 ;   btfsc ADCON0,ADGO
  ;  bra readanalog
    
    ;turn on the led if ADRESL is <10
    ;0x80 no turn on, 128, 0.64 volts
    ;0xA0 no turn on, 160, 0.8 volts
    ;0xC0 turn on, 192, 0.96 volts
    ;goes down between 0.8 and 0.96 volts
   ; banksel ADRESH
   ; movlw 0xA0
   ; banksel ADRESL
   ; cpfslt ADRESL
   ; bra readanalogcont
   ; banksel LATB
   ; bsf LATB,0
;readanalogcont:    
    btfsc PORTA,1, BANKED
    bra waitlow2
    
    banksel count
    decfsz count,1,1
    bra senddata
    
;send parity------------------------
    banksel LATA
    bcf LATA,2,BANKED
    
    ;wait for clock to be high
waithigh3:
    btfss PORTA,1, BANKED
    bra waithigh3
    
    ;wait for clock to be low
waitlow3:
    btfsc PORTA,1, BANKED
    bra waitlow3
    
    ;release data
    bsf TRISA,2, BANKED
    
    ;wait for data to be low
waitlow4:
    btfsc PORTA,2, BANKED
    bra waitlow4
    
    ;wait for clock to be low
waitclocklow4:
    btfsc PORTA,1, BANKED
    bra waitclocklow4
    
    ;wait for data and clock to be high
waitboth1:
    btfss PORTA,1, BANKED
    bra waitboth1
    btfss PORTA,2, BANKED
    bra waitboth1
    return
    
;set the data bit, sets 0xF4
setdata:
    banksel LATA
    bsf LATA,2, BANKED 
    bra finishsetdata
    
;set bit7 of wreg
setmdata1:
    banksel mdata1
    bsf mdata1,7, BANKED
    return
    
;read a byte from mouse, put in wreg
readbyte:
    banksel PORTA
    ;wait for clock low and data low, start bit
rbwaitlow4:
    btfsc PORTA,1, BANKED
    bra rbwaitlow4
    btfsc PORTA,2,BANKED
    bra rbwaitlow4
        
    ;get 8 data bits
    movlw 8
    banksel count5
    movwf count5, BANKED
    banksel mdata1
    clrf mdata1, BANKED
readloop:    
    ;wait for clock to be high
    banksel PORTA
rbwaithigh1:
    btfss PORTA,1, BANKED
    bra rbwaithigh1
    
    ;wait for clock to be low
rbwaitlow1:
    btfsc PORTA,1, BANKED
    bra rbwaitlow1
    
    ;shift in data to bit 7 of mdata1
    banksel mdata1 
    rrncf mdata1,1,BANKED
    banksel PORTA
    btfsc PORTA,2, BANKED
    call setmdata1
    
    banksel count5
    decfsz count5,1,BANKED
    bra readloop
    
    banksel PORTA
    ;get parity and stop
  ;wait for clock to be high
rbwaithigh2:
    btfss PORTA,1, BANKED
    bra rbwaithigh2
    
    ;wait for clock to be low
rbwaitlow2:
    btfsc PORTA,1, BANKED
    bra rbwaitlow2
   
  ;wait for clock to be high
rbwaithigh3:
    banksel PORTA
    btfss PORTA,1, BANKED
    bra rbwaithigh3
    
    ;wait for clock to be low
rbwaitlow3:
    btfsc PORTA,1, BANKED
    bra rbwaitlow3
    
    ;wait for clock to be high
rbwaithigh5:
    btfss PORTA,1, BANKED
    bra rbwaithigh5

    return
 
    ;clear lata 2 for parity
clearparity:
    banksel LATA
    bcf LATA,2, BANKED
    bra waithigh3
    
;NTSC functions-----------------------------------------
ISR: 
    
     banksel PIR4
     btfsc PIR4,TMR1IF
     bra spitchar
     
     ;set PWM3DCH to dutycycle
     movff dutycycle,PWM3DCH
    
    ; need 9 cycles in horizline and vertline
     banksel PIR0 ;1 cycle
 bcf PIR0,TMR0IF, BANKED ;1 cycle
 banksel state ;1 cycle
 movf state,w,BANKED ;1 cycle
 banksel WREG ;1 cycle
 rlncf WREG,w,BANKED ;1 cycle
 bra branchisr ;2 cycles
 
drawimage:
    call horizline ;2 cycles
    
    ;load NUMCHARS into spitcount
    banksel spitcount
    movlw NUMCHARS
    movwf spitcount,BANKED
    
    ;delay for blanking
    movlw 0x19
    banksel count2
    movwf count2,BANKED
blanking:
    decfsz count2,1,BANKED
    bra blanking
    
    ;set FSR register, FSR0=offsettable+index
    banksel index
    lfsr FSR0,offsettable
    movf index,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    
    ;turn on timer1
    banksel T1CON
    bsf T1CON,0
    movlw  TMR1THRESH    ;196 cycles
    movwf TMR1L,BANKED
    setf TMR1H,BANKED
    banksel PIR4
    bcf PIR4,TMR1IF,BANKED
    
    ;spit out first character
    banksel POSTINC0
    movf POSTINC0,w,BANKED
    banksel linemod
    addwf linemod,w,BANKED
    banksel WREG
    rlncf WREG,w,BANKED
    banksel PCL
    call getimage
    
    ;turn on spi and load register
    banksel SSP1CON1
    bsf SSP1CON1,SSPEN,BANKED
    movwf SSP1BUF,BANKED
    
    bra endline
    
gameprocess:
    call horizline
    call clear
    
    ;if count1 is 3, handle computer move:
    banksel count1
    movf count1,w,BANKED
    sublw 0x03
    bnz nomovecomp
    
    ;is compmoveflag set?...
    banksel compmoveflag
    movf compmoveflag,w,BANKED
    bz nocomprand
    
    ;first, if set, and compstate==TRACKING:
    banksel compstate
    movf compstate,w,BANKED
    sublw low TRACKING
    bnz notrack
    call comptrack
notrack:    
    ;if so, call rand1 and clear compdelaycnt
    movf compstate,w,BANKED
    sublw RANDOMCOMP
    bnz nodorand1
    call rand1
nodorand1:    
    banksel compdelaycnt
    clrf compdelaycnt,BANKED
    
nocomprand:    
    
    ;is compdelay set?
    banksel compdelay
    movf compdelay,w,BANKED
    bz nomovecomp
    
    ;if so, increment compdelaycnt
    banksel compdelaycnt
    incf compdelaycnt,BANKED
    
    ;is compdelaycnt==240?
    movf compdelaycnt,w,BANKED
    sublw 0xF0
    bnz nomovecomp
    
    ;clear and set appropriate registers, and clear offsets
    banksel compdelaycnt
    clrf compdelaycnt,BANKED
    banksel compdelay
    clrf compdelay,BANKED
    banksel plyrmoveflag
    setf plyrmoveflag,BANKED
    call clearoffsets
    
nomovecomp:    
    ;if count1 is 0x14, randomly select value for duty cycle up to 0x40
    banksel count1
    movf count1,w,BANKED
    sublw 0x14
    bnz noupdateduty
    movff amplitude,randmod
    call random
    banksel dutycycle
    movwf dutycycle,BANKED
    banksel PWM3CON
    bcf PWM3CON,EN,BANKED ;turn off pwm
    banksel amplitude
    movlw 0x0A
    cpfsgt amplitude,BANKED
    bra noupdateduty
    decf amplitude,BANKED
    banksel PWM3CON
    bsf PWM3CON,EN,BANKED ;turn on pwm if amplitude is high enough
noupdateduty:
    
    ;if count1 is 15, set FSR0 and FSR1, and updateoffsets
    banksel count1
    movf count1,w,BANKED
    sublw 0x0F
    bnz noudoffset1
    banksel FSR0
    lfsr FSR0,compships
    
    ;if compmoveflag is set or compdelay is set, load FSR0 with playerships
    banksel compmoveflag
    movf compmoveflag,w,BANKED
    bz nodispcompmove
    lfsr FSR0,playerships
nodispcompmove:
    
    banksel compdelay
    movf compdelay,w,BANKED
    bz nocompdelayset
    lfsr FSR0,playerships
nocompdelayset:    
    lfsr FSR1,offsettable
    call updateoffsets
noudoffset1:    
    ;if count1 is less than 15 and greater than 9, call updateoffsets
    banksel count1
    movlw 0x0F
    cpfslt count1,BANKED
    bra noudoffset
    movlw 0x09
    cpfsgt count1,BANKED
    bra noudoffset
    call updateoffsets
noudoffset:    
    ;handle mouse data at line 5
    ;banksel count1
    ;movf count1,w,BANKED
    ;sublw 0x05
    ;bnz noprcsmouse
    
    ;if mdone is set, get xpos and ypos, and mousebutton status
    banksel mdone
    btfss mdone,0,BANKED
    bra framecountnotzero
    
    ;reset mdone
    clrf mdone,BANKED
    
    ;get mousebutton status
    banksel mbutton
    clrf mbutton,BANKED
    banksel byte1
    btfss byte1,0,BANKED
    bra nobutton
    banksel mbutton
    setf mbutton,BANKED
nobutton:    
    ;if we are even, handle xpos if xmotion is 1
    banksel byte2
    movf byte2,w,BANKED
    bz xleft
    banksel xpos
    incf xpos,BANKED
    banksel byte1
    btfss byte1,4,BANKED ;test xdir
    bra xleft
    banksel xpos
    decf xpos,BANKED
    decf xpos,BANKED
xleft:
    
    ;if we are even, handle ypos if ymotion is 1
    banksel byte3
    movf byte3,w,BANKED 
    bz ynoup
    banksel ypos
    incf ypos,BANKED
    banksel byte1
    btfsc byte1,5,BANKED ;test ydir
    bra ynoup
    banksel ypos
    decf ypos,BANKED
    decf ypos,BANKED
ynoup:    
    ;clear byte1,byte2, and byte3
    banksel byte1
    clrf byte1,BANKED
    banksel byte2
    clrf byte2,BANKED
    banksel byte3
    clrf byte3,BANKED
    
framecountnotzero:
    
    
    
    ;test  if framecount is equal to 32, move cursorx and cursory, and reset framecount
    banksel framecount
    movf framecount,w,BANKED
    sublw 0x04 ;4
    bnz noprcsmouse
    
    ;if framecount is 16, get cursorx
    
    ;if cursorx <9 and xpos is >3 and less than 200, increment cursorx
    banksel cursorx
    movlw 0x09
    cpfslt cursorx,BANKED
    bra noincx
    banksel xpos
    movlw 0x01
    cpfsgt xpos,BANKED
    bra noincx
    movlw 0xC8
    cpfslt xpos
    bra noincx
    banksel cursorx
    call clearcursor
    banksel cursorx
    incf cursorx,BANKED
    call setcursor
    
noincx:
    
    ;if cursorx is > 0 and xpos >252, decrement cursorx
    banksel cursorx
    movlw 0x00
    cpfsgt cursorx,BANKED
    bra nodecx
    banksel xpos
    movlw 0xFE ;254
    cpfsgt xpos,BANKED
    bra nodecx
    banksel cursorx
    call clearcursor
    banksel cursorx
    decf cursorx,BANKED
    call setcursor
    
nodecx:
    
  ;if cursory <9 and ypos is >3 and less than 200, increment cursory
    banksel cursory
    movlw 0x09
    cpfslt cursory,BANKED
    bra noincy
    banksel ypos
    movlw 0x01
    cpfsgt ypos,BANKED
    bra noincy
    movlw 0xC8
    cpfslt ypos
    bra noincy
    banksel cursory
    call clearcursor
    banksel cursory
    incf cursory,BANKED
    call setcursor
    
noincy:
    
    ;if cursory is > 0 and ypos >252, decrement cursory
    banksel cursory
    movlw 0x00
    cpfsgt cursory,BANKED
    bra nodecy
    banksel ypos
    movlw 0xFE ;254
    cpfsgt ypos,BANKED
    bra nodecy
    banksel cursory
    call clearcursor
    banksel cursory
    decf cursory,BANKED
    call setcursor
    
nodecy:
    
    ;reset xpos and ypos
    banksel xpos
    clrf xpos,BANKED
    banksel ypos
    clrf ypos,BANKED
    
noprcsmouse:
    
    ;if count1 is five, handle framecount
    banksel count1
    movf count1,w,BANKED
    sublw 0x05
    bnz mousehandledone
    
    ;increment framecount
    banksel framecount
    incf framecount,BANKED
    
    ;if framecount is 5, clear it
    movf framecount,w,BANKED
    sublw 0x05 ;5
    bnz mousehandledone
    
    clrf framecount,BANKED
        
mousehandledone:    
    bra endline
    
;gap 
gap:
    call horizline
    call clear
    bra endline
    
;draw testmessage
drawmessage:
    call horizline
    
     ;delay for blanking
    movlw 0x19
    banksel count2
    movwf count2,BANKED
blanking1:
    decfsz count2,1,BANKED
    bra blanking1
    
    ;initialize messagepointer to linemod
    movff linemod,messagepointer   
    
    ;turn on spi 
    banksel SSP1CON1
    bsf SSP1CON1,SSPEN,BANKED
    
    ;loop 10x for each character
    banksel count2
    movlw 0x0A
    movwf count2,BANKED
messageloop:    
    banksel messagepointer
    rlncf messagepointer,w,BANKED
    banksel dispplyrships
    btfss dispplyrships,0,BANKED
    bra mlcont0
    banksel PCL
    call playershipsmsg
mlcont0:    
    banksel plyrmoveflag
    btfss plyrmoveflag,0,BANKED
    bra mlcont
    banksel PCL
    call playermovemsg
mlcont:
    banksel compdelay
    btfss compdelay,0,BANKED
    bra mlcont2
    banksel PCL
    call compmovemsg
mlcont2:
    banksel gamedonec
    btfss gamedonec,0,BANKED
    bra mlcont3
    banksel PCL
    call playerwinmsg
mlcont3:
    banksel gamedonep
    btfss gamedonep,0,BANKED
    bra mlcont4
    banksel PCL
    call compwinmsg
mlcont4:    
    banksel SSP1BUF
    xorlw 0xFF ;invert wreg
    
    movwf SSP1BUF,BANKED
    
    ;add 8 to messagepointer
    banksel messagepointer
    movlw 0x08
    addwf messagepointer,1,BANKED
    
    ;wait for transmission to complete
    banksel PIR3
syncspim:
    btfss PIR3,SSP1IF,BANKED
    bra syncspim
    bcf PIR3,SSP1IF,BANKED
    
    banksel count2
    decfsz count2,1,BANKED
    bra messageloop
    
    ;turn off spi
    banksel SSP1CON1
    bcf SSP1CON1,SSPEN,BANKED
    banksel LATC
    bsf LATC,0,BANKED
    
    ;increment linemod
    banksel linemod
    incf linemod,BANKED
    
    ;if linemod is 8, clear it
    movf linemod,w,BANKED
    sublw 0x08
    bnz noclrmlinemod
    clrf linemod,BANKED
noclrmlinemod:    
    bra endline
vsync:
    call vertline
    call clear
    
    ;if plyrmoveflag is set, and mousebutton is set, and count1 is 1, determine offset
    
    banksel plyrmoveflag
    movf plyrmoveflag,w,BANKED
    btfss plyrmoveflag,0,BANKED
    bra nooffset
    banksel mbutton
    btfss mbutton,0,BANKED ;left mouse button
    bra nooffset
    banksel count1
    movf count1,w,BANKED
    sublw 0x01
    bnz nooffset
    
    ;set FSR0 to compships+x+y*10
    lfsr FSR0,compships
    banksel cursorx
    movf cursorx,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    movlw 0x0A
    banksel cursory
    mulwf cursory,BANKED
    banksel PRODL
    movf PRODL,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    
    ;set bit 4 of compships[y][x], clear plyrmoveflag and set compmoveflag
    bsf INDF0,4,BANKED
    call checksinkp
    banksel plyrmoveflag
    clrf plyrmoveflag,BANKED
    banksel compmoveflag
    setf compmoveflag,BANKED
    call clearoffsets
    ;if indf0 bits 3..0 are non-zero:
    ;set amplitude for sound at 0x3B
    banksel INDF0
    movf INDF0,w,BANKED
    andlw 0x0F
    bz nooffset
    movlw 0x3B
    banksel amplitude
    movwf amplitude,BANKED
    call donec
nooffset:    
    ;branch to noincfrm if count1 is not 2
    banksel count1
    movf count1,w,BANKED
    sublw 0x02
    bnz noincfrm
    
    ;increment dispshipfrmcnt
    banksel dispshipfrmcnt
    incf dispshipfrmcnt,BANKED
    
    ;if dispshipfrmcnt is equal to dispfrmthresh and dispplyrships is set, set plyrmoveflag, clear dispshipfrmcnt
    banksel dispshipfrmcnt
    movf dispshipfrmcnt,w,BANKED
    sublw 0xEF
    bnz noincfrm
    
    banksel dispplyrships
    btfss dispplyrships,0,BANKED
    bra noincfrm
    
    banksel plyrmoveflag
    setf plyrmoveflag,BANKED
    banksel dispplyrships
    clrf dispplyrships,BANKED
    banksel dispshipfrmcnt
    clrf dispshipfrmcnt,BANKED
    call clearoffsets
noincfrm:    
    
    ;if clroffsetflag is set,
    bra endline
    
horizequal:
    call horizline
    call clear
endline:
    
    ;decrement count1
    banksel count1
    decf count1,BANKED
    
    ;if count1 is 0, reload with new data and increment state
    banksel count1
    movf count1,w,BANKED
    bnz endlinecont
    banksel state
    incf state,BANKED    
    call getcount
    banksel count1
    movwf count1,BANKED
endlinecont:
    
    ;if state is 6, clear it
    banksel state
    movf state,w,BANKED
    sublw 0x06
    bnz endlinecont1
    clrf state,BANKED
endlinecont1: 
    
 
   
   return
   
;spit out a character
spitchar:
    
    ;set timer1 registers
    banksel TMR1H
    setf TMR1H,BANKED
    movlw TMR1THRESH
    movwf TMR1L,BANKED
    
    ;clear PIR4,TMR1IF and set TMR register
    banksel PIR4
    bcf PIR4,TMR1IF,BANKED
    banksel TMR1H
    setf TMR1H,BANKED
    movlw TMR1THRESH
    movwf TMR1L,BANKED
    
    ;wait for transmission to complete
    banksel PIR3
syncspi:
    btfss PIR3,SSP1IF,BANKED
    bra syncspi
    bcf PIR3,SSP1IF,BANKED
    
    ;spit out character
    banksel POSTINC0
    movf POSTINC0,w,BANKED
    banksel linemod
    addwf linemod,w,BANKED
    banksel WREG
    rlncf WREG,w,BANKED
    banksel PCL
    call getimage
    
    ;load SPI register
    banksel SSP1BUF
    movwf SSP1BUF,BANKED
    
    ;decrement spitcount
    banksel spitcount
    decf spitcount,BANKED
    
    ;if spitcount is 0:
    movf spitcount,w,BANKED
    bnz spnotzero
    
    ;turn off timer 1
    banksel T1CON
    bcf T1CON,0,BANKED
    
    ;load NUMCHARS into spitcount
    movlw NUMCHARS
    movwf spitcount,BANKED
    
    ;increment linemod
    banksel linemod
    incf linemod,BANKED
    
    ;if linemod==8:
    movf linemod,w,BANKED
    sublw 0x08
    bnz linemodnoteight
    
    ;clear linemod
    clrf linemod,BANKED
    
    ;add 10 to index
    banksel index
    movlw 0x0A
    addwf index,1,BANKED
    
linemodnoteight:
    
    ;turn off spi and set RC0
    banksel SSP1CON1
    bcf SSP1CON1,SSPEN,BANKED
    banksel LATC
    bsf LATC,0
    
spnotzero:   
    return
    
;clear index,linemod, and spitcount
clear:
    ;clear index,linemod,spitcount
    banksel index
    clrf index,BANKED
    banksel linemod
    clrf linemod,BANKED
    banksel spitcount
    clrf spitcount,BANKED
    return
    
;set the led
turnonled:
    banksel LATB
    bcf TRISB,0,BANKED
    bsf LATB,0,BANKED
    
;clear the led
turnoffled:
    banksel LATB
    bcf TRISB,0,BANKED
    bcf LATB,0,BANKED
    
;set cursor
setcursor:
    
    ;return if plyrmoveflag is cleared
    banksel plyrmoveflag
    btfss plyrmoveflag,0,BANKED
    return
    
    ;fsr0 is tblstart+10*cursory+cursorx
    lfsr FSR0,offsettable
    banksel cursorx
    movf cursorx,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    movlw 0x0A
    banksel cursory
    mulwf cursory,BANKED
    banksel PRODL
    movf PRODL,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    
    movff INDF0,cell
    
    movlw 0x10
    banksel INDF0
    movwf INDF0
    
    return
    
;clear cursor
clearcursor:
    
     ;return if plyrmoveflag is cleared
    banksel plyrmoveflag
    btfss plyrmoveflag,0,BANKED
    return
    
   ;fsr0 is tblstart+10*cursory+cursorx
    lfsr FSR0,offsettable
    banksel cursorx
    movf cursorx,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    movlw 0x0A
    banksel cursory
    mulwf cursory,BANKED
    banksel PRODL
    movf PRODL,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
   
    ;move cell to INDF0
    movff cell,INDF0
    
    return 
    
;initialize the computer ships
initcomp:
    
    ;set indf1 to shipdircomp+8
    lfsr FSR2,shipdatacomp
    movlw 0x08
    banksel FSR1L
    addwf FSR2L,1,BANKED
   
    ;loop thru count from 5 to 1
    movlw 0x05
    banksel count
    movwf count,BANKED
initcomploop:
    
    ;get ship size
    call getshipsize
    
    
shipcstrt:
    ;move 11 to randmod and get direction
    banksel randmod
    movlw 0x0B
    movwf randmod,BANKED
    call random
    banksel direction
    movwf direction,BANKED
    
shipcont1:    
    
    ;if direction is 0, up down:
    ;banksel direction
    ;btfsc direction,0,BANKED
    ;bra plyrlr
    
    ;if direction <5, up down
    movlw 0x05
    banksel direction
    cpfsgt direction,BANKED
    bra complr
    
    ;we are up down. Set randmod to 10, and get x
    movlw 0x0A
    banksel randmod
    movwf randmod,BANKED
    call random
    banksel x
    movwf x,BANKED
    
    ;set randmod to 11-shipsize, and get y
    movlw 0x0B
    banksel shipsize
    subfwb shipsize,w,BANKED
    banksel randmod
    movwf randmod,BANKED
    call random
    banksel y
    movwf y,BANKED
    
    ;FSR0 is playerships+10*y+x
    lfsr FSR0,compships
    banksel x
    movf x,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    call incfsrh
    movlw 0x0A
    banksel y
    mulwf y,BANKED
    banksel PRODL
    movf PRODL,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED  
    call incfsrh
    
    ;loop count1 from shipsize to 1
    movff shipsize,count1
comploopcnt1:   
    
    banksel INDF0
    movf INDF0,w,BANKED
    bnz shipcstrt
    
    ;add 10 to FSR0L if playerships[x][y] is 0
    movlw 0x0A
    banksel FSR0L
    addwf FSR0L,1,BANKED
    call incfsrh
    
    banksel count1
    decfsz count1,1,BANKED
    bra comploopcnt1
    
    ;end of loop.
    
    call updateinfoc1
   
    ;subtract 10*shipsize from FSR0L
    movlw 0x0A
    banksel shipsize
    mulwf shipsize,BANKED
    banksel PRODL
    movf PRODL,w,BANKED
    subwf FSR0L,1,BANKED
    
    ;loop again with count1 from shipsize to 1
    movff shipsize,count1
comploopcnt2:
    
    ;move count to INDF0
    movff count,INDF0
    
    ;add 10 to FSR0L
    movlw 0x0A
    banksel FSR0L
    addwf FSR0L,1,BANKED
    
    banksel count1
    decfsz count1,1,BANKED
    bra comploopcnt2
    
    ;end second updown loop
    bra endcomploop
    
complr:
    
    ;randmod is 11-shipsize, then call random to get x
    movlw 0x0B
    banksel shipsize
    subfwb shipsize,w,BANKED
    banksel randmod
    movwf randmod,BANKED
    call random
    banksel x
    movwf x,BANKED

    ;set randmod to 10, use random to get y
    movlw 0x0A
    banksel randmod
    movwf randmod,BANKED
    call random
    banksel y
    movwf y,BANKED
    
    ;FSR0 is playerships+10*y+x
    lfsr FSR0,compships
    banksel x
    movf x,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    movlw 0x0A
    banksel y
    mulwf y,BANKED
    banksel PRODL
    movf PRODL,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    
    ;loop thru count1 from shipsize to 1
    movff shipsize,count1
complrloop1:
    
    ;if postinc0 > 0, bra shippstrt
    banksel POSTINC0
    movf POSTINC0,w,BANKED
    bnz shipcstrt
    
    banksel count1
    decfsz count1,1,BANKED
    bra complrloop1
    ;end plyrlrloop1
    
    call updateinfoc
    
    ;subtract shipsize from FSR0
    banksel shipsize
    movf shipsize,w,BANKED
    banksel FSR0L
    subwf FSR0L,1,BANKED
   
    
    
    ;loop count1 from shipsize to 1 again
    movff shipsize,count1
complrloop2:
    
    ;postinc is equal to count
    movff count,POSTINC0
    
    banksel count1
    decfsz count1,1,BANKED
    bra complrloop2    
    
endcomploop:    
    banksel count
    decfsz count,1,BANKED
    bra initcomploop
    
 return ;return from player ship placement   
    
    
;initialize the player's ships
initplayer:
    
    ;set FSR1 to playershipdir+8
    lfsr FSR2,shipdataplayer
    movlw 0x08
    banksel FSR1L
    addwf FSR2L,1,BANKED
    
    ;loop thru count from 5 to 1
    movlw 0x05
    banksel count
    movwf count,BANKED
initplayerloop:
    
    ;get ship size
    call getshipsize
    
    
shippstrt:
    ;move 11 to randmod and get direction
    banksel randmod
    movlw 0x0B
    movwf randmod,BANKED
    call random
    banksel direction
    movwf direction,BANKED
    
shipcont:    
    
    ;if direction is 0, up down:
    ;banksel direction
    ;btfsc direction,0,BANKED
    ;bra plyrlr
    
    ;if direction <5, up down
    movlw 0x05
    banksel direction
    cpfsgt direction,BANKED
    bra plyrlr
    
    ;we are up down. Set randmod to 10, and get x
    movlw 0x0A
    banksel randmod
    movwf randmod,BANKED
    call random
    banksel x
    movwf x,BANKED
    
    ;set randmod to 11-shipsize, and get y
    movlw 0x0B
    banksel shipsize
    subfwb shipsize,w,BANKED
    banksel randmod
    movwf randmod,BANKED
    call random
    banksel y
    movwf y,BANKED
    
    ;FSR0 is playerships+10*y+x
    lfsr FSR0,playerships
    banksel x
    movf x,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    call incfsrh
    movlw 0x0A
    banksel y
    mulwf y,BANKED
    banksel PRODL
    movf PRODL,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED  
    call incfsrh
    
    ;loop count1 from shipsize to 1
    movff shipsize,count1
plyrloopcnt1:   
    
    banksel INDF0
    movf INDF0,w,BANKED
    bnz shippstrt
    
    ;add 10 to FSR0L if playerships[x][y] is 0
    movlw 0x0A
    banksel FSR0L
    addwf FSR0L,1,BANKED
    call incfsrh
    
    banksel count1
    decfsz count1,1,BANKED
    bra plyrloopcnt1
    
    ;end of loop.
    
    call updateinfo
   
    ;subtract 10*shipsize from FSR0L
    movlw 0x0A
    banksel shipsize
    mulwf shipsize,BANKED
    banksel PRODL
    movf PRODL,w,BANKED
    subwf FSR0L,1,BANKED
    
    ;Set FSR1 with offsettable
    call setfsr1
    
    
    ;loop again with count1 from shipsize to 1
    movff shipsize,count1
plyrloopcnt2:
    
    ;move count to INDF0
    movff count,INDF0
    
    ;move 16 into INDF1
    movlw 0x10
    banksel INDF1
    movwf INDF1,BANKED
    
    ;add 10 to FSR0L and FSR1L
    movlw 0x0A
    banksel FSR0L
    addwf FSR0L,1,BANKED
    addwf FSR1L,1,BANKED
    
    banksel count1
    decfsz count1,1,BANKED
    bra plyrloopcnt2
    
    ;end second updown loop
    bra endplyrloop
    
plyrlr:
    
    ;randmod is 11-shipsize, then call random to get x
    movlw 0x0B
    banksel shipsize
    subfwb shipsize,w,BANKED
    banksel randmod
    movwf randmod,BANKED
    call random
    banksel x
    movwf x,BANKED

    ;set randmod to 10, use random to get y
    movlw 0x0A
    banksel randmod
    movwf randmod,BANKED
    call random
    banksel y
    movwf y,BANKED
    
    ;FSR0 is playerships+10*y+x
    lfsr FSR0,playerships
    banksel x
    movf x,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    movlw 0x0A
    banksel y
    mulwf y,BANKED
    banksel PRODL
    movf PRODL,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    
    ;loop thru count1 from shipsize to 1
    movff shipsize,count1
plyrlrloop1:
    
    ;if postinc0 > 0, bra shippstrt
    banksel POSTINC0
    movf POSTINC0,w,BANKED
    bnz shippstrt
    
    banksel count1
    decfsz count1,1,BANKED
    bra plyrlrloop1
    ;end plyrlrloop1
    
       ;put x and y into INDF2 
    movff x,INDF2
    movff y,tmp
    banksel tmp
    rlncf tmp,1,BANKED
    rlncf tmp,1,BANKED
    rlncf tmp,1,BANKED 
    rlncf tmp,1,BANKED
    movf tmp,w,BANKED
    banksel INDF2
    addwf INDF2,1,BANKED
    incf FSR2L,BANKED
    
     ;we are LFTRT. Set indf2 to LFTRT
    movlw LFTRT
    banksel POSTDEC1
    movwf INDF2,BANKED
    movf FSR2L,w,BANKED
    sublw 0x03
    banksel WREG
    negf WREG,BANKED
    banksel FSR2L
    movwf FSR2L,BANKED
    
    ;subtract shipsize from FSR0
    banksel shipsize
    movf shipsize,w,BANKED
    banksel FSR0L
    subwf FSR0L,1,BANKED
    
    ;set FSR1 with offsettable
    call setfsr1
    
    
    ;loop count1 from shipsize to 1 again
    movff shipsize,count1
plyrlrloop2:
    
    ;postinc is equal to shipindex+1
    movff count,POSTINC0
    
    ;postinc1 is equal to 16
    movlw 0x10
    banksel POSTINC1
    movwf POSTINC1,BANKED
    
    banksel count1
    decfsz count1,1,BANKED
    bra plyrlrloop2    
    
endplyrloop:
    
    banksel count
    decfsz count,1,BANKED
    bra initplayerloop
    
 return ;return from player ship placement
 
 ;random number generator
 random:
    ;movlw 0x64 ;multiply by 100
    ;mulwf rand,BANKED
    ;banksel PRODL
    ;movff PRODL,rand
    ;movf PRODL,w,BANKED
    
    banksel rand
    btfss rand,0,BANKED
    bra randlsb0
    rrncf rand,1,BANKED
    bcf rand,7,BANKED
    movf rand,w,BANKED
    xorlw 0xB8
    movwf rand,BANKED
    bra moverand
randlsb0:    
    rrncf rand,1,BANKED
    bcf rand,7,BANKED
moverand:
     banksel rand
    movf rand,w,BANKED
    ;keep subtracting randmod until wreg < randmod
modstart:
    banksel randmod
    cpfslt randmod, BANKED
    bra moddone
    subfwb randmod,w,BANKED
    bra modstart
moddone:
    ;if wreg == randmod, clear wreg
    cpfseq randmod,BANKED
    bra noextramod
    banksel WREG
    clrf WREG,BANKED
noextramod:    
    return
    
;random seed generation
seed:
    banksel ADCON0
    bsf ADCON0,ADGO
waitadc:
    btfsc ADCON0,ADGO
    bra waitadc
    movff ADRESL,rand
 
    ;if rand is 0, put 2D into it
    banksel rand
    movf rand,w,BANKED
    bnz nosetrand
    movlw 0x2D
    movwf rand,BANKED
nosetrand:    
    return    

;clear the player board
clearplayer:
    
    ;loop count from 100 to 1
    movlw 0x64
    banksel count
    movwf count,BANKED
    lfsr FSR0,playerships ;init FSR0
clrplyrloop:
    banksel POSTINC0
    clrf POSTINC0,BANKED
    
    banksel count
    decfsz count,1,BANKED
    bra clrplyrloop
    
    return
    
;clear the computer's ships, compships
clearcomp:
  ;loop count from 100 to 1
    movlw 0x64
    banksel count
    movwf count,BANKED
    lfsr FSR0,compships ;init FSR0
clrcomploop:
    banksel POSTINC0
    clrf POSTINC0,BANKED
    
    banksel count
    decfsz count,1,BANKED
    bra clrcomploop
    
    return   
    
;inc fsrh
incfsrh:
    bnc nto
    banksel LATB
    bsf LATB,0,BANKED
nto:    
    banksel WREG
    clrf WREG,BANKED
    banksel FSR0H
    addwfc FSR0H,1,BANKED
    return
    
;get the ship size
getshipsize:
    
    ;if count is 5, shipsize is 5
    banksel count
    movf count,w,BANKED
    sublw 0x05
    bnz gssnot5
    movlw 0x05
    banksel shipsize
    movwf shipsize,BANKED
    return
gssnot5:
    ;if count is 4, shipsize is 4
    banksel count
    movf count,w,BANKED
    sublw 0x04
    bnz gssnot4
    movlw 0x04
    banksel shipsize
    movwf shipsize,BANKED
    return
    
gssnot4:
    ;if count is 3, shipsize is 3
    banksel count
    movf count,w,BANKED
    sublw 0x03
    bnz gssnot3
    movlw 0x03
    banksel shipsize
    movwf shipsize,BANKED
    return
gssnot3:
    ;if count is 2, shipsize is 3
    banksel count
    movf count,w,BANKED
    sublw 0x02
    bnz gssnot2
    movlw 0x03
    banksel shipsize
    movwf shipsize,BANKED
    return
gssnot2:
    ;if count is 1, shipsize is 2
    movlw 0x02
    banksel shipsize
    movwf shipsize,BANKED
    return
    
;set fsr1 to offsettable+10*y+x
setfsr1:
    lfsr FSR1,offsettable
    banksel x
    movf x,w,BANKED
    banksel FSR1L
    addwf FSR1L,1,BANKED
    call incfsr1h
    movlw 0x0A
    banksel y
    mulwf y,BANKED
    banksel PRODL
    movf PRODL,w,BANKED
    banksel FSR1L
    addwf FSR1L,1,BANKED
    call incfsr1h
    return
    
;increment fsr1h if carry
incfsr1h:
    bnc incfsrhcont
    banksel FSR1H
    incf FSR1H,BANKED
incfsrhcont:
    return
    
;update the offset table
clearoffsets:
    lfsr FSR1,offsettable
    movlw 0x14
    banksel clroffsetcnt
    movwf clroffsetcnt,BANKED
clroffsetloop:
    banksel POSTINC1
    clrf POSTINC1,BANKED
    clrf POSTINC1,BANKED
    clrf POSTINC1,BANKED
    clrf POSTINC1,BANKED
    clrf POSTINC1,BANKED
    banksel clroffsetcnt
    decfsz clroffsetcnt,1,BANKED
    bra clroffsetloop
    return
    
;update the offset table
updateoffsets:
    movlw 0x14 ;20 cycles
    banksel udoffsetcnt
    movwf udoffsetcnt,BANKED
udoffsetloop:
    
    ;if POSTINC0 bit 5 is set:
    banksel INDF0
    btfss INDF0,5,BANKED
    bra nosinkudoffset
    movlw 0x18 ;24
    movwf INDF1,BANKED
    bra notry
nosinkudoffset:    
    ;if POSTINC0 bit 4 is set:
    banksel INDF0
    btfss INDF0,4,BANKED
    bra notry
    movlw 0x10 ; hit
    movwf INDF1,BANKED
    movf INDF0,w,BANKED
    andlw 0x0F
    bnz notry
    movlw 0x08 ;miss
    movwf INDF1,BANKED
notry:     
    banksel FSR0L
    incf FSR0L,BANKED
    incf FSR1L,BANKED
    
    banksel udoffsetcnt
    decfsz udoffsetcnt,1,BANKED
    bra udoffsetloop
    return

CODE 0x2000
testmessage:
    movff PCL,tmp
    banksel PCL
    addwf PCL,1,BANKED
    ;A
    retlw b'00111100'
    retlw b'00100100'
  retlw   b'00100100'
  retlw   b'00111100'
  retlw   b'00100100'
  retlw   b'00100100'
  retlw   b'00100100'
  retlw   b'00100100'

;B
  retlw b'00111000'
  retlw b'00100100'
  retlw b'00100100'
  retlw b'00111000'
  retlw b'00100100'
  retlw b'00100100'
  retlw b'00100100'
  retlw b'00111000'

;C
  retlw b'00111100'
  retlw b'00100000'
  retlw b'00100000'
  retlw b'00100000'
  retlw b'00100000'
  retlw b'00100000'
  retlw b'00100000'
  retlw b'00111100'

;D
  retlw b'00111000'
  retlw b'00100100'
  retlw b'00100100'
  retlw b'00100100'
  retlw b'00100100'
  retlw b'00100100'
  retlw b'00100100'
  retlw b'00111000'

;E
  retlw b'00111100'
  retlw b'00100000'
  retlw b'00100000'
  retlw b'00100000'
  retlw b'00111100'
  retlw b'00100000'
  retlw b'00100000'
  retlw b'00111100'
  
;space
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  
;space
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  
;space
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  
;space
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  
;space
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  retlw 0x00
  
;player ships message
section2 CODE 0x2100
playershipsmsg:  
      movff PCL,tmp
    banksel PCL
    addwf PCL,1,BANKED
    
    ;P
    retlw b'00111100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00111100'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    
    ;L
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00111100'
    
    ;Y
    retlw b'01000010'
    retlw b'00100100'
    retlw b'00011000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    
    ;R
    retlw b'00111100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00111100'
    retlw b'00101000'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    
    ;space
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    
    ;S
    retlw b'00011000'
    retlw b'00100100'
    retlw b'00100000'
    retlw b'00011000'
    retlw b'00000100'
    retlw b'00000100'
    retlw b'00100100'
    retlw b'00011000'
    
    ;H
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00111100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    
    ;I
    retlw b'00111100'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00111100'
    
    ;P
    retlw b'00111100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00111100'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    
    ;S
    retlw b'00011000'
    retlw b'00100100'
    retlw b'00100000'
    retlw b'00011000'
    retlw b'00000100'
    retlw b'00000100'
    retlw b'00100100'
    retlw b'00011000'
    
    
;player move message
section3 CODE 0x2200
playermovemsg:  
      movff PCL,tmp
    banksel PCL
    addwf PCL,1,BANKED
    
    ;P
    retlw b'00111100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00111100'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    
    ;L
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00111100'
    
    ;Y
    retlw b'01000010'
    retlw b'00100100'
    retlw b'00011000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    
    ;R
    retlw b'00111100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00111100'
    retlw b'00101000'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    
    ;space
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    
    ;M
    retlw b'01000010'
    retlw b'01100110'
    retlw b'01011010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    
    ;O
    retlw b'00111100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00111100'
    
    ;V
    retlw b'10000001'
    retlw b'10000001'
    retlw b'10000001'
    retlw b'10000001'
    retlw b'10000001'
    retlw b'01000010'
    retlw b'00100100'
    retlw b'00011000'
    
    ;E
    retlw b'00111100'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00111100'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00111100'
    
    ;space
    retlw 0x00
   retlw 0x00
   retlw 0x00
   retlw 0x00
   retlw 0x00
   retlw 0x00
   retlw 0x00
   retlw 0x00
   
section4 CODE 0x2300
playerwinmsg:
    movff PCL,tmp
    addwf PCL,1,BANKED
    
    ;P
    retlw b'00111100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00111100'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    
    ;L
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00111100'
    
    ;Y
    retlw b'01000010'
    retlw b'00100100'
    retlw b'00011000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    
    ;R
    retlw b'00111100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00111100'
    retlw b'00101000'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    
    ;space
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    
    ;W
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01011010'
    retlw b'01011010'
    retlw b'00100100'
    
    ;I
    retlw b'00111100'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00111100'
    
    ;N
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01100010'
    retlw b'01010010'
    retlw b'01001010'
    retlw b'01000110'
    retlw b'01000010'
    retlw b'01000010'
    
    ;S
    retlw b'00011000'
    retlw b'00100100'
    retlw b'00100000'
    retlw b'00011000'
    retlw b'00000100'
    retlw b'00000100'
    retlw b'00100100'
    retlw b'00011000'
    
    ;!
    retlw b'00011000'
    retlw b'00011000'
    retlw b'00011000'
    retlw b'00011000'
    retlw b'00000000'
    retlw b'00000000'
    retlw b'00011000'
    retlw b'00011000'
    
section5 CODE 0x2400
compwinmsg:
    movff PCL,tmp
    addwf PCL,1,BANKED
    
    ;C
    retlw b'00111100'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00111100'
    
    ;O
    retlw b'00111100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00111100'
    
    ;M
    retlw b'01000010'
    retlw b'01100110'
    retlw b'01011010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    
    ;P
    retlw b'00111100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00111100'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    
    ;space
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    
    ;W
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01011010'
    retlw b'01011010'
    retlw b'00100100'
    
    ;I
    retlw b'00111100'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00001000'
    retlw b'00111100'
    
    ;N
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01100010'
    retlw b'01010010'
    retlw b'01001010'
    retlw b'01000110'
    retlw b'01000010'
    retlw b'01000010'
    
    ;S
    retlw b'00011000'
    retlw b'00100100'
    retlw b'00100000'
    retlw b'00011000'
    retlw b'00000100'
    retlw b'00000100'
    retlw b'00100100'
    retlw b'00011000'
    
    ;!
    retlw b'00011000'
    retlw b'00011000'
    retlw b'00011000'
    retlw b'00011000'
    retlw b'00000000'
    retlw b'00000000'
    retlw b'00011000'
    retlw b'00011000'
    
    
;computer move message
section6 CODE 0x2500
compmovemsg:  
      movff PCL,tmp
    banksel PCL
    addwf PCL,1,BANKED
    
    ;C
    retlw b'00111100'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00111100'
    
    ;O
    retlw b'00111100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00111100'
    
    ;M
    retlw b'01000010'
    retlw b'01100110'
    retlw b'01011010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    
    ;P
    retlw b'00111100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00111100'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    
    ;space
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    retlw 0x00
    
    ;M
    retlw b'01000010'
    retlw b'01100110'
    retlw b'01011010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    retlw b'01000010'
    
    ;O
    retlw b'00111100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00100100'
    retlw b'00111100'
    
    ;V
    retlw b'10000001'
    retlw b'10000001'
    retlw b'10000001'
    retlw b'10000001'
    retlw b'10000001'
    retlw b'01000010'
    retlw b'00100100'
    retlw b'00011000'
    
    ;E
    retlw b'00111100'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00111100'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00100000'
    retlw b'00111100'
    
    ;space
    retlw 0x00
   retlw 0x00
   retlw 0x00
   retlw 0x00
   retlw 0x00
   retlw 0x00
   retlw 0x00
   retlw 0x00   
    
;sinksize routine
section7 CODE 0x2600
sinksize:
    movff PCL,tmp
    banksel PCL
    addwf PCL,1,BANKED
    retlw 0x02
    retlw 0x03
    retlw 0x03
    retlw 0x04
    retlw 0x05
  
;load pwm3dch with random value, modded at 0x20
loadpwm:
    movlw 0x20
    banksel randmod
    movwf randmod,BANKED
    call random
    banksel PWM3DCH
    movwf PWM3DCH,BANKED
    return
  
;subroutine to get a computer hit
rand1:
    banksel randmod
    movlw 0x0A
    movwf randmod,BANKED
    call random
    banksel x
    movwf x,BANKED
    call random
    banksel y
    movwf y,BANKED
    
    lfsr FSR0,playerships
    banksel x
    movf x,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    movlw 0x0A
    banksel y
    mulwf y,BANKED
    banksel PRODL
    movf PRODL,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    movf INDF0,w,BANKED
    andlw 0x10
    bnz nosetcompmovedone
    
    ;set bit4 in indf0
    bsf INDF0,4,BANKED
    banksel compdelay
    setf compdelay,BANKED
    banksel compmoveflag
    clrf compmoveflag,BANKED
    
    ;if a hit, set compstate to TRACKING
    banksel INDF0
    movf INDF0,w,BANKED
    andlw 0x0F
    bz nosetcompmovedone
    movlw TRACKING
    banksel compstate
    movwf compstate,BANKED
    call pushxy
    call checksink
nosetcompmovedone:
    return
    
;;;;;computer ship tracking;;;;;;;;;;;;;;;;;;;;;;;
comptrack:
    
;inc y if y < 9
    movlw 0x09
    banksel y
    cpfslt y,BANKED
    bra notrackincy
    lfsr FSR0, playerships
    banksel x
    movf x,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    banksel y
    movf y,w,BANKED
    banksel WREG
    incf WREG,BANKED
    mullw 0x0A
    banksel PRODL
    movf PRODL,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    btfsc INDF0,4,BANKED
    bra notrackincy

    bsf INDF0,4,BANKED
    banksel compdelay
    setf compdelay,BANKED
    banksel compmoveflag
    clrf compmoveflag,BANKED
    banksel INDF0
    movf INDF0,w,BANKED
    andlw 0x0F
    bz notrackincy1
    banksel y
    incf y,BANKED
    call pushxy
    call checksink
notrackincy1:
    return
notrackincy:

;dec y if y > 0
    banksel y
    movf y,w,BANKED
    bz notrackdecy
    lfsr FSR0, playerships
    banksel x
    movf x,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    banksel y
    movf y,w,BANKED
    banksel WREG
    decf WREG,BANKED
    mullw 0x0A
    banksel PRODL
    movf PRODL,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    btfsc INDF0,4,BANKED
    bra notrackdecy

    bsf INDF0,4,BANKED
    banksel compdelay
    setf compdelay,BANKED
    banksel compmoveflag
    clrf compmoveflag,BANKED
    banksel INDF0
    movf INDF0,w,BANKED
    andlw 0x0F
    bz notrackdecy1
    banksel y
    decf y,BANKED
    call pushxy
    call checksink
notrackdecy1:
    return
notrackdecy:

;... for inc x if x<9
    movlw 0x09
    banksel x
    cpfslt x,BANKED
    bra notrackincx
    lfsr FSR0, playerships
    banksel x
    movf x,w,BANKED
    banksel WREG
    incf WREG,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    banksel y
    movf y,w,BANKED
    mullw 0x0A
    banksel PRODL
    movf PRODL,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    btfsc INDF0,4,BANKED
    bra notrackincx

    bsf INDF0,4,BANKED
    banksel compdelay
    setf compdelay,BANKED
    banksel compmoveflag
    clrf compmoveflag,BANKED
    banksel INDF0
    movf INDF0,w,BANKED
    andlw 0x0F
    bz notrackincx1
    banksel x
    incf x,BANKED
    call pushxy
    call checksink
notrackincx1:
    return
notrackincx:    

;dec x if x > 0
    banksel x
    movf x,w,BANKED
    bz notrackdecx
    lfsr FSR0, playerships
    banksel x
    movf x,w,BANKED
    banksel WREG
    decf WREG,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    banksel y
    movf y,w,BANKED
    mullw 0x0A
    banksel PRODL
    movf PRODL,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    btfsc INDF0,4,BANKED
    bra notrackdecx

    bsf INDF0,4,BANKED
    banksel compdelay
    setf compdelay,BANKED
    banksel compmoveflag
    clrf compmoveflag,BANKED
    banksel INDF0
    movf INDF0,w,BANKED
    andlw 0x0F
    bz notrackdecx1
    banksel x
    decf x,BANKED
    call pushxy
    call checksink
notrackdecx1:
    return
notrackdecx:    
    
;if xypointer is 0:
    banksel xypointer
    movf xypointer,w,BANKED
    bnz nosettrack
    movlw RANDOMCOMP
    banksel compstate
    movwf compstate,BANKED
    return
nosettrack:
;pop x,y
    banksel FSR2L
    decf FSR2L,BANKED
    movf INDF2,w,BANKED
    andlw 0x0F
    banksel x
    movwf x,BANKED
    banksel INDF2
    movf INDF2,w,BANKED
    banksel WREG
    rrncf WREG,w,BANKED
    bcf WREG,7,BANKED
    rrncf WREG,w,BANKED
    bcf WREG,7,BANKED
    rrncf WREG,w,BANKED
    bcf WREG,7,BANKED
    rrncf WREG,w,BANKED
    bcf WREG,7,BANKED
    banksel y
    movwf y,BANKED
    banksel xypointer
    decf xypointer,BANKED
return
    
;push a value onto FSR2
pushxy:
    banksel tmp
    movff y,tmp
    rlncf tmp,1,BANKED
    rlncf tmp,1,BANKED
    rlncf tmp,1,BANKED
    rlncf tmp,1,BANKED
    banksel x
    movf x,w,BANKED
    banksel tmp
    addwf tmp,1,BANKED
    movff tmp,POSTINC2
    banksel xypointer
    incf xypointer,BANKED
    call donep
    ;we have a hit, set amplitude
    movlw 0x3B
    banksel amplitude
    movwf amplitude,BANKED
    return
    
;initialize hitcounts
inithitcounts:
    lfsr FSR0,hitcountscomp
    lfsr FSR1,hitcountsplayer
    banksel INDF0
    movlw 0x02
    movwf POSTINC0,BANKED
    movwf POSTINC1,BANKED
    movlw 0x03
    movwf POSTINC0,BANKED
    movwf POSTINC1,BANKED
    movlw 0x03
    movwf POSTINC0,BANKED
    movwf POSTINC1,BANKED
    movlw 0x04
    movwf POSTINC0,BANKED
    movwf POSTINC1,BANKED
    movlw 0x05
    movwf POSTINC0,BANKED
    movwf POSTINC1,BANKED
    return
    
;subroutine to update info arrays
updateinfo:
    
     ;put x and y into INDF2 
    movff x,INDF2
    movff y,tmp
    banksel tmp
    rlncf tmp,1,BANKED
    rlncf tmp,1,BANKED
    rlncf tmp,1,BANKED 
    rlncf tmp,1,BANKED
    movf tmp,w,BANKED
    banksel INDF2
    addwf INDF2,1,BANKED
    incf FSR2L,BANKED
    
     ;we are UPDN. Set indf2 to UPDN
    movlw UPDN
    banksel POSTDEC1
    movwf INDF2,BANKED
    movf FSR2L,w,BANKED
    sublw 0x03
    banksel WREG
    negf WREG,BANKED
    banksel FSR2L
    movwf FSR2L,BANKED
    return
    
updateinfoc:
     ;put x and y into INDF2 
    movff x,INDF2
    movff y,tmp
    banksel tmp
    rlncf tmp,1,BANKED
    rlncf tmp,1,BANKED
    rlncf tmp,1,BANKED 
    rlncf tmp,1,BANKED
    movf tmp,w,BANKED
    banksel INDF2
    addwf INDF2,1,BANKED
    incf FSR2L,BANKED
    
     ;we are LFTRT. Set indf2 to LFTRT
    movlw LFTRT
    banksel POSTDEC1
    movwf INDF2,BANKED
    movf FSR2L,w,BANKED
    sublw 0x03
    banksel WREG
    negf WREG,BANKED
    banksel FSR2L
    movwf FSR2L,BANKED
    return
    
;check for a sink
checksink:
    ;get shipindex from indf0
    banksel INDF0
    movf INDF0,w,BANKED
    andlw 0x0F
    banksel WREG
    decf WREG,BANKED
    banksel shipindex
    movwf shipindex,BANKED
    
    ;decrement hitcountsplayer[shipindex]
    lfsr FSR1,hitcountsplayer
    banksel FSR1L
    addwf FSR1L,1,BANKED
    decf INDF1,BANKED
    
    ;if INDF1 is 0:
    movf INDF1,w,BANKED
    bnz checksinkexit
    
    ;get x1,y1 from shipdataplayer[shipindex]
    lfsr FSR1,shipdataplayer
    banksel shipindex
    rlncf shipindex,w,BANKED
    banksel FSR1L
    addwf FSR1L,1,BANKED
    movf INDF1,w,BANKED
    andlw 0x0F
    banksel x1
    movwf x1,BANKED
    banksel INDF1
    movf INDF1,w,BANKED
    banksel WREG
    rrncf WREG,w,BANKED
    bcf WREG,7,BANKED
    rrncf WREG,w,BANKED
    bcf WREG,7,BANKED
    rrncf WREG,w,BANKED
    bcf WREG,7,BANKED
    rrncf WREG,w,BANKED
    bcf WREG,7,BANKED
    banksel y1
    movwf y1,BANKED
    banksel FSR1L
    incf FSR1L,BANKED
    
    ;if direction (INDF1) is UPDN:
    movf INDF1,w,BANKED
    sublw UPDN
    bnz notupdown
    call vertsink
notupdown:    
    banksel INDF1
    movf INDF1,w,BANKED
    sublw LFTRT
    bnz checksinkexit
    call horizsink    
   checksinkexit:
    return
    
;routine for vertical sinking of ship
vertsink:
    banksel shipindex
    rlncf shipindex,w,BANKED
    call sinksize
    banksel shipsize
    movwf shipsize,BANKED
    
    ;set indf0 to playerships[y1][x1], add 10 shipsize times
    lfsr FSR0,playerships
    banksel x1
    movf x1,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    banksel y1
    movf y1,w,BANKED
    mullw 0x0A
    banksel PRODL
    movf PRODL,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    
vertsinkloop:
    banksel INDF0
    bsf INDF0,5,BANKED
    movlw 0x0A
    addwf FSR0L,1,BANKED
    banksel shipsize
    decfsz shipsize,1,BANKED
    bra vertsinkloop
    return
;routine for horizontal sinking of ship
horizsink:
    banksel shipindex
    rlncf shipindex,w,BANKED
    call sinksize
    banksel shipsize
    movwf shipsize,BANKED
    
    ;set indf0 to playerships[y1][x1], add 10 shipsize times
    lfsr FSR0,playerships
    banksel x1
    movf x1,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    banksel y1
    movf y1,w,BANKED
    mullw 0x0A
    banksel PRODL
    movf PRODL,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    
horizsinkloop:
    banksel INDF0
    bsf INDF0,5,BANKED
    incf FSR0L,1,BANKED
    banksel shipsize
    decfsz shipsize,1,BANKED
    bra horizsinkloop
    return
    
updateinfoc1:
     ;put x and y into INDF2 
    movff x,INDF2
    movff y,tmp
    banksel tmp
    rlncf tmp,1,BANKED
    rlncf tmp,1,BANKED
    rlncf tmp,1,BANKED 
    rlncf tmp,1,BANKED
    movf tmp,w,BANKED
    banksel INDF2
    addwf INDF2,1,BANKED
    incf FSR2L,BANKED
    
     ;we are UPDN. Set indf2 to UPDN
    movlw UPDN
    banksel POSTDEC1
    movwf INDF2,BANKED
    movf FSR2L,w,BANKED
    sublw 0x03
    banksel WREG
    negf WREG,BANKED
    banksel FSR2L
    movwf FSR2L,BANKED
    return
    
    
;sink check for player
;check for a sink
checksinkp:
    ;get shipindex from indf0
    banksel INDF0
    movf INDF0,w,BANKED
    andlw 0x0F
    banksel WREG
    decf WREG,BANKED
    banksel shipindex
    movwf shipindex,BANKED
    
    ;decrement hitcountscomp[shipindex]
    lfsr FSR1,hitcountscomp
    banksel FSR1L
    addwf FSR1L,1,BANKED
    decf INDF1,BANKED
    
    ;if INDF1 is 0:
    movf INDF1,w,BANKED
    bnz checksinkexitp
    
    ;get x1,y1 from shipdatacomp[shipindex]
    lfsr FSR1,shipdatacomp
    banksel shipindex
    rlncf shipindex,w,BANKED
    banksel FSR1L
    addwf FSR1L,1,BANKED
    movf INDF1,w,BANKED
    andlw 0x0F
    banksel x1
    movwf x1,BANKED
    banksel INDF1
    movf INDF1,w,BANKED
    banksel WREG
    rrncf WREG,w,BANKED
    bcf WREG,7,BANKED
    rrncf WREG,w,BANKED
    bcf WREG,7,BANKED
    rrncf WREG,w,BANKED
    bcf WREG,7,BANKED
    rrncf WREG,w,BANKED
    bcf WREG,7,BANKED
    banksel y1
    movwf y1,BANKED
    banksel FSR1L
    incf FSR1L,BANKED
    
    ;if direction (INDF1) is UPDN:
    movf INDF1,w,BANKED
    sublw UPDN
    bnz notupdownp
    call vertsinkp
notupdownp:    
    banksel INDF1
    movf INDF1,w,BANKED
    sublw LFTRT
    bnz checksinkexitp
    call horizsinkp    
   checksinkexitp:
    return    
    
    
;vert sink walk player
;routine for vertical sinking of ship
vertsinkp:
    banksel shipindex
    rlncf shipindex,w,BANKED
    call sinksize
    banksel shipsize
    movwf shipsize,BANKED
    
    ;set indf0 to playerships[y1][x1], add 10 shipsize times
    lfsr FSR0,compships
    banksel x1
    movf x1,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    banksel y1
    movf y1,w,BANKED
    mullw 0x0A
    banksel PRODL
    movf PRODL,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    
vertsinkloopp:
    banksel INDF0
    bsf INDF0,5,BANKED
    movlw 0x0A
    addwf FSR0L,1,BANKED
    banksel shipsize
    decfsz shipsize,1,BANKED
    bra vertsinkloopp
    return
;routine for horizontal sinking of ship
horizsinkp:
    banksel shipindex
    rlncf shipindex,w,BANKED
    call sinksize
    banksel shipsize
    movwf shipsize,BANKED
    
    ;set indf0 to playerships[y1][x1], add 10 shipsize times
    lfsr FSR0,compships
    banksel x1
    movf x1,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    banksel y1
    movf y1,w,BANKED
    mullw 0x0A
    banksel PRODL
    movf PRODL,w,BANKED
    banksel FSR0L
    addwf FSR0L,1,BANKED
    
horizsinkloopp:
    banksel INDF0
    bsf INDF0,5,BANKED
    incf FSR0L,1,BANKED
    banksel shipsize
    decfsz shipsize,1,BANKED
    bra horizsinkloopp
    return    
    
;check done computer
donec:
    banksel hitcountc
    incf hitcountc,1,BANKED
    movf hitcountc,w,BANKED
    sublw 0x11 ;17
    bnz donecdone
    banksel gamedonec
    setf gamedonec,BANKED
    banksel plyrmoveflag
    clrf plyrmoveflag,BANKED
    banksel compdelay
    clrf compdelay,BANKED
donecdone:    
    return
    
;check done player
donep:
    banksel hitcountp
    incf hitcountp,1,BANKED
    movf hitcountp,w,BANKED
    sublw 0x11 ;17
    bnz donepdone
    banksel gamedonep
    setf gamedonep,BANKED
    banksel plyrmoveflag
    clrf plyrmoveflag,BANKED
    banksel compdelay
    clrf compdelay,BANKED
donepdone:    
    return    
end