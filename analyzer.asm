;;; 80 characters wide please ;;;;;;;;;;;;;;;;;;;;;;;;;; 8-space tabs please ;;;


;
;;;
;;;;;  ADB Protocol Analyzer
;;;
;


;;; Connections ;;;

;;;                                                         ;;;
;                          .--------.                         ;
;                  Supply -|01 \/ 08|- Ground                 ;
;         ADB <-->    RA5 -|02    07|- RA0    ---> UART TX    ;
;    Data LED <---    RA4 -|03    06|- RA1    <---            ;
;             --->    RA3 -|04    05|- RA2    <---            ;
;                          '--------'                         ;
;                                                             ;
;    Data LED is active low.                                  ;
;                                                             ;
;;;                                                         ;;;


;;; Assembler Directives ;;;

	list		P=PIC12F1840, F=INHX32, ST=OFF, MM=OFF, R=DEC, X=ON
	#include	P12F1840.inc
	__config	_CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
			;_FOSC_INTOSC	Internal oscillator, I/O on RA5
			;_WDTE_OFF	Watchdog timer disabled
			;_PWRTE_ON	Keep in reset for 64 ms on start
			;_MCLRE_OFF	RA3/!MCLR is RA3
			;_CP_OFF	Code protection off
			;_CPD_OFF	Data memory protection off
			;_BOREN_OFF	Brownout reset off
			;_CLKOUTEN_OFF	CLKOUT disabled, I/O on RA4
			;_IESO_OFF	Internal/External switch not needed
			;_FCMEN_OFF	Fail-safe clock monitor not needed
	__config	_CONFIG2, _WRT_OFF & _PLLEN_ON & _STVREN_ON & _LVP_ON
			;_WRT_OFF	Write protection off
			;_PLLEN_ON	4x PLL on
			;_STVREN_ON	Stack over/underflow causes reset
			;_LVP_OFF	High-voltage on Vpp to program


;;; Macros ;;;

DELAY	macro	value		;Delay 3*W cycles, set W to 0
	movlw	value
	decfsz	WREG,F
	bra	$-1
	endm

DNOP	macro
	bra	$+1
	endm


;;; Constants ;;;

;WARNING: do NOT use RA2 for ADB, the Schmitt Trigger takes too long to react
ADB_PIN	equ	RA5	;Pin on PORTA where ADB is connected
LED_PIN	equ	RA4	;Pin on PORTA where data LED is connected

			;AP_FLAG:
AP_RST	equ	7	;Set when a reset condition is detected, user clears
AP_COL	equ	6	;Set when the transmission collided, user clears
AP_RXCI	equ	5	;Set when command byte in AP_BUF, user clears
AP_RXDI	equ	4	;Set when data byte in AP_BUF, user clears
AP_DONE	equ	3	;Set when transmission or reception done, user clears
AP_TXI	equ	2	;User sets after filling AP_BUF, interrupt clears
AP_SRQ	equ	1	;User sets to request service, user clears
AP_RISE	equ	0	;Set when FSA should be entered on a rising edge too


;;; Variable Storage ;;;

	cblock	0x70	;Bank-common registers
	
	AP_FLAG	;ADB flags
	AP_FSAP	;Pointer to where to resume ADB state machine
	AP_SR	;ADB shift register
	AP_BUF	;ADB buffer
	AP_DTMR	;ADB down-cycle timer value
	T_QPTRS	;Tx queue pointers (high nibble is push, low nibble is pop)
	X9
	X8
	X7
	X6
	X5
	X4
	X3
	X2
	X1
	X0
	
	endc

	;Linear memory:
	;0x20E0-0x20EF - UART transmitter queue


;;; Vectors ;;;

	org	0x0		;Reset vector
	goto	Init

	org	0x4		;Interrupt vector


;;; Interrupt Handler ;;;

Interrupt
	movlp	0		;Copy the Timer0 flag into the carry bit so it
	bcf	STATUS,C	; doesn't change on us mid-stream
	btfsc	INTCON,TMR0IF	; "
	bsf	STATUS,C	; "
	btfsc	STATUS,C	;If the Timer0 flag is set and the interrupt is
	btfss	INTCON,TMR0IE	; enabled, handle it as an event for the ADB
	bra	$+2		; state machine
	call	IntAdbTimer	; "
	movlb	7		;If the ADB pin has had a negative or positive
	movlp	0		; edge, handle it as an event for the ADB state
	btfsc	IOCAF,ADB_PIN	; machine
	call	IntAdbEdge	; "
	btfss	INTCON,PEIE	;If peripheral interrupts are disabled, it's
	retfie			; not safe to service the UART right now
	movlb	0		;If the UART transmitter wants a byte, handle
	movlp	0		; it
	btfsc	PIR1,TXIF	; "
	call	IntTx		; "
	retfie

IntAdbTimer
	movlb	1		;Disable the Timer0 interrupt
	bcf	INTCON,TMR0IE	; "
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag and its mirror
	bcf	STATUS,C	; in the carry bit
	return
	
IntAdbEdge
	movlw	1 << ADB_PIN	;Toggle the edge that the IOC interrupt catches
	xorwf	IOCAN,F		; "
	xorwf	IOCAP,F		; "
	bcf	IOCAF,ADB_PIN	;Clear the interrupt flag
	btfsc	IOCAN,ADB_PIN	;If the edge we just caught is a rising edge,
	bra	IntAdbRising	; jump ahead, otherwise fall through
	;fall through

IntAdbFalling
	movlb	0		;If Timer0 overflowed, this falling edge is
	btfsc	STATUS,C	; the first after a too-long period, so handle
	bra	IntAdbTimeout	; it as a timeout
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag
	return

IntAdbRising
	movlb	0		;If Timer0 overflowed, this rising edge is at
	btfsc	STATUS,C	; the end of a reset pulse
	bra	IntAdbReset	; "
	movf	TMR0,W		;Save the current value of Timer0 so it can be
	movwf	AP_DTMR		; considered after its corresponding falling
	clrf	TMR0		; edge, then clear it and its flag
	bcf	INTCON,TMR0IF	; "
	btfss	AP_FLAG,AP_RISE	;If the flag isn't set that the state machine
	return			; wants to be resumed on a rising edge, done
	movf	AP_FSAP,W	;Resume the ADB state machine
	movlp	high AdbFsa	; "
	callw			; "
	movwf	AP_FSAP		;On return, save the address returned in W
	bcf	INTCON,TMR0IF	;Clear the Timer0 interrupt flag
	return

IntAdbReset
	bsf	AP_FLAG,AP_RST	;Set the reset flag
	clrf	AP_DTMR		;Clear the down timer
	;fall through

IntAdbTimeout
	clrf	AP_FSAP		;Reset the ADB state machine
	clrf	TMR0		;Reset Timer0 and its flag and disable its
	bcf	INTCON,TMR0IF	; interrupt
	bcf	INTCON,TMR0IE	; "
	return

IntTx
	swapf	T_QPTRS,W	;If the queue is empty, disable the TX
	xorwf	T_QPTRS,W	; interrupt and return
	btfss	STATUS,Z	; "
	bra	IntTx0		; "
	movlb	1		; "
	bcf	PIE1,TXIE	; "
	return			; "
IntTx0	movf	T_QPTRS,W	;Load the Tx pop pointer (low nibble of the
	andlw	B'00001111'	; combined pointer register) into FSR0
	iorlw	0xE0		; "
	movwf	FSR0L		; "
	moviw	FSR0++		;Pop the top byte off the TX queue and load it
	movlb	3		; for transmission, advancing the pointer
	movwf	TXREG		; "
	movlw	B'11110000'	;Copy the advanced pointer back into the low
	andwf	T_QPTRS,F	; nibble of the combined pointer register
	movf	FSR0L,W		; "
	andlw	B'00001111'	; "
	iorwf	T_QPTRS,F	; "
	return


;;; Mainline ;;;

Init
	banksel	OSCCON		;32 MHz (w/PLL) high-freq internal oscillator
	movlw	B'11110000'
	movwf	OSCCON

	banksel	RCSTA		;UART async mode, 115200 Hz, but receiver not
	movlw	B'01001000'	; enabled
	movwf	BAUDCON
	clrf	SPBRGH
	movlw	68
	movwf	SPBRGL
	movlw	B'00100110'
	movwf	TXSTA
	movlw	B'10000000'
	movwf	RCSTA
	clrf	TXREG
	
	banksel	IOCAN		;ADB sets IOCAF on negative edge
	movlw	1 << ADB_PIN
	movwf	IOCAN

	banksel	OPTION_REG	;Timer0 uses instruction clock, 1:32 prescaler,
	movlw	B'01010100'	; thus ticking every 4 us; weak pull-ups on
	movwf	OPTION_REG

	banksel	T1CON		;Timer1 ticks once per instruction cycle
	movlw	B'00000001'
	movwf	T1CON

	banksel	ANSELA		;All pins digital, not analog
	clrf	ANSELA

	banksel	LATA		;Ready to pull ADB low when output
	clrf	LATA

	banksel	TRISA		;TX and LED out, ADB is open-collector output,
	movlw	B'00101110'	; currently floating
	movwf	TRISA

	banksel	PIE1		;Receive interrupt enabled when peripheral
	movlw	1 << RCIE	; interrupts enabled
	movwf	PIE1

	movlw	12		;Delay approximately 2 ms at an instruction
	movwf	AP_BUF		; clock of 2 MHz until the PLL kicks in and the
PllWait	DELAY	110		; instruction clock gears up to 8 MHz
	decfsz	AP_BUF,F
	bra	PllWait

	clrf	AP_FLAG		;Set initial values of key globals
	clrf	AP_FSAP
	clrf	T_QPTRS

	movlw	0x20		;Set up FSRs to point more or less permanently
	movwf	FSR0H		; to linear memory
	movwf	FSR1H

	clrf	FSR0L		;Zero out linear memory
ClrLoop	clrf	INDF0
	incfsz	FSR0L,F
	bra	ClrLoop

	movlw	B'11001000'	;On-change interrupt, peripheral interrupts (for
	movwf	INTCON		; UART) and interrupt subsystem on


;;; Mainline ;;;

Analyzer
	movlb	2		;Turn off LED if it was on
	bsf	LATA,LED_PIN	; "
	btfsc	AP_FLAG,AP_RST	;Branch to reset if a reset was received, else
	goto	Analyz2		; wait until a command was received
	btfss	AP_FLAG,AP_RXCI	; "
	bra	Analyzer	; "
	bcf	AP_FLAG,AP_RXCI	;Clear the command flag
	bcf	AP_FLAG,AP_RXDI	;Clear other flags too from data activity that
	bcf	AP_FLAG,AP_DONE	; might have happened while waiting for a
	bcf	AP_FLAG,AP_COL	; command
	bcf	AP_FLAG,AP_TXI	; "
	bcf	AP_FLAG,AP_SRQ	;Not calling for service just yet
	swapf	T_QPTRS,W	;Load the Tx push pointer into FSR0
	andlw	B'00001111'	; "
	iorlw	0xE0		; "
	movwf	FSR0L		; "
	movf	AP_BUF,W	;Send the command byte
	movwf	INDF0		; "
	bcf	INTCON,PEIE	;Disable UART interrupts
	movlw	B'00001111'	;Advance the push pointer past the end of the
	andwf	T_QPTRS,F	; outbound command byte so it gets transmitted
	incf	FSR0L,W		; "
	andlw	B'00001111'	; "
	swapf	WREG,W		; "
	iorwf	T_QPTRS,F	; "
	bsf	INTCON,PEIE	;Reenable UART interrupts
	movlb	1		;Enable Tx interrupt
	bsf	PIE1,TXIE	; "
	btfss	AP_BUF,3	;If this command is not payload-bearing, loop to
	bra	Analyzer	; wait for the next one
	swapf	T_QPTRS,W	;Load the Tx push pointer into FSR0
	andlw	B'00001111'	; "
	iorlw	0xE0		; "
	movwf	FSR0L		; "
	clrf	INDF0		;Start with a payload length of zero
Analyz0	btfss	AP_FLAG,AP_RST	;If the flag is up for a reset or a command or
	btfsc	AP_FLAG,AP_RXCI	; the receive is done, finish up
	bra	Analyz1		; "
	btfsc	AP_FLAG,AP_DONE	; "
	bra	Analyz1		; "
	btfss	AP_FLAG,AP_RXDI	;Otherwise, keep waiting for a data byte
	bra	Analyz0		; "
	bcf	AP_FLAG,AP_RXDI	;Clear the byte-received flag
	movlb	2		;Payload has started, turn data LED on
	bcf	LATA,LED_PIN	; "
	swapf	T_QPTRS,W	;Load the Tx push pointer into FSR0
	andlw	B'00001111'	; "
	iorlw	0xE0		; "
	movwf	FSR0L		; "
	incf	INDF0,F		;Increment the length of the payload
	movf	INDF0,W		;Advance the pointer to where the next byte will
	andlw	B'00001111'	; be inserted
	addwf	FSR0L,F		; "
	bcf	FSR0L,4		; "
	movf	AP_BUF,W	;Add the received byte to the outbound packet
	movwf	INDF0		; "
	bra	Analyz0		;Loop for the next byte (if any)
Analyz1	bcf	INTCON,PEIE	;Disable UART interrupts
	movlw	B'00001111'	;Advance the push pointer past the end of the
	andwf	T_QPTRS,F	; outbound packet so it gets transmitted
	incf	FSR0L,W		; "
	andlw	B'00001111'	; "
	swapf	WREG,W		; "
	iorwf	T_QPTRS,F	; "
	bsf	INTCON,PEIE	;Reenable UART interrupts
	movlb	1		;Enable Tx interrupt
	bsf	PIE1,TXIE	; "
	goto	Analyzer	;Return to main
Analyz2	bcf	AP_FLAG,AP_RST	;Clear the reset flag
	swapf	T_QPTRS,W	;Load the Tx push pointer into FSR0
	andlw	B'00001111'	; "
	iorlw	0xE0		; "
	movwf	FSR0L		; "
	clrf	INDF0		;Send a SendReset byte
	bcf	INTCON,PEIE	;Disable UART interrupts
	movlw	B'00001111'	;Advance the push pointer past the end of the
	andwf	T_QPTRS,F	; outbound command byte so it gets transmitted
	incf	FSR0L,W		; "
	andlw	B'00001111'	; "
	swapf	WREG,W		; "
	iorwf	T_QPTRS,F	; "
	bsf	INTCON,PEIE	;Reenable UART interrupts
	movlb	1		;Enable Tx interrupt
	bsf	PIE1,TXIE	; "
	goto	Analyzer	;Return to main


;;; ADB State Machine ;;;

AdbFsa	org	0xF00

AdbFsaIdle
	clrf	TMR0		;Reset timer
	movf	AP_DTMR,W	;If the down time was 194-206 ticks (800 us +/-
	addlw	-207		; 3%), this is an attention pulse, so prepare
	btfsc	STATUS,C	; the shift register to receive a command byte
	retlw	low AdbFsaIdle	; and transition to receive the first bit
	addlw	13		; "
	btfss	STATUS,C	; "
	retlw	low AdbFsaIdle	; "
	movlw	0x01		; "
	movwf	AP_SR		; "
	retlw	low AdbFsaCmdBit; "

AdbFsaCmdBit
	btfsc	AP_DTMR,7	;If either the down time or the up time is over
	retlw	AdbFsaIdle	; 127 (508 us, ridiculous), throw up our hands
	btfsc	TMR0,7		; and wait for an attention pulse
	retlw	AdbFsaIdle	; "
	movf	TMR0,W		;Sum the value of Timer0 (the up time) with the
	addwf	AP_DTMR,W	; down time, then divide by two to get the mid-
	lsrf	WREG,W		; point; subtract the up time so carry contains
	subwf	TMR0,W		; 1 if up time was greater than the midpoint (a
	rlf	AP_SR,F		; 1 bit) else 0, rotate bit into shift register
	clrf	TMR0		;Reset Timer0 for next time
	btfss	STATUS,C	;If we rotated a 0 out of shift register, then
	retlw	low AdbFsaCmdBit; there are more command bits to come
	movf	AP_SR,W		;Else, move the contents of the filled shift
	movwf	AP_BUF		; register into the buffer and set flag to say
	bsf	AP_FLAG,AP_RXCI	; that a command has been received
	movlw	-12		;Set a timer to expire and interrupt after 48us
	movwf	TMR0		; so that the user program has time to decide
	bsf	INTCON,TMR0IE	; whether or not to request service or transmit
	bsf	AP_FLAG,AP_RISE	;Set to catch rising edge that starts Tlt
	retlw	low AdbFsaSrq	;Set to enter the state handling service reqs

AdbFsaSrq
	btfsc	BSR,0		;If for some reason we're here because of an
	bra	AFSrq0		; edge, cancel our timer interrupt, stop
	bcf	INTCON,TMR0IE	; catching rising edges, reset timer, and bail
	bcf	AP_FLAG,AP_RISE	; out to waiting for an attention pulse
	clrf	TMR0		; "
	retlw	low AdbFsaIdle	; "
AFSrq0	btfss	AP_FLAG,AP_SRQ	;If the user didn't call for a service request,
	retlw	low AdbFsaTlt	; just wait for Tlt to begin
	bcf	TRISA,ADB_PIN	;If the user did call for a service request,
	movlw	-63		; pull the pin low and set a timer for 252 us
	movlb	0		; above the 48 us we already waited to release
	movwf	TMR0		; it
	bsf	INTCON,TMR0IE	; "
	retlw	low AdbFsaSrqEnd; "

AdbFsaSrqEnd
	btfss	BSR,0		;On the off chance we're here because edge, go
	retlw	low AdbFsaSrqEnd; around again until the timer expires
	bsf	TRISA,ADB_PIN	;Release the pin that we pulled low to request
	retlw	low AdbFsaTlt	; service and wait for Tlt (could be right now)

AdbFsaTlt
	bcf	AP_FLAG,AP_RISE	;No longer need to catch rising edges
	movlw	-128		;Shorten the timeout period to 512 us, which is
	movwf	TMR0		; still too long to wait for a transmission
	btfss	AP_FLAG,AP_TXI	;If the user doesn't wish to transmit, just
	retlw	low AdbFsaTltEnd; wait for data to start
	movf	TMR1H,W		;Get a pseudorandom between 0 and 15, adjust it
	xorwf	TMR1L,W		; to between 199 and 214; that will make Timer0
	andlw	B'00001111'	; overflow in between 168us and 228us, which is
	addlw	-57		; close enough to the specced range of 160us to
	movwf	TMR0		; 240us to wait before transmitting
	movlw	B'11000000'	;Set the shift register so it outputs a 1 start
	movwf	AP_SR		; bit and then loads data from the buffer
	bsf	INTCON,TMR0IE	;Set timer to interrupt and bring us to the
	retlw	low AdbFsaTxBitD; transmission start state

AdbFsaTxBitD
	btfsc	BSR,0		;If we're here because of a timer interrupt,
	bra	AFTxBD0		; as we hope, skip ahead
	bsf	AP_FLAG,AP_COL	;If not, set the collision flag, clear the
	bcf	AP_FLAG,AP_TXI	; transmit flag, cancel the timer, and go back
	clrf	TMR0		; to waiting for an attention pulse
	bcf	INTCON,TMR0IE	; "
	retlw	low AdbFsaIdle	; "
AFTxBD0	bcf	TRISA,ADB_PIN	;Pull the pin low
	lslf	AP_SR,F		;Shift the next bit to send into carry bit
	btfss	STATUS,Z	;If we shifted the placeholder out of the shift
	bra	AFTxBD1		; register, continue, else skip ahead
	btfss	AP_FLAG,AP_TXI	;If there's no new byte ready to load, clear
	bcf	STATUS,C	; carry so we send a zero as our last bit
	btfss	AP_FLAG,AP_TXI	;If there's a new byte ready to load, load it,
	bra	AFTxBD1		; shift its MSB out and a 1 placeholder into
	rlf	AP_BUF,W	; its LSB and clear the transmit flag; else
	movwf	AP_SR		; leave the shift register all zeroes as a
	bcf	AP_FLAG,AP_TXI	; signal to the up phase state that we're done
AFTxBD1	movlw	-8		;Set a timer to interrupt in 8 cycles (32us) if
	movlb	0		; sending a 1 bit, double that to 16 cycles
	btfss	STATUS,C	; (64us) if we're sending a 0 bit, and also
	lslf	WREG,W		; save this value for use by the up phase state
	movwf	TMR0		; "
	movwf	AP_DTMR		; "
	bsf	INTCON,TMR0IE	; "
	retlw	low AdbFsaTxBitU; "

AdbFsaTxBitU
	btfss	BSR,0		;If we're here because of the falling edge we
	retlw	low AdbFsaTxBitU; just triggered, ignore and return posthaste
	bsf	TRISA,ADB_PIN	;Release the pin
	DELAY	2		;Wait for it to actually go high
	movlb	0		;If the pin is still low, we've collided; set
	btfsc	PORTA,ADB_PIN	; the collision flag, clear the transmit flag,
	bra	AFTxBU0		; and go back to waiting for an attention pulse
	bsf	AP_FLAG,AP_COL	; "
	bcf	AP_FLAG,AP_TXI	; "
	retlw	low AdbFsaIdle	; "
AFTxBU0	movf	AP_SR,W		;If the down phase let the shift register stay
	btfsc	STATUS,Z	; at zero, the bit we just transmitted is the
	bsf	AP_FLAG,AP_DONE	; stop bit and transmission is over, so set the
	btfsc	STATUS,Z	; done flag and return to waiting for an
	retlw	low AdbFsaIdle	; attention pulse
	movlw	B'00001000'	;Whatever delay (8 or 16) we did during the
	xorwf	AP_DTMR,W	; down phase, set a timer to do the other one
	movwf	TMR0		; "
	bsf	INTCON,TMR0IE	; "
	movlb	7		;Reverse the IOC interrupt and clear the flag
	bcf	IOCAP,ADB_PIN	; set by releasing the pin so the timer we just
	bsf	IOCAN,ADB_PIN	; set doesn't immediately get reset
	bcf	IOCAF,ADB_PIN	; "
	retlw	low AdbFsaTxBitD;Timer will take us to the down phase again

AdbFsaTltEnd
	clrf	TMR0		;This state is the end of Tlt and the start of
	retlw	low AdbFsaRxStrt; host or other device transmitting data

AdbFsaRxStrt
	movlw	0x01		;Start bit should be 1, but who cares, just set
	movwf	AP_SR		; up the shift register to receive the first
	clrf	TMR0		; data bit
	bsf	AP_FLAG,AP_RISE	;Catch rising edges to set receive timeout timer
	retlw	low AdbFsaRxBitD; "

AdbFsaRxBitD
	movlw	-31		;Set the timeout timer for 124 us, up time on a
	movwf	TMR0		; received bit should never be this long, and
	bsf	INTCON,TMR0IE	; wait for a falling edge or a timeout
	retlw	low AdbFsaRxBitU; "

AdbFsaRxBitU
	btfss	BSR,0		;If we got here because of a timer overflow, the
	bra	AFRxBD0		; data payload must be done, so disable catching
	bcf	AP_FLAG,AP_RISE	; rising edges, set the done flag, and return to
	bsf	AP_FLAG,AP_DONE	; idle
	retlw	low AdbFsaIdle	; "
AFRxBD0	movlw	31		;Compensate for us setting Timer0 to time out
	addwf	TMR0,F		; early
	btfsc	AP_DTMR,7	;If the down time is over 127 (508 us,
	bcf	AP_FLAG,AP_RISE	; ridiculous), throw up our hands and wait for
	btfsc	AP_DTMR,7	; an attention pulse
	retlw	low AdbFsaIdle	; "
	movf	TMR0,W		;Sum the value of Timer0 (the up time) with the
	addwf	AP_DTMR,W	; down time, then divide by two to get the mid-
	lsrf	WREG,W		; point; subtract the up time so carry contains
	subwf	TMR0,W		; 1 if up time was greater than the midpoint (a
	rlf	AP_SR,F		; 1 bit) else 0, rotate bit into shift register
	clrf	TMR0		;Reset Timer0 for next time
	btfss	STATUS,C	;If we rotated a 0 out of shift register, then
	retlw	low AdbFsaRxBitD; there are more data bits to come
	movf	AP_SR,W		;Else, move the contents of the filled shift
	movwf	AP_BUF		; register into the buffer and set flag to say
	bsf	AP_FLAG,AP_RXDI	; that a data byte has been received
	movlw	0x01		;Set up the shift register to receive the next
	movwf	AP_SR		; bit and wait for it
	retlw	low AdbFsaRxBitD; "


;;; End of Program ;;;
	end
