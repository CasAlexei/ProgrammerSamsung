;*******************************************************************************
;*******************************************************************************
;**                                                                           **
;**             ��������� ��� ������������� ����� �� ������ 24�xx.            **
;**                                                                           **
;**               ��� �� ATmega8A �������� ������� 4.00 ��                    **
;**                                Ver.1.3                                    **
;**                                                                           **
;*******************************************************************************
;*******************************************************************************

.include "m8def.inc"
 
    
	; ���������� ����������

.def	mulL=r0
.def	mulH=r1
.def	adwL=r2
.def	adwH=r3
.def	count_memL_t=r4						; ��������� �������� ������ �������� � ������
.def	count_memH_t=r5						;  �������������
;.def	=r6
;.def	=r7
.def	time_sleep=r8						; ������ ����� ��� ������� ��� �������� � sleep
.def	sleep_t=r9
.def	progres_temp=r10
.def	progres=r11
.def	col_prosh=r12						; ���������� �������� � ������ �������������
.def	count_m=r13
.def	temp0=r14
.def	count_s=r15
.def	temp_count=r16
.def	temp=r17
.def	temp1=r18
.def	temp2=r19
.def	char_ind=r20

.def	i2cstat_s=r21
.def	i2cstat=r22
.def	i2cdelay=r23
.def	i2cdata=r24
.def	i2cadr=r25

.def	count_memL=r26						; XL ����� ��������� �������� � ������ �������������
.def	count_memH=r27						; XH
;.def	=r28								; YL
;.def	=r29								; YH
;.def	=r30								; ZL
;.def	=r31								; ZH





;------------------------------------------------------------------------------
;	���������� ������ ��� ����������� LCD
                                            ;
.equ LCD_SCLK_port=PORTD					; ����� ����� ��� ������� SCLK 
.equ LCD_SCLK=5								; ������ SCLK ��� LCD
                                            ;     
.equ LCD_SDIN_port=PORTB					; ����� ����� ��� ������� SDIN
.equ LCD_SDIN=7								; ������ SDIN ��� LCD
                                            ;
.equ LCD_DC_port=PORTB						; ����� ����� ��� ������� DC
.equ LCD_DC=6								; ������ DC ��� LCD
                                            ;
.equ LCD_SCE_port=PORTD						; ����� ����� ��� ������� SCE
.equ LCD_SCE=4								; ������ SCE ��� LCD
                                            ;
.equ LCD_RES_port=PORTB						; ����� ����� ��� ������� RESET
.equ LCD_RES=4								; ������ RESET��� LCD




;------------------------------------------------------------------------------
;   ���� � ������ ��� I2C


.equ PORTI2C_M=PORTB						; ���� ���� ���������
.equ SDAP_M=3								; SDA Pin master
.equ SCLP_M=1								; SCL Pin master


.equ PORTI2C_S=PORTC						; ���� ���������� ������ ��������
.equ SDAP_S=1								; SDA Pin slave
.equ SCLP_S=0								; SCL Pin slave

.equ b_dir=0                                ; transfer direction bit in i2cadr

.equ i2crd=1
.equ i2cwr=0


;------------------------------------------------------------------------------
;   ���� � ������ ��� ������


.equ	PIN_BOT_L=PINC					; ������ "Left"
.equ	BOT_L=3

.equ	PIN_BOT_R=PIND					; ������ "Right"
.equ	BOT_R=3

.equ	PIN_BOT_Y=PINC					; ������ "Yes"
.equ	BOT_Y=5

.equ	PIN_BOT_N=PIND					; ������ "No"
.equ	BOT_N=2

.equ	PIN_BOT_1=PINC					; ������ "1"
.equ	BOT_1=4

.equ	PIN_BOT_2=PIND					; ������ "2"
.equ	BOT_2=0

.equ	PIN_BOT_3=PIND					; ������ "3"
.equ	BOT_3=1

;.equ	PIN_BOT_4=PINC					; ������ "4"
;.equ	BOT_4=3



	; ��� ������� ������. � ��� ���������� ����������� ������

.dseg

CHIP_NAME:	.byte 14						; �������� ���� (������ 0000h...0007h)
CHIP_CRUM:	.byte 11						; �������� ����� ���� (������ 0035h...003Fh)
CHIP_COUNT:	.byte 4							; ���������� ��������� ����� (������ 004Ah...004Bh)
TEMP_RAM:	.byte 1							; ��������� ���������� � ������ ���

BCD:		.byte 5

							; ���������� �������� � ������ �������������



.cseg

; ***** INTERRUPT VECTORS ************************************************
.org	$000
		rjmp RESET							; ��������� ������
.org	$0001
		rjmp aINT0							; 0x0001 - ������� ���������� 0
.org	$0002
		rjmp aINT1							; 0x0002 - ������� ���������� 1
.org	$0003
		rjmp aOC2							; 0x0003 - ���������� �������/�������� �2
.org	$0004
		rjmp aOVF2							; 0x0004 - ������������ �������/�������� �2
.org	$0005
		rjmp aICP1							; 0x0005 - ������ �������/�������� �1
.org	$0006
		rjmp aOC1A							; 0x0006 - ���������� � �������/�������� �1
.org	$0007
		rjmp aOC1B							; 0x0007 - ���������� � �������/�������� �1
.org	$0008
		rjmp aOVF1							; 0x0008 - ������������ �������/�������� �1
.org	$0009
		rjmp aOVF0							; 0x0009 - ������������ �������/�������� �0
.org	$000a
		rjmp aSPI							; 0x000a - �������� �� SPI ���������
.org	$000b
		rjmp aURXC							; 0x000b - USART, ����� ��������
.org	$000c
		rjmp aUDRE							; 0x000c - ������� ������ USART ����
.org	$000d
		rjmp aUTXC							; 0x000d - USART, �������� ���������
.org	$000e
		rjmp aADCC							; 0x000e - �������������� ADC ���������
.org	$000f
		rjmp aERDY							; 0x000f - EEPROM �����
.org	$0010
		rjmp aACI							; 0x0010 - Analog Comparator
.org	$0011
		rjmp aTWI							; 0x0011 - ���������� �� 2-wire Serial Interface
.org	$0012
		rjmp aSPMR							; 0x0012 - ���������� SPM





;aINT0:										; 0x0001 - ������� ���������� 0
aINT1:										; 0x0002 - ������� ���������� 1
aOC2:										; 0x0003 - ���������� �������/�������� �2
aOVF2:										; 0x0004 - ������������ �������/�������� �2
aICP1:										; 0x0005 - ������ �������/�������� �1
aOC1A:										; 0x0006 - ���������� � �������/�������� �1
aOC1B:										; 0x0007 - ���������� � �������/�������� �1
;aOVF1:										; 0x0008 - ������������ �������/�������� �1
aOVF0:										; 0x0009 - ������������ �������/�������� �0
aSPI:										; 0x000a - �������� �� SPI ���������
aURXC:										; 0x000b - USART, ����� ��������
aUDRE:										; 0x000c - ������� ������ USART ����
aUTXC:										; 0x000d - USART, �������� ���������
aADCC:										; 0x000e - �������������� ADC ���������
aERDY:										; 0x000f - EEPROM �����
aACI:										; 0x0010 - Analog Comparator
aTWI:										; 0x0011 - ���������� �� 2-wire Serial Interface
aSPMR:										; 0x0012 - ���������� SPM





RESET:

	; ������������� ����� 
 ldi 	temp, LOW(RAMEND)
 out	spl, Temp
 ldi 	temp, HIGH(RAMEND)
 out	sph, Temp



	; ������������� ������
		
 ldi	temp,0b11111111                       ; ������������� ����� B
 out	DDRB,temp

 ldi	temp,0b00000100                       ; ������������� ����� C
 out	DDRC,temp

 ldi	temp,0b11110000                       ; ������������� ����� D
 out	DDRD,temp 

 ldi	temp,0b11000000                       ; ������ �������� �� ����� ����� B
 out	PORTB,temp      
        
 ldi	temp,0b01111000                       ; ������ �������� �� ����� ����� C
 out	PORTC,temp      

 ldi	temp,0b00001111                       ; ������ �������� �� ����� ����� D
 out	PORTD,temp      



;------------------------------------------------------------------------------
	; ������������� ������� T1, ������������ ��� �������
		; 7 - OCIE2 ���� �� ���������� ���������� �� "����������"
		; 6 - TOIE2 ���� �� ���������� ���������� �� ������������ �������/�������� �1
		; 5 - TICIE1 ���� �� ���������� ���������� �� "������" ������� �������� �1
		; 4 - OCIE1A ���� �� ���������� ���������� �� "���������� �"
		; 3 - OCIE1B ���� �� ���������� ���������� �� "���������� �"
		; 2 - TOIE1 ���� �� ���������� ���������� �� ������������ �������/�������� �1
		; 1 - 
		; 0 - TOIE0 ���� �� ���������� ���������� �� ������������ �������/�������� �0

 ldi	temp,0b00000100                        ; ��������� ���������� �� "������������ T1"
 out	TIMSK,Temp


     ; ��������� �������/�������� T1 � ������ ������������
		; 7 - COM1A1 \ 
		; 6 - COM1A0  - ����� ������ ����� ��������� x 
		; 5 - COM1B1 /
		; 4 - COM1B0 /
		; 3 - FOC1A - �������������� ��������� ��������� ������ OCnx
		; 2 - FOC1B /
		; 1 - WGM11 - ����� ������ �������/�������� 
		; 0 - WGM10 / 

 ldi	temp,0b00000000
 out	TCCR1A,Temp


		; 7 - ICNC1 - ���������� ������ ���������� ����� ����� �������
		; 6 - ICES1 - ����� ��������� ������ ������� �������
		; 5 - 
		; 4 - WGM13 \
		; 3 - WGM12 - ����� ������ �������/��������
		; 2 - CS12 \
		; 1 - CS11  - ���������� �������� �������� (clk/64)
		; 0 - CS10 /

 ldi	temp,0b00000011
 out	TCCR1B,Temp



;------------------------------------------------------------------------------
	; ������������� ����������� �����������
		; 7 - ACD  - ���������� ����������� (0-�������, 1-��������)
		; 6 - ACBG - ����������� ����������� ��� (0-�� ���������, 1-���������)
		; 5 - AC0  - ��������� ���������
		; 4 - ACI  - ���� ���������� �� �����������
		; 3 - ACIE - ���������� ���������� �� �����������
		; 2 - ACIC - ����������� ����������� � ����� ������� �������/�������� �1 (0-��������,1-���������)
		; 1 - ACIS1 \
		; 0 - ACIS0  - ������� ������������� ���������� �� �����������

 ldi	temp,0b10010011
 out	ACSR,temp



;------------------------------------------------------------------------------

    ; ������������� ���

 ldi temp,0b000000010
 out ADMUX,temp
    ; 7 REFS1 = 0 
    ; 6 REFS0 = 0
    ; 5 ADLAR = 0 �������� �� ������ ����
    ; 4 MUX4 = 0
    ; 3 MUX3 = 0
    ; 2 MUX2 = 1
    ; 1 MUX1 = 1   ���� ���=ADC7
    ; 0 MUX0 = 1

 ldi temp,0b00010000
 out ADCSRA,temp
    ; 7 ADEN = ��������� ������ ���
    ; 6 ADSC = ������ ��������������
    ; 5 ADATE= ����������� �������������� 
    ; 4 ADIF = ���������� �� ��������� ��������������
    ; 3 ADIE = ���������� �� ��� ���������
    ; 2 ADPS2= 0 
    ; 1 ADPS1= 0
    ; 0 ADPS0= 0
 ldi temp,0b00000000                        ; ����������� ��������������
 out SFIOR,temp






;******************************************************************************
;*                                                                            *
;*                                                                            *
;*         ���������� ��������� � ������������� ���������� ���������          *
;*                                                                            *
;*                                                                            *
;******************************************************************************

 rcall	LCD_RESET							; ����� �������
 rcall	LCD_INIT							; ������������� �������
; rcall	LCD_CLR								; ������� �������
 clr	temp
 rcall	LCD_X								; ������������� ���������� X=0
 clr	temp
 rcall	LCD_Y								; ������������� ���������� Y=0
 rcall	i2c_init							; initialize I2C interface

 clr	sleep_t
 ldi	temp,10
 mov	time_sleep,temp						; ������ ���������� �� 40 ������

STEP0:
 clr	temp
 rcall	LCD_X							; ������������� ���������� X=0
 clr	temp
 rcall	LCD_Y							; ������������� ���������� Y=0

;	������� ������� �� �������
 clr	temp
 ldi	temp_count,84						; ��������� ������� �������� ��������� 130
PICLOGO:
 push	temp
 ldi	ZL,low(PICLOGO_TAB*2)				; ���������� ������ ������� ����� 
 ldi	ZH,high(PICLOGO_TAB*2)				;  ������� � ������� PICTURE1
 rcall	LCD_PUT_ARRAY					; ����� ������� �� ������� PICTURE1
 pop	temp
 inc	temp								; ����������� �������� ��������
 cpse	temp,temp_count
rjmp	PICLOGO								; ���������� ����� ������ 84, ����� ���������


 rcall	d500ms								; ���������� �������� 3 �������
 rcall	d500ms
 rcall	d500ms


 ldi	temp,18
 mov	col_prosh,temp						; ������������� ������� ���������� �������� (18 ����)






;********************************************************************************
;********************************************************************************
;**                                                                            **
;**                                                                            **
;**                     �������� ���� ������ ���������                         **
;**                                                                            **
;**                                                                            **
;********************************************************************************
;********************************************************************************

;********************************************************************************
;																				;
;	           ���1: ���� ����������� ���� � �������������						;
;																				;
;********************************************************************************

STEP1:
;	������� �������� ����������� ���� �� �������
 rcall	LCD_CLR
 clr	temp
 rcall	LCD_X
 clr	temp
 rcall	LCD_Y
 clr	temp
 ldi	temp_count,42						; ��������� ������� �������� ��������� 84
PIC_CONECT:
 push	temp
 ldi	ZL,low(PIC_CONECT_TAB*2)			; ���������� ������ ������� ����� 
 ldi	ZH,high(PIC_CONECT_TAB*2)			;  ������� � ������� PICTURE1
 rcall	LCD_PUT_ARRAY					; ����� ������� �� ������� PICTURE1
 pop	temp
 inc	temp								; ����������� �������� ��������
 cpse	temp,temp_count
rjmp	PIC_CONECT							; ���������� ����� ������ 84, ����� ���������

;	������� ����� "���������� ���"
 ldi	temp,0
 rcall	LCD_X
 ldi	temp,3
 rcall	LCD_Y
 clr	temp_count
FRAZA4_V:
 ldi	ZL,low(FRAZA4*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA4*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA4_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA4_V							; ���������� �������� ����� �� �������
FRAZA4_END:									; ������� �� ����� ������ ����� �� �������

;	������� ����� "����� ..."
 ldi	temp,31
 rcall	LCD_X
 ldi	temp,5
 rcall	LCD_Y
 clr	temp_count
FRAZA8_V:
 ldi	ZL,low(FRAZA8*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA8*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA8_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA8_V							; ���������� �������� ����� �� �������
FRAZA8_END:									; ������� �� ����� ������ ����� �� �������
 rcall	d500ms

 sei										; �������� ����������, ��������� ������
 clr	sleep_t								; �������� ������ ��������������

 sbi	PORTB,2								; ���������� ��������� �������� ��� ����
 sbi	PORTC,2								; ���������� ��������� �������� ��� EEPROM ��������

;	���������� ������
WAIT_STEP1:

 sbis	PIN_BOT_L,BOT_L						; �������� ��������� ������ "<"
 rjmp	BOT1_WAIT1

 sbis	PIN_BOT_R,BOT_R						; �������� ��������� ������ ">"
 rjmp	BOT2_WAIT1

 sbis	PIN_BOT_Y,BOT_Y						; �������� ��������� ������ "YES"
 rjmp	BOT3_WAIT1

 sbis	PIN_BOT_N,BOT_N						; �������� ��������� ������ "NO"
 rjmp	BOT4_WAIT1


; ��������� ������ �������������� ����� 1 ������
 cpse	time_sleep,sleep_t					; ������� ��������� ������� ��� ���������
 rjmp	WAIT_STEP1							; ���� ������ �� ������, ���������� ���������� ������

 rjmp	AUTO_OFF							; ���� ������ ��������, �� ��������� � �������� �����



BOT1_WAIT1:									; ��������� ������� ���������
 rcall	d13ms
 sbic	PIN_BOT_L,BOT_L
 rjmp	WAIT_STEP1
BOT1_WAIT1_N:								; ���� ���������� ������
 rcall	d13ms
 sbis	PIN_BOT_L,BOT_L
 rjmp	BOT1_WAIT1_N
 rjmp	STEP2								; ���� ������ ��������, �� ��������� �� 2-�� ���� 


BOT2_WAIT1:
 rcall	d13ms
 sbic	PIN_BOT_R,BOT_R
 rjmp	WAIT_STEP1
BOT2_WAIT1_N:								; ���� ���������� ������
 rcall	d13ms
 sbis	PIN_BOT_R,BOT_R
 rjmp	BOT2_WAIT1_N
 rjmp	STEP2								; ���� ������ ��������, �� ��������� �� 2-�� ���� 


BOT3_WAIT1:
 rcall	d13ms
 sbic	PIN_BOT_Y,BOT_Y
 rjmp	WAIT_STEP1
BOT3_WAIT1_N:								; ���� ���������� ������
 rcall	d13ms
 sbis	PIN_BOT_Y,BOT_Y
 rjmp	BOT3_WAIT1_N
 rjmp	STEP2								; ���� ������ ��������, �� ��������� �� 2-�� ���� 


BOT4_WAIT1:
 rcall	d13ms
 sbic	PIN_BOT_N,BOT_N
 rjmp	WAIT_STEP1
BOT4_WAIT1_N:								; ���� ���������� ������
 rcall	d13ms
 sbis	PIN_BOT_N,BOT_N
 rjmp	BOT4_WAIT1_N
 rjmp	STEP2								; ���� ������ ��������, �� ��������� �� 2-�� ���� 




;********************************************************************************
;																				;
;	���2: ������ ���������� ���� � ������� �� ������� �������� ���������		;
;																				;
;********************************************************************************
STEP2:
 cli										; ��������� ���������� � ��� ����� ������������� ������
 rcall	lcd_clr
;-----------------------------------------------------------------------------
; ������ �������� ���� � ���������� Master ������ �� ��������� I2C

 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$00							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_NAME,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$01							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_NAME+1,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$02							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_NAME+2,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$03							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_NAME+3,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$04							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_NAME+4,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$05							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_NAME+5,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$06							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_NAME+6,i2cdata



;-----------------------------------------------------------------------------
; ������ �������� ����� ���� � ���������� ������ �� ��������� I2C

 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$35							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_CRUM,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$36							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_CRUM+1,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$37							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_CRUM+2,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$38							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_CRUM+3,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$39							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_CRUM+4,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$3A							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_CRUM+5,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$3B							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_CRUM+6,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$3C							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_CRUM+7,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$3D							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_CRUM+8,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$3E							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_CRUM+9,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 ldi	i2cdata,$3F							; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
; rcall	i2c_stop							; �������� "����" - ����������� �����
 sts	CHIP_CRUM+10,i2cdata




;-----------------------------------------------------------------------------------
;	������� ��������� ���������� � ���� �� �������
;-----------------------------------------------------------------------------------

 rcall	LCD_CLR
;-------------------------------------------
;	������� ������� "�������� ����:"
 clr	temp_count
FRAZA1_V:
 ldi	ZL,low(FRAZA1*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA1*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA1_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA1_V							; ���������� �������� ����� �� �������
FRAZA1_END:									; ������� �� ����� ������ ����� �� �������
 ldi	temp,23								; ������ ���������� X
 rcall	LCD_X
 ldi	temp,1
 rcall	LCD_Y							; ������ ���������� Y
 lds	temp,CHIP_NAME
 rcall	LCD_PUT_CHAR					; ������� �������� ����
 lds	temp,CHIP_NAME+1
 rcall	LCD_PUT_CHAR
 lds	temp,CHIP_NAME+2
 rcall	LCD_PUT_CHAR
 lds	temp,CHIP_NAME+3
 rcall	LCD_PUT_CHAR
 lds	temp,CHIP_NAME+4
 rcall	LCD_PUT_CHAR
 lds	temp,CHIP_NAME+5
 rcall	LCD_PUT_CHAR
 lds	temp,CHIP_NAME+6
 rcall	LCD_PUT_CHAR


;-------------------------------------------
;	������� ������� "CRUM ����:"

 ldi	temp,10								; ������ ���������� X
 rcall	LCD_X
 ldi	temp,2
 rcall	LCD_Y							; ������ ���������� Y

 clr	temp_count
FRAZA2_V:
 ldi	ZL,low(FRAZA2*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA2*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA2_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA2_V							; ���������� �������� ����� �� �������
FRAZA2_END:									; ������� �� ����� ������ ����� �� �������
 ldi	temp,10								; ������ ���������� X
 rcall	LCD_X
 ldi	temp,3
 rcall	LCD_Y								; ������ ���������� Y
 lds	temp,CHIP_CRUM
 rcall	LCD_PUT_CHAR						; ������� �������� ����� ����
 lds	temp,CHIP_CRUM+1	
 rcall	LCD_PUT_CHAR
 lds	temp,CHIP_CRUM+2
 rcall	LCD_PUT_CHAR
 lds	temp,CHIP_CRUM+3
 rcall	LCD_PUT_CHAR
 lds	temp,CHIP_CRUM+4
 rcall	LCD_PUT_CHAR
 lds	temp,CHIP_CRUM+5
 rcall	LCD_PUT_CHAR
 lds	temp,CHIP_CRUM+6
 rcall	LCD_PUT_CHAR
 lds	temp,CHIP_CRUM+7
 rcall	LCD_PUT_CHAR
 lds	temp,CHIP_CRUM+8
 rcall	LCD_PUT_CHAR
 lds	temp,CHIP_CRUM+9
 rcall	LCD_PUT_CHAR
 lds	temp,CHIP_CRUM+10
 rcall	LCD_PUT_CHAR


;-------------------------------------------
;	������� ������� "<         >"

 ldi	temp,0								; ������ ���������� X
 rcall	LCD_X
 ldi	temp,5
 rcall	LCD_Y							; ������ ���������� Y
 ldi	temp,60
 rcall	LCD_PUT_CHAR
 ldi	temp,73								; ������ ���������� X
 rcall	LCD_X
 ldi	temp,5
 rcall	LCD_Y							; ������ ���������� Y
 ldi	temp,62
 rcall	LCD_PUT_CHAR


 sei										; �������� ����������, ��������� ������
 clr	sleep_t								; �������� ������ ��������������

;--------------------------------------------
;	���� ������� ������

WAIT_STEP2:

 sbis	PIN_BOT_L,BOT_L						; �������� ��������� ������ "<"
 rjmp	BOT1_WAIT2

 sbis	PIN_BOT_R,BOT_R						; �������� ��������� ������ ">"
 rjmp	BOT4_WAIT2

 sbis	PIN_BOT_N,BOT_N						; �������� ��������� ������ "NO"
 rjmp	BOT3_WAIT2


; ��������� ������ ��������������
 cpse	time_sleep,sleep_t					; ������� ��������� ������� ��� ���������
 rjmp	WAIT_STEP2							; ���� ������ �� ������, ���������� ���������� ������

 rjmp	AUTO_OFF							; ���� ������ ��������, �� ��������� � �������� �����



BOT1_WAIT2:									; ��������� ������� ���������
 rcall	d13ms
 sbic	PIN_BOT_L,BOT_L
 rjmp	WAIT_STEP2
BOT1_WAIT2_N:								; ���� ���������� ������
 rcall	d13ms
 sbis	PIN_BOT_L,BOT_L
 rjmp	BOT1_WAIT2_N
 rjmp	STEP1

BOT4_WAIT2:									; ��������� ������� ���������
 rcall	d13ms
 sbic	PIN_BOT_R,BOT_R
 rjmp	WAIT_STEP2
BOT4_WAIT2_N:								; ���� ���������� ������
 rcall	d13ms
 sbis	PIN_BOT_R,BOT_R
 rjmp	BOT4_WAIT2_N
 rjmp	STEP3								; ���� ������ ">" ������, �� ���� ������


BOT3_WAIT2:									; ��������� ������� ���������
 rcall	d13ms
 sbic	PIN_BOT_N,BOT_N
 rjmp	WAIT_STEP2
BOT3_WAIT2_N:								; ���� ���������� ������
 rcall	d13ms
 sbis	PIN_BOT_N,BOT_N
 rjmp	BOT3_WAIT2_N
 rjmp	STEP1								; ���� ������ ">" ������, �� ���� ������




;********************************************************************************
;																				;
;					���3: �������� �������� � ��������� ���						;
;																				;
;********************************************************************************

STEP3:

 rcall	LCD_CLR
;-------------------------------------------
;	������� ������� "��������"

 ldi	temp,19								; ������ ���������� X
 rcall	LCD_X
 ldi	temp,0
 rcall	LCD_Y							; ������ ���������� Y

 clr	temp_count
FRAZA5_V:
 ldi	ZL,low(FRAZA5*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA5*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA5_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA5_V							; ���������� �������� ����� �� �������
FRAZA5_END:									; ������� �� ����� ������ ����� �� �������

;-------------------------------------------
;	������� ������� "�������� ����"

 ldi	temp,0								; ������ ���������� X
 rcall	LCD_X
 ldi	temp,1
 rcall	LCD_Y							; ������ ���������� Y

 clr	temp_count
FRAZA6_V:
 ldi	ZL,low(FRAZA6*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA6*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA6_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA6_V							; ���������� �������� ����� �� �������
FRAZA6_END:									; ������� �� ����� ������ ����� �� �������


;-------------------------------------------
;	������� ������� "�������"

 ldi	temp,25								; ������ ���������� X
 rcall	LCD_X
 ldi	temp,4
 rcall	LCD_Y								; ������ ���������� Y

 clr	temp_count
FRAZA9_V:
 ldi	ZL,low(FRAZA9*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA9*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA9_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA9_V							; ���������� �������� ����� �� �������
FRAZA9_END:									; ������� �� ����� ������ ����� �� �������




;-------------------------------------------
;	������� ������� "<   ���   >"

 ldi	temp,0								; ������ ���������� X
 rcall	LCD_X
 ldi	temp,5
 rcall	LCD_Y								; ������ ���������� Y

 clr	temp_count
FRAZA7_V:
 ldi	ZL,low(FRAZA7*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA7*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA7_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	temp_count
 rjmp	FRAZA7_V							; ���������� �������� ����� �� �������
FRAZA7_END:									; ������� �� ����� ������ ����� �� �������



;--------------------------------------------
;	���� ������� ������
 
 ldi	temp_count,1						; ������������� ������� �� ������ ��������
 ldi	count_memH,0x01						; ������ ����� � ������ slave ��� ������ ��������
 ldi	count_memL,0x20						;
 rcall	VIBOR								; ���������� ������ ��������

 clr	sleep_t								; �������� ������ ��������������
 sei										; �������� ����������, ��������� ������


WAIT_STEP3:

 sbis	PIN_BOT_L,BOT_L						; �������� ��������� ������ "<"
 rjmp	BOT1_WAIT3

 sbis	PIN_BOT_Y,BOT_Y						; �������� ��������� ������ "YES"
 rjmp	BOT2_WAIT3

 sbis	PIN_BOT_N,BOT_N						; �������� ��������� ������ "NO"
 rjmp	BOT3_WAIT3

 sbis	PIN_BOT_R,BOT_R						; �������� ��������� ������ ">"
 rjmp	BOT4_WAIT3

 sbis	PIN_BOT_1,BOT_1						; �������� ��������� ������ "1"
 rjmp	BOT5_WAIT3

 sbis	PIN_BOT_2,BOT_2						; �������� ��������� ������ "2"
 rjmp	BOT6_WAIT3

 sbis	PIN_BOT_3,BOT_3						; �������� ��������� ������ "3"
 rjmp	BOT7_WAIT3


; ��������� ������ ��������������
 cpse	time_sleep,sleep_t					; ������� ��������� ������� ��� ���������
 rjmp	WAIT_STEP3							; ���� ������ �� ������, ���������� ���������� ������

 rjmp	AUTO_OFF							; ���� ������ ��������, �� ��������� � �������� �����



BOT1_WAIT3:									; ������ "<" ������. ������� "����� ���������� ��������"		
 rcall	d13ms
 sbic	PIN_BOT_L,BOT_L
 rjmp	WAIT_STEP3
BOT1_WAIT3_N:								; ���� ���������� ������
 rcall	d13ms
 sbis	PIN_BOT_L,BOT_L
 rjmp	BOT1_WAIT3_N
 rjmp	SELECT_PREW							; ���� ������ "<" ������, �� �������� ���������� ��������


BOT2_WAIT3:									; ������ "YES" ������. ������� "�������� ���"
 rcall	d13ms
 sbic	PIN_BOT_Y,BOT_Y
 rjmp	WAIT_STEP3
BOT2_WAIT3_N:								; ���� ���������� ������
 rcall	d13ms
 sbis	PIN_BOT_Y,BOT_Y
 rjmp	BOT2_WAIT3_N
 rjmp	STEP4								; ���� ������ ��������, �� ��������� � 4-�� ���� 


BOT3_WAIT3:									; ������ "NO" ������. ������� "��������� �� ���� ����� �����"
 rcall	d13ms
 sbic	PIN_BOT_N,BOT_N
 rjmp	WAIT_STEP3
BOT3_WAIT3_N:								; ���� ���������� ������
 rcall	d13ms
 sbis	PIN_BOT_N,BOT_N
 rjmp	BOT3_WAIT3_N
 rjmp	STEP1								; ���� ������ ��������, �� ��������� � 1-�� ���� 


BOT4_WAIT3:									; ������ ">" ������. ������� "����� �������� ��������"
 rcall	d13ms
 sbic	PIN_BOT_R,BOT_R
 rjmp	WAIT_STEP3
BOT4_WAIT3_N:								; ���� ���������� ������
 rcall	d13ms
 sbis	PIN_BOT_R,BOT_R
 rjmp	BOT4_WAIT3_N
 rjmp	SELECT_NEXT							; ���� ������ ">" ������, �� �������� ��������� ��������



BOT5_WAIT3:									; ������ "1" ������.
 rcall	d13ms
 sbic	PIN_BOT_1,BOT_1
 rjmp	WAIT_STEP3
BOT5_WAIT3_N:								; ���� ������ ������ ����� 2 ������, �� ��������� ������� ��������
 rcall	d500ms
 sbic	PIN_BOT_1,BOT_1
 rjmp	SELECT_1							;  ����� �������� ����� ���������� �������
 rcall	d500ms
 sbic	PIN_BOT_1,BOT_1
 rjmp	SELECT_1
 rcall	d500ms
 sbic	PIN_BOT_1,BOT_1
 rjmp	SELECT_1

 rjmp	SAVE_SELECT1						; ���� ������ "1" ������ ����� 2 ������, �� ��������� ������� ��������




BOT6_WAIT3:									; ������ "2" ������.
 rcall	d13ms
 sbic	PIN_BOT_2,BOT_2
 rjmp	WAIT_STEP3
BOT6_WAIT3_N:								; ���� ���������� ������
 rcall	d500ms
 sbic	PIN_BOT_2,BOT_2
 rjmp	SELECT_2							;  ����� �������� ����� ���������� �������
 rcall	d500ms
 sbic	PIN_BOT_2,BOT_2
 rjmp	SELECT_2
 rcall	d500ms
 sbic	PIN_BOT_2,BOT_2
 rjmp	SELECT_2

 rjmp	SAVE_SELECT2						; ���� ������ "2" ������ ����� 2 ������, �� ��������� ������� ��������



BOT7_WAIT3:									; ������ "3" ������.
 rcall	d13ms
 sbic	PIN_BOT_3,BOT_3
 rjmp	WAIT_STEP3
BOT7_WAIT3_N:								; ���� ���������� ������
 rcall	d500ms
 sbic	PIN_BOT_3,BOT_3
 rjmp	SELECT_3							;  ����� �������� ����� ���������� �������
 rcall	d500ms
 sbic	PIN_BOT_3,BOT_3
 rjmp	SELECT_3
 rcall	d500ms
 sbic	PIN_BOT_3,BOT_3
 rjmp	SELECT_3

 rjmp	SAVE_SELECT3						; ���� ������ "3" ������ ����� 2 ������, �� ��������� ������� ��������



SELECT_PREW:								; ������� ���������� �������� �� ������ �����
 clr	sleep_t								; �������� ������ ��������������

 cpi	temp_count,1						; ���� ��� ������ ��������, �� �������� ���������
 breq	VIBOR_P_Z 

 sbiw	XH:XL,60							; ������� ���������� �������� �� ������ �����,
 sbiw	XH:XL,60							;  �������� �� ����������� ���� ����� 176 (60+60+56)
 sbiw	XH:XL,60							;  ��� ���������� �������� � ����� ������
 sbiw	XH:XL,60
 sbiw	XH:XL,48

 rcall	VIBOR

 dec	temp_count

 rjmp	WAIT_STEP3

VIBOR_P_Z:
 mov	temp_count,col_prosh				; ������������� ������� �� ������ ��������
 ldi	count_memH,0x14						; ������ ����� � ������ slave ��� ������ ��������
 ldi	count_memL,0x40						;
 rcall	VIBOR								; ���������� ������ ��������

 rjmp	WAIT_STEP3




SELECT_NEXT:
 clr	sleep_t								; �������� ������ ��������������

 cp		temp_count,col_prosh				; ���������� ���������� ����� �������� � ����������� ��������
 breq	VIBOR_N_Z							; ���� �������� ���������, �� �������� � ������

 adiw	XH:XL,60							; ������� ��������� �������� �� ������ �����
 adiw	XH:XL,60							;  ���������� � ����������� ���� ����� 288 (60+60+60+60+48)
 adiw	XH:XL,60							;  ��� ���������� ��������� �������� � ����� ������
 adiw	XH:XL,60
 adiw	XH:XL,48
			
 rcall	VIBOR
 inc	temp_count							; ����������� ������� ������� ��������
 rjmp	WAIT_STEP3

VIBOR_N_Z:									; ���� ��������� �� ��������� ��������, ��
 ldi	temp_count,1						; ������������� ������� �� ������ ��������
 ldi	count_memH,0x01						; ������ ����� � ������ slave ��� ������ ��������
 ldi	count_memL,0x20						;
 rcall	VIBOR								; ���������� ������ ��������

 rjmp	WAIT_STEP3



VIBOR:										; ������� �������� �������� �� �������
 push	count_memH
 push	count_memL

 adiw	XH:XL,1								; ������ ��� ����� ��������
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start_s							; �������� "�����" ������� � �����
 mov	i2cdata,count_memH					; ����� ����� ������
 rcall	i2c_do_transfer_s					; ��������� ��������
 mov	i2cdata,count_memL					; ����� ����� ������
 rcall	i2c_do_transfer_s					; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start_s						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������
 rcall	i2c_do_transfer_s					; ��������� �������� (������)
 mov	temp,i2cdata
 
 cpi	temp,0x11							; ��������� ��� ����� ��������
 breq	VIBOR_S								; ������� ������� SAMSUNG
 cpi	temp,0x22							; ��������� ��� ����� ��������
 breq	VIBOR_X								; ������� ������� XEROX
 cpi	temp,0x33							; ��������� ��� ����� ��������
 breq	VIBOR_D								; ������� ������� DELL
 cpi	temp,0x44							; ��������� ��� ����� ��������
 breq	VIBOR_M								; ������� ������� MB

VIBOR_S:
 ldi	temp,0								; ������ ���������� X
 rcall	LCD_X
 ldi	temp,2
 rcall	LCD_Y								; ������ ���������� Y

 clr	count_s
FRAZA12_V:									; ����� "SAMSUNG"
 ldi	ZL,low(FRAZA12*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA12*2)					;
 ldi	temp,0
 add	ZL,count_s							; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA12_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	count_s
 rjmp	FRAZA12_V							; ���������� �������� ����� �� �������
FRAZA12_END:									; ������� �� ����� ������ ����� �� �������
 rjmp	VIBOR1

VIBOR_X:
 ldi	temp,0								; ������ ���������� X
 rcall	LCD_X
 ldi	temp,2
 rcall	LCD_Y								; ������ ���������� Y
 
 clr	count_s
FRAZA13_V:									; ����� "XEROX"
 ldi	ZL,low(FRAZA13*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA13*2)					;
 ldi	temp,0
 add	ZL,count_s							; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA13_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	count_s
 rjmp	FRAZA13_V							; ���������� �������� ����� �� �������
FRAZA13_END:								; ������� �� ����� ������ ����� �� �������
 rjmp	VIBOR1

VIBOR_D:
 ldi	temp,0								; ������ ���������� X
 rcall	LCD_X
 ldi	temp,2
 rcall	LCD_Y								; ������ ���������� Y
 clr	count_s
FRAZA14_V:									; ����� "Dell"
 ldi	ZL,low(FRAZA14*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA14*2)					;
 ldi	temp,0
 add	ZL,count_s							; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA14_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	count_s
 rjmp	FRAZA14_V							; ���������� �������� ����� �� �������
FRAZA14_END:								; ������� �� ����� ������ ����� �� ������� 
 rjmp	VIBOR1

VIBOR_M:
 ldi	temp,0								; ������ ���������� X
 rcall	LCD_X
 ldi	temp,2
 rcall	LCD_Y								; ������ ���������� Y
 clr	count_s
FRAZA15_V:									; ����� "MB"
 ldi	ZL,low(FRAZA15*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA15*2)					;
 ldi	temp,0
 add	ZL,count_s							; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA15_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	count_s
 rjmp	FRAZA15_V							; ���������� �������� ����� �� �������
FRAZA15_END:								; ������� �� ����� ������ ����� �� ������� 
; rjmp	VIBOR1

VIBOR1:
 ldi	temp,0								; ������ ���������� X
 rcall	LCD_X
 ldi	temp,3
 rcall	LCD_Y								; ������ ���������� Y

 clr	count_s

 adiw	XH:XL,1								; ������ �������� �������� ����� ��������� ������
;-----------------------------------------------------------------------------
; ������ �������� ���� � ���������� ������ �� ��������� I2C slave � ������� �� �������

VIBOR2:
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start_s							; �������� "�����" ������� � �����
 mov	i2cdata,count_memH					; ����� ����� ������
 rcall	i2c_do_transfer_s					; ��������� ��������
 mov	i2cdata,count_memL					; ����� ����� ������
 rcall	i2c_do_transfer_s					; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start_s						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer_s					; ��������� �������� (������)
 mov	temp,i2cdata
 rcall	LCD_PUT_CHAR						; ������� ������ �� �������

 adiw	XH:XL,1
 inc	count_s
 mov	temp,count_s
 cpi	temp,14								; ��������� ���� 14 ���
 brne	VIBOR2

 pop	count_memL
 pop	count_memH

 clr	sleep_t								; �������� ������ ��������������

ret



;--------------------------------------------
SELECT_1:									; �������� ��������, ����������� � ������ �������

EEPROM_read11:
 sbic	EECR,EEWE							; ����� ���������� ���������� ������, ���� ����
 rjmp	EEPROM_read11
 clr	temp
 out	EEARH,temp
 ldi	temp,0x10
 out	EEARL,temp							; ������� ����� � ������� ������
 sbi	EECR,EERE							; ������ ������
 in		temp_count,EEDR						; ��������� ���������� ����� �������� �� EEPROM
 
EEPROM_read12:
 sbic	EECR,EEWE							; ����� ���������� ���������� ������, ���� ����
 rjmp	EEPROM_read12
 clr	temp
 out	EEARH,temp
 ldi	temp,0x11
 out	EEARL,temp							; ������� ����� � ������� ������
 sbi	EECR,EERE							; ������ ������
 in		count_memH,EEDR						; ��������� ������� ����� �������� �� EEPROM

EEPROM_read13:
 sbic	EECR,EEWE							; ����� ���������� ���������� ������, ���� ����
 rjmp	EEPROM_read13
 clr	temp
 out	EEARH,temp
 ldi	temp,0x12
 out	EEARL,temp							; ������� ����� � ������� ������
 sbi	EECR,EERE							; ������ ������
 in		count_memL,EEDR						; ��������� ������� ����� �������� �� EEPROM

 rcall	VIBOR								; ���������� ������ ��������
 rjmp	WAIT_STEP3





;--------------------------------------------
SELECT_2:
EEPROM_read21:
 sbic	EECR,EEWE							; ����� ���������� ���������� ������, ���� ����
 rjmp	EEPROM_read21
 clr	temp
 out	EEARH,temp
 ldi	temp,0x20
 out	EEARL,temp							; ������� ����� � ������� ������
 sbi	EECR,EERE							; ������ ������
 in		temp_count,EEDR						; ��������� ���������� ����� �������� �� EEPROM
 
EEPROM_read22:
 sbic	EECR,EEWE							; ����� ���������� ���������� ������, ���� ����
 rjmp	EEPROM_read22
 clr	temp
 out	EEARH,temp
 ldi	temp,0x21
 out	EEARL,temp							; ������� ����� � ������� ������
 sbi	EECR,EERE							; ������ ������
 in		count_memH,EEDR						; ��������� ������� ����� �������� �� EEPROM

EEPROM_read23:
 sbic	EECR,EEWE							; ����� ���������� ���������� ������, ���� ����
 rjmp	EEPROM_read23
 clr	temp
 out	EEARH,temp
 ldi	temp,0x22
 out	EEARL,temp							; ������� ����� � ������� ������
 sbi	EECR,EERE							; ������ ������
 in		count_memL,EEDR						; ��������� ������� ����� �������� �� EEPROM

 rcall	VIBOR								; ���������� ������ ��������
 rjmp	WAIT_STEP3



;--------------------------------------------
SELECT_3:
EEPROM_read31:
 sbic	EECR,EEWE							; ����� ���������� ���������� ������, ���� ����
 rjmp	EEPROM_read31
 clr	temp
 out	EEARH,temp
 ldi	temp,0x30
 out	EEARL,temp							; ������� ����� � ������� ������
 sbi	EECR,EERE							; ������ ������
 in		temp_count,EEDR						; ��������� ���������� ����� �������� �� EEPROM
 
EEPROM_read32:
 sbic	EECR,EEWE							; ����� ���������� ���������� ������, ���� ����
 rjmp	EEPROM_read32
 clr	temp
 out	EEARH,temp
 ldi	temp,0x31
 out	EEARL,temp							; ������� ����� � ������� ������
 sbi	EECR,EERE							; ������ ������
 in		count_memH,EEDR						; ��������� ������� ����� �������� �� EEPROM

EEPROM_read33:
 sbic	EECR,EEWE							; ����� ���������� ���������� ������, ���� ����
 rjmp	EEPROM_read33
 clr	temp
 out	EEARH,temp
 ldi	temp,0x32
 out	EEARL,temp							; ������� ����� � ������� ������
 sbi	EECR,EERE							; ������ ������
 in		count_memL,EEDR						; ��������� ������� ����� �������� �� EEPROM

 rcall	VIBOR								; ���������� ������ ��������
 rjmp	WAIT_STEP3




;--------------------------------------------
SAVE_SELECT1:								; ��������� �������� � ������ �������

 rcall	d100ms
 cli										; ��������� ���������� �� ����� ������ � ������ EEPROM

EEPROM_write11:
 sbic	EECR,EEWE
 rjmp	EEPROM_write11						; ���� ���������� ���������� ������, ���� ����
 clr	temp
 out	EEARH,temp
 ldi	temp,0x10
 out	EEARL,temp							; ������� ����� � ������� ������
 out	EEDR,temp_count							; ������� ���������� ����� �������� � ������� ������
 sbi	EECR,EEMWE							; ���������� ���� EEMWE
 sbi	EECR,EEWE							; ������ ������ � EEPROM
 rcall	d100ms

EEPROM_write12:
 sbic	EECR,EEWE
 rjmp	EEPROM_write12						; ���� ���������� ���������� ������, ���� ����
 clr	temp
 out	EEARH,temp
 ldi	temp,0x11
 out	EEARL,temp							; ������� ����� � ������� ������
 out	EEDR,count_memH							; ������� ���������� ����� �������� � ������� ������
 sbi	EECR,EEMWE							; ���������� ���� EEMWE
 sbi	EECR,EEWE							; ������ ������ � EEPROM
 rcall	d100ms

EEPROM_write13:
 sbic	EECR,EEWE
 rjmp	EEPROM_write13						; ���� ���������� ���������� ������, ���� ����
 clr	temp
 out	EEARH,temp
 ldi	temp,0x12
 out	EEARL,temp							; ������� ����� � ������� ������
 out	EEDR,count_memL							; ������� ���������� ����� �������� � ������� ������
 sbi	EECR,EEMWE							; ���������� ���� EEMWE
 sbi	EECR,EEWE							; ������ ������ � EEPROM
 rcall	d100ms

 sei 

 rcall	SAVE_END							; ������� ������� "������"

BOT5_WAIT_SAVE:								; ���� ���������� ������
 rcall	d13ms
 sbis	PIN_BOT_1,BOT_1
 rjmp	BOT5_WAIT_SAVE

 rjmp	WAIT_STEP3
 

;--------------------------------------------
SAVE_SELECT2:								; ��������� �������� �� ������ �������
 rcall	d100ms
 cli										; ��������� ���������� �� ����� ������ � ������ EEPROM

EEPROM_write21:
 sbic	EECR,EEWE
 rjmp	EEPROM_write21						; ���� ���������� ���������� ������, ���� ����
 clr	temp
 out	EEARH,temp
 ldi	temp,0x20
 out	EEARL,temp							; ������� ����� � ������� ������
 out	EEDR,temp_count						; ������� ���������� ����� �������� � ������� ������
 sbi	EECR,EEMWE							; ���������� ���� EEMWE
 sbi	EECR,EEWE							; ������ ������ � EEPROM
 rcall	d100ms

EEPROM_write22:
 sbic	EECR,EEWE
 rjmp	EEPROM_write22						; ���� ���������� ���������� ������, ���� ����
 clr	temp
 out	EEARH,temp
 ldi	temp,0x21
 out	EEARL,temp							; ������� ����� � ������� ������
 out	EEDR,count_memH						; ������� ������� ����� �������� � ������� ������
 sbi	EECR,EEMWE							; ���������� ���� EEMWE
 sbi	EECR,EEWE							; ������ ������ � EEPROM
 rcall	d100ms

EEPROM_write23:
 sbic	EECR,EEWE
 rjmp	EEPROM_write23						; ���� ���������� ���������� ������, ���� ����
 clr	temp
 out	EEARH,temp
 ldi	temp,0x22
 out	EEARL,temp							; ������� ����� � ������� ������
 out	EEDR,count_memL						; ������� ������� ����� �������� � ������� ������
 sbi	EECR,EEMWE							; ���������� ���� EEMWE
 sbi	EECR,EEWE							; ������ ������ � EEPROM
 rcall	d100ms

 sei 

 rcall	SAVE_END							; ������� ������� "������"

BOT6_WAIT_SAVE:								; ���� ���������� ������
 rcall	d13ms
 sbis	PIN_BOT_2,BOT_2
 rjmp	BOT6_WAIT_SAVE

 rjmp	WAIT_STEP3




;--------------------------------------------
SAVE_SELECT3:								; ��������� �������� � ������ �������
 rcall	d100ms
 cli										; ��������� ���������� �� ����� ������ � ������ EEPROM

EEPROM_write31:
 sbic	EECR,EEWE
 rjmp	EEPROM_write31						; ���� ���������� ���������� ������, ���� ����
 clr	temp
 out	EEARH,temp
 ldi	temp,0x30
 out	EEARL,temp							; ������� ����� � ������� ������
 out	EEDR,temp_count						; ������� ���������� ����� �������� � ������� ������
 sbi	EECR,EEMWE							; ���������� ���� EEMWE
 sbi	EECR,EEWE							; ������ ������ � EEPROM
 rcall	d100ms

EEPROM_write32:
 sbic	EECR,EEWE
 rjmp	EEPROM_write32						; ���� ���������� ���������� ������, ���� ����
 clr	temp
 out	EEARH,temp
 ldi	temp,0x31
 out	EEARL,temp							; ������� ����� � ������� ������
 out	EEDR,count_memH						; ������� ������� ����� �������� � ������� ������
 sbi	EECR,EEMWE							; ���������� ���� EEMWE
 sbi	EECR,EEWE							; ������ ������ � EEPROM
 rcall	d100ms

EEPROM_write33:
 sbic	EECR,EEWE
 rjmp	EEPROM_write33						; ���� ���������� ���������� ������, ���� ����
 clr	temp
 out	EEARH,temp
 ldi	temp,0x32
 out	EEARL,temp							; ������� ����� � ������� ������
 out	EEDR,count_memL						; ������� ������� ����� �������� � ������� ������
 sbi	EECR,EEMWE							; ���������� ���� EEMWE
 sbi	EECR,EEWE							; ������ ������ � EEPROM
 rcall	d100ms

 sei 

 rcall	SAVE_END							; ������� ������� "������"

BOT7_WAIT_SAVE:								; ���� ���������� ������
 rcall	d13ms
 sbis	PIN_BOT_3,BOT_3
 rjmp	BOT7_WAIT_SAVE

 rjmp	WAIT_STEP3




;--------------------------------------------
SAVE_END:
 ldi	temp,0
 rcall	LCD_X
 ldi	temp,3
 rcall	LCD_Y

 clr	count_s
FRAZA19_V1:
 ldi	ZL,low(FRAZA19*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA19*2)					;
 ldi	temp,0
 add	ZL,count_s							; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA19_END1						; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	count_s
 rjmp	FRAZA19_V1							; ���������� �������� ����� �� �������
FRAZA19_END1:								; ������� �� ����� ������ ����� �� ������� 

ret

;********************************************************************************
;																				;
;					���4: ��������� �������� � ���������						;
;																				;
;********************************************************************************

STEP4:
 clr	sleep_t								; �������� ������ ��������������
 cli										; ��������� ���������� � ��� ����� ������������� ������


 rcall	LCD_CLR
 ldi	temp,0								; ������ ���������� X
 rcall	LCD_X
 ldi	temp,0
 rcall	LCD_Y							; ������ ���������� Y
 
 clr	count_s
FRAZA16_V:
 ldi	ZL,low(FRAZA16*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA16*2)					;
 ldi	temp,0
 add	ZL,count_s							; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA16_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	count_s
 rjmp	FRAZA16_V							; ���������� �������� ����� �� �������
FRAZA16_END:								; ������� �� ����� ������ ����� �� ������� 


;	������ ��� �� ������ ������������� ��������� ��������

 clr	temp_count							; ������� ������� �������� �����
 clr	count_m								; ������� ������� �������� ������ ���� 
 clr	progres								; ������� ������� ��������� ������ (0%)
 clr	progres_temp
 adiw	XH:XL,16							; ������������ �� ������ ��������� ��������
 mov	count_memH_t,count_memH
 mov	count_memL_t,count_memL
STEP4_1:
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start_s							; �������� "�����" ������� � �����
 mov	i2cdata,count_memH					; ����� ������� ����� ������
 rcall	i2c_do_transfer_s					; ��������� ��������
 mov	i2cdata,count_memL					; ����� ������� ����� ������
 rcall	i2c_do_transfer_s					; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start_s						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������, �.�. ������� ������� "����"
 rcall	i2c_do_transfer_s					; ��������� �������� (������)
 mov	temp,i2cdata

;	����� ����������� ��� � ������������ � ������������� ���
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 mov	i2cdata,count_m						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 mov	i2cdata,temp						; ������������� ������
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d6ms								; ���� ���� ������ ���������� � ������ ����


 inc	progres_temp
 mov	temp,progres_temp
 cpi	temp,50
 breq	STEP4_PROGRES
 rjmp	STEP4_1_1

 STEP4_PROGRES:
 ldi	temp,37
 rcall	LCD_X
 ldi	temp,1
 rcall	LCD_Y
 
 inc	progres								; ����������� ������� �� �������
 mov	temp,progres
 rcall LCD_PUT_CHAR						; ������� ������� ����������
 ldi	temp,48
 rcall LCD_PUT_CHAR						; ��������� 0
 ldi	temp,37
 rcall LCD_PUT_CHAR						; ��������� %

 clr	progres_temp


STEP4_1_1:
;	��������� ���� 255 ���
 inc	temp_count							; ����������� ������� ���������� ������
 inc	count_m								; ���������� �� 1 ����� ������ ��� ������
 adiw	XH:XL,1								; ������� ��������� ����� ������ �����
											;  ���������� �� ������� count_memH:count_memL
 cpi	temp_count,255						; ��������� ���� 255 ���
 brne	STEP4_1	



;	��������� ���������� 255 ����� ������������� ���� FF
 clr	temp_count							; ���������� ������� �����
STEP4_3:
 ldi	i2cadr,$A2+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 mov	i2cdata,count_m						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cdata,0xFF						; ������������� ������ FF
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d6ms								; ���� ���� ������ ���������� � ������ ����

 inc	progres_temp
 mov	temp,progres_temp
 cpi	temp,50
 breq	STEP4_PROGRES1
 rjmp	STEP4_3_1

STEP4_PROGRES1:
 ldi	temp,37
 rcall	LCD_X
 ldi	temp,1
 rcall	LCD_Y
 
 inc	progres
 mov	temp,progres
 rcall LCD_PUT_CHAR
 ldi	temp,48
 rcall LCD_PUT_CHAR
 ldi	temp,37
 rcall LCD_PUT_CHAR

 clr	progres_temp

;	��������� ���� 255 ���
STEP4_3_1:
 inc	temp_count							; ����������� ������� ���������� ������
 inc	count_m								; ���������� �� 1 ����� ������ ��� ������
											;  ���������� �� ������� count_memH:count_memL
 cpi	temp_count,255						; ��������� ���� 255 ����
 brne	STEP4_3	


 ldi	temp,37
 rcall	LCD_X
 ldi	temp,1
 rcall	LCD_Y
 
 ldi	temp,1								; ������� ������� 100%
 rcall LCD_PUT_CHAR
 ldi	temp,0
 rcall LCD_PUT_CHAR
 ldi	temp,0
 rcall LCD_PUT_CHAR
 ldi	temp,37
 rcall LCD_PUT_CHAR

 rcall	d500ms
 clr	sleep_t								; �������� ������ ��������������

;--------------------------------------------
;	��������� ���������� ������ � ����

;	������� ����� "��������� ���" 
 ldi	temp,0
 rcall	LCD_X
 ldi	temp,3
 rcall	LCD_Y

 clr	count_s
FRAZA18_V:
 ldi	ZL,low(FRAZA18*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA18*2)					;
 ldi	temp,0
 add	ZL,count_s							; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA18_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	count_s
 rjmp	FRAZA18_V							; ���������� �������� ����� �� �������
FRAZA18_END:									; ������� �� ����� ������ ����� �� ������� 

 rcall	d500ms
 rcall	d500ms

 clr	temp_count
 mov	count_memH,count_memH_t				; ��������������� ����� ��������� �������� � ������ �������������
 mov	count_memL,count_memL_t

STEP4_4:
;	������ ������ �� ����
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start							; �������� "�����" ������� � �����
 mov	i2cdata,temp_count					; ����� ����� ������
 rcall	i2c_do_transfer						; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; ��������� �������� (������)
 mov	temp,i2cdata

;	������ ������ �� ������ �������������
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���������� � �����
 rcall	i2c_start_s							; �������� "�����" ������� � �����
 mov	i2cdata,count_memH					; ����� ������� ����� ������
 rcall	i2c_do_transfer_s					; ��������� ��������
 mov	i2cdata,count_memL					; ����� ������� ����� ������
 rcall	i2c_do_transfer_s					; ��������� ��������
 ldi	i2cadr,$A0+i2crd					; ������ ����� ���������� � ������
 rcall	i2c_rep_start_s						; �������� "������ ������" � �����
 sec										; ��������� ����� ��������, �.�. ������� ������� "����"
 rcall	i2c_do_transfer_s					; ��������� �������� (������)
 mov	temp1,i2cdata

 cp		temp,temp1
 brne	ERROR_CP

 adiw	XH:XL,1
 inc	temp_count
 cpi	temp_count,255
 brne	STEP4_4	

 rjmp	STEP4_5

ERROR_CP:
 rcall	LCD_CLR

 ldi	temp,25
 rcall	LCD_X
 ldi	temp,1
 rcall	LCD_Y

;	������� ����� "������"
 clr	count_s
FRAZA20_V:
 ldi	ZL,low(FRAZA20*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA20*2)					;
 ldi	temp,0
 add	ZL,count_s							; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA20_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	count_s
 rjmp	FRAZA20_V							; ���������� �������� ����� �� �������
FRAZA20_END:								; ������� �� ����� ������ ����� �� ������� 

 ldi	temp,0
 rcall	LCD_X
 ldi	temp,2
 rcall	LCD_Y

;	������� ����� "�� ������"
 clr	count_s
FRAZA21_V:
 ldi	ZL,low(FRAZA21*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA21*2)					;
 ldi	temp,0
 add	ZL,count_s							; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA21_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	count_s
 rjmp	FRAZA21_V							; ���������� �������� ����� �� �������
FRAZA21_END:								; ������� �� ����� ������ ����� �� ������� 

 mov	temp,temp_count						; ������� �����, �� �������� ������
 rcall	LCD_PUT_CHAR

 clr	sleep_t								; �������� ������ ��������������
 sei										; �������� ����������, ��������� ������

 rjmp	STEP5


;-------------------------------------------
;	������� ����� "������ CRUM"
STEP4_5:
 ldi	temp,0
 rcall	LCD_X
 ldi	temp,5
 rcall	LCD_Y

 clr	count_s
FRAZA17_V:
 ldi	ZL,low(FRAZA17*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA17*2)					;
 ldi	temp,0
 add	ZL,count_s							; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA17_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	count_s
 rjmp	FRAZA17_V							; ���������� �������� ����� �� �������
FRAZA17_END:									; ������� �� ����� ������ ����� �� ������� 

 rcall	d500ms
 rcall	d500ms
 rcall	d500ms

;	��������� ������������ ��������� ������ ����. � ��� ������ ���� ������ ����� �� 0 �� 9 (0x30...0x39). ���� �� ���,
;		�� ������ ��������������� ������ �� 0x30
PROBAB_CRUM:
 lds	temp,CHIP_CRUM						; ��������� 1 ������ CRUM ����
 cpi	temp,0x40							; ��������� ��� ��������
 brsh	EXCHANGE_CRUM_1						; ���� ��� ������ ��� ����� 9, �� ������ ��� �� 0

 lds	temp,CHIP_CRUM+1					; ��������� 2 ������ CRUM ����
 cpi	temp,0x40							; ��������� ��� ��������
 brsh	EXCHANGE_CRUM_2						; ���� ��� ������ ��� ����� 9, �� ������ ��� �� 0

 lds	temp,CHIP_CRUM+2					; ��������� 3 ������ CRUM ����
 cpi	temp,0x40							; ��������� ��� ��������
 brsh	EXCHANGE_CRUM_3						; ���� ��� ������ ��� ����� 9, �� ������ ��� �� 0

 lds	temp,CHIP_CRUM+3					; ��������� 4 ������ CRUM ����
 cpi	temp,0x40							; ��������� ��� ��������
 brsh	EXCHANGE_CRUM_4						; ���� ��� ������ ��� ����� 9, �� ������ ��� �� 0

 lds	temp,CHIP_CRUM+4					; ��������� 5 ������ CRUM ����
 cpi	temp,0x40							; ��������� ��� ��������
 brsh	EXCHANGE_CRUM_5						; ���� ��� ������ ��� ����� 9, �� ������ ��� �� 0

 lds	temp,CHIP_CRUM+5					; ��������� 6 ������ CRUM ����
 cpi	temp,0x40							; ��������� ��� ��������
 brsh	EXCHANGE_CRUM_6						; ���� ��� ������ ��� ����� 9, �� ������ ��� �� 0

 lds	temp,CHIP_CRUM+6					; ��������� 7 ������ CRUM ����
 cpi	temp,0x40							; ��������� ��� ��������
 brsh	EXCHANGE_CRUM_7						; ���� ��� ������ ��� ����� 9, �� ������ ��� �� 0

 lds	temp,CHIP_CRUM+7					; ��������� 8 ������ CRUM ����
 cpi	temp,0x40							; ��������� ��� ��������
 brsh	EXCHANGE_CRUM_8						; ���� ��� ������ ��� ����� 9, �� ������ ��� �� 0

 lds	temp,CHIP_CRUM+8					; ��������� 9 ������ CRUM ����
 cpi	temp,0x40							; ��������� ��� ��������
 brsh	EXCHANGE_CRUM_9						; ���� ��� ������ ��� ����� 9, �� ������ ��� �� 0

 lds	temp,CHIP_CRUM+9					; ��������� 10 ������ CRUM ����
 cpi	temp,0x40							; ��������� ��� ��������
 brsh	EXCHANGE_CRUM_10					; ���� ��� ������ ��� ����� 9, �� ������ ��� �� 0

 lds	temp,CHIP_CRUM+10					; ��������� 11 ������ CRUM ����
 cpi	temp,0x40							; ��������� ��� ��������
 brsh	EXCHANGE_CRUM_11					; ���� ��� ������ ��� ����� 9, �� ������ ��� �� 0


 rjmp	EXCHANGE_CRUM						; ���� �������� ����� � �������, ����� ����������� ��� �� 1

EXCHANGE_CRUM_1:
 ldi	temp,0x30
 sts	CHIP_CRUM,temp
 rjmp	PROBAB_CRUM
EXCHANGE_CRUM_2:
 ldi	temp,0x30
 sts	CHIP_CRUM+1,temp
 rjmp	PROBAB_CRUM
EXCHANGE_CRUM_3:
 ldi	temp,0x30
 sts	CHIP_CRUM+2,temp
 rjmp	PROBAB_CRUM
EXCHANGE_CRUM_4:
 ldi	temp,0x30
 sts	CHIP_CRUM+3,temp
 rjmp	PROBAB_CRUM
EXCHANGE_CRUM_5:
 ldi	temp,0x30
 sts	CHIP_CRUM+4,temp
 rjmp	PROBAB_CRUM
EXCHANGE_CRUM_6:
 ldi	temp,0x30
 sts	CHIP_CRUM+5,temp
 rjmp	PROBAB_CRUM
EXCHANGE_CRUM_7:
 ldi	temp,0x30
 sts	CHIP_CRUM+6,temp
 rjmp	PROBAB_CRUM
EXCHANGE_CRUM_8:
 ldi	temp,0x30
 sts	CHIP_CRUM+7,temp
 rjmp	PROBAB_CRUM
EXCHANGE_CRUM_9:
 ldi	temp,0x30
 sts	CHIP_CRUM+8,temp
 rjmp	PROBAB_CRUM
EXCHANGE_CRUM_10:
 ldi	temp,0x30
 sts	CHIP_CRUM+9,temp
 rjmp	PROBAB_CRUM
EXCHANGE_CRUM_11:
 ldi	temp,0x30
 sts	CHIP_CRUM+10,temp



;	������ �������� ����� ����, ����� ���������� ��������� 7 �������� �� 1

EXCHANGE_CRUM:
 lds temp,CHIP_CRUM+10						; ��������� �� ��� 11-� ������ �������� CRUM
 inc temp									;  ��������� �� 1 11-� ������ CRUM
 sts CHIP_CRUM+10,temp						;  ��������� � ��� ���������
 cpi temp,0x3A								;  ���������� �������� � 10
 brne EXCHANGE_CRUM_END						;  ���� ������ ��� ����� 10, �� ������� 
 ldi temp,0x30								;  ���� ����� 10, �� ����������� 10 ������
 sts CHIP_CRUM+10,temp						;  ����� ����������� �������
 
 lds temp,CHIP_CRUM+9						; ��������� �� ��� 10-� ������ �������� CRUM
 inc temp									;  ��������� �� 1 10-� ������ CRUM
 sts CHIP_CRUM+9,temp						;  ��������� � ��� ���������
 cpi temp,0x3A								;  ���������� �������� � 10
 brne EXCHANGE_CRUM_END						;  ���� ������ ��� ����� 10, �� ������� 
 ldi temp,0x30								;  ���� ����� 10, �� ����������� 9 ������
 sts CHIP_CRUM+9,temp						;  ����� ����������� �������
 
 lds temp,CHIP_CRUM+8						; ��������� �� ��� 9-� ������ �������� CRUM
 inc temp									;  ��������� �� 1 9-� ������ CRUM
 sts CHIP_CRUM+8,temp						;  ��������� � ��� ���������
 cpi temp,0x3A								;  ���������� �������� � 10
 brne EXCHANGE_CRUM_END						;  ���� ������ ��� ����� 10, �� ������� 
 ldi temp,0x30								;  ���� ����� 10, �� ����������� 8 ������
 sts CHIP_CRUM+8,temp						;  ����� ����������� �������
 
 lds temp,CHIP_CRUM+7						; ��������� �� ��� 8-� ������ �������� CRUM
 inc temp									;  ��������� �� 1 8-� ������ CRUM
 sts CHIP_CRUM+7,temp						;  ��������� � ��� ���������
 cpi temp,0x3A								;  ���������� �������� � 10
 brne EXCHANGE_CRUM_END						;  ���� ������ ��� ����� 10, �� ������� 
 ldi temp,0x30								;  ���� ����� 10, �� ����������� 7 ������
 sts CHIP_CRUM+7,temp						;  ����� ����������� �������
 
 lds temp,CHIP_CRUM+6						; ��������� �� ��� 7-� ������ �������� CRUM
 inc temp									;  ��������� �� 1 7-� ������ CRUM
 sts CHIP_CRUM+6,temp						;  ��������� � ��� ���������
 cpi temp,0x3A								;  ���������� �������� � 10
 brne EXCHANGE_CRUM_END						;  ���� ������ ��� ����� 10, �� ������� 
 ldi temp,0x30								;  ���� ����� 10, �� ����������� 6 ������
 sts CHIP_CRUM+6,temp						;  ����� ����������� �������
 
 lds temp,CHIP_CRUM+5						; ��������� �� ��� 6-� ������ �������� CRUM
 inc temp									;  ��������� �� 1 6-� ������ CRUM
 sts CHIP_CRUM+5,temp						;  ��������� � ��� ���������
 cpi temp,0x3A								;  ���������� �������� � 10
 brne EXCHANGE_CRUM_END						;  ���� ������ ��� ����� 10, �� ������� 
 ldi temp,0x30								;  ���� ����� 10, �� ����������� 5 ������
 sts CHIP_CRUM+5,temp						;  ����� ����������� �������
 
 lds temp,CHIP_CRUM+4						; ��������� �� ��� 5-� ������ �������� CRUM
 inc temp									;  ��������� �� 1 5-� ������ CRUM
 sts CHIP_CRUM+4,temp						;  ��������� � ��� ���������
 cpi temp,0x3A								;  ���������� �������� � 10
 brne EXCHANGE_CRUM_END						;  ���� ������ ��� ����� 10, �� ������� 
 ldi temp,0x30								;  ���� ����� 10, �� ����������� 4 ������
 sts CHIP_CRUM+4,temp						;  ����� ����������� �������

EXCHANGE_CRUM_END:

;	������������ �������� ����� ����� ����������
 ldi	temp,0x35
 mov	count_m,temp						; ������������� ������� ������ ���� �� 0x0018
 
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 mov	i2cdata,count_m						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 lds	temp,CHIP_CRUM
 mov	i2cdata,temp						; ������������� ������
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d6ms								; ���� ���� ������ ���������� � ������ ����
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 mov	i2cdata,count_m						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 lds	temp,CHIP_CRUM+1
 mov	i2cdata,temp						; ������������� ������
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d6ms								; ���� ���� ������ ���������� � ������ ����
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 mov	i2cdata,count_m						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 lds	temp,CHIP_CRUM+2
 mov	i2cdata,temp						; ������������� ������
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d6ms								; ���� ���� ������ ���������� � ������ ����
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 mov	i2cdata,count_m						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 lds	temp,CHIP_CRUM+3
 mov	i2cdata,temp						; ������������� ������
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d6ms								; ���� ���� ������ ���������� � ������ ����
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 mov	i2cdata,count_m						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 lds	temp,CHIP_CRUM+4
 mov	i2cdata,temp						; ������������� ������
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d6ms								; ���� ���� ������ ���������� � ������ ����
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 mov	i2cdata,count_m						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 lds	temp,CHIP_CRUM+5
 mov	i2cdata,temp						; ������������� ������
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d6ms								; ���� ���� ������ ���������� � ������ ����
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 mov	i2cdata,count_m						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 lds	temp,CHIP_CRUM+6
 mov	i2cdata,temp						; ������������� ������
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d6ms								; ���� ���� ������ ���������� � ������ ����
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 mov	i2cdata,count_m						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 lds	temp,CHIP_CRUM+7
 mov	i2cdata,temp						; ������������� ������
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d6ms								; ���� ���� ������ ���������� � ������ ����
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 mov	i2cdata,count_m						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 lds	temp,CHIP_CRUM+8
 mov	i2cdata,temp						; ������������� ������
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d6ms								; ���� ���� ������ ���������� � ������ ����
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 mov	i2cdata,count_m						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 lds	temp,CHIP_CRUM+9
 mov	i2cdata,temp						; ������������� ������
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d6ms								; ���� ���� ������ ���������� � ������ ����
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 mov	i2cdata,count_m						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 lds	temp,CHIP_CRUM+10
 mov	i2cdata,temp						; ������������� ������
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d6ms								; ���� ���� ������ ���������� � ������ ����


;	����� ����� � ��� "Crum By Hwang.sk"
 ldi	temp,0x18
 mov	count_m,temp						; ������������� ������� ������ ���� �� 0x0118
 clr	temp_count
FRAZA10_V:									; ����� ����� "Crum By Hwang.sk"
 ldi	ZL,low(FRAZA10*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA10*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp

 ldi	i2cadr,$A2+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 mov	i2cdata,count_m						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 mov	i2cdata,temp						; ������������� ������ FF
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d6ms								; ���� ���� ������ ���������� � ������ ����

 inc	temp_count
 inc	count_m
 cpi	temp_count,16
 brne	FRAZA10_V							; ���������� �������� ����� �� �������
FRAZA10_END:								; ������� �� �����


;	����� ����� � ��� "ChipResetter V.1"
 ldi	temp,0x40
 mov	count_m,temp						; ������������� ������� ������ ���� �� 0x0118
 clr	temp_count

 ldi	temp,0
 rcall	LCD_X
 ldi	temp,4
 rcall	LCD_Y


; ����� ����� "Crum By Hwang.sk"
FRAZA11_V:
 ldi	ZL,low(FRAZA11*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA11*2)					;
 ldi	temp,0
 add	ZL,temp_count						; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp

 ldi	i2cadr,$A2+i2cwr					; ������ ����� ���� � ������� "������"
 rcall	i2c_start							; �������� ����� � �������
 mov	i2cdata,count_m						; ������ ����� ������ ��� ������
 rcall	i2c_do_transfer						; ��������� ��������
 mov	i2cdata,temp						; ������������� ������ FF
 rcall	i2c_do_transfer						; ��������� ��������
 rcall	i2c_stop							; �������� ���� �������
 rcall	d6ms								; ���� ���� ������ ���������� � ������ ����

 inc	temp_count
 inc	count_m
 cpi	temp_count,16
 brne	FRAZA11_V							; ���������� �������� ����� �� �������
FRAZA11_END:								; ������� �� �����



 rcall	i2c_stop							; �������� "����" - ����������� �����
 rcall	i2c_stop_s



;	������� ������� "������"
 rcall	LCD_CLR
 ldi	temp,0
 rcall	LCD_X
 ldi	temp,2
 rcall	LCD_Y

 clr	count_s
FRAZA19_V:
 ldi	ZL,low(FRAZA19*2)					; ��������� �� ������ ������
 ldi	ZH,high(FRAZA19*2)					;
 ldi	temp,0
 add	ZL,count_s							; ��������� ��������� ����� ����� � �������
 adc	ZH,temp
 lpm										; ��������� �� ������� ��������� �����
 mov	temp,r0								; ��������� ����������� ����� � ���������� temp
 cpi	temp,23								; ��������� �� ����� �������
 breq	FRAZA19_END							; ���� ������� �������� ������� ����� 10, ������ ������ ����������
 rcall	LCD_PUT_CHAR						; ������� ����� �� �������
 inc	count_s
 rjmp	FRAZA19_V							; ���������� �������� ����� �� �������
FRAZA19_END:									; ������� �� ����� ������ ����� �� ������� 



;********************************************************************************
;																				;
;                         ���5: ��� ������ � �����                              ;
;																				;
;********************************************************************************

 clr	sleep_t								; �������� ������ ��������������
 sei										; �������� ����������, ��������� ������

 cbi	PORTB,2								; ��������� ��������� �������� ��� ����
 cbi	PORTC,2								; ��������� ��������� �������� ��� EEPROM ��������

 rcall	d500ms
 rcall	d500ms
 rcall	d500ms

 rjmp	Step1




STEP5:
 sbis	PIN_BOT_R,BOT_R								; �������� ��������� ������ ">"
 rjmp	BOT4_WAIT5

 sbis	PIN_BOT_N,BOT_N								; �������� ��������� ������ "NO"
 rjmp	BOT3_WAIT1

; ��������� ������ ��������������
 cpse	time_sleep,sleep_t					; ������� ��������� ������� ��� ���������
 rjmp	STEP5								; ���� ������ �� ������, ���������� ���������� ������

 rjmp	AUTO_OFF							; ���� ������ ��������, �� ��������� � �������� �����



BOT4_WAIT5:									; ��������� ������� ���������
 rcall	d13ms
 sbic	PIN_BOT_R,BOT_R
 rjmp	STEP5
BOT4_WAIT5_N:								; ���� ���������� ������
 rcall	d13ms
 sbis	PIN_BOT_R,BOT_R
 rjmp	BOT4_WAIT5_N

 rjmp	STEP0								; ���� ������ ">" ������, �� ��������� � ����� ������ ���������


BOT3_WAIT5:									; ��������� ������� ���������
 rcall	d13ms
 sbic	PIN_BOT_N,BOT_N
 rjmp	STEP5
BOT3_WAIT5_N:								; ���� ���������� ������
 rcall	d13ms
 sbis	PIN_BOT_N,BOT_N
 rjmp	BOT3_WAIT5_N

 rjmp	STEP0								; ���� ������ ">" ������, �� ��������� � ����� ������ ���������


MAIN_LOOP:
rjmp	MAIN_LOOP							; ��������� ���� ����������










;****************************************************************************
;																			*
;             ������ �������������� ��� ������� �������������               *
;																			*
;****************************************************************************

AUTO_OFF:

 ldi	temp,0b00000000                        ; ��������� ���������� �� "������������ T1"
 out	TIMSK,Temp

 sei
;	��������� ������� ���������� INT0 ��� ������ �� ������� ������
 ldi	temp,0b01000000
 out	GICR,temp



;	������� �������� ��������� ������
 rcall	LCD_CLR
 clr	temp
 rcall	LCD_X
 clr	temp
 rcall	LCD_Y
 clr	temp
 ldi	temp_count,84						; ��������� ������� �������� ��������� 84
PIC_SLEEP:
 push	temp
 ldi	ZL,low(PIC_SLEEP_TAB*2)				; ���������� ������ ������� ����� 
 ldi	ZH,high(PIC_SLEEP_TAB*2)			;  ������� � ������� SLEEP
 rcall	LCD_PUT_ARRAY					; ����� ������� �� ������� SLEEP
 pop	temp
 inc	temp								; ����������� �������� ��������
 cpse	temp,temp_count
rjmp	PIC_SLEEP							; ���������� ����� ������ 84, ����� ���������



; ��������� ������� � ������ ����� � �������� ������� ��������� ����������
;   �� ��������� �� ������

;-------------------------------------------
;	������ ����������� �����������������
		; 7 - SE   - ���������� �������� � ����� ����������� ����������� (1-���������)
		; 6 - SM2  \
		; 5 - SM1   - ����� ������ ����������� �����������������
		; 4 - SM0  /
		; 3 - ISC11\
		; 2 - ISC10 - ������� ��������� �������� ���������� INT1 
		; 1 - ISC01\
		; 0 - ISC00 - ������� ��������� �������� ���������� INT0

 ldi	temp,0b10100000						; �� ���������� ������ �� ������
 out	MCUCR,temp

 nop
 nop
 nop

 sleep										; ��������� � �������� �����


AUTO_OFF_END:
 rcall	d1ms								; ��������� �������� ����� ���

 rjmp	RESET								; ����� ��������� ��������� � ������ ���������






;******************************************************************************
;******************************************************************************
;**                                                                          **
;**                                                                          **
;**                                 ���������                                **
;**                                                                          **
;**                                                                          **
;******************************************************************************
;******************************************************************************


;******************************************************************************
;*   ��� ��������� ������������� ��� ������ �                                 *        
;*   ��-�������� LPH7779 (Nokia3310) �� ����                                  *        
;*   ��������� PCD8544                                                        *        
;*   (�) 2004 ������� ��������, ��������� ������ (Rander)                     *       
;*                                                                            *
;*   � ������� ��������� ��������� �������. ������ ��������, ���� ���         *
;******************************************************************************

;------------------------------------------------------------------------------
LCD_INIT:     ;��������� ������������� LCD
;------------------------------------------------------------------------------

 ldi	ZL,low(LCD_INIT_TAB*2)				; ��������� �� ������ ������
 ldi	ZH,high(LCD_INIT_TAB*2)			;
 ldi	temp1,5								; ����� ������� = 5
 cbi	LCD_DC_port,LCD_DC					; ���������� ������ D/C
 rcall	LCD_SEND							; ���������� ������� � LCD
ret											;

;------------------------------------------------------------------------------
LCD_SEND:     ;�������� N ���� � LCD �� program memory
;------------------------------------------------------------------------------
                                            ;  [temp1 = N] - ����� ����
                                            ;  [ZL = �����] - ����� �������
 cbi	LCD_SCE_port,LCD_SCE				; ���������� ������ SCE
LCD_LOOP:									;
 lpm										; ��������� ��������� ����
 adiw	ZL,1								; ������� ���������
LCD_SEND1:									;
 ldi	temp2,8								; �������� 8-�� ��������
LCD_LOOP1:									;
 cbi	LCD_SCLK_port,LCD_SCLK				; ���������� ������ SCLK
 sbrc	r0,7								; ��������� ������� ���
 sbi	LCD_SDIN_port,LCD_SDIN				;  �������� ��������������
 sbrs	r0,7								;  �������� � �����
 cbi	LCD_SDIN_port,LCD_SDIN				;  ������� SDIN
 lsl	r0									; ����� ����� (��������� ���)
 sbi	LCD_SCLK_port,LCD_SCLK				; ������ ���� �� ������ SCLK
 dec	temp2								; �������� ����������� �����
 brne	LCD_LOOP1							;
 dec	temp1								; �������� �������� �����
 brne	LCD_LOOP							;
 sbi	LCD_SCE_port,LCD_SCE				; ������������� ������ SCE
 sbi	LCD_DC_port,LCD_DC					; ������������� ������ DC
ret											; ����� ���������

;------------------------------------------------------------------------------
LCD_PUT_CHAR:     ;�������� ���� ������ � LCD
;------------------------------------------------------------------------------
 ldi	ZL,low(FONT_TAB*2)					; ���������� ������ ������� ����� 
 ldi	ZH,high(FONT_TAB*2)					;  ������� �� ��������� �������

LCD_PUT_ARRAY:								; ����� ������� �� ������ �������
                                            ;  [temp = char] - ����� �������
 clr	temp1
 clr	temp0
 mov	char_ind,temp						; �������� ������ �� �������
                                            ; adres=n*6+adres0
 add	temp,char_ind						; ���������� temp (n*2) 
 adc	temp1,temp0
 add	temp,char_ind						; ���������� temp (n*3) 
 adc	temp1,temp0
 add	temp,char_ind						; ���������� temp (n*4) 
 adc	temp1,temp0
 add	temp,char_ind						; ���������� temp (n*5) 
 adc	temp1,temp0
 add	temp,char_ind						; ���������� temp (n*6) 
 adc	temp1,temp0
 
 add	ZL,temp								; ������� ����� ������� n � �������
 adc	ZH,temp1							;  adres=adres+n*6

 
 ldi	temp1,6
 clr	r0
 cbi	LCD_SCE_port,LCD_SCE				; ���������� ������ SCE
 nop
 rjmp	LCD_LOOP

;------------------------------------------------------------------------------
LCD_X:     ;���������� ����� ���������� X (0<X<83) �� LCD
;------------------------------------------------------------------------------
 
 ori	temp,0x80
 mov	r0,temp								; ���������� ��� X
 ldi	temp1,1								; �������� ���� �������
 cbi	LCD_DC_port,LCD_DC					; ���������� ������ D/C
 cbi	LCD_SCE_port,LCD_SCE				; ���������� ������ SCE
 rjmp	LCD_SEND1							;
 
;------------------------------------------------------------------------------
LCD_Y:     ;���������� ����� ���������� Y(0<Y<5) �� LCD
;------------------------------------------------------------------------------
 ori	temp,0x40
 mov	r0,temp								; ���������� ��� Y
 ldi	temp1,1								; �������� ���� �������
 cbi	LCD_DC_port,LCD_DC					; ���������� ������ D/C
 cbi	LCD_SCE_port,LCD_SCE				; ���������� ������ SCE
 rjmp	LCD_SEND1							;

;------------------------------------------------------------------------------
LCD_CLR:     ;�������� LCD
;------------------------------------------------------------------------------
 ldi	temp,0								; X = 0
 rcall	LCD_X								;
 ldi	temp,0								; Y = 0
 rcall	LCD_Y								;
 ldi	temp1,84							; �������� 84 �������
LCD_CLR1:
 push	temp1								; ��������� temp1
 ldi	temp,32								; ��� �������
 rcall	LCD_PUT_CHAR						; ������
 pop	temp1								; ��������������� temp1
 dec	temp1								; ����
 brne	LCD_CLR1							;
 ldi	temp,0								; X = 0
 rcall	LCD_X								;
 ldi	temp,0								; Y = 0
 rcall	LCD_Y								;

ret




;------------------------------------------------------------------------------
LCD_INIT_TAB:     ;������� ����� �������������
;------------------------------------------------------------------------------
 .db 0x21, 0xc5, 0x05, 0x20, 0x0C, 0x00
                                            ; 0x21 ;����� (PD=0, V=0, H=1)
                                            ; 0xC5 ;��������� Vop=69
                                            ; 0x13 ;��������� Bias (n=4)
                                            ; 0x20 ;����� (PD=0, V=0, H=0)
                                            ; 0x0D ;���������� ����� (D=1, E=0)



;------------------------------------------------------------------------------
LCD_RESET:        ;������������ �������� ������ ����������
;------------------------------------------------------------------------------

 cbi LCD_RES_port,LCD_RES
 rcall d100ms
 sbi LCD_RES_port,LCD_RES
 nop

ret















;**** A P P L I C A T I O N   N O T E   A V R 3 0 0 ***************************
;*
;* Title		: I2C (Single) Master Implementation
;* Version		: 1.0 (BETA)
;* Last updated		: 97.08.27
;* Target		: AT90Sxxxx (any AVR device)
;*
;* Support email	: avr@atmel.com
;*
;* DESCRIPTION
;* 	Basic routines for communicating with I2C slave devices. This
;*	"single" master implementation is limited to one bus master on the
;*	I2C bus. Most applications do not need the multimaster ability
;*	the I2C bus provides. A single master implementation uses, by far,
;*	less resources and is less XTAL frequency dependent.
;*
;*	Some features :
;*	* All interrupts are free, and can be used for other activities.
;*	* Supports normal and fast mode.
;*	* Supports both 7-bit and 10-bit addressing.
;*	* Supports the entire AVR microcontroller family.
;*
;*	Main I2C functions :
;*	'i2c_start' -		Issues a start condition and sends address
;*				and transfer direction.
;*	'i2c_rep_start' -	Issues a repeated start condition and sends
;*				address and transfer direction.
;*	'i2c_do_transfer' -	Sends or receives data depending on
;*				direction given in address/dir byte.
;*	'i2c_stop' -		Terminates the data transfer by issue a
;*				stop condition.
;*
;* USAGE
;*	Transfer formats is described in the AVR300 documentation.
;*	(An example is shown in the 'main' code).	
;*
;* NOTES
;*	The I2C routines can be called either from non-interrupt or
;*	interrupt routines, not both.
;*
;* STATISTICS
;*	Code Size	: 81 words (maximum)
;*	Register Usage	: 4 High, 0 Low
;*	Interrupt Usage	: None
;*	Other Usage	: Uses two I/O pins on port D
;*	XTAL Range	: N/A
;*
;******************************************************************************




;******************************************************************************
;*
;* FUNCTION
;*	i2c_hp_delay
;*	i2c_qp_delay
;*
;* DESCRIPTION
;*	hp - half i2c clock period delay (normal: 5.0us / fast: 1.3us)
;*	qp - quarter i2c clock period delay (normal: 2.5us / fast: 0.6us)
;*
;*	SEE DOCUMENTATION !!!
;*
;* USAGE
;*	no parameters
;*
;* RETURN
;*	none
;*
;******************************************************************************

i2c_hp_delay:
 ldi	i2cdelay,2
i2c_hp_delay_loop:
 dec	i2cdelay
 brne	i2c_hp_delay_loop
ret

i2c_qp_delay:
 ldi	i2cdelay,1	
i2c_qp_delay_loop:
 dec	i2cdelay
 brne	i2c_qp_delay_loop
ret


;******************************************************************************
;*
;* FUNCTION
;*	i2c_rep_start
;*
;* DESCRIPTION
;*	Assert repeated start condition and sends slave address.
;*
;* USAGE
;*	i2cadr - Contains the slave address and transfer direction.
;*
;* RETURN
;*	Carry flag - Cleared if a slave responds to the address.
;*
;* NOTE
;*	IMPORTANT! : This funtion must be directly followed by i2c_start.
;*
;******************************************************************************

i2c_rep_start:
 sbi	DDRB,SCLP_M							; force SCL low
 cbi	DDRB,SDAP_M							; release SDA
 rcall	i2c_hp_delay						; half period delay
 cbi	DDRB,SCLP_M							; release SCL
 rcall	i2c_qp_delay						; quarter period delay


;******************************************************************************
;*
;* FUNCTION
;*	i2c_start
;*
;* DESCRIPTION
;*	Generates start condition and sends slave address.
;*
;* USAGE
;*	i2cadr - Contains the slave address and transfer direction.
;*
;* RETURN
;*	Carry flag - Cleared if a slave responds to the address.
;*
;* NOTE
;*	IMPORTANT! : This funtion must be directly followed by i2c_write.
;*
;******************************************************************************

i2c_start:				
 mov	i2cdata,i2cadr						; copy address to transmitt register
 sbi	DDRB,SDAP_M							; force SDA low
 rcall	i2c_qp_delay						; quarter period delay


;******************************************************************************
;*
;* FUNCTION
;*	i2c_write
;*
;* DESCRIPTION
;*	Writes data (one byte) to the I2C bus. Also used for sending
;*	the address.
;*
;* USAGE
;*	i2cdata - Contains data to be transmitted.
;*
;* RETURN
;*	Carry flag - Set if the slave respond transfer.
;*
;* NOTE
;*	IMPORTANT! : This funtion must be directly followed by i2c_get_ack.
;*
;******************************************************************************

i2c_write:
 sec										; set carry flag
 rol	i2cdata								; shift in carry and out bit one
 rjmp	i2c_write_first
i2c_write_bit:
 lsl	i2cdata								; if transmit register empty
i2c_write_first:
 breq	i2c_get_ack							; goto get acknowledge
 sbi	DDRB,SCLP_M							; force SCL low

 brcc	i2c_write_low						; if bit high
 nop										; (equalize number of cycles)
 cbi	DDRB,SDAP_M							; release SDA
 rjmp	i2c_write_high
i2c_write_low:								; else
 sbi	DDRB,SDAP_M							; force SDA low
 rjmp	i2c_write_high						; (equalize number of cycles)
i2c_write_high:
 rcall	i2c_hp_delay						; half period delay
 cbi	DDRB,SCLP_M							; release SCL
 rcall	i2c_hp_delay						; half period delay

 rjmp	i2c_write_bit


;******************************************************************************
;*
;* FUNCTION
;*	i2c_get_ack
;*
;* DESCRIPTION
;*	Get slave acknowledge response.
;*
;* USAGE
;*	(used only by i2c_write in this version)
;*
;* RETURN
;*	Carry flag - Cleared if a slave responds to a request.
;*
;******************************************************************************

i2c_get_ack:
 sbi	DDRB,SCLP_M							; force SCL low
 cbi	DDRB,SDAP_M							; release SDA
 rcall	i2c_hp_delay						; half period delay
 cbi	DDRB,SCLP_M							; release SCL

i2c_get_ack_wait:
 sbis	PINB,SCLP_M							; wait SCL high 
					                        ;(In case wait states are inserted)
 rjmp	i2c_get_ack_wait

 clc										; clear carry flag
 sbic	PINB,SDAP_M							; if SDA is high
 sec										; set carry flag
 rcall	i2c_hp_delay						; half period delay
ret


;******************************************************************************
;*
;* FUNCTION
;*	i2c_do_transfer
;*
;* DESCRIPTION
;*	Executes a transfer on bus. This is only a combination of i2c_read
;*	and i2c_write for convenience.
;*
;* USAGE
;*	i2cadr - Must have the same direction as when i2c_start was called.
;*	see i2c_read and i2c_write for more information.
;*
;* RETURN
;*	(depends on type of transfer, read or write)
;*
;* NOTE
;*	IMPORTANT! : This funtion must be directly followed by i2c_read.
;*
;******************************************************************************

i2c_do_transfer:
 sbrs	i2cadr,b_dir						; if dir = write
 rjmp	i2c_write							; goto write data


;******************************************************************************
;*
;* FUNCTION
;*	i2c_read
;*
;* DESCRIPTION
;*	Reads data (one byte) from the I2C bus.
;*
;* USAGE
;*	Carry flag - 	If set no acknowledge is given to the slave
;*			indicating last read operation before a STOP.
;*			If cleared acknowledge is given to the slave
;*			indicating more data.
;*
;* RETURN
;*	i2cdata - Contains received data.
;*
;* NOTE
;*	IMPORTANT! : This funtion must be directly followed by i2c_put_ack.
;*
;******************************************************************************

i2c_read:
 rol	i2cstat								; store acknowledge
					                        ; (used by i2c_put_ack)
 ldi	i2cdata,0x01						; data = 0x01
i2c_read_bit:                               ; do
 sbi	DDRB,SCLP_M							; force SCL low
 rcall	i2c_hp_delay						; half period delay

 cbi	DDRB,SCLP_M							; release SCL
 rcall	i2c_hp_delay						; half period delay

 clc										; clear carry flag
 sbic	PINB,SDAP_M							; if SDA is high
 sec										; set carry flag

 rol	i2cdata								; store data bit
 brcc	i2c_read_bit						; while receive register not full


;******************************************************************************
;*
;* FUNCTION
;*	i2c_put_ack
;*
;* DESCRIPTION
;*	Put acknowledge.
;*
;* USAGE
;*	(used only by i2c_read in this version)
;*
;* RETURN
;*	none
;*
;******************************************************************************

i2c_put_ack:
 sbi	DDRB,SCLP_M							; force SCL low

 ror	i2cstat								; get status bit
 brcc	i2c_put_ack_low						; if bit low goto assert low
 cbi	DDRB,SDAP_M							; release SDA
 rjmp	i2c_put_ack_high
i2c_put_ack_low:							; else
 sbi	DDRB,SDAP_M							; force SDA low
i2c_put_ack_high:

 rcall	i2c_hp_delay						; half period delay
 cbi	DDRB,SCLP_M							; release SCL
i2c_put_ack_wait:
 sbis	PINB,SCLP_M							; wait SCL high
 rjmp	i2c_put_ack_wait
 rcall	i2c_hp_delay						; half period delay
ret


;******************************************************************************
;*
;* FUNCTION
;*	i2c_stop
;*
;* DESCRIPTION
;*	Assert stop condition.
;*
;* USAGE
;*	No parameters.
;*
;* RETURN
;*	None.
;*
;******************************************************************************

i2c_stop:
 sbi	DDRB,SCLP_M							; force SCL low
 sbi	DDRB,SDAP_M							; force SDA low
 rcall	i2c_hp_delay						; half period delay
 cbi	DDRB,SCLP_M							; release SCL
 rcall	i2c_qp_delay						; quarter period delay
 cbi	DDRB,SDAP_M							; release SDA
 rcall	i2c_hp_delay						; half period delay
ret


;******************************************************************************
;*
;* FUNCTION
;*	i2c_init
;*
;* DESCRIPTION
;*	Initialization of the I2C bus interface.
;*
;* USAGE
;*	Call this function once to initialize the I2C bus. No parameters
;*	are required.
;*
;* RETURN
;*	None
;*
;* NOTE
;*	PORTI2C and DDRC pins not used by the I2C bus interface will be
;*	set to Hi-Z (!).
;*
;* COMMENT
;*	This function can be combined with other PORTI2C initializations.
;*
;******************************************************************************

i2c_init:
 clr	i2cstat								; clear I2C status register (used
											; as a temporary register)
 clr	i2cstat_s
ret



/*
;------------------------------------------------------------------------------
;  ������ ������ � I2C 


rcall	i2c_init		; initialize I2C interface

;**** Write data => Adr(00) = 0x55 ****

 ldi	i2cadr,$A0+i2cwr					; Set device address and write
 rcall	i2c_start							; Send start condition and address

 ldi	i2cdata,$00							; Write word address (0x00)
 rcall	i2c_do_transfer						; Execute transfer

 ldi	i2cdata,$55							; Set write data to 01010101b
 rcall	i2c_do_transfer						; Execute transfer

 rcall	i2c_stop							; Send stop condition

;**** Read data => i2cdata = Adr(00) ****

 ldi	i2cadr,$A0+i2cwr					; Set device address and write
 rcall	i2c_start							; Send start condition and address

 ldi	i2cdata,$00							; Write word address
 rcall	i2c_do_transfer						; Execute transfer

 ldi	i2cadr,$A0+i2crd					; Set device address and read
 rcall	i2c_rep_start						; Send repeated start condition and address

 sec										; Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Execute transfer (read)

 rcall	i2c_stop							; Send stop condition - releases bus


*/

;********************************************************************************
;*																				*
;*         ��������� ��� ������� � ����������� ������ �������� (slave)          *
;*																				*
;********************************************************************************

i2c_rep_start_s:
 sbi	DDRC,SCLP_S							; force SCL low
 cbi	DDRC,SDAP_S							; release SDA
 rcall	i2c_hp_delay						; half period delay
 cbi	DDRC,SCLP_S							; release SCL
 rcall	i2c_qp_delay						; quarter period delay


;******************************************************************************
;* FUNCTION
;*	i2c_start_s
;******************************************************************************

i2c_start_s:				
 mov	i2cdata,i2cadr						; copy address to transmitt register
 sbi	DDRC,SDAP_S							; force SDA low
 rcall	i2c_qp_delay						; quarter period delay


;******************************************************************************
;* FUNCTION
;*	i2c_write_s
;******************************************************************************

i2c_write_s:
 sec										; set carry flag
 rol	i2cdata								; shift in carry and out bit one
 rjmp	i2c_write_first_s
i2c_write_bit_s:
 lsl	i2cdata								; if transmit register empty
i2c_write_first_s:
 breq	i2c_get_ack_s							; goto get acknowledge
 sbi	DDRC,SCLP_S							; force SCL low

 brcc	i2c_write_low_s						; if bit high
 nop										; (equalize number of cycles)
 cbi	DDRC,SDAP_S							; release SDA
 rjmp	i2c_write_high_s
i2c_write_low_s:								; else
 sbi	DDRC,SDAP_S							; force SDA low
 rjmp	i2c_write_high_s						; (equalize number of cycles)
i2c_write_high_s:
 rcall	i2c_hp_delay						; half period delay
 cbi	DDRC,SCLP_S							; release SCL
 rcall	i2c_hp_delay						; half period delay

 rjmp	i2c_write_bit_s


;******************************************************************************
;* FUNCTION
;*	i2c_get_ack_s
;******************************************************************************

i2c_get_ack_s:
 sbi	DDRC,SCLP_S							; force SCL low
 cbi	DDRC,SDAP_S							; release SDA
 rcall	i2c_hp_delay						; half period delay
 cbi	DDRC,SCLP_S							; release SCL

i2c_get_ack_wait_s:
 sbis	PINC,SCLP_S							; wait SCL high 
					                        ;(In case wait states are inserted)
 rjmp	i2c_get_ack_wait_s

 clc										; clear carry flag
 sbic	PINC,SDAP_S							; if SDA is high
 sec										; set carry flag
 rcall	i2c_hp_delay						; half period delay
ret


;******************************************************************************
;* FUNCTION
;*	i2c_do_transfer_s
;******************************************************************************

i2c_do_transfer_s:
 sbrs	i2cadr,b_dir						; if dir = write
 rjmp	i2c_write_s							; goto write data


;******************************************************************************
;* FUNCTION
;*	i2c_read_s
;******************************************************************************

i2c_read_s:
 rol	i2cstat_s								; store acknowledge
					                        ; (used by i2c_put_ack)
 ldi	i2cdata,0x01						; data = 0x01
i2c_read_bit_s:                               ; do
 sbi	DDRC,SCLP_S							; force SCL low
 rcall	i2c_hp_delay						; half period delay

 cbi	DDRC,SCLP_S							; release SCL
 rcall	i2c_hp_delay						; half period delay

 clc										; clear carry flag
 sbic	PINC,SDAP_S							; if SDA is high
 sec										; set carry flag

 rol	i2cdata								; store data bit
 brcc	i2c_read_bit_s						; while receive register not full


;******************************************************************************
;* FUNCTION
;*	i2c_put_ack_s
;******************************************************************************

i2c_put_ack_s:
 sbi	DDRC,SCLP_S							; force SCL low

 ror	i2cstat_s								; get status bit
 brcc	i2c_put_ack_low_s						; if bit low goto assert low
 cbi	DDRC,SDAP_S							; release SDA
 rjmp	i2c_put_ack_high_s
i2c_put_ack_low_s:							; else
 sbi	DDRC,SDAP_S							; force SDA low
i2c_put_ack_high_s:

 rcall	i2c_hp_delay						; half period delay
 cbi	DDRC,SCLP_S							; release SCL
i2c_put_ack_wait_s:
 sbis	PINC,SCLP_S							; wait SCL high
 rjmp	i2c_put_ack_wait_s
 rcall	i2c_hp_delay						; half period delay
ret


;******************************************************************************
;* FUNCTION
;*	i2c_stop_s
;******************************************************************************

i2c_stop_s:
 sbi	DDRC,SCLP_S							; force SCL low
 sbi	DDRC,SDAP_S							; force SDA low
 rcall	i2c_hp_delay						; half period delay
 cbi	DDRC,SCLP_S							; release SCL
 rcall	i2c_qp_delay						; quarter period delay
 cbi	DDRC,SDAP_S							; release SDA
 rcall	i2c_hp_delay						; half period delay
ret











;***********************************************************************************
;*                                                                                 *
;*                                                                                 *
;*            �������������� ��������� ����� � ����� ��� ����������                *
;*                                                                                 *
;*                                                                                 *
;***********************************************************************************

;   ���������� ����������: temp, temp1, adwL, adwH, mulL, mulH
                          
; Bin2ToBcd5
; ==========
; converts a 16-bit-binary to a 5-digit-BCD
; In: 16-bit-binary in adwH,adwL
; Out: 5-digit-BCD
; Used registers:temp
; Called subroutines: Bin2ToDigit
;
Bin2ToBCD5:
 ldi	temp,high(10000)					; ��������� 10000 ������
 mov	mulH,temp
 ldi	temp,low(10000)
 mov	mulL,temp
 rcall	Bin2ToDigit							; ���������, ��������� � ���������� temp
 sts	BCD,temp							; ���� �� ����, ����� ���������� ���������
Bin2ToBCD4:
 ldi	temp,high(1000)						; ��������� 1000 ������
 mov	mulH,temp
 ldi	temp,low(1000)
 mov	mulL,temp
 rcall	Bin2ToDigit							; ���������
 sts	BCD+1,temp							; ��������� � ���������� temp 
Bin2ToBCD3:
 clr	mulH								; ��������� 100 ������
 ldi	temp,100
 mov	mulL,temp
 rcall	Bin2ToDigit							; ���������
 sts	BCD+2,temp							; ��������� � ���������� temp
Bin2ToBCD2:
 clr	mulH								; ��������� 10 ������
 ldi	temp,10
 mov	mulL,temp
 rcall	Bin2ToDigit							; ���������
 sts	BCD+4,adwL							; ������� �������� � adiw0
 sts	BCD+3,temp							; ��������� � ���������� temp
ret

; Bin2ToDigit
; ===========
; converts one decimal digit by continued subraction of a binary coded decimal
; Used by: Bin2ToBcd5
; In: 16-bit-binary in adw1,adw0, binary coded decimal in mul0,mul1
; Out: Result in temp
; Used registers: adiw0,adiw1, mul0,mul1, temp
; Called subroutines: -

Bin2ToDigit:
 clr	temp								; digit count is zero
Bin2ToDigita:
 cp		adwH,mulH							; Number bigger than decimal?
 brcs	Bin2ToDigitc						; MSB smaller than decimal
 brne	Bin2ToDigitb						; MSB bigger than decimal
 cp		adwL,mulL							; LSB bigger or equal decimal
 brcs	Bin2ToDigitc						; LSB smaller than decimal
Bin2ToDigitb:
 sub	adwL,mulL							; Subtract LSB decimal
 sbc	adwH,mulH							; Subtract MSB decimal
 inc	temp								; Increment digit count
 rjmp	Bin2ToDigita						; Next loop
Bin2ToDigitc:
ret											; done




;************************************************************************************
;*																					*
;*																					*
;*                                  ����������                                      *
;*																					*
;*																					*
;************************************************************************************

;	������������ ������/������� T1 (����������� ��� ������� �������� � �������� �����)
aOVF1:

 inc	sleep_t

reti

;------------------------------------------------------------------------------------
;	������� ���������� INT0 (����������� ��� ������ �� ������� ������)


aINT0:
 ; ��������� ������� ����������
 ldi	temp,0b00000000
 out	GICR,temp

reti


;************************************************************************************
;*																					*
;*																					*
;*                           ������ ���� ��� �������                                *
;*																					*
;*																					*
;************************************************************************************

;------------------------------------------------------------------------------------
;	"�������� ����:"
FRAZA1:
.db 141,160,167,162,160,173,168,165,32,183,168,175,160,58,23,0

;------------------------------------------------------------------------------------
;	"CRUM ����:"
FRAZA2:
.db 67,82,85,77,32,183,168,175,160,58,23,0

;------------------------------------------------------------------------------------
;	"���-�� �����:"
FRAZA3:
.db 138,174,171,45,162,174,32,170,174,175,168,169,23,0

;------------------------------------------------------------------------------------
;	"���������� ���"
FRAZA4:
.db 143,174,164,170,171,190,183,168,178,165,32,183,168,175,23,0

;------------------------------------------------------------------------------------
;	"��������"
FRAZA5:
.db 130,187,161,165,176,168,178,165,23,0

;------------------------------------------------------------------------------------
;	"�������� ����"
FRAZA6:
.db 175,176,174,184,168,162,170,179,32,183,168,175,160,23

;------------------------------------------------------------------------------------
;	"<    ���    >"
FRAZA7:
.db 60,32,32,32,32,32,183,168,175,32,32,32,32,62,23,0

;------------------------------------------------------------------------------------
;	"�����..."
FRAZA8:
.db 164,160,171,165,165,46,46,46,23,0


;------------------------------------------------------------------------------------
;	"�������"
FRAZA9:
.db 175,176,174,184,168,178,188,23


;------------------------------------------------------------------------------------
;	"Crum By Hwang.sk"
FRAZA10:
.db 0x43,0x72,0x75,0x6D,0x20,0x42,0x79,0x20,0x48,0x77,0x61,0x6E,0x67,0x2E,0x73,0x6B


;------------------------------------------------------------------------------------
;	"ChipResetter V.1"
FRAZA11:
.db 67,104,105,112,82,101,115,101,116,116,101,114,32,86,46,32,23,0


;------------------------------------------------------------------------------------
;	"SAMSUNG"
FRAZA12:
.db 32,32,32,83,65,77,83,85,78,71,32,32,32,23


;------------------------------------------------------------------------------------
;	"XEROX"
FRAZA13:
.db 32,32,32,32,88,69,82,79,88,32,32,32,32,23


;------------------------------------------------------------------------------------
;	"DELL"
FRAZA14:
.db 32,32,32,32,32,68,69,76,76,32,32,32,32,23


;------------------------------------------------------------------------------------
;	"MB"
FRAZA15:
.db 32,32,32,32,32,32,77,66,32,32,32,32,32,23


;------------------------------------------------------------------------------------
;	"����� ���..."
FRAZA16:
.db 32,143,168,184,168,172,32,183,168,175,46,46,46,23

;------------------------------------------------------------------------------------
;	"������ CRUM"
FRAZA17:
.db 32,140,165,173,191,165,172,32,67,82,85,77,23,0


;------------------------------------------------------------------------------------
;	"��������� ���"
FRAZA18:
.db 143,176,174,162,165,176,191,165,172,32,183,168,175,23


;------------------------------------------------------------------------------------
;	"������"
FRAZA19:
.db 32,32,32,32,131,174,178,174,162,174,32,32,32,32,23,0


;------------------------------------------------------------------------------------
;	"������"
FRAZA20:
.db 142,184,168,161,170,160,23,0

;------------------------------------------------------------------------------------
;	"�� ������"
FRAZA21:
.db 175,174,32,160,164,176,165,177,179,32,36,23



;************************************************************************************
;************************************************************************************
;**                                                                                **
;**                                                                                **
;**                 ������������ �������� ��� ������� ������ 4���                  **
;**                                                                                **
;**                                                                                **
;************************************************************************************
;************************************************************************************

;-------------------------------------------------------------------------
;	�������� �� 0,25�� (250���)

d025ms:
 ldi YL,low(248)                            ; �������� � YH:YL ��������� 497
 ldi YH,high(248)

d025_1:
 sbiw YL,1                                  ; ��������� �� ����������� YH:YL
                                            ;  �������
 brne d025_1                                ; ���� ���� Z<>0 (��������� ����������
                                            ;  ���������� ������� �� ����� ����), ��
									        ;  ������� �� ����� d05_1
ret





;-------------------------------------------------------------------------
;	�������� �� 0,5�� (500���)

d05ms:
 ldi	YL,low(497)							; �������� � YH:YL ��������� 497
 ldi	YH,high(497)

d05_1:
 sbiw	YL,1								; ��������� �� ����������� YH:YL �������
 brne	d05_1								; ���� ���� Z<>0 (��������� ����������
											;  ���������� ������� �� ����� ����), ��
											;  ������� �� ����� d05_1
ret





;-------------------------------------------------------------------------
;	�������� 1 ms

d1ms:  
 ldi	temp,5
m2:
 ldi	temp1,255
m3:
 dec	temp1
 brne	m3
 dec	temp
 brne	m2
ret





;-------------------------------------------------------------------------
;	�������� 2,8 ms

d2_8ms:  
 ldi	temp,15
m:
 ldi	temp1,255
m1:
 dec	temp1
 brne	m1
 dec	temp
 brne	m
ret





;-------------------------------------------------------------------------
;	�������� 6 ms (5,7ms)

d6ms:  
 ldi	temp,30
ms6:
 ldi	temp1,255
ms61:
 dec	temp1
 brne	ms61
 dec	temp
 brne	ms6
ret






;-------------------------------------------------------------------------
;	�������� 13 ms

d13ms:  
 ldi	temp,70								; 100-19ms, 70-13ms
ms:
 ldi	temp1,255
ms1:
 dec	temp1
 brne	ms1
 dec	temp
 brne	ms
ret





;-------------------------------------------------------------------------
;	�������� 20 ms

d20ms:  
 ldi	temp,100
m20s:
 ldi	temp1,255
m20s1:
 dec	temp1
 brne	m20s1
 dec	temp
 brne	m20s
ret





;-------------------------------------------------------------------------
;	�������� �� 50��

d50ms:
 ldi	temp,100

d50_1:
 rcall	d05ms								; ����� ������������ �������� �� 0,5��
 dec	temp								; ��������� ������� �� temp
 brne	d50_1								; ���� ��������� �� ����� ����, ������� �� ����� d50_1
ret





;-------------------------------------------------------------------------
;	�������� �� 100��

d100ms:
 ldi	temp,200							; �������� � temp ��������� 200

d100_1:
 rcall	d05ms								; ����� ������������ �������� �� 0,5��
 dec	temp								; ��������� ������� �� temp
 brne	d100_1								; ���� ��������� �� ����� ����, ������� �� ����� d100_1
ret





;-------------------------------------------------------------------------
;	�������� �� 300��

d300ms:
 ldi	XL,low(700)							; �������� � YH:YL ��������� 700
 ldi	XH,high(700)

d300_1:
 rcall	d05ms								; ����� ������������ �������� �� 0,5��
 sbiw	XL,1								; ��������� ������� �� ����������� XH:XL
 brne	d300_1								; ���� ��������� �� ����� ����, ������� �� ����� d500_1
ret





;-------------------------------------------------------------------------
;	�������� �� 500��

d500ms:
 ldi	ZL,low(1000)						; �������� � ZH:ZL ��������� 1000
 ldi	ZH,high(1000)

d500_1:
 rcall	d05ms								; ����� ������������ �������� �� 0,5��
 sbiw	ZL,1								; ��������� ������� �� ����������� ZH:ZL
 brne	d500_1								; ���� ��������� �� ����� ����, ������� �� ����� d500_1
ret









;������� ����� ���������������
;------------------------------------------------------------------------------
FONT_TAB:         ; �������� ������ ������
;------------------------------------------------------------------------------
											;������  Dec   Hex
.db 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E 		;   0    (0)   (0)
.db 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00 		;   1    (1)   (1)
.db 0x00, 0x42, 0x61, 0x51, 0x49, 0x46 		;   2    (2)   (2)
.db 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31 		;   3    (3)   (3)
.db 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10 		;   4    (4)   (4)
.db 0x00, 0x27, 0x45, 0x45, 0x45, 0x39 		;   5    (5)   (5)
.db 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30 		;   6    (6)   (6)
.db 0x00, 0x03, 0x01, 0x71, 0x09, 0x07 		;   7    (7)   (7)
.db 0x00, 0x36, 0x49, 0x49, 0x49, 0x36 		;   8    (8)   (8)
.db 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E 		;   9    (9)   (9)
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 		; �����  (10)  (A)
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF 		;        (11)  (B)
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF 		;        (12)  (C)
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF 		;        (13)  (D)
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF 		;        (14)  (E)
.db 0x00, 0x00, 0x0C, 0x0C, 0x00, 0x00		;        (15)  (F)
.db 0x00, 0x7F, 0x3E, 0x1C, 0x08, 0x00 		;        (16)  (10)
.db 0x00, 0x08, 0x1C, 0x3E, 0x7F, 0x00 		;        (17)  (11)
.db 0x00, 0x06, 0x05, 0x00, 0x06, 0x05 		;        (18)  (12)
.db 0x00, 0x05, 0x03, 0x00, 0x05, 0x03 		;        (19)  (13)
.db 0x00, 0x44, 0x66, 0x77, 0x66, 0x44 		;        (20)  (14)
.db 0x00, 0x11, 0x33, 0x77, 0x33, 0x11 		;        (21)  (15)
.db 0x00, 0x1C, 0x3E, 0x3E, 0x3E, 0x1C 		;        (22)  (16)
.db 0x00, 0x10, 0x38, 0x54, 0x10, 0x1F 		;        (23)  (17)
.db 0x00, 0x04, 0x02, 0x7F, 0x02, 0x04 		;        (24)  (18)
.db 0x00, 0x10, 0x20, 0x7F, 0x20, 0x10 		;        (25)  (19)
.db 0x00, 0x10, 0x10, 0x54, 0x38, 0x10 		;        (26)  (1A)
.db 0x00, 0x10, 0x38, 0x54, 0x10, 0x10 		;        (27)  (1B)
.db 0x00, 0x40, 0x44, 0x4A, 0x51, 0x40 		;        (28)  (1C)
.db 0x00, 0x40, 0x51, 0x4A, 0x44, 0x40 		;        (29)  (1D)
.db 0x00, 0x20, 0x30, 0x38, 0x30, 0x20 		;        (30)  (1E)
.db 0x00, 0x08, 0x18, 0x38, 0x18, 0x08 		;        (31)  (1F)
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00		; space  (32)  (20)
.db 0x00, 0x00, 0x00, 0x5F, 0x00, 0x00 		;   !    (33)  (21)
.db 0x00, 0x00, 0x07, 0x00, 0x07, 0x00 		;   "    (34)  (22)
.db 0x00, 0x14, 0x7F, 0x14, 0x7F, 0x14 		;   #    (35)  (23)
.db 0x00, 0x24, 0x2A, 0x7F, 0x2A, 0x12 		;   $    (36)  (24)
.db 0x00, 0x23, 0x13, 0x08, 0x64, 0x62 		;   %    (37)  (25)
.db 0x00, 0x36, 0x49, 0x55, 0x22, 0x50 		;   &    (38)  (26)
.db 0x00, 0x00, 0x05, 0x03, 0x00, 0x00 		;   '    (39)  (27)
.db 0x00, 0x00, 0x1C, 0x22, 0x41, 0x00 		;   (    (40)  (28)
.db 0x00, 0x00, 0x41, 0x22, 0x1C, 0x00 		;   )    (41)  (29)
.db 0x00, 0x0A, 0x04, 0x1F, 0x04, 0x0A 		;   *    (42)  (2A)
.db 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08 		;   +    (43)  (2B)
.db 0x00, 0x00, 0x50, 0x30, 0x00, 0x00 		;   ,    (44)  (2C)
.db 0x00, 0x08, 0x08, 0x08, 0x08, 0x08 		;   -    (45)  (2D)
.db 0x00, 0x00, 0x60, 0x60, 0x00, 0x00 		;   .    (46)  (2E)
.db 0x00, 0x20, 0x10, 0x08, 0x04, 0x02 		;   /    (47)  (2F)
.db 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E 		;   0    (48)  (30)
.db 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00 		;   1    (49)  (31)
.db 0x00, 0x42, 0x61, 0x51, 0x49, 0x46 		;   2    (50)  (32)
.db 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31 		;   3    (51)  (33)
.db 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10 		;   4    (52)  (34)
.db 0x00, 0x27, 0x45, 0x45, 0x45, 0x39 		;   5    (53)  (35)
.db 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30 		;   6    (54)  (36)
.db 0x00, 0x03, 0x01, 0x71, 0x09, 0x07 		;   7    (55)  (37)
.db 0x00, 0x36, 0x49, 0x49, 0x49, 0x36 		;   8    (56)  (38)
.db 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E 		;   9    (57)  (39)
.db 0x00, 0x00, 0x36, 0x36, 0x00, 0x00 		;   :    (58)  (3A)
.db 0x00, 0x00, 0x56, 0x36, 0x00, 0x00 		;   ;    (59)  (3B)
.db 0x00, 0x08, 0x14, 0x22, 0x41, 0x00 		;   <    (60)  (3C)
.db 0x00, 0x14, 0x14, 0x14, 0x14, 0x14 		;   =    (61)  (3D)
.db 0x00, 0x00, 0x41, 0x22, 0x14, 0x08 		;   >    (62)  (3E)
.db 0x00, 0x02, 0x01, 0x51, 0x09, 0x06 		;   ?    (63)  (3F)
.db 0x00, 0x32, 0x49, 0x79, 0x41, 0x3E 		;   @    (64)  (40)
;--------------------------------------------
.db 0x00, 0x7E, 0x09, 0x09, 0x09, 0x7E 		;   A    (65)  (41)
.db 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 		;   B    (66)  (42)
.db 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 		;   C    (67)  (43)
.db 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C 		;   D    (68)  (44)
.db 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 		;   E    (69)  (45)
.db 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01 		;   F    (70)  (46)
.db 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A 		;   G    (71)  (47)
.db 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F 		;   H    (72)  (48)
.db 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00 		;   I    (73)  (49)
.db 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01 		;   J    (74)  (4A)
.db 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 		;   K    (75)  (4B)
.db 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40 		;   L    (76)  (4C)
.db 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F 		;   M    (77)  (4D)
.db 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F 		;   N    (78)  (4E)
.db 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E 		;   O    (79)  (4F)
.db 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 		;   P    (80)  (50)
.db 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E 		;   Q    (81)  (51)
.db 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46 		;   R    (82)  (52)
.db 0x00, 0x46, 0x49, 0x49, 0x49, 0x31 		;   S    (83)  (53)
.db 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 		;   T    (84)  (54)
.db 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F 		;   U    (85)  (55)
.db 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F 		;   V    (86)  (56)
.db 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F 		;   W    (87)  (57)
.db 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 		;   X    (88)  (58)
.db 0x00, 0x07, 0x08, 0x70, 0x08, 0x07 		;   Y    (89)  (59)
.db 0x00, 0x61, 0x51, 0x49, 0x45, 0x43 		;   Z    (90)  (5A)
.db 0x00, 0x7F, 0x41, 0x41, 0x00, 0x00 		;   [    (91)  (5B)
.db 0x00, 0x02, 0x04, 0x08, 0x10, 0x20 		;   \    (92)  (5C)
.db 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00 		;   ]    (93)  (5D)
.db 0x00, 0x04, 0x02, 0x01, 0x02, 0x04 		;   ^    (94)  (5E)
.db 0x00, 0x40, 0x40, 0x40, 0x40, 0x40 		;   _    (95)  (5F)
.db 0x00, 0x00, 0x01, 0x02, 0x04, 0x00 		;   '    (96)  (60)
;--------------------------------------------
.db 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 		;   a    (97)  (61)
.db 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38 		;   b    (98)  (62)
.db 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 		;   c    (99)  (63)
.db 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F 		;   d    (100)  (64)
.db 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 		;   e    (101)  (65)
.db 0x00, 0x08, 0x7E, 0x09, 0x01, 0x02 		;   f    (102)  (66)
.db 0x00, 0x0C, 0x52, 0x52, 0x52, 0x3E 		;   g    (103)  (67)
.db 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78 		;   h    (104)  (68)
.db 0x00, 0x00, 0x44, 0x7D, 0x40, 0x00 		;   i    (105)  (69)
.db 0x00, 0x20, 0x40, 0x44, 0x3D, 0x00 		;   j    (106)  (6A)
.db 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00 		;   k    (107)  (6B)
.db 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00 		;   l    (108)  (6C)
.db 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78 		;   m    (109)  (6D)
.db 0x00, 0x7C, 0x08, 0x04, 0x04, 0x78 		;   n    (110)  (6E)
.db 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 		;   o    (111)  (6F)
.db 0x00, 0x7C, 0x14, 0x14, 0x14, 0x08 		;   p    (112)  (70)
.db 0x00, 0x08, 0x14, 0x14, 0x18, 0x7C 		;   q    (113)  (71)
.db 0x00, 0x7C, 0x08, 0x04, 0x04, 0x08 		;   r    (114)  (72)
.db 0x00, 0x08, 0x54, 0x54, 0x54, 0x20 		;   s    (115)  (73)
.db 0x00, 0x04, 0x3F, 0x44, 0x40, 0x20 		;   t    (116)  (74)
.db 0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C 		;   u    (117)  (75)
.db 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C 		;   v    (118)  (76)
.db 0x00, 0x3C, 0x40, 0x38, 0x40, 0x3C		;   w    (119)  (77)
.db 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 		;   x    (120)  (78)
.db 0x00, 0x0C, 0x50, 0x50, 0x50, 0x3C 		;   y    (121)  (79)
.db 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44 		;   z    (122)  (7A)
.db 0x00, 0x00, 0x08, 0x36, 0x41, 0x00 		;   {    (123)  (7B)
.db 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00 		;   |    (124)  (7C)
.db 0x00, 0x00, 0x41, 0x36, 0x08, 0x00 		;   }    (125)  (7D)
.db 0x00, 0x08, 0x04, 0x08, 0x04, 0x08 		;   ~    (126)  (7E)
.db 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF 		; black  (127)  (7F)
;--------------------------------------------
.db 0x00, 0x7E, 0x09, 0x09, 0x09, 0x7E		;   �    (128)  (80)
.db 0x00, 0x7F, 0x45, 0x45, 0x45, 0x39 		;   �    (129)  (81)
.db 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 		;   �    (130)  (82)
.db 0x00, 0x7F, 0x01, 0x01, 0x01, 0x03 		;   �    (131)  (83)
.db 0x00, 0xC0, 0x7E, 0x41, 0x7F, 0xC0		;   �    (132)  (84)
.db 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 		;   �    (133)  (85)
.db 0x00, 0x77, 0x08, 0x7F, 0x08, 0x77 		;   �    (134)  (86)
.db 0x00, 0x41, 0x49, 0x49, 0x49, 0x36 		;   �    (135)  (87)
.db 0x00, 0x7F, 0x10, 0x08, 0x04, 0x7F 		;   �    (136)  (88)
.db 0x00, 0x7E, 0x20, 0x13, 0x08, 0x7E 		;   �    (137)  (89)
.db 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 		;   �    (138)  (8A)
.db 0x00, 0x40, 0x3C, 0x02, 0x01, 0x7F 		;   �    (139)  (8B)
.db 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F 		;   �    (140)  (8C)
.db 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F 		;   �    (141)  (8D)
.db 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E 		;   �    (142)  (8E)
.db 0x00, 0x7F, 0x01, 0x01, 0x01, 0x7F 		;   �    (143)  (8F)
.db 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 		;   �    (144)  (90)
.db 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 		;   �    (145)  (91)
.db 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 		;   �    (146)  (92)
.db 0x00, 0x07, 0x48, 0x48, 0x48, 0x3F 		;   �    (147)  (93)
.db 0x00, 0x1C, 0x22, 0x7F, 0x22, 0x1C 		;   �    (148)  (94)
.db 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 		;   �    (149)  (95)
.db 0x00, 0x3F, 0x20, 0x20, 0x3F, 0x60 		;   �    (150)  (96)
.db 0x00, 0x0F, 0x08, 0x08, 0x08, 0x7F 		;   �    (151)  (97)
.db 0x00, 0x7F, 0x40, 0x7E, 0x40, 0x7F 		;   �    (152)  (98)
.db 0x00, 0x3F, 0x20, 0x3C, 0x20, 0x7F 		;   �    (153)  (99)
.db 0x00, 0x01, 0x7F, 0x48, 0x48, 0x30 		;   �    (154)  (9A)
.db 0x00, 0x7F, 0x48, 0x30, 0x00, 0x7F 		;   �    (155)  (9B)
.db 0x00, 0x7F, 0x48, 0x48, 0x48, 0x30		;   �    (156)  (9C)
.db 0x00, 0x22, 0x49, 0x49, 0x49, 0x3E 		;   �    (157)  (9D)
.db 0x00, 0x7F, 0x08, 0x3E, 0x41, 0x3E 		;   �    (158)  (9E)
.db 0x00, 0x46, 0x29, 0x19, 0x09, 0x7F 		;   �    (159)  (9F)
.db 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 		;   �    (160)  (A0)
.db 0x00, 0x3C, 0x4A, 0x4A, 0x49, 0x30 		;   �    (161)  (A1)
.db 0x00, 0x7C, 0x54, 0x54, 0x28, 0x00   	;   �    (162)  (A2)
.db 0x00, 0x7C, 0x04, 0x04, 0x0C, 0x00 		;   �    (163)  (A3)
.db 0x00, 0xC0, 0x78, 0x44, 0x7C, 0xC0 		;   �    (164)  (A4)
.db 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 		;   �    (165)  (A5)
.db 0x00, 0x6C, 0x10, 0x7C, 0x10, 0x6C 		;   �    (166)  (A6)
.db 0x00, 0x44, 0x54, 0x54, 0x28, 0x00 		;   �    (167)  (A7)
.db 0x00, 0x7C, 0x20, 0x10, 0x08, 0x7C 		;   �    (168)  (A8)
.db 0x00, 0x7C, 0x21, 0x12, 0x09, 0x7C 		;   �    (169)  (A9)
.db 0x00, 0x7C, 0x10, 0x28, 0x44, 0x00 		;   �    (170)  (AA)
.db 0x00, 0x70, 0x08, 0x04, 0x7C, 0x00 		;   �    (171)  (AB)
.db 0x00, 0x7C, 0x08, 0x10, 0x08, 0x7C 		;   �    (172)  (AC)
.db 0x00, 0x7C, 0x10, 0x10, 0x10, 0x7C 		;   �    (173)  (AD)
.db 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 		;   �    (174)  (AE)
.db 0x00, 0x7C, 0x04, 0x04, 0x04, 0x7C 		;   �    (175)  (AF)
.db 0x00, 0x7C, 0x14, 0x14, 0x14, 0x08 		;   �    (176)  (B0)
.db 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 		;   �    (177)  (B1)
.db 0x00, 0x04, 0x04, 0x7C, 0x04, 0x04 		;   �    (178)  (B2)
.db 0x00, 0x0C, 0x50, 0x50, 0x50, 0x3C 		;   �    (179)  (B3)
.db 0x00, 0x30, 0x48, 0xFC, 0x48, 0x30 		;   �    (180)  (B4)
.db 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 		;   �    (181)  (B5)
.db 0x00, 0x7C, 0x40, 0x40, 0x7C, 0xC0 		;   �    (182)  (B6)
.db 0x00, 0x0C, 0x10, 0x10, 0x10, 0x7C 		;   �    (183)  (B7)
.db 0x00, 0x7C, 0x40, 0x7C, 0x40, 0x7C 		;   �    (184)  (B8)
.db 0x00, 0x7C, 0x40, 0x7C, 0x40, 0xFC 		;   �    (185)  (B9)
.db 0x00, 0x04, 0x7C, 0x50, 0x50, 0x20 		;   �    (186)  (BA)
.db 0x00, 0x7C, 0x50, 0x20, 0x00, 0x7C 		;   �    (187)  (BB)
.db 0x00, 0x7C, 0x50, 0x50, 0x20, 0x00 		;   �    (188)  (BC)
.db 0x00, 0x28, 0x44, 0x54, 0x54, 0x38 		;   �    (189)  (BD)
.db 0x00, 0x7C, 0x10, 0x38, 0x44, 0x38 		;   �    (190)  (BE)
.db 0x00, 0x08, 0x54, 0x34, 0x14, 0x7C 		;   �    (191)  (BF)

;------------------------------------------------------------------------------
FONT2_TAB:     ; ������� ������ ������
;------------------------------------------------------------------------------
.db 0x00, 0xFC, 0xFE, 0x07, 0x03, 0x83		;   0    (192)
.db 0x83, 0x63, 0x67, 0xFE, 0xFC, 0x00
.db 0x00, 0x3F, 0x7F, 0xE6, 0xC6, 0xC1
.db 0xC1, 0xC0, 0xE0, 0x7F, 0x3F, 0x00

.db 0x00, 0x00, 0x00, 0x18, 0x1C, 0xFF		;   1    (193)
.db 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0xC0, 0xC0, 0xFF
.db 0xFF, 0xC0, 0xC0, 0x00, 0x00, 0x00

.db 0x00, 0x06, 0x07, 0x03, 0x03, 0x03		;   2    (194)
.db 0x03, 0x83, 0xC7, 0xFE, 0x7C, 0x00
.db 0x00, 0xE0, 0xF0, 0xF8, 0xDC, 0xCE
.db 0xC7, 0xC3, 0xC1, 0xC0, 0xC0, 0x00

.db 0x00, 0x0C, 0x0E, 0x87, 0x83, 0x83		;   3    (195)
.db 0x83, 0x83, 0xC7, 0xFE, 0x3C, 0x00
.db 0x00, 0x30, 0x70, 0xE1, 0xC1, 0xC1
.db 0xC1, 0xC1, 0xE3, 0x7F, 0x3E, 0x00

.db 0x00, 0x00, 0x00, 0xC0, 0xF0, 0x3C		;   4    (196)
.db 0x0E, 0x06, 0xFF, 0xFF, 0x00, 0x00
.db 0x00, 0x06, 0x07, 0x07, 0x06, 0x06
.db 0x06, 0x06, 0xFF, 0xFF, 0x06, 0x00

.db 0x00, 0x7E, 0xFF, 0xC3, 0xC3, 0xC3		;   5    (197)
.db 0xC3, 0xC3, 0xC3, 0x83, 0x03, 0x00
.db 0x00, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0
.db 0xC0, 0xC0, 0xE1, 0x7F, 0x3F, 0x00

.db 0x00, 0xFE, 0xFF, 0xC3, 0xC3, 0xC3		;   6    (198)
.db 0xC3, 0xC3, 0xC3, 0x87, 0x06, 0x00
.db 0x00, 0x7F, 0xFF, 0xE1, 0xC0, 0xC0
.db 0xC0, 0xC0, 0xE1, 0x7F, 0x3F, 0x00

.db 0x00, 0x03, 0x03, 0x03, 0x03, 0x03		;   7    (199)
.db 0x03, 0xC3, 0xF3, 0x3F, 0x0F, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFC
.db 0x0F, 0x03, 0x00, 0x00, 0x00, 0x00

.db 0x00, 0x7C, 0xFE, 0xC7, 0x83, 0x83		;   8    (200)
.db 0x83, 0x83, 0xC7, 0xFE, 0x7C, 0x00
.db 0x00, 0x3E, 0x7F, 0xE3, 0xC1, 0xC1
.db 0xC1, 0xC1, 0xE3, 0x7F, 0x3E, 0x00

.db 0x00, 0x7C, 0xFE, 0xC7, 0x83, 0x83		;   9    (201)
.db 0x83, 0x83, 0xC7, 0xFE, 0xFC, 0x00
.db 0x00, 0x30, 0x70, 0xE1, 0xC1, 0xC1
.db 0xC1, 0xC1, 0xE1, 0x7F, 0x3F, 0x00





;******************************************************************************
;*                                                                            *
;*                                  ��������                                  *
;*                                                                            *
;******************************************************************************

;******************************************************************************
;*                                                                            *
;*                                  ��������                                  *
;*                                                                            *
;******************************************************************************

; ��������-������� �������������
PICLOGO_TAB:
.db 0x00, 0x00, 0xC0, 0xE0, 0x70, 0x30
.db 0x30, 0x70, 0xE0, 0x00, 0xF0, 0xF0
.db 0xC0, 0xC0, 0xC0, 0x80, 0x00, 0xC0
.db 0xD8, 0xD8, 0x00, 0x80, 0xC0, 0xC0
.db 0xC0, 0xC0, 0xC0, 0x80, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x07, 0x08, 0x10
.db 0x08, 0x07, 0x00, 0x1F, 0x15, 0x11
.db 0x00, 0x1F, 0x05, 0x0D, 0x12, 0x00
.db 0x10, 0x00, 0x12, 0x1F, 0x10, 0x00
.db 0x10, 0x00, 0x11, 0x15, 0x1F, 0x00
.db 0x00, 0x00, 0x07, 0x0F, 0x1C, 0x18
.db 0x18, 0x1C, 0x0E, 0x00, 0x1F, 0x1F
.db 0x00, 0x00, 0x1F, 0x1F, 0x00, 0x18
.db 0x1F, 0x1F, 0x00, 0xFF, 0xFF, 0xD8
.db 0x18, 0x18, 0x1F, 0x0F, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0xFC, 0xFE, 0x66
.db 0x66, 0xE6, 0xBE, 0x1C, 0xE0, 0xF0
.db 0xF8, 0x48, 0x48, 0x58, 0x70, 0x60
.db 0x00, 0x30, 0x78, 0x58, 0x58, 0xD8
.db 0xD8, 0x00, 0xE0, 0xF0, 0xF8, 0x48
.db 0x48, 0x58, 0x70, 0x60, 0x00, 0xFE
.db 0xFE, 0x88, 0x08, 0x88, 0xC8, 0x00
.db 0xFE, 0xFE, 0x88, 0x08, 0x88, 0xC8
.db 0x00, 0xE0, 0xF0, 0x78, 0x48, 0x48
.db 0x58, 0x70, 0x60, 0x00, 0xF0, 0xF8
.db 0x18, 0x18, 0x38, 0x70, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x03, 0x03, 0x00
.db 0x00, 0x01, 0x03, 0x03, 0x00, 0x01
.db 0x03, 0x03, 0x03, 0x03, 0x01, 0x00
.db 0x00, 0x03, 0x03, 0x83, 0x83, 0x03
.db 0x01, 0x00, 0x00, 0x01, 0x03, 0x03
.db 0x03, 0x03, 0x01, 0x00, 0x00, 0x01
.db 0x03, 0x03, 0x03, 0x03, 0x01, 0x00
.db 0x01, 0x03, 0x03, 0x03, 0x03, 0x01
.db 0x00, 0x00, 0x01, 0x03, 0x03, 0x03
.db 0x03, 0x01, 0x00, 0x00, 0x03, 0x03
.db 0x03, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x04, 0x3F, 0x04, 0x00, 0x01
.db 0x00, 0x1C, 0x22, 0x22, 0x22, 0x1C
.db 0x00, 0x3E, 0x04, 0x02, 0x02, 0x04
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x46, 0x49, 0x49, 0x49, 0x31
.db 0x00, 0x7E, 0x09, 0x09, 0x09, 0x7E
.db 0x00, 0x7F, 0x02, 0x04, 0x02, 0x7F
.db 0x00, 0x46, 0x49, 0x49, 0x49, 0x31
.db 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F
.db 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F
.db 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A
.db 0x00, 0x00, 0x00, 0x00, 0x36, 0x49
.db 0x55, 0x22, 0x50, 0x00, 0x00, 0x00
.db 0x63, 0x14, 0x08, 0x14, 0x63, 0x00
.db 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00
.db 0x7F, 0x09, 0x19, 0x29, 0x46, 0x00
.db 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00
.db 0x63, 0x14, 0x08, 0x14, 0x63, 0x00


; �������� ����������� �������������
PIC_CONECT_TAB:
.db 0xFF, 0x01, 0x01, 0x81, 0x01, 0x01
.db 0x01, 0x81, 0x01, 0x01, 0x03, 0x1F
.db 0x1F, 0x03, 0x01, 0x01, 0x01, 0x01
.db 0x01, 0x01, 0x01, 0x01, 0xFF, 0x01
.db 0x81, 0x81, 0x81, 0x81, 0x01, 0x01
.db 0x01, 0x01, 0x03, 0x1F, 0x1F, 0x83
.db 0x01, 0x01, 0x01, 0x01, 0x01, 0x01
.db 0x01, 0x01, 0xFF, 0x01, 0x01, 0x01
.db 0x81, 0x81, 0x81, 0x01, 0x83, 0x9F
.db 0x1F, 0x03, 0x81, 0x01, 0x01, 0x01
.db 0x01, 0x01, 0xFF, 0x01, 0x01, 0x01
.db 0x81, 0x81, 0x81, 0x81, 0x01, 0x01
.db 0x03, 0x1F, 0x1F, 0x03, 0x01, 0x01
.db 0x01, 0x01, 0x81, 0x01, 0x01, 0xFF
.db 0xFF, 0x00, 0x00, 0x0F, 0x10, 0x20
.db 0x10, 0x0F, 0x00, 0x04, 0x2A, 0x2A
.db 0x2A, 0x10, 0x00, 0x04, 0x2A, 0x2A
.db 0x2A, 0x10, 0x00, 0x00, 0xFF, 0x00
.db 0x3F, 0x20, 0x20, 0x20, 0x1F, 0x00
.db 0x10, 0x2A, 0x2A, 0x3C, 0x01, 0x1F
.db 0x21, 0x10, 0x00, 0x10, 0x2A, 0x2A
.db 0x3C, 0x00, 0xFF, 0x00, 0x00, 0x1F
.db 0x20, 0x20, 0x20, 0x00, 0x20, 0x3F
.db 0x20, 0x00, 0x3F, 0x08, 0x14, 0x22
.db 0x00, 0x00, 0xFF, 0x00, 0x00, 0x1F
.db 0x20, 0x20, 0x28, 0x38, 0x00, 0x00
.db 0x3C, 0x04, 0x04, 0x38, 0x00, 0x10
.db 0x28, 0x28, 0x3F, 0x00, 0x00, 0xFF
.db 0x0F, 0x0D, 0x0C, 0x08, 0x08, 0x08
.db 0x08, 0x08, 0x08, 0x08, 0x0C, 0x0F
.db 0x0F, 0x0C, 0x08, 0x08, 0x08, 0x08
.db 0x08, 0x08, 0x08, 0x0C, 0x0F, 0x0C
.db 0x08, 0x08, 0x08, 0x08, 0x08, 0x08
.db 0x08, 0x08, 0x0C, 0x0F, 0x0F, 0x0C
.db 0x08, 0x08, 0x08, 0x08, 0x08, 0x08
.db 0x08, 0x0C, 0x0F, 0x0C, 0x08, 0x08
.db 0x08, 0x08, 0x08, 0x08, 0x0C, 0x0F
.db 0x0F, 0x0C, 0x08, 0x08, 0x08, 0x08
.db 0x08, 0x0C, 0x0F, 0x0C, 0x08, 0x08
.db 0x08, 0x08, 0x08, 0x08, 0x08, 0x08
.db 0x0C, 0x0F, 0x0F, 0x0C, 0x08, 0x08
.db 0x08, 0x08, 0x08, 0x08, 0x0C, 0x0F



;	�������� ������� ������
PIC_SLEEP_TAB:
.db 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0
.db 0xE0, 0xF0, 0x70, 0x38, 0x18, 0x18
.db 0x08, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x40, 0x80, 0x40, 0x00, 0x00, 0x00
.db 0x14, 0x08, 0x14, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x14, 0x08
.db 0x14, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x28, 0x10, 0x28, 0x00
.db 0x00, 0x80, 0x00, 0x80, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x0F, 0x3F, 0x7F
.db 0x7F, 0xF0, 0xE0, 0xC0, 0xC0, 0x80
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x01, 0x00, 0x01, 0x50, 0x20, 0x50
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A
.db 0x04, 0x0A, 0x00, 0x80, 0x00, 0x80
.db 0x00, 0x00, 0x00, 0x14, 0x08, 0x14
.db 0x00, 0x00, 0x00, 0x00, 0xA0, 0x40
.db 0xA0, 0x02, 0x01, 0x02, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x01, 0x01, 0x01
.db 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x02, 0x01, 0x02
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x7E, 0x81, 0x81, 0x81, 0x81, 0x42
.db 0x00, 0xF8, 0x08, 0x08, 0x08, 0xF8
.db 0x00, 0xF8, 0x40, 0x20, 0x10, 0xF8
.db 0x00, 0xF8, 0x10, 0x20, 0x10, 0xF8
.db 0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00
.db 0xC0, 0xC0, 0x00, 0x00, 0xC0, 0xC0
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0xF8, 0xF0, 0x00
.db 0x80, 0xC0, 0xC0, 0xC0, 0xC0, 0x80
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x80
.db 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x1E
.db 0x0E, 0x07, 0x07, 0x07, 0x07, 0x06
.db 0x06, 0x06, 0x06, 0x06, 0x06, 0x06
.db 0x06, 0x06, 0x06, 0x06, 0x06, 0x06
.db 0x06, 0x06, 0x06, 0x0E, 0x1E, 0xFF
.db 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00








;******************************************************************************

.exit                                       ; ����� ���������
