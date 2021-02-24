;*******************************************************************************
;*******************************************************************************
;**                                                                           **
;**             Программа для программатора чипов на основе 24Сxx.            **
;**                                                                           **
;**               тип МК ATmega8A Тактовая частота 4.00 МГ                    **
;**                                Ver.1.3                                    **
;**                                                                           **
;*******************************************************************************
;*******************************************************************************

.include "m8def.inc"
 
    
	; определяем переменные

.def	mulL=r0
.def	mulH=r1
.def	adwL=r2
.def	adwH=r3
.def	count_memL_t=r4						; Временные регистры адреса прошивки в памяти
.def	count_memH_t=r5						;  программатора
;.def	=r6
;.def	=r7
.def	time_sleep=r8						; Задает время для таймера при переходе в sleep
.def	sleep_t=r9
.def	progres_temp=r10
.def	progres=r11
.def	col_prosh=r12						; Количество прошивок в памяти программатора
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

.def	count_memL=r26						; XL Адрес выбранной прошивки в памяти программатора
.def	count_memH=r27						; XH
;.def	=r28								; YL
;.def	=r29								; YH
;.def	=r30								; ZL
;.def	=r31								; ZH





;------------------------------------------------------------------------------
;	Распиновка портов для подключения LCD
                                            ;
.equ LCD_SCLK_port=PORTD					; Выбор порта для сигнала SCLK 
.equ LCD_SCLK=5								; сигнал SCLK для LCD
                                            ;     
.equ LCD_SDIN_port=PORTB					; Выбор порта для сигнала SDIN
.equ LCD_SDIN=7								; сигнал SDIN для LCD
                                            ;
.equ LCD_DC_port=PORTB						; Выбор порта для сигнала DC
.equ LCD_DC=6								; сигнал DC для LCD
                                            ;
.equ LCD_SCE_port=PORTD						; Выбор порта для сигнала SCE
.equ LCD_SCE=4								; сигнал SCE для LCD
                                            ;
.equ LCD_RES_port=PORTB						; Выбор порта для сигнала RESET
.equ LCD_RES=4								; сигнал RESETдля LCD




;------------------------------------------------------------------------------
;   Порт и выводы для I2C


.equ PORTI2C_M=PORTB						; Порт чипа картриджа
.equ SDAP_M=3								; SDA Pin master
.equ SCLP_M=1								; SCL Pin master


.equ PORTI2C_S=PORTC						; Порт микросхемы памяти прошивок
.equ SDAP_S=1								; SDA Pin slave
.equ SCLP_S=0								; SCL Pin slave

.equ b_dir=0                                ; transfer direction bit in i2cadr

.equ i2crd=1
.equ i2cwr=0


;------------------------------------------------------------------------------
;   Порт и выводы для кнопок


.equ	PIN_BOT_L=PINC					; кнопка "Left"
.equ	BOT_L=3

.equ	PIN_BOT_R=PIND					; кнопка "Right"
.equ	BOT_R=3

.equ	PIN_BOT_Y=PINC					; кнопка "Yes"
.equ	BOT_Y=5

.equ	PIN_BOT_N=PIND					; кнопка "No"
.equ	BOT_N=2

.equ	PIN_BOT_1=PINC					; кнопка "1"
.equ	BOT_1=4

.equ	PIN_BOT_2=PIND					; кнопка "2"
.equ	BOT_2=0

.equ	PIN_BOT_3=PIND					; кнопка "3"
.equ	BOT_3=1

;.equ	PIN_BOT_4=PINC					; кнопка "4"
;.equ	BOT_4=3



	; это сегмент данных. В нем выделяется оперативная память

.dseg

CHIP_NAME:	.byte 14						; Название чипа (адреса 0000h...0007h)
CHIP_CRUM:	.byte 11						; Серийный номер чипа (адреса 0035h...003Fh)
CHIP_COUNT:	.byte 4							; Количество сделанных копий (адреса 004Ah...004Bh)
TEMP_RAM:	.byte 1							; Временная переменная в памяти ОЗУ

BCD:		.byte 5

							; Количество прошивок в памяти программатора



.cseg

; ***** INTERRUPT VECTORS ************************************************
.org	$000
		rjmp RESET							; Обработка сброса
.org	$0001
		rjmp aINT0							; 0x0001 - Внешнее прерывание 0
.org	$0002
		rjmp aINT1							; 0x0002 - Внешнее прерывание 1
.org	$0003
		rjmp aOC2							; 0x0003 - Совпадение таймера/счетчика Т2
.org	$0004
		rjmp aOVF2							; 0x0004 - Переполнение таймера/счетчика Т2
.org	$0005
		rjmp aICP1							; 0x0005 - Захват таймера/счетчика Т1
.org	$0006
		rjmp aOC1A							; 0x0006 - Совпадение А таймера/счетчика Т1
.org	$0007
		rjmp aOC1B							; 0x0007 - Совпадение В таймера/счетчика Т1
.org	$0008
		rjmp aOVF1							; 0x0008 - Переполнение таймера/счетчика Т1
.org	$0009
		rjmp aOVF0							; 0x0009 - Переполнение таймера/счетчика Т0
.org	$000a
		rjmp aSPI							; 0x000a - Передача по SPI завершена
.org	$000b
		rjmp aURXC							; 0x000b - USART, Прием завершен
.org	$000c
		rjmp aUDRE							; 0x000c - Регистр данных USART пуст
.org	$000d
		rjmp aUTXC							; 0x000d - USART, Передача завершена
.org	$000e
		rjmp aADCC							; 0x000e - Преобразование ADC завершено
.org	$000f
		rjmp aERDY							; 0x000f - EEPROM готов
.org	$0010
		rjmp aACI							; 0x0010 - Analog Comparator
.org	$0011
		rjmp aTWI							; 0x0011 - Прерывание от 2-wire Serial Interface
.org	$0012
		rjmp aSPMR							; 0x0012 - Готовность SPM





;aINT0:										; 0x0001 - Внешнее прерывание 0
aINT1:										; 0x0002 - Внешнее прерывание 1
aOC2:										; 0x0003 - Совпадение таймера/счетчика Т2
aOVF2:										; 0x0004 - Переполнение таймера/счетчика Т2
aICP1:										; 0x0005 - Захват таймера/счетчика Т1
aOC1A:										; 0x0006 - Совпадение А таймера/счетчика Т1
aOC1B:										; 0x0007 - Совпадение В таймера/счетчика Т1
;aOVF1:										; 0x0008 - Переполнение таймера/счетчика Т1
aOVF0:										; 0x0009 - Переполнение таймера/счетчика Т0
aSPI:										; 0x000a - Передача по SPI завершена
aURXC:										; 0x000b - USART, Прием завершен
aUDRE:										; 0x000c - Регистр данных USART пуст
aUTXC:										; 0x000d - USART, Передача завершена
aADCC:										; 0x000e - Преобразование ADC завершено
aERDY:										; 0x000f - EEPROM готов
aACI:										; 0x0010 - Analog Comparator
aTWI:										; 0x0011 - Прерывание от 2-wire Serial Interface
aSPMR:										; 0x0012 - Готовность SPM





RESET:

	; Инициализация стека 
 ldi 	temp, LOW(RAMEND)
 out	spl, Temp
 ldi 	temp, HIGH(RAMEND)
 out	sph, Temp



	; Инициализация портов
		
 ldi	temp,0b11111111                       ; инициализация порта B
 out	DDRB,temp

 ldi	temp,0b00000100                       ; инициализация порта C
 out	DDRC,temp

 ldi	temp,0b11110000                       ; инициализация порта D
 out	DDRD,temp 

 ldi	temp,0b11000000                       ; запись сигналов на вывод порта B
 out	PORTB,temp      
        
 ldi	temp,0b01111000                       ; запись сигналов на вывод порта C
 out	PORTC,temp      

 ldi	temp,0b00001111                       ; запись сигналов на вывод порта D
 out	PORTD,temp      



;------------------------------------------------------------------------------
	; Инициализация таймера T1, используется для таймера
		; 7 - OCIE2 флаг по разрешению прерывания по "совпадению"
		; 6 - TOIE2 флаг по разрешению прерывания по переполнению таймера/счетчика Т1
		; 5 - TICIE1 флаг по разрешению прерывания по "Захват" таймера счетчика Т1
		; 4 - OCIE1A флаг по разрешению прерывания по "совпадению А"
		; 3 - OCIE1B флаг по разрешению прерывания по "совпадению В"
		; 2 - TOIE1 флаг по разрешению прерывания по переполнению таймера/счетчика Т1
		; 1 - 
		; 0 - TOIE0 флаг по разрешению прерывания по переполнению таймера/счетчика Т0

 ldi	temp,0b00000100                        ; разрешить прерывание по "переполнению T1"
 out	TIMSK,Temp


     ; Настройка таймера/счетчика T1 в режиме переполнения
		; 7 - COM1A1 \ 
		; 6 - COM1A0  - режим работы блока сравнения x 
		; 5 - COM1B1 /
		; 4 - COM1B0 /
		; 3 - FOC1A - принудительное изменение состояния вывода OCnx
		; 2 - FOC1B /
		; 1 - WGM11 - режим работы таймера/счетчика 
		; 0 - WGM10 / 

 ldi	temp,0b00000000
 out	TCCR1A,Temp


		; 7 - ICNC1 - управление схемой подавления помех блока захвата
		; 6 - ICES1 - выбор активного фронта сигнала захвата
		; 5 - 
		; 4 - WGM13 \
		; 3 - WGM12 - режим работы таймера/счетчика
		; 2 - CS12 \
		; 1 - CS11  - управление тактовым сигналом (clk/64)
		; 0 - CS10 /

 ldi	temp,0b00000011
 out	TCCR1B,Temp



;------------------------------------------------------------------------------
	; Инициализация аналогового компаратора
		; 7 - ACD  - выключение компаратора (0-включен, 1-выключен)
		; 6 - ACBG - подключение внутреннего ИОН (0-не подключен, 1-подключен)
		; 5 - AC0  - результат сравнения
		; 4 - ACI  - флаг прерывания от компаратора
		; 3 - ACIE - разрешение прерывания от компаратора
		; 2 - ACIC - подключение компаратора к блоку захвата таймера/счетчика Т1 (0-отключен,1-подключен)
		; 1 - ACIS1 \
		; 0 - ACIS0  - условие возникновения прерывания от компаратора

 ldi	temp,0b10010011
 out	ACSR,temp



;------------------------------------------------------------------------------

    ; Инициализация АЦП

 ldi temp,0b000000010
 out ADMUX,temp
    ; 7 REFS1 = 0 
    ; 6 REFS0 = 0
    ; 5 ADLAR = 0 равнение по левому краю
    ; 4 MUX4 = 0
    ; 3 MUX3 = 0
    ; 2 MUX2 = 1
    ; 1 MUX1 = 1   вход АЦП=ADC7
    ; 0 MUX0 = 1

 ldi temp,0b00010000
 out ADCSRA,temp
    ; 7 ADEN = разрешает работу АЦП
    ; 6 ADSC = начать преобразование
    ; 5 ADATE= непрерывное преобразование 
    ; 4 ADIF = прерывание об окончании преобразования
    ; 3 ADIE = прерывание от АЦП запрещено
    ; 2 ADPS2= 0 
    ; 1 ADPS1= 0
    ; 0 ADPS0= 0
 ldi temp,0b00000000                        ; Непрерывное преобразование
 out SFIOR,temp






;******************************************************************************
;*                                                                            *
;*                                                                            *
;*         Подготовка периферии и инициализация переменных программы          *
;*                                                                            *
;*                                                                            *
;******************************************************************************

 rcall	LCD_RESET							; Сброс дисплея
 rcall	LCD_INIT							; Инициализация дисплея
; rcall	LCD_CLR								; Очищаем дичплей
 clr	temp
 rcall	LCD_X								; Устанавливаем координаты X=0
 clr	temp
 rcall	LCD_Y								; Устанавливаем координаты Y=0
 rcall	i2c_init							; initialize I2C interface

 clr	sleep_t
 ldi	temp,10
 mov	time_sleep,temp						; Таймер установлен на 40 секунд

STEP0:
 clr	temp
 rcall	LCD_X							; Устанавливаем координаты X=0
 clr	temp
 rcall	LCD_Y							; Устанавливаем координаты Y=0

;	Выводим логотип на дисплей
 clr	temp
 ldi	temp_count,84						; Загружаем счетчик символов значением 130
PICLOGO:
 push	temp
 ldi	ZL,low(PICLOGO_TAB*2)				; Вычисление адреса первого байта 
 ldi	ZH,high(PICLOGO_TAB*2)				;  символа в массиве PICTURE1
 rcall	LCD_PUT_ARRAY					; Вывод символа из массива PICTURE1
 pop	temp
 inc	temp								; Увеличиваем значение счетчика
 cpse	temp,temp_count
rjmp	PICLOGO								; Количество шагов меньше 84, тогда повторяем


 rcall	d500ms								; Показываем заставку 3 секунды
 rcall	d500ms
 rcall	d500ms


 ldi	temp,18
 mov	col_prosh,temp						; Устанавливаем счетчик количества прошивок (18 штук)






;********************************************************************************
;********************************************************************************
;**                                                                            **
;**                                                                            **
;**                     Основной цикл работы программы                         **
;**                                                                            **
;**                                                                            **
;********************************************************************************
;********************************************************************************

;********************************************************************************
;																				;
;	           ШАГ1: Ждем подключения чипа к программатору						;
;																				;
;********************************************************************************

STEP1:
;	Выводим картинку подключения чипа на дисплей
 rcall	LCD_CLR
 clr	temp
 rcall	LCD_X
 clr	temp
 rcall	LCD_Y
 clr	temp
 ldi	temp_count,42						; Загружаем счетчик символов значением 84
PIC_CONECT:
 push	temp
 ldi	ZL,low(PIC_CONECT_TAB*2)			; Вычисление адреса первого байта 
 ldi	ZH,high(PIC_CONECT_TAB*2)			;  символа в массиве PICTURE1
 rcall	LCD_PUT_ARRAY					; Вывод символа из массива PICTURE1
 pop	temp
 inc	temp								; Увеличиваем значение счетчика
 cpse	temp,temp_count
rjmp	PIC_CONECT							; Количество шагов меньше 84, тогда повторяем

;	Выводим текст "Подключите чип"
 ldi	temp,0
 rcall	LCD_X
 ldi	temp,3
 rcall	LCD_Y
 clr	temp_count
FRAZA4_V:
 ldi	ZL,low(FRAZA4*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA4*2)					;
 ldi	temp,0
 add	ZL,temp_count						; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA4_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	temp_count
 rjmp	FRAZA4_V							; Продолжаем выводить буквы на дисплей
FRAZA4_END:									; Выходим из цикла вывода фразы на дисплей

;	Выводим текст "далее ..."
 ldi	temp,31
 rcall	LCD_X
 ldi	temp,5
 rcall	LCD_Y
 clr	temp_count
FRAZA8_V:
 ldi	ZL,low(FRAZA8*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA8*2)					;
 ldi	temp,0
 add	ZL,temp_count						; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA8_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	temp_count
 rjmp	FRAZA8_V							; Продолжаем выводить буквы на дисплей
FRAZA8_END:									; Выходим из цикла вывода фразы на дисплей
 rcall	d500ms

 sei										; Рзрешаем прерывание, запускаем таймер
 clr	sleep_t								; Обнуляем таймер автовыключения

 sbi	PORTB,2								; Подключаем резисторы подтяжки для чипа
 sbi	PORTC,2								; Подключаем резисторы подтяжки для EEPROM прошивок

;	Опрашиваем кнопки
WAIT_STEP1:

 sbis	PIN_BOT_L,BOT_L						; Проеряем состояние кнопки "<"
 rjmp	BOT1_WAIT1

 sbis	PIN_BOT_R,BOT_R						; Проеряем состояние кнопки ">"
 rjmp	BOT2_WAIT1

 sbis	PIN_BOT_Y,BOT_Y						; Проеряем состояние кнопки "YES"
 rjmp	BOT3_WAIT1

 sbis	PIN_BOT_N,BOT_N						; Проеряем состояние кнопки "NO"
 rjmp	BOT4_WAIT1


; Проверяем таймер автовыключения через 1 минуту
 cpse	time_sleep,sleep_t					; Пропуск следующей команды при равенстве
 rjmp	WAIT_STEP1							; Если кнопка не нажата, продолжаем опрашивать кнопки

 rjmp	AUTO_OFF							; Если таймер сработал, то переходим в дежурный режим



BOT1_WAIT1:									; Устраняем дребезг контактов
 rcall	d13ms
 sbic	PIN_BOT_L,BOT_L
 rjmp	WAIT_STEP1
BOT1_WAIT1_N:								; Ждем отпускания кнопки
 rcall	d13ms
 sbis	PIN_BOT_L,BOT_L
 rjmp	BOT1_WAIT1_N
 rjmp	STEP2								; Если кнопка отпущена, то переходим ко 2-му шагу 


BOT2_WAIT1:
 rcall	d13ms
 sbic	PIN_BOT_R,BOT_R
 rjmp	WAIT_STEP1
BOT2_WAIT1_N:								; Ждем отпускания кнопки
 rcall	d13ms
 sbis	PIN_BOT_R,BOT_R
 rjmp	BOT2_WAIT1_N
 rjmp	STEP2								; Если кнопка отпущена, то переходим ко 2-му шагу 


BOT3_WAIT1:
 rcall	d13ms
 sbic	PIN_BOT_Y,BOT_Y
 rjmp	WAIT_STEP1
BOT3_WAIT1_N:								; Ждем отпускания кнопки
 rcall	d13ms
 sbis	PIN_BOT_Y,BOT_Y
 rjmp	BOT3_WAIT1_N
 rjmp	STEP2								; Если кнопка отпущена, то переходим ко 2-му шагу 


BOT4_WAIT1:
 rcall	d13ms
 sbic	PIN_BOT_N,BOT_N
 rjmp	WAIT_STEP1
BOT4_WAIT1_N:								; Ждем отпускания кнопки
 rcall	d13ms
 sbis	PIN_BOT_N,BOT_N
 rjmp	BOT4_WAIT1_N
 rjmp	STEP2								; Если кнопка отпущена, то переходим ко 2-му шагу 




;********************************************************************************
;																				;
;	ШАГ2: читаем содержимое чипа и выводим на дисплей основные параметры		;
;																				;
;********************************************************************************
STEP2:
 cli										; Запрещаем прерывание и тем самым останавливаем таймер
 rcall	lcd_clr
;-----------------------------------------------------------------------------
; Читаем название чипа с микросхемы Master памяти по протоколу I2C

 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$00							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_NAME,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$01							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_NAME+1,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$02							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_NAME+2,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$03							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_NAME+3,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$04							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_NAME+4,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$05							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_NAME+5,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$06							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_NAME+6,i2cdata



;-----------------------------------------------------------------------------
; Читаем серийный номер чипа с микросхемы памяти по протоколу I2C

 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$35							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_CRUM,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$36							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_CRUM+1,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$37							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_CRUM+2,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$38							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_CRUM+3,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$39							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_CRUM+4,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$3A							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_CRUM+5,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$3B							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_CRUM+6,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$3C							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_CRUM+7,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$3D							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_CRUM+8,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$3E							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_CRUM+9,i2cdata
nop
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 ldi	i2cdata,$3F							; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
; rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 sts	CHIP_CRUM+10,i2cdata




;-----------------------------------------------------------------------------------
;	Выводим считанную информацию с чипа на дисплей
;-----------------------------------------------------------------------------------

 rcall	LCD_CLR
;-------------------------------------------
;	Выводим надпись "название чипа:"
 clr	temp_count
FRAZA1_V:
 ldi	ZL,low(FRAZA1*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA1*2)					;
 ldi	temp,0
 add	ZL,temp_count						; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA1_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	temp_count
 rjmp	FRAZA1_V							; Продолжаем выводить буквы на дисплей
FRAZA1_END:									; Выходим из цикла вывода фразы на дисплей
 ldi	temp,23								; Задаем координаты X
 rcall	LCD_X
 ldi	temp,1
 rcall	LCD_Y							; Задаем координаты Y
 lds	temp,CHIP_NAME
 rcall	LCD_PUT_CHAR					; Выводим название чипа
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
;	Выводим надпись "CRUM чипа:"

 ldi	temp,10								; Задаем координаты X
 rcall	LCD_X
 ldi	temp,2
 rcall	LCD_Y							; Задаем координаты Y

 clr	temp_count
FRAZA2_V:
 ldi	ZL,low(FRAZA2*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA2*2)					;
 ldi	temp,0
 add	ZL,temp_count						; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA2_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	temp_count
 rjmp	FRAZA2_V							; Продолжаем выводить буквы на дисплей
FRAZA2_END:									; Выходим из цикла вывода фразы на дисплей
 ldi	temp,10								; Задаем координаты X
 rcall	LCD_X
 ldi	temp,3
 rcall	LCD_Y								; Задаем координаты Y
 lds	temp,CHIP_CRUM
 rcall	LCD_PUT_CHAR						; Выводим серийный номер чипа
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
;	Выводим надпись "<         >"

 ldi	temp,0								; Задаем координаты X
 rcall	LCD_X
 ldi	temp,5
 rcall	LCD_Y							; Задаем координаты Y
 ldi	temp,60
 rcall	LCD_PUT_CHAR
 ldi	temp,73								; Задаем координаты X
 rcall	LCD_X
 ldi	temp,5
 rcall	LCD_Y							; Задаем координаты Y
 ldi	temp,62
 rcall	LCD_PUT_CHAR


 sei										; Рзрешаем прерывание, запускаем таймер
 clr	sleep_t								; Обнуляем таймер автовыключения

;--------------------------------------------
;	Ждем нажатия кнопок

WAIT_STEP2:

 sbis	PIN_BOT_L,BOT_L						; Проеряем состояние кнопки "<"
 rjmp	BOT1_WAIT2

 sbis	PIN_BOT_R,BOT_R						; Проеряем состояние кнопки ">"
 rjmp	BOT4_WAIT2

 sbis	PIN_BOT_N,BOT_N						; Проеряем состояние кнопки "NO"
 rjmp	BOT3_WAIT2


; Проверяем таймер автовыключения
 cpse	time_sleep,sleep_t					; Пропуск следующей команды при равенстве
 rjmp	WAIT_STEP2							; Если кнопка не нажата, продолжаем опрашивать кнопки

 rjmp	AUTO_OFF							; Если таймер сработал, то переходим в дежурный режим



BOT1_WAIT2:									; Устраняем дребезг контактов
 rcall	d13ms
 sbic	PIN_BOT_L,BOT_L
 rjmp	WAIT_STEP2
BOT1_WAIT2_N:								; Ждем отпускания кнопки
 rcall	d13ms
 sbis	PIN_BOT_L,BOT_L
 rjmp	BOT1_WAIT2_N
 rjmp	STEP1

BOT4_WAIT2:									; Устраняем дребезг контактов
 rcall	d13ms
 sbic	PIN_BOT_R,BOT_R
 rjmp	WAIT_STEP2
BOT4_WAIT2_N:								; Ждем отпускания кнопки
 rcall	d13ms
 sbis	PIN_BOT_R,BOT_R
 rjmp	BOT4_WAIT2_N
 rjmp	STEP3								; Если кнопка ">" нажата, то идем дальше


BOT3_WAIT2:									; Устраняем дребезг контактов
 rcall	d13ms
 sbic	PIN_BOT_N,BOT_N
 rjmp	WAIT_STEP2
BOT3_WAIT2_N:								; Ждем отпускания кнопки
 rcall	d13ms
 sbis	PIN_BOT_N,BOT_N
 rjmp	BOT3_WAIT2_N
 rjmp	STEP1								; Если кнопка ">" нажата, то идем дальше




;********************************************************************************
;																				;
;					ШАГ3: Выбираем прошивку и прошиваем чип						;
;																				;
;********************************************************************************

STEP3:

 rcall	LCD_CLR
;-------------------------------------------
;	Выводим надпись "Выберите"

 ldi	temp,19								; Задаем координаты X
 rcall	LCD_X
 ldi	temp,0
 rcall	LCD_Y							; Задаем координаты Y

 clr	temp_count
FRAZA5_V:
 ldi	ZL,low(FRAZA5*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA5*2)					;
 ldi	temp,0
 add	ZL,temp_count						; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA5_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	temp_count
 rjmp	FRAZA5_V							; Продолжаем выводить буквы на дисплей
FRAZA5_END:									; Выходим из цикла вывода фразы на дисплей

;-------------------------------------------
;	Выводим надпись "прошивку чипа"

 ldi	temp,0								; Задаем координаты X
 rcall	LCD_X
 ldi	temp,1
 rcall	LCD_Y							; Задаем координаты Y

 clr	temp_count
FRAZA6_V:
 ldi	ZL,low(FRAZA6*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA6*2)					;
 ldi	temp,0
 add	ZL,temp_count						; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA6_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	temp_count
 rjmp	FRAZA6_V							; Продолжаем выводить буквы на дисплей
FRAZA6_END:									; Выходим из цикла вывода фразы на дисплей


;-------------------------------------------
;	Выводим надпись "прошить"

 ldi	temp,25								; Задаем координаты X
 rcall	LCD_X
 ldi	temp,4
 rcall	LCD_Y								; Задаем координаты Y

 clr	temp_count
FRAZA9_V:
 ldi	ZL,low(FRAZA9*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA9*2)					;
 ldi	temp,0
 add	ZL,temp_count						; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA9_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	temp_count
 rjmp	FRAZA9_V							; Продолжаем выводить буквы на дисплей
FRAZA9_END:									; Выходим из цикла вывода фразы на дисплей




;-------------------------------------------
;	Выводим надпись "<   чип   >"

 ldi	temp,0								; Задаем координаты X
 rcall	LCD_X
 ldi	temp,5
 rcall	LCD_Y								; Задаем координаты Y

 clr	temp_count
FRAZA7_V:
 ldi	ZL,low(FRAZA7*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA7*2)					;
 ldi	temp,0
 add	ZL,temp_count						; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA7_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	temp_count
 rjmp	FRAZA7_V							; Продолжаем выводить буквы на дисплей
FRAZA7_END:									; Выходим из цикла вывода фразы на дисплей



;--------------------------------------------
;	Ждем нажатия кнопок
 
 ldi	temp_count,1						; Устанавливаем счетчик на первую прошивки
 ldi	count_memH,0x01						; Задаем адрес в памяти slave для первой прошивки
 ldi	count_memL,0x20						;
 rcall	VIBOR								; Отображаем первую прошивку

 clr	sleep_t								; Обнуляем таймер автовыключения
 sei										; Рзрешаем прерывание, запускаем таймер


WAIT_STEP3:

 sbis	PIN_BOT_L,BOT_L						; Проеряем состояние кнопки "<"
 rjmp	BOT1_WAIT3

 sbis	PIN_BOT_Y,BOT_Y						; Проеряем состояние кнопки "YES"
 rjmp	BOT2_WAIT3

 sbis	PIN_BOT_N,BOT_N						; Проеряем состояние кнопки "NO"
 rjmp	BOT3_WAIT3

 sbis	PIN_BOT_R,BOT_R						; Проеряем состояние кнопки ">"
 rjmp	BOT4_WAIT3

 sbis	PIN_BOT_1,BOT_1						; Проеряем состояние кнопки "1"
 rjmp	BOT5_WAIT3

 sbis	PIN_BOT_2,BOT_2						; Проеряем состояние кнопки "2"
 rjmp	BOT6_WAIT3

 sbis	PIN_BOT_3,BOT_3						; Проеряем состояние кнопки "3"
 rjmp	BOT7_WAIT3


; Проверяем таймер автовыключения
 cpse	time_sleep,sleep_t					; Пропуск следующей команды при равенстве
 rjmp	WAIT_STEP3							; Если кнопка не нажата, продолжаем опрашивать кнопки

 rjmp	AUTO_OFF							; Если таймер сработал, то переходим в дежурный режим



BOT1_WAIT3:									; Кнопка "<" нажата. Команда "выбор превидущей прошивки"		
 rcall	d13ms
 sbic	PIN_BOT_L,BOT_L
 rjmp	WAIT_STEP3
BOT1_WAIT3_N:								; Ждем отпускания кнопки
 rcall	d13ms
 sbis	PIN_BOT_L,BOT_L
 rjmp	BOT1_WAIT3_N
 rjmp	SELECT_PREW							; Если кнопка "<" нажата, то выбираем превидущую прошивку


BOT2_WAIT3:									; Кнопка "YES" нажата. Команда "записать чип"
 rcall	d13ms
 sbic	PIN_BOT_Y,BOT_Y
 rjmp	WAIT_STEP3
BOT2_WAIT3_N:								; Ждем отпускания кнопки
 rcall	d13ms
 sbis	PIN_BOT_Y,BOT_Y
 rjmp	BOT2_WAIT3_N
 rjmp	STEP4								; Если кнопка отпущена, то переходим к 4-му шагу 


BOT3_WAIT3:									; Кнопка "NO" нажата. Команда "вернуться на один пункт назат"
 rcall	d13ms
 sbic	PIN_BOT_N,BOT_N
 rjmp	WAIT_STEP3
BOT3_WAIT3_N:								; Ждем отпускания кнопки
 rcall	d13ms
 sbis	PIN_BOT_N,BOT_N
 rjmp	BOT3_WAIT3_N
 rjmp	STEP1								; Если кнопка отпущена, то переходим к 1-му шагу 


BOT4_WAIT3:									; Кнопка ">" нажата. Команда "выбор слеующей прошивки"
 rcall	d13ms
 sbic	PIN_BOT_R,BOT_R
 rjmp	WAIT_STEP3
BOT4_WAIT3_N:								; Ждем отпускания кнопки
 rcall	d13ms
 sbis	PIN_BOT_R,BOT_R
 rjmp	BOT4_WAIT3_N
 rjmp	SELECT_NEXT							; Если кнопка ">" нажата, то выбираем следующую прошивку



BOT5_WAIT3:									; Кнопка "1" нажата.
 rcall	d13ms
 sbic	PIN_BOT_1,BOT_1
 rjmp	WAIT_STEP3
BOT5_WAIT3_N:								; Если кнопка нажата более 2 секунд, то сохраняем позицию прошивки
 rcall	d500ms
 sbic	PIN_BOT_1,BOT_1
 rjmp	SELECT_1							;  иначе выбираем ранее записанную позицию
 rcall	d500ms
 sbic	PIN_BOT_1,BOT_1
 rjmp	SELECT_1
 rcall	d500ms
 sbic	PIN_BOT_1,BOT_1
 rjmp	SELECT_1

 rjmp	SAVE_SELECT1						; Если кнопка "1" нажата более 2 секунд, то сохраняем позицию прошивки




BOT6_WAIT3:									; Кнопка "2" нажата.
 rcall	d13ms
 sbic	PIN_BOT_2,BOT_2
 rjmp	WAIT_STEP3
BOT6_WAIT3_N:								; Ждем отпускания кнопки
 rcall	d500ms
 sbic	PIN_BOT_2,BOT_2
 rjmp	SELECT_2							;  иначе выбираем ранее записанную позицию
 rcall	d500ms
 sbic	PIN_BOT_2,BOT_2
 rjmp	SELECT_2
 rcall	d500ms
 sbic	PIN_BOT_2,BOT_2
 rjmp	SELECT_2

 rjmp	SAVE_SELECT2						; Если кнопка "2" нажата более 2 секунд, то сохраняем позицию прошивки



BOT7_WAIT3:									; Кнопка "3" нажата.
 rcall	d13ms
 sbic	PIN_BOT_3,BOT_3
 rjmp	WAIT_STEP3
BOT7_WAIT3_N:								; Ждем отпускания кнопки
 rcall	d500ms
 sbic	PIN_BOT_3,BOT_3
 rjmp	SELECT_3							;  иначе выбираем ранее записанную позицию
 rcall	d500ms
 sbic	PIN_BOT_3,BOT_3
 rjmp	SELECT_3
 rcall	d500ms
 sbic	PIN_BOT_3,BOT_3
 rjmp	SELECT_3

 rjmp	SAVE_SELECT3						; Если кнопка "3" нажата более 2 секунд, то сохраняем позицию прошивки



SELECT_PREW:								; Находим превидущую прошивку по номеру счета
 clr	sleep_t								; Обнуляем таймер автовыключения

 cpi	temp_count,1						; Если это первая прошивка, то выбираем последнюю
 breq	VIBOR_P_Z 

 sbiw	XH:XL,60							; Находим превидущую прошивку по номеру счета,
 sbiw	XH:XL,60							;  отнимаем от регистровой пары сдвиг 176 (60+60+56)
 sbiw	XH:XL,60							;  для нахождения прошивки в общей памяти
 sbiw	XH:XL,60
 sbiw	XH:XL,48

 rcall	VIBOR

 dec	temp_count

 rjmp	WAIT_STEP3

VIBOR_P_Z:
 mov	temp_count,col_prosh				; Устанавливаем счетчик на первую прошивки
 ldi	count_memH,0x14						; Задаем адрес в памяти slave для первой прошивки
 ldi	count_memL,0x40						;
 rcall	VIBOR								; Отображаем первую прошивку

 rjmp	WAIT_STEP3




SELECT_NEXT:
 clr	sleep_t								; Обнуляем таймер автовыключения

 cp		temp_count,col_prosh				; Сравниваем порядковый номер прошивки с количеством прошивок
 breq	VIBOR_N_Z							; Если прошивка последняя, то начинаем с начала

 adiw	XH:XL,60							; Находим следующую прошивку по номеру счета
 adiw	XH:XL,60							;  прибавляем к регистровой паре сдвиг 288 (60+60+60+60+48)
 adiw	XH:XL,60							;  для нахождения следующей прошивки в общей памяти
 adiw	XH:XL,60
 adiw	XH:XL,48
			
 rcall	VIBOR
 inc	temp_count							; Увеличиваем счетчик текущей прошивки
 rjmp	WAIT_STEP3

VIBOR_N_Z:									; Если добрались до последней прошивки, то
 ldi	temp_count,1						; Устанавливаем счетчик на первую прошивки
 ldi	count_memH,0x01						; Задаем адрес в памяти slave для первой прошивки
 ldi	count_memL,0x20						;
 rcall	VIBOR								; Отображаем первую прошивку

 rjmp	WAIT_STEP3



VIBOR:										; Выводим название прошивки на дисплей
 push	count_memH
 push	count_memL

 adiw	XH:XL,1								; Читаем код фирмы прошивки
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start_s							; Передаем "старт" условие и адрес
 mov	i2cdata,count_memH					; Пишем адрес ячейки
 rcall	i2c_do_transfer_s					; Выполняем трансфер
 mov	i2cdata,count_memL					; Пишем адрес ячейки
 rcall	i2c_do_transfer_s					; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start_s						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса
 rcall	i2c_do_transfer_s					; Выполняем трансфер (читаем)
 mov	temp,i2cdata
 
 cpi	temp,0x11							; Проверяем код фирмы прошивки
 breq	VIBOR_S								; Выводим надпись SAMSUNG
 cpi	temp,0x22							; Проверяем код фирмы прошивки
 breq	VIBOR_X								; Выводим надпись XEROX
 cpi	temp,0x33							; Проверяем код фирмы прошивки
 breq	VIBOR_D								; Выводим надпись DELL
 cpi	temp,0x44							; Проверяем код фирмы прошивки
 breq	VIBOR_M								; Выводим надпись MB

VIBOR_S:
 ldi	temp,0								; Задаем координаты X
 rcall	LCD_X
 ldi	temp,2
 rcall	LCD_Y								; Задаем координаты Y

 clr	count_s
FRAZA12_V:									; Фраза "SAMSUNG"
 ldi	ZL,low(FRAZA12*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA12*2)					;
 ldi	temp,0
 add	ZL,count_s							; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA12_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	count_s
 rjmp	FRAZA12_V							; Продолжаем выводить буквы на дисплей
FRAZA12_END:									; Выходим из цикла вывода фразы на дисплей
 rjmp	VIBOR1

VIBOR_X:
 ldi	temp,0								; Задаем координаты X
 rcall	LCD_X
 ldi	temp,2
 rcall	LCD_Y								; Задаем координаты Y
 
 clr	count_s
FRAZA13_V:									; Фраза "XEROX"
 ldi	ZL,low(FRAZA13*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA13*2)					;
 ldi	temp,0
 add	ZL,count_s							; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA13_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	count_s
 rjmp	FRAZA13_V							; Продолжаем выводить буквы на дисплей
FRAZA13_END:								; Выходим из цикла вывода фразы на дисплей
 rjmp	VIBOR1

VIBOR_D:
 ldi	temp,0								; Задаем координаты X
 rcall	LCD_X
 ldi	temp,2
 rcall	LCD_Y								; Задаем координаты Y
 clr	count_s
FRAZA14_V:									; Фраза "Dell"
 ldi	ZL,low(FRAZA14*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA14*2)					;
 ldi	temp,0
 add	ZL,count_s							; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA14_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	count_s
 rjmp	FRAZA14_V							; Продолжаем выводить буквы на дисплей
FRAZA14_END:								; Выходим из цикла вывода фразы на дисплей 
 rjmp	VIBOR1

VIBOR_M:
 ldi	temp,0								; Задаем координаты X
 rcall	LCD_X
 ldi	temp,2
 rcall	LCD_Y								; Задаем координаты Y
 clr	count_s
FRAZA15_V:									; Фраза "MB"
 ldi	ZL,low(FRAZA15*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA15*2)					;
 ldi	temp,0
 add	ZL,count_s							; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA15_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	count_s
 rjmp	FRAZA15_V							; Продолжаем выводить буквы на дисплей
FRAZA15_END:								; Выходим из цикла вывода фразы на дисплей 
; rjmp	VIBOR1

VIBOR1:
 ldi	temp,0								; Задаем координаты X
 rcall	LCD_X
 ldi	temp,3
 rcall	LCD_Y								; Задаем координаты Y

 clr	count_s

 adiw	XH:XL,1								; Читаем название прошивки после порядкого номера
;-----------------------------------------------------------------------------
; Читаем название чипа с микросхемы памяти по протоколу I2C slave и выводим на дисплей

VIBOR2:
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start_s							; Передаем "старт" условие и адрес
 mov	i2cdata,count_memH					; Пишем адрес ячейки
 rcall	i2c_do_transfer_s					; Выполняем трансфер
 mov	i2cdata,count_memL					; Пишем адрес ячейки
 rcall	i2c_do_transfer_s					; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start_s						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer_s					; Выполняем трансфер (читаем)
 mov	temp,i2cdata
 rcall	LCD_PUT_CHAR						; Выводим символ на дисплей

 adiw	XH:XL,1
 inc	count_s
 mov	temp,count_s
 cpi	temp,14								; Повторяем цикл 14 раз
 brne	VIBOR2

 pop	count_memL
 pop	count_memH

 clr	sleep_t								; Обнуляем таймер автовыключения

ret



;--------------------------------------------
SELECT_1:									; Выбираем прошивку, сохраненную в первой позиции

EEPROM_read11:
 sbic	EECR,EEWE							; Ждать завершения предыдущей записи, если была
 rjmp	EEPROM_read11
 clr	temp
 out	EEARH,temp
 ldi	temp,0x10
 out	EEARL,temp							; Заносим адрес в регистр адреса
 sbi	EECR,EERE							; Начать чтение
 in		temp_count,EEDR						; Считываем порядковый номер прошивки из EEPROM
 
EEPROM_read12:
 sbic	EECR,EEWE							; Ждать завершения предыдущей записи, если была
 rjmp	EEPROM_read12
 clr	temp
 out	EEARH,temp
 ldi	temp,0x11
 out	EEARL,temp							; Заносим адрес в регистр адреса
 sbi	EECR,EERE							; Начать чтение
 in		count_memH,EEDR						; Считываем старшый адрес прошивки из EEPROM

EEPROM_read13:
 sbic	EECR,EEWE							; Ждать завершения предыдущей записи, если была
 rjmp	EEPROM_read13
 clr	temp
 out	EEARH,temp
 ldi	temp,0x12
 out	EEARL,temp							; Заносим адрес в регистр адреса
 sbi	EECR,EERE							; Начать чтение
 in		count_memL,EEDR						; Считываем младший адрес прошивки из EEPROM

 rcall	VIBOR								; Отображаем первую прошивку
 rjmp	WAIT_STEP3





;--------------------------------------------
SELECT_2:
EEPROM_read21:
 sbic	EECR,EEWE							; Ждать завершения предыдущей записи, если была
 rjmp	EEPROM_read21
 clr	temp
 out	EEARH,temp
 ldi	temp,0x20
 out	EEARL,temp							; Заносим адрес в регистр адреса
 sbi	EECR,EERE							; Начать чтение
 in		temp_count,EEDR						; Считываем порядковый номер прошивки из EEPROM
 
EEPROM_read22:
 sbic	EECR,EEWE							; Ждать завершения предыдущей записи, если была
 rjmp	EEPROM_read22
 clr	temp
 out	EEARH,temp
 ldi	temp,0x21
 out	EEARL,temp							; Заносим адрес в регистр адреса
 sbi	EECR,EERE							; Начать чтение
 in		count_memH,EEDR						; Считываем старшый адрес прошивки из EEPROM

EEPROM_read23:
 sbic	EECR,EEWE							; Ждать завершения предыдущей записи, если была
 rjmp	EEPROM_read23
 clr	temp
 out	EEARH,temp
 ldi	temp,0x22
 out	EEARL,temp							; Заносим адрес в регистр адреса
 sbi	EECR,EERE							; Начать чтение
 in		count_memL,EEDR						; Считываем младший адрес прошивки из EEPROM

 rcall	VIBOR								; Отображаем первую прошивку
 rjmp	WAIT_STEP3



;--------------------------------------------
SELECT_3:
EEPROM_read31:
 sbic	EECR,EEWE							; Ждать завершения предыдущей записи, если была
 rjmp	EEPROM_read31
 clr	temp
 out	EEARH,temp
 ldi	temp,0x30
 out	EEARL,temp							; Заносим адрес в регистр адреса
 sbi	EECR,EERE							; Начать чтение
 in		temp_count,EEDR						; Считываем порядковый номер прошивки из EEPROM
 
EEPROM_read32:
 sbic	EECR,EEWE							; Ждать завершения предыдущей записи, если была
 rjmp	EEPROM_read32
 clr	temp
 out	EEARH,temp
 ldi	temp,0x31
 out	EEARL,temp							; Заносим адрес в регистр адреса
 sbi	EECR,EERE							; Начать чтение
 in		count_memH,EEDR						; Считываем старшый адрес прошивки из EEPROM

EEPROM_read33:
 sbic	EECR,EEWE							; Ждать завершения предыдущей записи, если была
 rjmp	EEPROM_read33
 clr	temp
 out	EEARH,temp
 ldi	temp,0x32
 out	EEARL,temp							; Заносим адрес в регистр адреса
 sbi	EECR,EERE							; Начать чтение
 in		count_memL,EEDR						; Считываем младший адрес прошивки из EEPROM

 rcall	VIBOR								; Отображаем первую прошивку
 rjmp	WAIT_STEP3




;--------------------------------------------
SAVE_SELECT1:								; Сохраняем прошивку в первую позицию

 rcall	d100ms
 cli										; Запрещаем прерывание на время записи в память EEPROM

EEPROM_write11:
 sbic	EECR,EEWE
 rjmp	EEPROM_write11						; Ждем завершения предыдущей записи, если была
 clr	temp
 out	EEARH,temp
 ldi	temp,0x10
 out	EEARL,temp							; Заносим адрес в регистр адреса
 out	EEDR,temp_count							; Запишем порядковый номер прошивки в регистр данных
 sbi	EECR,EEMWE							; Установить флаг EEMWE
 sbi	EECR,EEWE							; Начать запись в EEPROM
 rcall	d100ms

EEPROM_write12:
 sbic	EECR,EEWE
 rjmp	EEPROM_write12						; Ждем завершения предыдущей записи, если была
 clr	temp
 out	EEARH,temp
 ldi	temp,0x11
 out	EEARL,temp							; Заносим адрес в регистр адреса
 out	EEDR,count_memH							; Запишем порядковый номер прошивки в регистр данных
 sbi	EECR,EEMWE							; Установить флаг EEMWE
 sbi	EECR,EEWE							; Начать запись в EEPROM
 rcall	d100ms

EEPROM_write13:
 sbic	EECR,EEWE
 rjmp	EEPROM_write13						; Ждем завершения предыдущей записи, если была
 clr	temp
 out	EEARH,temp
 ldi	temp,0x12
 out	EEARL,temp							; Заносим адрес в регистр адреса
 out	EEDR,count_memL							; Запишем порядковый номер прошивки в регистр данных
 sbi	EECR,EEMWE							; Установить флаг EEMWE
 sbi	EECR,EEWE							; Начать запись в EEPROM
 rcall	d100ms

 sei 

 rcall	SAVE_END							; Выводим надпись "Готово"

BOT5_WAIT_SAVE:								; Ждем отпускания кнопки
 rcall	d13ms
 sbis	PIN_BOT_1,BOT_1
 rjmp	BOT5_WAIT_SAVE

 rjmp	WAIT_STEP3
 

;--------------------------------------------
SAVE_SELECT2:								; Сохраняем прошивку во вторую позицию
 rcall	d100ms
 cli										; Запрещаем прерывание на время записи в память EEPROM

EEPROM_write21:
 sbic	EECR,EEWE
 rjmp	EEPROM_write21						; Ждем завершения предыдущей записи, если была
 clr	temp
 out	EEARH,temp
 ldi	temp,0x20
 out	EEARL,temp							; Заносим адрес в регистр адреса
 out	EEDR,temp_count						; Запишем порядковый номер прошивки в регистр данных
 sbi	EECR,EEMWE							; Установить флаг EEMWE
 sbi	EECR,EEWE							; Начать запись в EEPROM
 rcall	d100ms

EEPROM_write22:
 sbic	EECR,EEWE
 rjmp	EEPROM_write22						; Ждем завершения предыдущей записи, если была
 clr	temp
 out	EEARH,temp
 ldi	temp,0x21
 out	EEARL,temp							; Заносим адрес в регистр адреса
 out	EEDR,count_memH						; Запишем старший адрес прошивки в регистр данных
 sbi	EECR,EEMWE							; Установить флаг EEMWE
 sbi	EECR,EEWE							; Начать запись в EEPROM
 rcall	d100ms

EEPROM_write23:
 sbic	EECR,EEWE
 rjmp	EEPROM_write23						; Ждем завершения предыдущей записи, если была
 clr	temp
 out	EEARH,temp
 ldi	temp,0x22
 out	EEARL,temp							; Заносим адрес в регистр адреса
 out	EEDR,count_memL						; Запишем младший адрес прошивки в регистр данных
 sbi	EECR,EEMWE							; Установить флаг EEMWE
 sbi	EECR,EEWE							; Начать запись в EEPROM
 rcall	d100ms

 sei 

 rcall	SAVE_END							; Выводим надпись "Готово"

BOT6_WAIT_SAVE:								; Ждем отпускания кнопки
 rcall	d13ms
 sbis	PIN_BOT_2,BOT_2
 rjmp	BOT6_WAIT_SAVE

 rjmp	WAIT_STEP3




;--------------------------------------------
SAVE_SELECT3:								; Сохраняем прошивку в третью позицию
 rcall	d100ms
 cli										; Запрещаем прерывание на время записи в память EEPROM

EEPROM_write31:
 sbic	EECR,EEWE
 rjmp	EEPROM_write31						; Ждем завершения предыдущей записи, если была
 clr	temp
 out	EEARH,temp
 ldi	temp,0x30
 out	EEARL,temp							; Заносим адрес в регистр адреса
 out	EEDR,temp_count						; Запишем порядковый номер прошивки в регистр данных
 sbi	EECR,EEMWE							; Установить флаг EEMWE
 sbi	EECR,EEWE							; Начать запись в EEPROM
 rcall	d100ms

EEPROM_write32:
 sbic	EECR,EEWE
 rjmp	EEPROM_write32						; Ждем завершения предыдущей записи, если была
 clr	temp
 out	EEARH,temp
 ldi	temp,0x31
 out	EEARL,temp							; Заносим адрес в регистр адреса
 out	EEDR,count_memH						; Запишем старший адрес прошивки в регистр данных
 sbi	EECR,EEMWE							; Установить флаг EEMWE
 sbi	EECR,EEWE							; Начать запись в EEPROM
 rcall	d100ms

EEPROM_write33:
 sbic	EECR,EEWE
 rjmp	EEPROM_write33						; Ждем завершения предыдущей записи, если была
 clr	temp
 out	EEARH,temp
 ldi	temp,0x32
 out	EEARL,temp							; Заносим адрес в регистр адреса
 out	EEDR,count_memL						; Запишем младший адрес прошивки в регистр данных
 sbi	EECR,EEMWE							; Установить флаг EEMWE
 sbi	EECR,EEWE							; Начать запись в EEPROM
 rcall	d100ms

 sei 

 rcall	SAVE_END							; Выводим надпись "Готово"

BOT7_WAIT_SAVE:								; Ждем отпускания кнопки
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
 ldi	ZL,low(FRAZA19*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA19*2)					;
 ldi	temp,0
 add	ZL,count_s							; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA19_END1						; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	count_s
 rjmp	FRAZA19_V1							; Продолжаем выводить буквы на дисплей
FRAZA19_END1:								; Выходим из цикла вывода фразы на дисплей 

ret

;********************************************************************************
;																				;
;					ШАГ4: Прошиваем прошивку и проверяем						;
;																				;
;********************************************************************************

STEP4:
 clr	sleep_t								; Обнуляем таймер автовыключения
 cli										; Запрещаем прерывание и тем самым останавливаем таймер


 rcall	LCD_CLR
 ldi	temp,0								; Задаем координаты X
 rcall	LCD_X
 ldi	temp,0
 rcall	LCD_Y							; Задаем координаты Y
 
 clr	count_s
FRAZA16_V:
 ldi	ZL,low(FRAZA16*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA16*2)					;
 ldi	temp,0
 add	ZL,count_s							; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA16_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	count_s
 rjmp	FRAZA16_V							; Продолжаем выводить буквы на дисплей
FRAZA16_END:								; Выходим из цикла вывода фразы на дисплей 


;	Читаем бит из памяти программатора выбранной прошивки

 clr	temp_count							; Очищаем регистр счетчика цикла
 clr	count_m								; Очищаем регистр счетчика ячейки чипа 
 clr	progres								; Очищаем регистр прогресса записи (0%)
 clr	progres_temp
 adiw	XH:XL,16							; Перемещаемся на начало выбранной прошивки
 mov	count_memH_t,count_memH
 mov	count_memL_t,count_memL
STEP4_1:
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start_s							; Передаем "старт" условие и адрес
 mov	i2cdata,count_memH					; Пишем старший адрес ячейки
 rcall	i2c_do_transfer_s					; Выполняем трансфер
 mov	i2cdata,count_memL					; Пишем младший адрес ячейки
 rcall	i2c_do_transfer_s					; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start_s						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса, т.к. следует условие "стоп"
 rcall	i2c_do_transfer_s					; Выполняем трансфер (читаем)
 mov	temp,i2cdata

;	Пишем прочитанный бит в подключенный к программатору чип
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес чипа и команду "писать"
 rcall	i2c_start							; Посылаем адрес и команду
 mov	i2cdata,count_m						; Задаем адрес ячейки для записи
 rcall	i2c_do_transfer						; Выполняем передачу
 mov	i2cdata,temp						; Устанавливаем данные
 rcall	i2c_do_transfer						; Выполняем передачу
 rcall	i2c_stop							; Передаем стоп условие
 rcall	d6ms								; Ждем пока данные запишуться в память чипа


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
 
 inc	progres								; Увеличиваем процент на единицу
 mov	temp,progres
 rcall LCD_PUT_CHAR						; Выводим процент выполнения
 ldi	temp,48
 rcall LCD_PUT_CHAR						; Добавляем 0
 ldi	temp,37
 rcall LCD_PUT_CHAR						; Добавляем %

 clr	progres_temp


STEP4_1_1:
;	Повторяем цикл 255 раз
 inc	temp_count							; Увеличиваем счетчик количества циклов
 inc	count_m								; Увеичиваем на 1 адрес ячейки для записи
 adiw	XH:XL,1								; Находим следующий адрес памяти путем
											;  увеличения на единицу count_memH:count_memL
 cpi	temp_count,255						; Повторяем цикл 255 раз
 brne	STEP4_1	



;	Заполняем оставшиеся 255 ячеек подключенного чипа FF
 clr	temp_count							; Сбрасываем счетчик цикла
STEP4_3:
 ldi	i2cadr,$A2+i2cwr					; Задаем адрес чипа и команду "писать"
 rcall	i2c_start							; Посылаем адрес и команду
 mov	i2cdata,count_m						; Задаем адрес ячейки для записи
 rcall	i2c_do_transfer						; Выполняем передачу
 ldi	i2cdata,0xFF						; Устанавливаем данные FF
 rcall	i2c_do_transfer						; Выполняем передачу
 rcall	i2c_stop							; Передаем стоп условие
 rcall	d6ms								; Ждем пока данные запишуться в память чипа

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

;	Повторяем цикл 255 раз
STEP4_3_1:
 inc	temp_count							; Увеличиваем счетчик количества циклов
 inc	count_m								; Увеичиваем на 1 адрес ячейки для записи
											;  увеличения на единицу count_memH:count_memL
 cpi	temp_count,255						; Повторяем цикл 255 раза
 brne	STEP4_3	


 ldi	temp,37
 rcall	LCD_X
 ldi	temp,1
 rcall	LCD_Y
 
 ldi	temp,1								; Выводим надпись 100%
 rcall LCD_PUT_CHAR
 ldi	temp,0
 rcall LCD_PUT_CHAR
 ldi	temp,0
 rcall LCD_PUT_CHAR
 ldi	temp,37
 rcall LCD_PUT_CHAR

 rcall	d500ms
 clr	sleep_t								; Обнуляем таймер автовыключения

;--------------------------------------------
;	Проверяем записанные данные в чипе

;	Выводим фразу "Проверяем чип" 
 ldi	temp,0
 rcall	LCD_X
 ldi	temp,3
 rcall	LCD_Y

 clr	count_s
FRAZA18_V:
 ldi	ZL,low(FRAZA18*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA18*2)					;
 ldi	temp,0
 add	ZL,count_s							; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA18_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	count_s
 rjmp	FRAZA18_V							; Продолжаем выводить буквы на дисплей
FRAZA18_END:									; Выходим из цикла вывода фразы на дисплей 

 rcall	d500ms
 rcall	d500ms

 clr	temp_count
 mov	count_memH,count_memH_t				; Восстанавливаем адрес выбранной прошивки в памяти программатора
 mov	count_memL,count_memL_t

STEP4_4:
;	Читаем данные из чипа
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start							; Передаем "старт" условие и адрес
 mov	i2cdata,temp_count					; Пишем адрес ячейки
 rcall	i2c_do_transfer						; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса. Set no acknowledge (read is followed by a stop condition)
 rcall	i2c_do_transfer						; Выполняем трансфер (читаем)
 mov	temp,i2cdata

;	Читаем данные из памяти программатора
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес устройства и пишем
 rcall	i2c_start_s							; Передаем "старт" условие и адрес
 mov	i2cdata,count_memH					; Пишем старший адрес ячейки
 rcall	i2c_do_transfer_s					; Выполняем трансфер
 mov	i2cdata,count_memL					; Пишем младший адрес ячейки
 rcall	i2c_do_transfer_s					; Выполняем трансфер
 ldi	i2cadr,$A0+i2crd					; Задаем адрес устройства и читаем
 rcall	i2c_rep_start_s						; Передаем "повтор старта" и адрес
 sec										; Установка флага переноса, т.к. следует условие "стоп"
 rcall	i2c_do_transfer_s					; Выполняем трансфер (читаем)
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

;	Выводим фразу "Ошибка"
 clr	count_s
FRAZA20_V:
 ldi	ZL,low(FRAZA20*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA20*2)					;
 ldi	temp,0
 add	ZL,count_s							; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA20_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	count_s
 rjmp	FRAZA20_V							; Продолжаем выводить буквы на дисплей
FRAZA20_END:								; Выходим из цикла вывода фразы на дисплей 

 ldi	temp,0
 rcall	LCD_X
 ldi	temp,2
 rcall	LCD_Y

;	Выводим фразу "по адресу"
 clr	count_s
FRAZA21_V:
 ldi	ZL,low(FRAZA21*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA21*2)					;
 ldi	temp,0
 add	ZL,count_s							; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA21_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	count_s
 rjmp	FRAZA21_V							; Продолжаем выводить буквы на дисплей
FRAZA21_END:								; Выходим из цикла вывода фразы на дисплей 

 mov	temp,temp_count						; Выводим адрес, по которому ошибка
 rcall	LCD_PUT_CHAR

 clr	sleep_t								; Обнуляем таймер автовыключения
 sei										; Рзрешаем прерывание, запускаем таймер

 rjmp	STEP5


;-------------------------------------------
;	Выводим фразу "Меняем CRUM"
STEP4_5:
 ldi	temp,0
 rcall	LCD_X
 ldi	temp,5
 rcall	LCD_Y

 clr	count_s
FRAZA17_V:
 ldi	ZL,low(FRAZA17*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA17*2)					;
 ldi	temp,0
 add	ZL,count_s							; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA17_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	count_s
 rjmp	FRAZA17_V							; Продолжаем выводить буквы на дисплей
FRAZA17_END:									; Выходим из цикла вывода фразы на дисплей 

 rcall	d500ms
 rcall	d500ms
 rcall	d500ms

;	Проверяем правильность серийного номера чипа. В нем должны быть только цифры от 0 до 9 (0x30...0x39). Если не так,
;		то меняем соответствующий разряд на 0x30
PROBAB_CRUM:
 lds	temp,CHIP_CRUM						; Загружаем 1 разряд CRUM чипа
 cpi	temp,0x40							; Проверяем его значение
 brsh	EXCHANGE_CRUM_1						; Если оно больше или равно 9, то меняем его на 0

 lds	temp,CHIP_CRUM+1					; Загружаем 2 разряд CRUM чипа
 cpi	temp,0x40							; Проверяем его значение
 brsh	EXCHANGE_CRUM_2						; Если оно больше или равно 9, то меняем его на 0

 lds	temp,CHIP_CRUM+2					; Загружаем 3 разряд CRUM чипа
 cpi	temp,0x40							; Проверяем его значение
 brsh	EXCHANGE_CRUM_3						; Если оно больше или равно 9, то меняем его на 0

 lds	temp,CHIP_CRUM+3					; Загружаем 4 разряд CRUM чипа
 cpi	temp,0x40							; Проверяем его значение
 brsh	EXCHANGE_CRUM_4						; Если оно больше или равно 9, то меняем его на 0

 lds	temp,CHIP_CRUM+4					; Загружаем 5 разряд CRUM чипа
 cpi	temp,0x40							; Проверяем его значение
 brsh	EXCHANGE_CRUM_5						; Если оно больше или равно 9, то меняем его на 0

 lds	temp,CHIP_CRUM+5					; Загружаем 6 разряд CRUM чипа
 cpi	temp,0x40							; Проверяем его значение
 brsh	EXCHANGE_CRUM_6						; Если оно больше или равно 9, то меняем его на 0

 lds	temp,CHIP_CRUM+6					; Загружаем 7 разряд CRUM чипа
 cpi	temp,0x40							; Проверяем его значение
 brsh	EXCHANGE_CRUM_7						; Если оно больше или равно 9, то меняем его на 0

 lds	temp,CHIP_CRUM+7					; Загружаем 8 разряд CRUM чипа
 cpi	temp,0x40							; Проверяем его значение
 brsh	EXCHANGE_CRUM_8						; Если оно больше или равно 9, то меняем его на 0

 lds	temp,CHIP_CRUM+8					; Загружаем 9 разряд CRUM чипа
 cpi	temp,0x40							; Проверяем его значение
 brsh	EXCHANGE_CRUM_9						; Если оно больше или равно 9, то меняем его на 0

 lds	temp,CHIP_CRUM+9					; Загружаем 10 разряд CRUM чипа
 cpi	temp,0x40							; Проверяем его значение
 brsh	EXCHANGE_CRUM_10					; Если оно больше или равно 9, то меняем его на 0

 lds	temp,CHIP_CRUM+10					; Загружаем 11 разряд CRUM чипа
 cpi	temp,0x40							; Проверяем его значение
 brsh	EXCHANGE_CRUM_11					; Если оно больше или равно 9, то меняем его на 0


 rjmp	EXCHANGE_CRUM						; Если серийный номер в порядке, тогда увеличиваем его на 1

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



;	Меняем серийный номер чипа, путем увеличения последних 7 разрядов на 1

EXCHANGE_CRUM:
 lds temp,CHIP_CRUM+10						; Загружаем из ОЗУ 11-й разряд счетчика CRUM
 inc temp									;  увеличили на 1 11-й разряд CRUM
 sts CHIP_CRUM+10,temp						;  сохраняем в ОЗУ результат
 cpi temp,0x3A								;  сравниваем значение с 10
 brne EXCHANGE_CRUM_END						;  если больше или равно 10, то считаем 
 ldi temp,0x30								;  если равно 10, то расчитываем 10 разряд
 sts CHIP_CRUM+10,temp						;  сброс превидущего разряда
 
 lds temp,CHIP_CRUM+9						; Загружаем из ОЗУ 10-й разряд счетчика CRUM
 inc temp									;  увеличили на 1 10-й разряд CRUM
 sts CHIP_CRUM+9,temp						;  сохраняем в ОЗУ результат
 cpi temp,0x3A								;  сравниваем значение с 10
 brne EXCHANGE_CRUM_END						;  если больше или равно 10, то считаем 
 ldi temp,0x30								;  если равно 10, то расчитываем 9 разряд
 sts CHIP_CRUM+9,temp						;  сброс превидущего разряда
 
 lds temp,CHIP_CRUM+8						; Загружаем из ОЗУ 9-й разряд счетчика CRUM
 inc temp									;  увеличили на 1 9-й разряд CRUM
 sts CHIP_CRUM+8,temp						;  сохраняем в ОЗУ результат
 cpi temp,0x3A								;  сравниваем значение с 10
 brne EXCHANGE_CRUM_END						;  если больше или равно 10, то считаем 
 ldi temp,0x30								;  если равно 10, то расчитываем 8 разряд
 sts CHIP_CRUM+8,temp						;  сброс превидущего разряда
 
 lds temp,CHIP_CRUM+7						; Загружаем из ОЗУ 8-й разряд счетчика CRUM
 inc temp									;  увеличили на 1 8-й разряд CRUM
 sts CHIP_CRUM+7,temp						;  сохраняем в ОЗУ результат
 cpi temp,0x3A								;  сравниваем значение с 10
 brne EXCHANGE_CRUM_END						;  если больше или равно 10, то считаем 
 ldi temp,0x30								;  если равно 10, то расчитываем 7 разряд
 sts CHIP_CRUM+7,temp						;  сброс превидущего разряда
 
 lds temp,CHIP_CRUM+6						; Загружаем из ОЗУ 7-й разряд счетчика CRUM
 inc temp									;  увеличили на 1 7-й разряд CRUM
 sts CHIP_CRUM+6,temp						;  сохраняем в ОЗУ результат
 cpi temp,0x3A								;  сравниваем значение с 10
 brne EXCHANGE_CRUM_END						;  если больше или равно 10, то считаем 
 ldi temp,0x30								;  если равно 10, то расчитываем 6 разряд
 sts CHIP_CRUM+6,temp						;  сброс превидущего разряда
 
 lds temp,CHIP_CRUM+5						; Загружаем из ОЗУ 6-й разряд счетчика CRUM
 inc temp									;  увеличили на 1 6-й разряд CRUM
 sts CHIP_CRUM+5,temp						;  сохраняем в ОЗУ результат
 cpi temp,0x3A								;  сравниваем значение с 10
 brne EXCHANGE_CRUM_END						;  если больше или равно 10, то считаем 
 ldi temp,0x30								;  если равно 10, то расчитываем 5 разряд
 sts CHIP_CRUM+5,temp						;  сброс превидущего разряда
 
 lds temp,CHIP_CRUM+4						; Загружаем из ОЗУ 5-й разряд счетчика CRUM
 inc temp									;  увеличили на 1 5-й разряд CRUM
 sts CHIP_CRUM+4,temp						;  сохраняем в ОЗУ результат
 cpi temp,0x3A								;  сравниваем значение с 10
 brne EXCHANGE_CRUM_END						;  если больше или равно 10, то считаем 
 ldi temp,0x30								;  если равно 10, то расчитываем 4 разряд
 sts CHIP_CRUM+4,temp						;  сброс превидущего разряда

EXCHANGE_CRUM_END:

;	Переписываем серийный номер нашим измененным
 ldi	temp,0x35
 mov	count_m,temp						; Устанавливаем счетчик адреса чипа на 0x0018
 
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес чипа и команду "писать"
 rcall	i2c_start							; Посылаем адрес и команду
 mov	i2cdata,count_m						; Задаем адрес ячейки для записи
 rcall	i2c_do_transfer						; Выполняем передачу
 lds	temp,CHIP_CRUM
 mov	i2cdata,temp						; Устанавливаем данные
 rcall	i2c_do_transfer						; Выполняем передачу
 rcall	i2c_stop							; Передаем стоп условие
 rcall	d6ms								; Ждем пока данные запишуться в память чипа
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес чипа и команду "писать"
 rcall	i2c_start							; Посылаем адрес и команду
 mov	i2cdata,count_m						; Задаем адрес ячейки для записи
 rcall	i2c_do_transfer						; Выполняем передачу
 lds	temp,CHIP_CRUM+1
 mov	i2cdata,temp						; Устанавливаем данные
 rcall	i2c_do_transfer						; Выполняем передачу
 rcall	i2c_stop							; Передаем стоп условие
 rcall	d6ms								; Ждем пока данные запишуться в память чипа
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес чипа и команду "писать"
 rcall	i2c_start							; Посылаем адрес и команду
 mov	i2cdata,count_m						; Задаем адрес ячейки для записи
 rcall	i2c_do_transfer						; Выполняем передачу
 lds	temp,CHIP_CRUM+2
 mov	i2cdata,temp						; Устанавливаем данные
 rcall	i2c_do_transfer						; Выполняем передачу
 rcall	i2c_stop							; Передаем стоп условие
 rcall	d6ms								; Ждем пока данные запишуться в память чипа
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес чипа и команду "писать"
 rcall	i2c_start							; Посылаем адрес и команду
 mov	i2cdata,count_m						; Задаем адрес ячейки для записи
 rcall	i2c_do_transfer						; Выполняем передачу
 lds	temp,CHIP_CRUM+3
 mov	i2cdata,temp						; Устанавливаем данные
 rcall	i2c_do_transfer						; Выполняем передачу
 rcall	i2c_stop							; Передаем стоп условие
 rcall	d6ms								; Ждем пока данные запишуться в память чипа
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес чипа и команду "писать"
 rcall	i2c_start							; Посылаем адрес и команду
 mov	i2cdata,count_m						; Задаем адрес ячейки для записи
 rcall	i2c_do_transfer						; Выполняем передачу
 lds	temp,CHIP_CRUM+4
 mov	i2cdata,temp						; Устанавливаем данные
 rcall	i2c_do_transfer						; Выполняем передачу
 rcall	i2c_stop							; Передаем стоп условие
 rcall	d6ms								; Ждем пока данные запишуться в память чипа
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес чипа и команду "писать"
 rcall	i2c_start							; Посылаем адрес и команду
 mov	i2cdata,count_m						; Задаем адрес ячейки для записи
 rcall	i2c_do_transfer						; Выполняем передачу
 lds	temp,CHIP_CRUM+5
 mov	i2cdata,temp						; Устанавливаем данные
 rcall	i2c_do_transfer						; Выполняем передачу
 rcall	i2c_stop							; Передаем стоп условие
 rcall	d6ms								; Ждем пока данные запишуться в память чипа
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес чипа и команду "писать"
 rcall	i2c_start							; Посылаем адрес и команду
 mov	i2cdata,count_m						; Задаем адрес ячейки для записи
 rcall	i2c_do_transfer						; Выполняем передачу
 lds	temp,CHIP_CRUM+6
 mov	i2cdata,temp						; Устанавливаем данные
 rcall	i2c_do_transfer						; Выполняем передачу
 rcall	i2c_stop							; Передаем стоп условие
 rcall	d6ms								; Ждем пока данные запишуться в память чипа
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес чипа и команду "писать"
 rcall	i2c_start							; Посылаем адрес и команду
 mov	i2cdata,count_m						; Задаем адрес ячейки для записи
 rcall	i2c_do_transfer						; Выполняем передачу
 lds	temp,CHIP_CRUM+7
 mov	i2cdata,temp						; Устанавливаем данные
 rcall	i2c_do_transfer						; Выполняем передачу
 rcall	i2c_stop							; Передаем стоп условие
 rcall	d6ms								; Ждем пока данные запишуться в память чипа
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес чипа и команду "писать"
 rcall	i2c_start							; Посылаем адрес и команду
 mov	i2cdata,count_m						; Задаем адрес ячейки для записи
 rcall	i2c_do_transfer						; Выполняем передачу
 lds	temp,CHIP_CRUM+8
 mov	i2cdata,temp						; Устанавливаем данные
 rcall	i2c_do_transfer						; Выполняем передачу
 rcall	i2c_stop							; Передаем стоп условие
 rcall	d6ms								; Ждем пока данные запишуться в память чипа
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес чипа и команду "писать"
 rcall	i2c_start							; Посылаем адрес и команду
 mov	i2cdata,count_m						; Задаем адрес ячейки для записи
 rcall	i2c_do_transfer						; Выполняем передачу
 lds	temp,CHIP_CRUM+9
 mov	i2cdata,temp						; Устанавливаем данные
 rcall	i2c_do_transfer						; Выполняем передачу
 rcall	i2c_stop							; Передаем стоп условие
 rcall	d6ms								; Ждем пока данные запишуться в память чипа
inc	count_m
 ldi	i2cadr,$A0+i2cwr					; Задаем адрес чипа и команду "писать"
 rcall	i2c_start							; Посылаем адрес и команду
 mov	i2cdata,count_m						; Задаем адрес ячейки для записи
 rcall	i2c_do_transfer						; Выполняем передачу
 lds	temp,CHIP_CRUM+10
 mov	i2cdata,temp						; Устанавливаем данные
 rcall	i2c_do_transfer						; Выполняем передачу
 rcall	i2c_stop							; Передаем стоп условие
 rcall	d6ms								; Ждем пока данные запишуться в память чипа


;	Пишем фразу в чип "Crum By Hwang.sk"
 ldi	temp,0x18
 mov	count_m,temp						; Устанавливаем счетчик адреса чипа на 0x0118
 clr	temp_count
FRAZA10_V:									; Пишем фразу "Crum By Hwang.sk"
 ldi	ZL,low(FRAZA10*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA10*2)					;
 ldi	temp,0
 add	ZL,temp_count						; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp

 ldi	i2cadr,$A2+i2cwr					; Задаем адрес чипа и команду "писать"
 rcall	i2c_start							; Посылаем адрес и команду
 mov	i2cdata,count_m						; Задаем адрес ячейки для записи
 rcall	i2c_do_transfer						; Выполняем передачу
 mov	i2cdata,temp						; Устанавливаем данные FF
 rcall	i2c_do_transfer						; Выполняем передачу
 rcall	i2c_stop							; Передаем стоп условие
 rcall	d6ms								; Ждем пока данные запишуться в память чипа

 inc	temp_count
 inc	count_m
 cpi	temp_count,16
 brne	FRAZA10_V							; Продолжаем выводить буквы на дисплей
FRAZA10_END:								; Выходим из цикла


;	Пишем фразу в чип "ChipResetter V.1"
 ldi	temp,0x40
 mov	count_m,temp						; Устанавливаем счетчик адреса чипа на 0x0118
 clr	temp_count

 ldi	temp,0
 rcall	LCD_X
 ldi	temp,4
 rcall	LCD_Y


; Пишем фразу "Crum By Hwang.sk"
FRAZA11_V:
 ldi	ZL,low(FRAZA11*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA11*2)					;
 ldi	temp,0
 add	ZL,temp_count						; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp

 ldi	i2cadr,$A2+i2cwr					; Задаем адрес чипа и команду "писать"
 rcall	i2c_start							; Посылаем адрес и команду
 mov	i2cdata,count_m						; Задаем адрес ячейки для записи
 rcall	i2c_do_transfer						; Выполняем передачу
 mov	i2cdata,temp						; Устанавливаем данные FF
 rcall	i2c_do_transfer						; Выполняем передачу
 rcall	i2c_stop							; Передаем стоп условие
 rcall	d6ms								; Ждем пока данные запишуться в память чипа

 inc	temp_count
 inc	count_m
 cpi	temp_count,16
 brne	FRAZA11_V							; Продолжаем выводить буквы на дисплей
FRAZA11_END:								; Выходим из цикла



 rcall	i2c_stop							; Передаем "стоп" - освобождаем линию
 rcall	i2c_stop_s



;	Выводим надпись "Готово"
 rcall	LCD_CLR
 ldi	temp,0
 rcall	LCD_X
 ldi	temp,2
 rcall	LCD_Y

 clr	count_s
FRAZA19_V:
 ldi	ZL,low(FRAZA19*2)					; Указатель на начало буфера
 ldi	ZH,high(FRAZA19*2)					;
 ldi	temp,0
 add	ZL,count_s							; Вычисляем следующий адрес буквы в массиве
 adc	ZH,temp
 lpm										; Загружаем из массива следующую букву
 mov	temp,r0								; Переносим загруженную букву в переменную temp
 cpi	temp,23								; Проверяем на конец массива
 breq	FRAZA19_END							; Если текущее значение массива равно 10, значит массив закончился
 rcall	LCD_PUT_CHAR						; Выводим букву на дисплей
 inc	count_s
 rjmp	FRAZA19_V							; Продолжаем выводить буквы на дисплей
FRAZA19_END:									; Выходим из цикла вывода фразы на дисплей 



;********************************************************************************
;																				;
;                         ШАГ5: Чип прошит и готов                              ;
;																				;
;********************************************************************************

 clr	sleep_t								; Обнуляем таймер автовыключения
 sei										; Рзрешаем прерывание, запускаем таймер

 cbi	PORTB,2								; Одключаем резисторы подтяжки для чипа
 cbi	PORTC,2								; Одключаем резисторы подтяжки для EEPROM прошивок

 rcall	d500ms
 rcall	d500ms
 rcall	d500ms

 rjmp	Step1




STEP5:
 sbis	PIN_BOT_R,BOT_R								; Проеряем состояние кнопки ">"
 rjmp	BOT4_WAIT5

 sbis	PIN_BOT_N,BOT_N								; Проеряем состояние кнопки "NO"
 rjmp	BOT3_WAIT1

; Проверяем таймер автовыключения
 cpse	time_sleep,sleep_t					; Пропуск следующей команды при равенстве
 rjmp	STEP5								; Если кнопка не нажата, продолжаем опрашивать кнопки

 rjmp	AUTO_OFF							; Если таймер сработал, то переходим в дежурный режим



BOT4_WAIT5:									; Устраняем дребезг контактов
 rcall	d13ms
 sbic	PIN_BOT_R,BOT_R
 rjmp	STEP5
BOT4_WAIT5_N:								; Ждем отпускания кнопки
 rcall	d13ms
 sbis	PIN_BOT_R,BOT_R
 rjmp	BOT4_WAIT5_N

 rjmp	STEP0								; Если кнопка ">" нажата, то переходим в самое начало программы


BOT3_WAIT5:									; Устраняем дребезг контактов
 rcall	d13ms
 sbic	PIN_BOT_N,BOT_N
 rjmp	STEP5
BOT3_WAIT5_N:								; Ждем отпускания кнопки
 rcall	d13ms
 sbis	PIN_BOT_N,BOT_N
 rjmp	BOT3_WAIT5_N

 rjmp	STEP0								; Если кнопка ">" нажата, то переходим в самое начало программы


MAIN_LOOP:
rjmp	MAIN_LOOP							; Повторяем цикл непрерывно










;****************************************************************************
;																			*
;             Таймер автовыключения при простое программатора               *
;																			*
;****************************************************************************

AUTO_OFF:

 ldi	temp,0b00000000                        ; запрещаем прерывание по "переполнению T1"
 out	TIMSK,Temp

 sei
;	Разрешаем внешнее прерывание INT0 для выхода из спящего режима
 ldi	temp,0b01000000
 out	GICR,temp



;	Выводим картинку дежурного режима
 rcall	LCD_CLR
 clr	temp
 rcall	LCD_X
 clr	temp
 rcall	LCD_Y
 clr	temp
 ldi	temp_count,84						; Загружаем счетчик символов значением 84
PIC_SLEEP:
 push	temp
 ldi	ZL,low(PIC_SLEEP_TAB*2)				; Вычисление адреса первого байта 
 ldi	ZH,high(PIC_SLEEP_TAB*2)			;  символа в массиве SLEEP
 rcall	LCD_PUT_ARRAY					; Вывод символа из массива SLEEP
 pop	temp
 inc	temp								; Увеличиваем значение счетчика
 cpse	temp,temp_count
rjmp	PIC_SLEEP							; Количество шагов меньше 84, тогда повторяем



; Разрешаем переход в спящий режим и выбираем условие генерации прерывания
;   по изменению на выводе

;-------------------------------------------
;	Режимы пониженного энергопотребления
		; 7 - SE   - разрешение перехода в режим пониженного потребления (1-разрешено)
		; 6 - SM2  \
		; 5 - SM1   - выбор режима пониженного энергопотребления
		; 4 - SM0  /
		; 3 - ISC11\
		; 2 - ISC10 - условие генерации внешнего прерывания INT1 
		; 1 - ISC01\
		; 0 - ISC00 - условие генерации внешнего прерывания INT0

 ldi	temp,0b10100000						; по спадающему фронту на выводе
 out	MCUCR,temp

 nop
 nop
 nop

 sleep										; Переходим в дежурный режим


AUTO_OFF_END:
 rcall	d1ms								; Выполняем задержку после сна

 rjmp	RESET								; После включения переходим в начало программы






;******************************************************************************
;******************************************************************************
;**                                                                          **
;**                                                                          **
;**                                 Процедуры                                **
;**                                                                          **
;**                                                                          **
;******************************************************************************
;******************************************************************************


;******************************************************************************
;*   Эта процедура предназначена для работы с                                 *        
;*   ЖК-дисплеем LPH7779 (Nokia3310) на базе                                  *        
;*   кристалла PCD8544                                                        *        
;*   (С) 2004 Великий Новгород, Захаревич Михаил (Rander)                     *       
;*                                                                            *
;*   Я немного переделал некоторые функции. Смотри оригинал, если что         *
;******************************************************************************

;------------------------------------------------------------------------------
LCD_INIT:     ;Начальная инициализация LCD
;------------------------------------------------------------------------------

 ldi	ZL,low(LCD_INIT_TAB*2)				; Указатель на начало буфера
 ldi	ZH,high(LCD_INIT_TAB*2)			;
 ldi	temp1,5								; Длина таблицы = 5
 cbi	LCD_DC_port,LCD_DC					; Сбрасываем сигнал D/C
 rcall	LCD_SEND							; Отправляем команды в LCD
ret											;

;------------------------------------------------------------------------------
LCD_SEND:     ;Передать N байт в LCD из program memory
;------------------------------------------------------------------------------
                                            ;  [temp1 = N] - число байт
                                            ;  [ZL = Адрес] - адрес первого
 cbi	LCD_SCE_port,LCD_SCE				; Сбрасываем сигнал SCE
LCD_LOOP:									;
 lpm										; Загружаем очередной байт
 adiw	ZL,1								; Смещаем указатель
LCD_SEND1:									;
 ldi	temp2,8								; Передача 8-ми разрядов
LCD_LOOP1:									;
 cbi	LCD_SCLK_port,LCD_SCLK				; Сбрасываем сигнал SCLK
 sbrc	r0,7								; Переносим старший бит
 sbi	LCD_SDIN_port,LCD_SDIN				;  регистра промежуточного
 sbrs	r0,7								;  хранения в линию
 cbi	LCD_SDIN_port,LCD_SDIN				;  сигнала SDIN
 lsl	r0									; Сдвиг влево (очередной бит)
 sbi	LCD_SCLK_port,LCD_SCLK				; Запись бита по фронту SCLK
 dec	temp2								; Итерация внутреннего цикла
 brne	LCD_LOOP1							;
 dec	temp1								; Итерация внешнего цикла
 brne	LCD_LOOP							;
 sbi	LCD_SCE_port,LCD_SCE				; Устанавливаем сигнал SCE
 sbi	LCD_DC_port,LCD_DC					; Устанавливаем сигнал DC
ret											; Конец процедуры

;------------------------------------------------------------------------------
LCD_PUT_CHAR:     ;Записать один символ в LCD
;------------------------------------------------------------------------------
 ldi	ZL,low(FONT_TAB*2)					; Вычисление адреса первого байта 
 ldi	ZH,high(FONT_TAB*2)					;  символа из основного массива

LCD_PUT_ARRAY:								; Вывод символа из любого массива
                                            ;  [temp = char] - номер символа
 clr	temp1
 clr	temp0
 mov	char_ind,temp						; Выбираем символ из массива
                                            ; adres=n*6+adres0
 add	temp,char_ind						; Прибовляем temp (n*2) 
 adc	temp1,temp0
 add	temp,char_ind						; Прибавляем temp (n*3) 
 adc	temp1,temp0
 add	temp,char_ind						; Прибавляем temp (n*4) 
 adc	temp1,temp0
 add	temp,char_ind						; Прибавляем temp (n*5) 
 adc	temp1,temp0
 add	temp,char_ind						; Прибавляем temp (n*6) 
 adc	temp1,temp0
 
 add	ZL,temp								; Находим адрес символа n в массиве
 adc	ZH,temp1							;  adres=adres+n*6

 
 ldi	temp1,6
 clr	r0
 cbi	LCD_SCE_port,LCD_SCE				; Сбрасываем сигнал SCE
 nop
 rjmp	LCD_LOOP

;------------------------------------------------------------------------------
LCD_X:     ;Установить новые координаты X (0<X<83) на LCD
;------------------------------------------------------------------------------
 
 ori	temp,0x80
 mov	r0,temp								; Координата для X
 ldi	temp1,1								; Передаем одну команду
 cbi	LCD_DC_port,LCD_DC					; Сбрасываем сигнал D/C
 cbi	LCD_SCE_port,LCD_SCE				; Сбрасываем сигнал SCE
 rjmp	LCD_SEND1							;
 
;------------------------------------------------------------------------------
LCD_Y:     ;Установить новые координаты Y(0<Y<5) на LCD
;------------------------------------------------------------------------------
 ori	temp,0x40
 mov	r0,temp								; Координата для Y
 ldi	temp1,1								; Передаем одну команду
 cbi	LCD_DC_port,LCD_DC					; Сбрасываем сигнал D/C
 cbi	LCD_SCE_port,LCD_SCE				; Сбрасываем сигнал SCE
 rjmp	LCD_SEND1							;

;------------------------------------------------------------------------------
LCD_CLR:     ;Очистить LCD
;------------------------------------------------------------------------------
 ldi	temp,0								; X = 0
 rcall	LCD_X								;
 ldi	temp,0								; Y = 0
 rcall	LCD_Y								;
 ldi	temp1,84							; печатаем 84 пробела
LCD_CLR1:
 push	temp1								; сохраняем temp1
 ldi	temp,32								; код пробела
 rcall	LCD_PUT_CHAR						; печать
 pop	temp1								; восстанавливаем temp1
 dec	temp1								; цикл
 brne	LCD_CLR1							;
 ldi	temp,0								; X = 0
 rcall	LCD_X								;
 ldi	temp,0								; Y = 0
 rcall	LCD_Y								;

ret




;------------------------------------------------------------------------------
LCD_INIT_TAB:     ;Таблица кодов инициализации
;------------------------------------------------------------------------------
 .db 0x21, 0xc5, 0x05, 0x20, 0x0C, 0x00
                                            ; 0x21 ;Режим (PD=0, V=0, H=1)
                                            ; 0xC5 ;Установка Vop=69
                                            ; 0x13 ;Установка Bias (n=4)
                                            ; 0x20 ;Режим (PD=0, V=0, H=0)
                                            ; 0x0D ;Нормальный режим (D=1, E=0)



;------------------------------------------------------------------------------
LCD_RESET:        ;Формирования импульса сброса индикатора
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
;  ПРИМЕР РАБОТЫ С I2C 


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
;*         Процедуры для общения с микросхемой памяти прошивок (slave)          *
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
;*            Преобразование двоичного числа в число для индикатора                *
;*                                                                                 *
;*                                                                                 *
;***********************************************************************************

;   Использует переменные: temp, temp1, adwL, adwH, mulL, mulH
                          
; Bin2ToBcd5
; ==========
; converts a 16-bit-binary to a 5-digit-BCD
; In: 16-bit-binary in adwH,adwL
; Out: 5-digit-BCD
; Used registers:temp
; Called subroutines: Bin2ToDigit
;
Bin2ToBCD5:
 ldi	temp,high(10000)					; Вычисляем 10000 разряд
 mov	mulH,temp
 ldi	temp,low(10000)
 mov	mulL,temp
 rcall	Bin2ToDigit							; Вычисляем, результат в переменной temp
 sts	BCD,temp							; Если не ноль, тогда записываем результат
Bin2ToBCD4:
 ldi	temp,high(1000)						; Вычисляем 1000 разряд
 mov	mulH,temp
 ldi	temp,low(1000)
 mov	mulL,temp
 rcall	Bin2ToDigit							; Выяисляем
 sts	BCD+1,temp							; Результат в переменной temp 
Bin2ToBCD3:
 clr	mulH								; Вычисляем 100 разряд
 ldi	temp,100
 mov	mulL,temp
 rcall	Bin2ToDigit							; Вычисляем
 sts	BCD+2,temp							; Результат в переменной temp
Bin2ToBCD2:
 clr	mulH								; Вычисляем 10 разряд
 ldi	temp,10
 mov	mulL,temp
 rcall	Bin2ToDigit							; Вычисляем
 sts	BCD+4,adwL							; Единицы остаются в adiw0
 sts	BCD+3,temp							; Результат в переменной temp
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
;*                                  Прерывания                                      *
;*																					*
;*																					*
;************************************************************************************

;	Переполнение таймер/счетчик T1 (применяется для таймера перехода в дежурный режим)
aOVF1:

 inc	sleep_t

reti

;------------------------------------------------------------------------------------
;	Внешнее прерывание INT0 (применяется для выхода из спящего режима)


aINT0:
 ; Запрещаем внешнее прерывание
 ldi	temp,0b00000000
 out	GICR,temp

reti


;************************************************************************************
;*																					*
;*																					*
;*                           Массив фраз для дисплея                                *
;*																					*
;*																					*
;************************************************************************************

;------------------------------------------------------------------------------------
;	"Название чипа:"
FRAZA1:
.db 141,160,167,162,160,173,168,165,32,183,168,175,160,58,23,0

;------------------------------------------------------------------------------------
;	"CRUM чипа:"
FRAZA2:
.db 67,82,85,77,32,183,168,175,160,58,23,0

;------------------------------------------------------------------------------------
;	"Кол-во копий:"
FRAZA3:
.db 138,174,171,45,162,174,32,170,174,175,168,169,23,0

;------------------------------------------------------------------------------------
;	"Подключите чип"
FRAZA4:
.db 143,174,164,170,171,190,183,168,178,165,32,183,168,175,23,0

;------------------------------------------------------------------------------------
;	"Выберите"
FRAZA5:
.db 130,187,161,165,176,168,178,165,23,0

;------------------------------------------------------------------------------------
;	"прошивку чипа"
FRAZA6:
.db 175,176,174,184,168,162,170,179,32,183,168,175,160,23

;------------------------------------------------------------------------------------
;	"<    чип    >"
FRAZA7:
.db 60,32,32,32,32,32,183,168,175,32,32,32,32,62,23,0

;------------------------------------------------------------------------------------
;	"далее..."
FRAZA8:
.db 164,160,171,165,165,46,46,46,23,0


;------------------------------------------------------------------------------------
;	"прошить"
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
;	"Пишим чип..."
FRAZA16:
.db 32,143,168,184,168,172,32,183,168,175,46,46,46,23

;------------------------------------------------------------------------------------
;	"Меняем CRUM"
FRAZA17:
.db 32,140,165,173,191,165,172,32,67,82,85,77,23,0


;------------------------------------------------------------------------------------
;	"Проверяем чип"
FRAZA18:
.db 143,176,174,162,165,176,191,165,172,32,183,168,175,23


;------------------------------------------------------------------------------------
;	"Готово"
FRAZA19:
.db 32,32,32,32,131,174,178,174,162,174,32,32,32,32,23,0


;------------------------------------------------------------------------------------
;	"Ошибка"
FRAZA20:
.db 142,184,168,161,170,160,23,0

;------------------------------------------------------------------------------------
;	"по адресу"
FRAZA21:
.db 175,174,32,160,164,176,165,177,179,32,36,23



;************************************************************************************
;************************************************************************************
;**                                                                                **
;**                                                                                **
;**                 Формирование задержек при частоте кварца 4мГц                  **
;**                                                                                **
;**                                                                                **
;************************************************************************************
;************************************************************************************

;-------------------------------------------------------------------------
;	Задержка на 0,25мс (250мкс)

d025ms:
 ldi YL,low(248)                            ; Загрузка в YH:YL константы 497
 ldi YH,high(248)

d025_1:
 sbiw YL,1                                  ; Вычитание из содержимого YH:YL
                                            ;  единицы
 brne d025_1                                ; Если флаг Z<>0 (результат выполнения
                                            ;  предыдущей команды не равен нулю), то
									        ;  перейти на метку d05_1
ret





;-------------------------------------------------------------------------
;	Задержка на 0,5мс (500мкс)

d05ms:
 ldi	YL,low(497)							; Загрузка в YH:YL константы 497
 ldi	YH,high(497)

d05_1:
 sbiw	YL,1								; Вычитание из содержимого YH:YL единицы
 brne	d05_1								; Если флаг Z<>0 (результат выполнения
											;  предыдущей команды не равен нулю), то
											;  перейти на метку d05_1
ret





;-------------------------------------------------------------------------
;	Задержка 1 ms

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
;	Задержка 2,8 ms

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
;	Задержка 6 ms (5,7ms)

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
;	Задержка 13 ms

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
;	Задержка 20 ms

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
;	Задержка на 50мс

d50ms:
 ldi	temp,100

d50_1:
 rcall	d05ms								; Вызов подпрограммы задержки на 0,5мс
 dec	temp								; Вычитание единицы из temp
 brne	d50_1								; Если результат не равен нулю, перейти на метку d50_1
ret





;-------------------------------------------------------------------------
;	Задержка на 100мс

d100ms:
 ldi	temp,200							; Загрузка в temp константы 200

d100_1:
 rcall	d05ms								; Вызов подпрограммы задержки на 0,5мс
 dec	temp								; Вычитание единицы из temp
 brne	d100_1								; Если результат не равен нулю, перейти на метку d100_1
ret





;-------------------------------------------------------------------------
;	Задержка на 300мс

d300ms:
 ldi	XL,low(700)							; Загрузка в YH:YL константы 700
 ldi	XH,high(700)

d300_1:
 rcall	d05ms								; Вызов подпрограммы задержки на 0,5мс
 sbiw	XL,1								; Вычитание единицы из содержимого XH:XL
 brne	d300_1								; Если результат не равен нулю, перейти на метку d500_1
ret





;-------------------------------------------------------------------------
;	Задержка на 500мс

d500ms:
 ldi	ZL,low(1000)						; Загрузка в ZH:ZL константы 1000
 ldi	ZH,high(1000)

d500_1:
 rcall	d05ms								; Вызов подпрограммы задержки на 0,5мс
 sbiw	ZL,1								; Вычитание единицы из содержимого ZH:ZL
 brne	d500_1								; Если результат не равен нулю, перейти на метку d500_1
ret









;Таблица кодов знакогенератора
;------------------------------------------------------------------------------
FONT_TAB:         ; Одиарный размер шрифта
;------------------------------------------------------------------------------
											;Символ  Dec   Hex
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
.db 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 		; конец  (10)  (A)
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
.db 0x00, 0x7E, 0x09, 0x09, 0x09, 0x7E		;   А    (128)  (80)
.db 0x00, 0x7F, 0x45, 0x45, 0x45, 0x39 		;   Б    (129)  (81)
.db 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 		;   В    (130)  (82)
.db 0x00, 0x7F, 0x01, 0x01, 0x01, 0x03 		;   Г    (131)  (83)
.db 0x00, 0xC0, 0x7E, 0x41, 0x7F, 0xC0		;   Д    (132)  (84)
.db 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 		;   Е    (133)  (85)
.db 0x00, 0x77, 0x08, 0x7F, 0x08, 0x77 		;   Ж    (134)  (86)
.db 0x00, 0x41, 0x49, 0x49, 0x49, 0x36 		;   З    (135)  (87)
.db 0x00, 0x7F, 0x10, 0x08, 0x04, 0x7F 		;   И    (136)  (88)
.db 0x00, 0x7E, 0x20, 0x13, 0x08, 0x7E 		;   Й    (137)  (89)
.db 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 		;   К    (138)  (8A)
.db 0x00, 0x40, 0x3C, 0x02, 0x01, 0x7F 		;   Л    (139)  (8B)
.db 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F 		;   М    (140)  (8C)
.db 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F 		;   Н    (141)  (8D)
.db 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E 		;   О    (142)  (8E)
.db 0x00, 0x7F, 0x01, 0x01, 0x01, 0x7F 		;   П    (143)  (8F)
.db 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 		;   Р    (144)  (90)
.db 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 		;   С    (145)  (91)
.db 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 		;   Т    (146)  (92)
.db 0x00, 0x07, 0x48, 0x48, 0x48, 0x3F 		;   У    (147)  (93)
.db 0x00, 0x1C, 0x22, 0x7F, 0x22, 0x1C 		;   Ф    (148)  (94)
.db 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 		;   Х    (149)  (95)
.db 0x00, 0x3F, 0x20, 0x20, 0x3F, 0x60 		;   Ц    (150)  (96)
.db 0x00, 0x0F, 0x08, 0x08, 0x08, 0x7F 		;   Ч    (151)  (97)
.db 0x00, 0x7F, 0x40, 0x7E, 0x40, 0x7F 		;   Ш    (152)  (98)
.db 0x00, 0x3F, 0x20, 0x3C, 0x20, 0x7F 		;   Щ    (153)  (99)
.db 0x00, 0x01, 0x7F, 0x48, 0x48, 0x30 		;   Ъ    (154)  (9A)
.db 0x00, 0x7F, 0x48, 0x30, 0x00, 0x7F 		;   Ы    (155)  (9B)
.db 0x00, 0x7F, 0x48, 0x48, 0x48, 0x30		;   Ь    (156)  (9C)
.db 0x00, 0x22, 0x49, 0x49, 0x49, 0x3E 		;   Э    (157)  (9D)
.db 0x00, 0x7F, 0x08, 0x3E, 0x41, 0x3E 		;   Ю    (158)  (9E)
.db 0x00, 0x46, 0x29, 0x19, 0x09, 0x7F 		;   Я    (159)  (9F)
.db 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 		;   а    (160)  (A0)
.db 0x00, 0x3C, 0x4A, 0x4A, 0x49, 0x30 		;   б    (161)  (A1)
.db 0x00, 0x7C, 0x54, 0x54, 0x28, 0x00   	;   в    (162)  (A2)
.db 0x00, 0x7C, 0x04, 0x04, 0x0C, 0x00 		;   г    (163)  (A3)
.db 0x00, 0xC0, 0x78, 0x44, 0x7C, 0xC0 		;   д    (164)  (A4)
.db 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 		;   е    (165)  (A5)
.db 0x00, 0x6C, 0x10, 0x7C, 0x10, 0x6C 		;   ж    (166)  (A6)
.db 0x00, 0x44, 0x54, 0x54, 0x28, 0x00 		;   з    (167)  (A7)
.db 0x00, 0x7C, 0x20, 0x10, 0x08, 0x7C 		;   и    (168)  (A8)
.db 0x00, 0x7C, 0x21, 0x12, 0x09, 0x7C 		;   й    (169)  (A9)
.db 0x00, 0x7C, 0x10, 0x28, 0x44, 0x00 		;   к    (170)  (AA)
.db 0x00, 0x70, 0x08, 0x04, 0x7C, 0x00 		;   л    (171)  (AB)
.db 0x00, 0x7C, 0x08, 0x10, 0x08, 0x7C 		;   м    (172)  (AC)
.db 0x00, 0x7C, 0x10, 0x10, 0x10, 0x7C 		;   н    (173)  (AD)
.db 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 		;   о    (174)  (AE)
.db 0x00, 0x7C, 0x04, 0x04, 0x04, 0x7C 		;   п    (175)  (AF)
.db 0x00, 0x7C, 0x14, 0x14, 0x14, 0x08 		;   р    (176)  (B0)
.db 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 		;   с    (177)  (B1)
.db 0x00, 0x04, 0x04, 0x7C, 0x04, 0x04 		;   т    (178)  (B2)
.db 0x00, 0x0C, 0x50, 0x50, 0x50, 0x3C 		;   у    (179)  (B3)
.db 0x00, 0x30, 0x48, 0xFC, 0x48, 0x30 		;   ф    (180)  (B4)
.db 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 		;   х    (181)  (B5)
.db 0x00, 0x7C, 0x40, 0x40, 0x7C, 0xC0 		;   ц    (182)  (B6)
.db 0x00, 0x0C, 0x10, 0x10, 0x10, 0x7C 		;   ч    (183)  (B7)
.db 0x00, 0x7C, 0x40, 0x7C, 0x40, 0x7C 		;   ш    (184)  (B8)
.db 0x00, 0x7C, 0x40, 0x7C, 0x40, 0xFC 		;   щ    (185)  (B9)
.db 0x00, 0x04, 0x7C, 0x50, 0x50, 0x20 		;   ъ    (186)  (BA)
.db 0x00, 0x7C, 0x50, 0x20, 0x00, 0x7C 		;   ы    (187)  (BB)
.db 0x00, 0x7C, 0x50, 0x50, 0x20, 0x00 		;   ь    (188)  (BC)
.db 0x00, 0x28, 0x44, 0x54, 0x54, 0x38 		;   э    (189)  (BD)
.db 0x00, 0x7C, 0x10, 0x38, 0x44, 0x38 		;   ю    (190)  (BE)
.db 0x00, 0x08, 0x54, 0x34, 0x14, 0x7C 		;   я    (191)  (BF)

;------------------------------------------------------------------------------
FONT2_TAB:     ; Двойной размер шрифта
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
;*                                  Картинки                                  *
;*                                                                            *
;******************************************************************************

;******************************************************************************
;*                                                                            *
;*                                  Картинки                                  *
;*                                                                            *
;******************************************************************************

; Картинка-логотип программатора
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


; Картинка подключения программатора
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



;	Картинка спящего режима
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

.exit                                       ; Конец программы
