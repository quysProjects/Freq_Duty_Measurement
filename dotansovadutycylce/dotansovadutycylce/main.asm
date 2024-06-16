;
; dotansovadutycylce.asm
;
; Created: 12/9/2023 8:29:43 AM
; Author : Le Duc Quy
;

; Replace with your application code
// Cac dinh nghia
.DEF OPD1_L=R20 ;byte thap cua so nhi phan 16 bit
.DEF OPD1_H=R21 ;byte cao cua so nhi phan 16 bit
.DEF OPD2=R22
.DEF OPD3=R23
.DEF DIVD_QU_L=R24
.DEF DIVD_QU_H=R25
.DEF DIVSR_L=R4
.DEF DIVSR_H=R5
.DEF REMDR_L=R2
.DEF REMDR_H=R3
.DEF COUNT=R18
.EQU OUTPORT=PORTB					;PORTB la port xuat ra LCD
.EQU INPORT=PINB
.EQU IOSETB=DDRB
.EQU CONT=PORTC						;PORTC dieu khien LCD
.EQU CONT_DR=DDRC
.EQU CONT_IN=PINC
.EQU RS=0							;bit RS
.EQU RW=1							;bit RW
.EQU E=2							;bit E
.EQU NULL=$00 ;ma ket thuc
.EQU BCD_BUF=0X200 ;d/c luu SRAM luu so BCD (kq chuyen tu so 16 bit)
			.ORG 0
			RJMP MAIN
			.ORG 0X40
// Cau hinh cac chan
MAIN:		LDI R16,HIGH(RAMEND) ;dua stack len vung d/c cao
			OUT SPH,R16
			LDI R16,LOW(RAMEND)
			OUT SPL,R16
			LDI R16,0X07
			OUT CONT_DR,R16 ;khai bao PC0,PC1,PC2 la output
			CBI CONT,RS ;RS=PC0=0
			CBI CONT,RW ;RW=PC1=0 truy xuat ghi
			CBI CONT,E ;E=PC2=0 cam LCD
			LDI R16,0XFF
			OUT IOSETB,R16 ;khai bao outport
			CBI DDRD,6; khai bao PD6 la input
			;--------------------------------
			RCALL POWER_RESET_LCD8 ;reset cap nguon LCD 8 bit
			RCALL INIT_LCD8 ;ctc khoi dong LCD 8 bit
			;--------------------------------
			LDI ZH,HIGH(MSG1<<1) ;Z tro dau bang tra MSG1
			LDI ZL,LOW(MSG1<<1)
			RCALL MSG_DISP ;ghi MSG1 ra LCD
			LDI R17,$C0 ;con tro bat dau o dau dong 2
			RCALL CURS_POS ;xuat lenh ra LCD
			LDI ZH,HIGH(MSG2<<1) ;Z tro dau bang tra MSG2
			LDI ZL,LOW(MSG2<<1)
			RCALL MSG_DISP ;ghi MSG2 ra LCD
;---------------------------------------------------
; do tan so
;---------------------------------------------------
; Setup Timer1
START:		LDI R17,0X00 ;Timer1 mode NOR
			STS TCCR1A,R17
			LDI R17,0X00 ;Timer1 mode NOR, ban dau cho Timer dung
			STS TCCR1B,R17
			ldi r16, high(-31250) ; 
			sts TCNT1H, r16
			ldi r16, low(-31250) ;
			sts TCNT1L, r16

			ldi r24, 0x00 ; byte thap bien dem xung
			ldi r25, 0x00 ; byte cao bien dem xung

			LDI R17,0X04 ;Timer1 chay,he so chia N=256	
			STS TCCR1B,R17
; Main loop
loop:	
			sbis PIND, 6 ; cho xuat hien canh len cua PD6
			rjmp loop
loop_nes:
			sbic PIND, 6 ; cho xuat hien canh xuong cua PD6
			rjmp loop_nes

			adiw r24, 1 ; tang bien dem len 1

			in r17,TIFR1 ; doc thanh ghi co TIMER1
			SBRS R17,TOV1; skip rjmp neu co TOV1=1 bao TIMER1 tran
			RJMP loop ; neu chua tran, chua du 1 giay thi tiep tuc dem xung

			out TIFR1,R17; timer1 tran, xoa co
			LDI R17,0X00 ;Timer1 mode NOR, cho Timer d?ng
			STS TCCR1B,R17

; r25:r24 now contains the number of pulses in one second
			MOVW R20,R24 ; R20 <-- R24, R21 <-- R25
			DEC R20 ; sai so 1Hz
			RCALL BIN16_BCD5DG ;chuyen so nhi phan 16 bit sang 5 ky so BCD
			LDI R17,$88 ;con tro bat dau o dau vi tri 9 dong 2
			RCALL CURS_POS ;xuet lenh ra LCD
			RCALL NUM_DISP ;hien thi tan so cua xung
;---------------------------------------------------
; do chu ki cua xung
;---------------------------------------------------
			LDI R16,0X00
			STS TCCR1A,R16 ;Timer1 mode NOR
			LDI R16,0B01000010 ;Timer1 mode NOR,N=8,bat ngo vao canh len
			STS TCCR1B,R16
			SBI TIFR1,ICF1 ;xoa co ICF1 chuan bi do
WAIT0:		IN R17,TIFR1 ;doc thanh ghi co Timer1
			SBRS R17,ICF1 ;cho co ICF1=1 bao bat duoc xung kich khoi
			RJMP WAIT0 ;ICF1=0: cho co ICF1=1
			OUT TIFR1,R17 ;ICF1=1: xoa co ICF1
			LDS R18,ICR1L ;cat thoi diem lay mau canh len
			LDS R19,ICR1H
WAIT1:		IN R17,TIFR1 ;doc thanh ghi co Timer1
			SBRS R17,ICF1 ;cho co ICF1=1 bao bat duoc xung kich khoi
			RJMP WAIT1 ;ICF1=0: cho co ICF1=1
			OUT TIFR1,R17 ;ICF1=1: xoa co ICF1
			LDS R20,ICR1L ;cat thoi diem lay mau canh len tiep theo
			LDS R21,ICR1H
			SUB R20,R18 ;tru thoi diem sau voi thoi diem dau
			SBC R21,R19
;---------------------------------------------------
; kiem tra chu ki cua xung co lon hon 725us khong
; tuc la kiem tra thoi gian muc cao cua xung co lon hon 655us khong
;---------------------------------------------------
KIEMTRA:
			LDI R16,LOW(725) ; kiem tra xem chu ki co >= 725us khong
			LDI R17,HIGH(725)
			PUSH R20
			PUSH R21
			SUB R20,R16
			SBC R21,R17
			BRCS PERIOD2 ; neu am thi <725us, chuyen toi PERIOD2
			RJMP PERIOD1 ; so nay >725us chuyen den PERIOD1
;---------------------------------------------------
; Truong hop 1: chu ki lon hon 725us, ta phai chia chu ki cho 100 truoc
;---------------------------------------------------
PERIOD1:
			POP R21
			POP R20
			LDI OPD2,100
			RCALL DIV16_8 ; chia cho 100
			MOVW R4,R20; R4 <-- R20, R5 <-- R21, nap chu ki (da chia 100) vao so chia cua DIV_16
			LDI R22,1; danh dau truong hop 1
			RJMP NEXTDUTY
;---------------------------------------------------
; Truong hop 2: chu ki nho hon 725us, ta khong can chia
;---------------------------------------------------
PERIOD2:	
			POP R21
			POP R20
			MOVW R4,R20 ; khong chia 100, nap chu ki vao so chia cua DIV_16
			LDI R22,2; danh dau truong hop 2
;---------------------------------------------------
; do thoi gian muc cao cua xung
;---------------------------------------------------
NEXTDUTY:	LDI R16,0X00
			STS TCCR1A,R16 ;Timer1 mode NOR
			LDI R16,0B01000010 ;Timer1 mode NOR,N=8,bat ngo vao canh len
			STS TCCR1B,R16
DUTYCYCLE:	SBI TIFR1,ICF1 ;xoa co ICF1 chuan bi do
	WAIT2:  IN R17,TIFR1 ;doc thanh ghi co Timer1
			SBRS R17,ICF1 ;cho co ICF1=1 bao bat duoc xung kich khoi
			RJMP WAIT2 ;ICF1=0: cho co ICF1=1
			OUT TIFR1,R17 ;ICF1=1: xoa co ICF1
			LDS R18,ICR1L ;cat thoi diem lay mau canh len
			LDS R19,ICR1H
			LDI R16,0B00000010 ;Timer1 mode NOR,N=8,bat ngo vao canh xuong
			STS TCCR1B,R16
	WAIT3:	IN R17,TIFR1 ;doc thanh ghi co Timer1
			SBRS R17,ICF1 ;cho co ICF1=1 bao bat duoc xung kich khoi
			RJMP WAIT3 ;ICF1=0: cho co ICF1=1
			OUT TIFR1,R17 ;ICF1=1: xoa co ICF1
			LDS R20,ICR1L ;cat thoi diem lay mau canh xuong
			LDS R21,ICR1H
			LDI R16,0B01000010 ;Timer1 mode NOR,N=8,bat ngo vao canh len (xung tiep theo)
			STS TCCR1B,R16
			SUB R20,R18 ;tru thoi diem sau voi thoi diem dau
			SBC R21,R19  ; chu ky lam viec
			CPI R22,1; kiem tra truong hop 1
			BREQ DUTY1 ; truong hop 1
			RJMP DUTY2 ; truong hop 2
;---------------------------------------------------
; Truong hop 1: chu ki tong lon hon 725us (T_high > 655us)
; So bi chia la thoi gian muc cao
; So chia la chu ki tong (da bi chia 100)
; Ta chia thoi gian muc cao cho chu ki tong (da bi chia 100)
;---------------------------------------------------
DUTY1:		
			MOV DIVD_QU_H,R21 ; nap so bi chia
			MOV DIVD_QU_L,R20 ; nap so bi chia
			RCALL DIV_16 ; chia chu ki muc cao cho chu ki tong (da bi chia 100)
			MOV R20,DIVD_QU_L
			MOV R21,DIVD_QU_H
			RJMP DONE2
;---------------------------------------------------
; Truong hop 2: chu ki tong nho hon 725us (T_high < 655us)
; So bi chia la thoi gian muc cao (nhan voi 100)
; So chia la chu ki tong 
; Ta chia thoi gian muc cao (da nhan voi 100) cho chu ki tong 
;---------------------------------------------------
DUTY2:
			LDI R16,100 ; nhan chu ky muc cao voi 100 de chia lay %
			MUL R20,R16 ; nhan byte thap voi 100
			MOVW R18,R0 ; chuyen tich vao R18,R19
			MUL R21,R16; nhan byte cao voi 100
			ADD R19,R0 ; cong tich byte cao vao R19
			MOVW R20,R18; chuyen du lieu tu R18,R19 --> R20,R21
			MOV DIVD_QU_H,R21 ; nap so bi chia 
			MOV DIVD_QU_L,R20 ; nap so bi chia
			RCALL DIV_16 ; chia chu ki muc cao (da nhan 100) cho chu ki tong
			MOV R20,DIVD_QU_L ; tra lai thuong so
			MOV R21,DIVD_QU_H ; tra lai thuong so
			RJMP DONE2

DONE2:		RCALL BIN16_BCD5DG ;chuyen so nhi phan 16 bit sang 5 ky so BCD
			LDI R17,$C8 ;con tro bat dau o dau vi tri 9 dong 2
			RCALL CURS_POS ;xuat lenh ra LCD
			RCALL NUM_DISP ;hien thi chu ki nhiem vu
			RJMP START
;---------------------------------------
;DIV16_8 chia so nhi phan 16 bit OPD1 cho 8 bit OPD2 (Xem l?u ?? ph?p chia ? H?nh 7.19 ? Gi?o tr?nh)
;Input: OPD1_H,OPD1_L= SBC(GPR16-31)
; OPD2=SC(GPR0-31)
;Output:OPD1_H,OPD1_L=thuong so
; OPD3=DS(GPR0-31)
;Su dung COUNT(GPR16-31)
;---------------------------------------
DIV16_8:	LDI COUNT,16 ;COUNT=??m 16
			CLR OPD3 ;x?a d? s?
	SH_NXT: CLC ;C=0=bit th??ng s?
			LSL OPD1_L ;d?ch tr?i SBC L,bit0=C=th??ng s?
			ROL OPD1_H ;quay tr?i SBC H,C=bit7
			ROL OPD3 ;d?ch bit7 SBC H v?o d? s?
			BRCS OV_C ;tr?n bit C=1,chia ???c
			SUB OPD3,OPD2 ;tr? d? s? v?i s? chia
			BRCC GT_TH ;C=0 chia ???c
			ADD OPD3,OPD2 ;C=1 kh?ng chia ???c,kh?ng tr?
			RJMP NEXT
	OV_C:	SUB OPD3,OPD2 ;tr? d? s? v?i s? chia
	GT_TH:	SBR OPD1_L,1 ;chia ???c,th??ng s?=1
	NEXT:	DEC COUNT ;??m s? l?n d?ch SBC
			BRNE SH_NXT ;ch?a ?? ti?p t?c d?ch bit
			RET
;----------------------------------------------------------------------
; DIV_16 chia 2 so nhi phan 16 bit DIVD_QU / DIVSR
; Input: DIVD_QU_H,DIVD_QU_L = so bi chia(SBC)R24:R25
;	DIVSR_H,DIVSR_L = so chia R4,R5
; Output: DIVD_QU_H,DIVD_QU_L = thuong so R24:R25
;		REMDR_H,REMDR_L = du so R2:R3
; Su dung COUNT
;----------------------------------------------------------------------
DIV_16:		LDI R16,16
			MOV COUNT,R16
			CLR REMDR_L
			CLR REMDR_H
	AGAIN:	CLC
			LSL DIVD_QU_L
			ROL DIVD_QU_H
			ROL REMDR_L
			ROL REMDR_H
			BRCS GTN16
			SUB REMDR_L,DIVSR_L
			SBC REMDR_H,DIVSR_H
			BRCC GT_TH16
			ADD REMDR_L,DIVSR_L
			ADC REMDR_H,DIVSR_H
			RJMP NEXT16
	GTN16:	SUB REMDR_L,DIVSR_L
			SBC REMDR_H,DIVSR_H;
GT_TH16:	SBR DIVD_QU_L,1
	NEXT16:	DEC COUNT
			BRNE AGAIN
			RET
;----------------------------------------------------------------------
// Xuat gia tri len LCD
;----------------------------------------------------------------------
;----------------------------------------------------------------------
;khoi tao LCD gom:
;reset cap nguon
;cau hinh LCD hoat dong che do 8 bit
;khoi tao cac gia tri
;----------------------------------------------------------------------
;-----------------------------------------------------------------
;Cac lenh reset cap nguon LCD 8 bit
;Cho han 15ms
;Ghi ma lenh 30H lan 1, cho it nhat 4.1ms
;Ghi ma lenh 30H lan 2, cho it nhat 100us
;Ghi ma lenh 30H lan 3, cho it nhat 100us
;-----------------------------------------------------------------
POWER_RESET_LCD8:
		LDI R16,200 ;delay 20ms
		RCALL DELAY_US ;ctc delay 100usxR16
		;Ghi m? lenh 30H lan 1, cho 4.2ms
		CBI CONT,RS ;RS=0 ghi lenh
		LDI R17,$30 ;m? lenh=$30 lan 1,RS=RW=E=0
		RCALL OUT_LCD ;ctc ghi ra LCD
		LDI R16,42 ;delay 4.2ms
		RCALL DELAY_US
		;Ghi m? lenh 30H lan 2, cho 200us
		CBI CONT,RS ;RS=0 ghi lenh
		LDI R17,$30 ;m? lenh=$30 lan 2
		RCALL OUT_LCD
		LDI R16,2 ;delay 200?s
		RCALL DELAY_US
		;Ghi m? lenh 30H lan 3, cho 200us
		CBI CONT,RS ;RS=0 ghi lenh
		LDI R17,$30
		RCALL OUT_LCD
		LDI R16,2 ;delay 200us
		RCALL DELAY_US
		RET
;-----------------------------------------------------------------
;INIT_LCD8 khoi dong LCD ghi 4 byte m? lenh
;Function set: 0x38: 8 bit, 2 d?ng font 5x8
;Clear display: 0x01: x?a m?n h?nh
;Display on/off control: 0x0C: m?n h?nh on, con tro off
;Entry mode set: 0x06: dich phai con tro, dia chi DDRAM tang 1 khi ghi data
;----------------------------------------------------------------
INIT_LCD8:	
		CBI CONT,RS ;RS=0 ghi lenh
		LDI R17,0x38 ;che do giao tiep 8 bit, 2 d?ng font 5x8
		RCALL OUT_LCD
		LDI R16,1 ;cho 100us
		RCALL DELAY_US
		CBI CONT,RS ;RS=0 ghi lenh
		LDI R17,0x01 ;x?a m?n h?nh
		RCALL OUT_LCD
		LDI R16,20 ;cho 2ms sau lenh Clear display
		RCALL DELAY_US
		CBI CONT,RS ;RS=0 ghi lenh
		LDI R17,0x0C ;m?n h?nh on, con tro off
		RCALL OUT_LCD
		LDI R16,1 ;cho 100us
		RCALL DELAY_US
		CBI CONT,RS ;RS=0 ghi lenh
		LDI R17,0x06 ;dich phai con tro, dia chi DDRAM tang 1 khi ghi data
		RCALL OUT_LCD
		LDI R16,1 ;cho 100us
		RCALL DELAY_US
		RET
;----------------------------------------------------------
;CURS_POS ??t con tr? t?i v? tr? c? ??a ch? trong R17
;Input: R17=$80 -$8F d?ng 1,$C0-$CF d?ng 2
;R17= ??a ch? v? tr? con tr?
;S? d?ng R16,ctc DEAY_US,OUT_LCD
;----------------------------------------------------------
CURS_POS: 
		LDI R16,1 ;ch? 100?s
		RCALL DELAY_US
		CBI CONT,RS ;RS=0 ghi l?nh
		RCALL OUT_LCD
		RET
;--------------------------------------------------
;OUT_LCD ghi m? lenh/data ra LCD
;Input: R17 chua m? lenh/data
;--------------------------------------------------
OUT_LCD:	
		OUT OUTPORT,R17 ;1MC,ghi lenh/data ra LCD
		SBI CONT,E ;2MC,xuat xung cho ph?p LCD
		CBI CONT,E ;2MC
		RET
;-----------------------------------------------------------------
;MSG_DISP hi?n th? chu?i k? t? k?t th?c b?ng m? NULL ??t trong Flash ROM
;Input: Z ch?a ??a ch? ??u chu?i k? t?
;Output: hi?n th? chu?i k? t? ra LCD t?i v? tr? con tr? hi?n h?nh
;S? d?ng R16,R17,ctc DELAY_US,OUT_LCD
;------------------------------------------------------------------
MSG_DISP:	LPM R17,Z+ ;l?y m? ASCII k? t? t? Flash ROM
			CPI R17,NULL ;ki?m tra k? t? k?t th?c
			BREQ EXIT_MSG ;k? t? NULL tho?t
			LDI R16,1 ;ch? 100?s
			RCALL DELAY_US
			SBI CONT,RS ;RS=1 ghi data hi?n th? LCD
			RCALL OUT_LCD ;ghi m? ASCII k? t? ra LCD
			RJMP MSG_DISP ;ti?p t?c hi?n th? k? t?
EXIT_MSG:	RET
;-------------------------------------------------------
;DELAY_US tao thoi gian tre =R16x100us(Fosc=8MHz, CKDIV8 = 1)
;Input:R16 he so nh?n thoi gian 
;-------------------------------------------------------
DELAY_US: 
		MOV R15,R16 ;1MC nap data cho R15
		LDI R16,200 ;1MC su dung R16
		L1: MOV R14,R16 ;1MC nap data cho R14
		L2: DEC R14 ;1MC
		NOP ;1MC
		BRNE L2 ;2/1MC
		DEC R15 ;1MC
		BRNE L1 ;2/1MC
		RET ;4MC
;---------------------------------------------------------
;BIN16_BCD5DG chuyen doi so nhi phan 16 bit sang so BCD 5 digit
;Inputs: OPD1_H=R21:OPD1_L=R20 chua so nhi phan 16 bit
;Outputs: BCD_BUF:BCD_BUF+4:dia chi SRAM chua 5 digit BCD tu cao den thap
;Su dung R17,COUNT,X,ctc DIV16_8
;---------------------------------------------------------
BIN16_BCD5DG:
			LDI XH,HIGH(BCD_BUF);X tr? ??a ch? ??u buffer BCD
			LDI XL,LOW(BCD_BUF)
			LDI COUNT,5 ;??m s? byte b? nh?
			LDI R17,0X00 ;n?p gi? tr? 0
LOOP_CL:	ST X+,R17 ;x?a buffer b? nh?
			DEC COUNT ;??m ?? 5 byte
			BRNE LOOP_CL
			LDI OPD2,10 ;n?p s? chia (SC)
DIV_NXT:	RCALL DIV16_8 ;chia s? nh? ph?n 16 bit cho s? nh? ph?n 8 bit
			ST -X,OPD3 ;c?t s? d? v?o buffer
			CPI OPD1_L,0 ;th??ng s?=0?
			BRNE DIV_NXT ;kh?c 0 chia ti?p
			RET
;------------------------------------------------
;NUM_DISP hi?n th? 5 k? t? ??a ch? ??u BCD_BUF trong SRAM
;(BCD_BUF)=digit cao nh?t,(BCD_BUF+4)=digit th?p nh?t
;S? d?ng R16,R17,COUNT,X,ctc DELAY_US,ctc OUT_LCD
;------------------------------------------------
NUM_DISP:	LDI COUNT,5 ;hi?n th? 5 k? t?
			LDI XH,HIGH(BCD_BUF) ;X tr? ??a ch? ??u buffer
			LDI XL,LOW(BCD_BUF)
DISP_NXT:	LD R17,X+ ;l?y s? BCD t? buffer,t?ng ??a ch? buffer k? ti?p
			LDI R16,0X30 ;chuy?n sang m? ASCII
			ADD R17,R16
			LDI R16,1 ;ch? 100?s
			RCALL DELAY_US
			SBI CONT,RS ;RS=1 ghi data hi?n th? LCD
			RCALL OUT_LCD ;ghi k? t? ra LCD
			DEC COUNT ;hi?n th? ?? 5 k? t?
			BRNE DISP_NXT
			RET
;------------------------------------------------
			.ORG 0X0200
;------------------------------------------------
MSG1: .DB "f(Hz)  : ",$00
MSG2: .DB "CKNV(%): ",$00;$E4 m? k? t? ?
