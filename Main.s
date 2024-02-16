	/*
	* Main.s
	*
	* Created on: Aug 24, 2022
	* Author: rozman
	*/

	.syntax unified
	.cpu cortex-m7
	.thumb


	///////////////////////////////////////////////////////////////////////////////
	// Definitions
	///////////////////////////////////////////////////////////////////////////////
	// Definitions section. Define all the registers and
	// constants here for code readability.

	// Constants

	// Register Addresses
	.equ DWT_CYCCNT, 0xE0001004 // DWT_CYCCNT reg (RM, pp.3211)

	// RCC base address is 0x58024400
	// AHB4ENR register offset is 0xE0
	.equ RCC_AHB4ENR, 0x580244E0 // RCC AHB4 peripheral clock reg
	.equ GPIOJ_BASE, 0x58022400 // GPIOJ base address)
	.equ GPIOx_MODER, 0x00 // GPIOx port mode register
	.equ GPIOx_ODR, 0x14 // GPIOx output data register
	.equ GPIOx_BSRR, 0x18 // GPIOx port set/reset register

	// Values for BSSR register - pin 2: LED is on, when GPIO is off
	.equ LEDs_OFF, 0x00000004 // Setting pin to 1 -> LED is off
	.equ LEDs_ON, 0x00040000 // Setting pin to 0 -> LED is on

	.equ msDelay, 640


	// SysTick Timer definitions
	.equ SCS_BASE,0xe000e000
	.equ SCS_SYST_CSR,0x10// Control/Status register
	.equ SCS_SYST_RVR,0x14// Value to countdown from
	.equ SCS_SYST_CVR,0x18// Current value
	.equ SYSTICK_RELOAD_1MS, 63999 //1 msec at 64MHz ...


	// AHB4ENR register offset is 0xE0
	.equ RCC_AHB4ENR, 0x580244E0 // RCC AHB4 peripheral clock reg

	// RCC base address is 0x58024400
	.equ RCC_BASE, 0x58024400 // RCC base reg

	// APB1LENR register offset is 0xE8
	.equ RCC_APB1LENR, 0xE8 // RCC APB1LENR peripheral clock reg

	// GPIOB base address is 0x58020400
	.equ GPIOB_BASE, 0x58020400 // GPIOB base address)

	// AFRH register offset is 0x24
	.equ GPIOx_AFRH, 0x24 //
	.equ GPIO_AFRH_VALUE, 0x7700 // AF7 on PB10,11

	// USART3 base address is 0x40004800
	.equ USART3_BASE, 0x40004800 // USART3 base address)

	// CRx registers
	// CR1 register
	.equ USART_CR1, 0x00 // CR1 register
	.equ USART_CR1_VAL, 0b1101 // CR1 register value

	// CRx registers
	.equ USART_CR2, 0x04 // CR2 register
	.equ USART_CR3, 0x08 // CR3 register


	// BRR register
	.equ USART_BRR, 0x0C // BRR register
	.equ USART_BRR_VAL,556 // BRR register 64 000 000 / 115 200 = 555.55 = 556
	// ISR register
	.equ USART_ISR, 0x1c // ISR register

	// Data registers
	.equ USART_RDR, 0x24 // Receive Data Register
	.equ USART_TDR, 0x28 // Transmit Data Register


	//blue user button
	.equ USER_BUTTON_PIN, 13	// user button pin
	.equ GPIOC_BASE, 0x58020800 // GPIOC base address
	.equ GPIOx_IDR, 0x10 		// GPIOx input register

// Start of data section
.data

.align

PIKA:		.ascii "."
A:			.ascii "A"

MORSE:		.ascii ".-"
			.byte 0,0,0,0	//A
			.ascii "-..."
			.byte 0,0		//B
			.ascii "-.-."
			.byte 0,0		//C
			.ascii "-.."
			.byte 0,0,0		//D
			.ascii "."
			.byte 0,0,0,0,0	//E
			.ascii "..-."
			.byte 0,0		//F
			.ascii "--."
			.byte 0,0,0		//G
			.ascii "...."
			.byte 0,0		//H
			.ascii ".."
			.byte 0,0,0,0	//I
			.ascii ".---"
			.byte 0,0		//J
			.ascii "-.-"
			.byte 0,0,0		//K
			.ascii ".-.."
			.byte 0,0		//L
			.ascii "--"
			.byte 0,0,0,0	//M
			.ascii "-."
			.byte 0,0,0,0	//N
			.ascii "---"
			.byte 0,0,0		//O
			.ascii ".--."
			.byte 0,0		//P
			.ascii "--.-"
			.byte 0,0		//Q
			.ascii ".-."
			.byte 0,0,0		//R
			.ascii "..."
			.byte 0,0,0		//S
			.ascii "-"
			.byte 0,0,0,0,0	//T
			.ascii "..-"
			.byte 0,0,0		//U
			.ascii "...-"
			.byte 0,0		//V
			.ascii ".--"
			.byte 0,0,0		//W
			.ascii "-..-"
			.byte 0,0		//X
			.ascii "-.--"
			.byte 0,0		//Y
			.ascii "--.."
			.byte 0,0		//Z


NIZ:		.byte 0,0,0,0,0,0,0,0,0	// 9 praznih rezerviranih za input (ena dodatna, da se funkcija ustavi, ko naleti na 0)

// Start of text section
.text

.type main, %function
.global main

.align
main:
   	bl INIT_IO    	// Priprava za kontrolo LED diode
    bl INIT_TC    	// Priprava SysTick časovnika
    bl INIT_USER_BUTTON
    bl INIT_USART3 	// Priprava USART3 naprave


    bl LED_OFF
    mov r0, #500
    bl DELAYTC

	mov r1, #9		//števec za dolžino besede
	ldr r2, =NIZ 	//Kazalec na naslov kamor shranjuje črke

//zanka pridobiva znake iz računalnika in jih hkrati pošilja nazaj, da uporabnik vidi kar je vpisal
loop:
	subs r1, r1, #1
	beq morse_blink

	bl RECV_UART

	cmp r0, #13
	beq morse_blink

	bl SEND_UART

	strb r0, [r2]
	add r2, r2, #1
 	mov r0, #50
    bl DELAYTC
	b loop

_end: b _end


// inicializacija user gumba
INIT_USER_BUTTON:
	push {r0, r1, r2, lr}
    ldr r1, =RCC_AHB4ENR
    ldr r0, [r1]
    orr r0, r0, #(1 << 2) 		// Set bit 2 to enable GPIOC clock
    str r0, [r1]

    ldr r1, =GPIOC_BASE
    ldr r0, [r1, #GPIOx_MODER]
    mov r2, #0xF3FFFFFF			// Maska za nastavitev na input
	and r0, r0, r2
    str r0, [r1, #GPIOx_MODER]	// Nastavi na input v MODER

   	mov r2, #0x10000000
	str r2, [r1,#GPIOx_BSRR] 	// nastavi IDR register na 0

    pop {r0, r1, r2, pc}

// funkcija, ki preveri, če je bil user gumb pritisnjen
CHECK_USER_BUTTON:
	push {r0, r1, lr}
    ldr r1, =GPIOC_BASE

    ldr r0, [r1, #GPIOx_IDR]
    and r0, r0, #(1 << USER_BUTTON_PIN)
    cmp r0, #(1 << USER_BUTTON_PIN) // Naloži IDR register in pogled 13-i pin, če je na 1
    beq morse_blink

    pop {r0, r1, pc}






//Ko je uporabnik pritisnil enter ali pa vnesel 8 znakov, ta podprogram resetira r0 števec na prvo črko ter pokliče XWORD
morse_blink:
	mov r0, #1000
	bl DELAYTC

    ldr r0, =NIZ
	bl XWORD
	b _end

//Pridobivanje znakov
RECV_UART:
	push {r1, r2, lr}
	ldr r1, =USART3_BASE
RECV_LP:
	bl CHECK_USER_BUTTON
	ldr r2, [r1, #USART_ISR]
	tst r2, #(1 << 5) 			// RXNE flag
	beq RECV_LP
	ldr r0, [r1, #USART_RDR]
	pop {r1, r2, pc}

//Vračanje znakov
SEND_UART:
	push {r1, r2, lr}
	ldr r1, =USART3_BASE
SEND_LP:
	ldr r2, [r1, #USART_ISR]
	tst r2, #(1 << 7) // TXE flag
	beq SEND_LP
	str r0, [r1, #USART_TDR]
	pop {r1, r2, pc}

//Iz naslova v pomnilniku bere črko za črko in za vsako kliče GETMCODE ter XMCODE
XWORD:
	push {r1, lr}
loop_word:
	ldrb r1, [r0]
	cmp r1, #0
	beq exit_XWORD

	push {r0}
	ldrb r0, [r0]
	bl GETMCODE
	bl XMCODE
	pop {r0}

	add r0, r0, #1
	b loop_word

exit_XWORD:
	pop {r1, pc}



//Podano črko spremenit v naslov, kjer se nahaja njena morse koda
GETMCODE:
	push {r1, r2, lr}
	ldr r1, =A
	ldrb r1, [r1]
	sub r1, r0, r1	//Naložim črko A ter jo odštejem podani črki

	mov r2, #6
	mul r1, r1, r2	//V r1 naložim odmik odmik črke v abecedi

	ldr r0, =MORSE
	add r0, r0, r1	//naslovu MORSE tabele prištejem odmik, da pridem do podane črke

	pop {r1, r2, pc}



//zažmiga eno črko
XMCODE:
	push {r0, r1, r2, lr}

loop_letter:
	ldrb r1, [r0]
	cmp r1, #0
	beq exit_XMCODE

	push {r0}
	ldrb r0, [r0]
	bl XMCHAR
	pop {r0}

	add r0, r0, #1
	b loop_letter

exit_XMCODE:
	pop {r0, r1, r2, pc}



//zažmiga en znak
XMCHAR:
	push {r0,r1, r2, lr}
	ldr r1, =PIKA
	ldrb r2, [r1]
	cmp r0, '.'
	it eq
	moveq 	r0, #150
	it ne
	movne 	r0, #300

	bl LED_ON
	bl DELAYTC
	bl LED_OFF

	mov	r0, #150	//pause for 150
	bl DELAYTC

	pop {r0, r1, r2, pc}


INIT_USART3:
	push {r0, r1, r2, lr}
	// Enable USART3 Peripheral Clock (bit 18 in APB1LENR register)
	ldr r1, =RCC_BASE 				// Load peripheral clock reg base address to r1
	ldr r0, [r1,#RCC_APB1LENR] 		// Read its content to r0
	orr r0, r0, #(1<<18) 			// Set bit 18 to enable USART3 clock
	str r0, [r1,#RCC_APB1LENR] 		// Store result in peripheral clock register
	// Enable GPIOB Peripheral Clock (bit 1 in AHB4ENR register)
	ldr r1, = RCC_AHB4ENR 			// Load peripheral clock reg address to r6
	ldr r0, [r1] 					// Read its content to r5
	orr r0, r0, #0b10 				// Set bit 1 to enable GPIOB clock
	str r0, [r1] 					// Store result in peripheral clock register
	ldr r1, =GPIOB_BASE 			// Load GPIOB BASE address to r1
	// Make GPIOB Pins 10,11 as AF (bits 20:23 in MODER register)
	ldr r0, [r1,#GPIOx_MODER] 		// Read GPIO_MODER content to r0
	ldr r2, =0xFF0FFFFF 			// Clear mask
	and r0, r0, r2 					// Clear bits
	orr r0, #0x00A00000 			// Write 10 to bits
	str r0, [r1,#GPIOx_MODER]		// Store result in GPIO MODER register
	// Make GPIOB Pins 10,11 as AF7 (bits 8:15 in AFRH register)
	ldr r0, [r1,#GPIOx_AFRH] 		// Read GPIOB AFRH content to r0
	ldr r2, =0xFFFF00FF 			// Clear mask
	and r0, r0, r2 					// Clear bits
	orr r0, r0, #GPIO_AFRH_VALUE
	str r0, [r1,#GPIOx_AFRH] 		// Store result in GPIOB AFRH register
	ldr r1, =USART3_BASE 			// Load USART3 BASE address to r1
	// Disable USART3
	mov r0, #0
	str r0, [r1,#USART_CR1] 		// Store result
	// Set USART3 BaudRate
	ldr r0, =USART_BRR_VAL
	str r0, [r1,#USART_BRR] 		// Store result
	// Start USART3
	mov r0, #USART_CR1_VAL
	str r0, [r1,#USART_CR1] 		// Store result
	pop {r0, r1, r2, pc}


INIT_TC:
	push {r0, r1, lr}
	ldr r1, =SCS_BASE
	ldr r0, =SYSTICK_RELOAD_1MS
	str r0, [r1, #SCS_SYST_RVR]
	mov r0, #0
	str r0, [r1, #SCS_SYST_CVR]
	mov r0, #0b101
	str r0, [r1, #SCS_SYST_CSR]
	pop {r0, r1, pc}



// Delay with internal timer based loop approx.
//r0 x ms
DELAYTC:
	push {r1, r2, lr}
	ldr r1, =SCS_BASE

LOOPTC: ldr r2, [r1, #SCS_SYST_CSR]
	tst r2, #0x10000 // COUNT_FLAG=1?
	beq LOOPTC
	subs r0, r0, #1
	bne LOOPTC
	pop {r1, r2, pc}



INIT_IO:
	push {lr}

	ldr r6, = RCC_AHB4ENR 		// Load peripheral clock reg address to r6
	ldr r5, [r6] 				// Read its content to r5
	orr r5, #0x00000200 		// Set bit 8 to enable GPIOJ clock
	str r5, [r6] 				// Store result in peripheral clock register

	//2. MODER (Mode Register): 01: General purpose output mode
	// Make GPIOJ Pin2 as output pin (bits 4:5 in MODER register)
	ldr r6, =GPIOJ_BASE 		// Load GPIOD BASE address to r6
	ldr r5, [r6,#GPIOx_MODER] 	// Read GPIOD_MODER content to r5
	and r5, #0xFFFFFFCF 		// Clear bits 4-5 for P2
	orr r5, #0x00000010 		// Write 01 to bits 4-5 for P2
	str r5, [r6] 				// Store result in GPIO MODER register

	pop {pc}


LED_ON:
	push {r5, r6, lr}
	// Set GPIOx Pins to 0 (through BSSR register)
	ldr r6, =GPIOJ_BASE 		// Load GPIOJ BASE address to r6
	mov r5, #LEDs_ON
	str r5, [r6,#GPIOx_BSRR] 	// Write to BSRR register
	pop {r5, r6, pc}

LED_OFF:
	push {r5, r6, lr}
	// Set GPIOx Pins to 1 (through BSSR register)
	ldr r6, =GPIOJ_BASE 		// Load GPIOJ BASE address to r6
	mov r5, #LEDs_OFF
	str r5, [r6,#GPIOx_BSRR] 	// Write to BSRR register
	pop {r5, r6, pc}
