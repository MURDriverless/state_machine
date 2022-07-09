/*
 * Copyright (c) 2014 - 2016, Freescale Semiconductor, Inc.
 * Copyright (c) 2016 - 2018, NXP.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * main.c sets up ports, interrupts and updates the state of the autonomous car, i.e.,
 * it governs the overall behaviour of the car under autonomous operation.
 *
 * Author:	Vinamra Goswami
 * Last updated: 02/06/2022
 *
 * Autonomous Safety Engineer
 * Melbourne University Racing (MUR) Motorsports
 *
 * University of Melbourne
 */



/* Header files */
#include "setup.h"
#include "LPUART.h"


/* Array of state function pointers */
enum state_codes (* AS_states[])(void) = {AS_off_state, AS_ready_state, AS_driving_state, AS_emergency_state, AS_finished_state};

/* Variable holding code for next state */
enum state_codes func_output;

/* Function pointer: pointing to different state functions */
enum state_codes (* state_func)(void);

/* LPIT0 channel 0 timeout counter */
int lpit0_ch0_flag_counter = 0;

/* Counter to keep track of length of command */
int counter = 0;

/* Variable to store the time value at which connection with Jetson is lost */
int Jetson_timer = 0;


/* Function defines interrupt request on Low-Power Interrupt Timer (LPIT) channel 0 */
void NVIC_init_IRQs (void) {
	
	S32_NVIC->ICPR[1] = 1 << (48 % 32);  /* IRQ48-LPIT0 ch0: clr any pending IRQ	*/
	S32_NVIC->ISER[1] = 1 << (48 % 32);  /* IRQ48-LPIT0 ch0: enable IRQ 			*/
	S32_NVIC->IP[48] = 0xA;              /* IRQ48-LPIT0 ch0: priority 10 of 0-15	*/
}



/* Function sets up LPIT module (channel 0) for interrupt defined in NVIC_init_IRQs() */
void LPIT0_init (void) {

	/*!
	 * LPIT Clocking:
	 * ==============================
	 */
  PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);    /* Clock Src = 6 (SPLL2_DIV2_CLK)*/
  PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clk to LPIT0 regs 		*/

  /*!
   * LPIT Initialization:
   */
  LPIT0->MCR |= LPIT_MCR_M_CEN_MASK;  /* DBG_EN-0: Timer chans stop in Debug mode */
                              	  	  /* DOZE_EN=0: Timer chans are stopped in DOZE mode */
                              	  	  /* SW_RST=0: SW reset does not reset timer chans, regs */
                              	  	  /* M_CEN=1: enable module clk (allows writing other LPIT0 regs) */
  LPIT0->MIER = LPIT_MIER_TIE0_MASK;  /* TIE0=1: Timer Interrupt Enabled for Chan 0 */

  LPIT0->TMR[0].TVAL = INTERRUPT_CLKS;      /* Chan 0 Timeout period: 40M clocks corresponds to 1s */

  LPIT0->TMR[0].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK;
  	  	  	  	  	  	  	  /* T_EN=1: Timer channel is enabled */
                              /* CHAIN=0: channel chaining is disabled */
                              /* MODE=0: 32 periodic counter mode */
                              /* TSOT=0: Timer decrements immediately based on restart */
                              /* TSOI=0: Timer does not stop after timeout */
                              /* TROT=0 Timer will not reload on trigger */
                              /* TRG_SRC=0: External trigger source */
                              /* TRG_SEL=0: Timer chan 0 trigger source is selected*/

}



/* Function defines initial state of car and flags for pre-driving checks */
void state_init(void) {

	/* Initial state of car */
	AS_state.previous_state = INITIAL_STATE;
	AS_state.present_state = INITIAL_STATE;
	state_func = AS_states[INITIAL_STATE];

	R2D = off;
	go = false;
	UART_flag = false;
	ACK_sent = false;
	Jetson_alive = false;
	mission_complete = false;

	/* Initialise ASSI */
	ASSI(AS_state.present_state, lpit0_ch0_flag_counter);
}



/* Function initialises the ports and GPIO pins (for pin definitions refer to setup.h) */
void PORT_init (void) {

  PCC->PCCn[PCC_PORTA_INDEX ]|=PCC_PCCn_CGC_MASK; /* Enable clock for PORTA */
  PCC->PCCn[PCC_PORTB_INDEX ]|=PCC_PCCn_CGC_MASK; /* Enable clock for PORTB */
  PCC->PCCn[PCC_PORTC_INDEX ]|=PCC_PCCn_CGC_MASK; /* Enable clock for PORTC */
  PCC->PCCn[PCC_PORTD_INDEX ]|=PCC_PCCn_CGC_MASK; /* Enable clock for PORTD */
  PCC->PCCn[PCC_PORTE_INDEX ]|=PCC_PCCn_CGC_MASK; /* Enable clock for PORTE */
  

  /* Configuring port functionality */
  PORTA->PCR[PTA21]= PORT_PCR_MUX(1);	/* Port A21: MUX = IO_D0  */
  PORTA->PCR[PTA22]= PORT_PCR_MUX(1);	/* Port A22: MUX = IO_D0  */
  PORTA->PCR[PTA23]= PORT_PCR_MUX(1);	/* Port A23: MUX = IO_D0  */
  PORTA->PCR[PTA24]= PORT_PCR_MUX(1);	/* Port A24: MUX = IO_D0  */

  PORTB->PCR[PTB27]= PORT_PCR_MUX(1);	/* Port B27: MUX = IO_D0  */
  PORTB->PCR[PTB28]= PORT_PCR_MUX(1);	/* Port B28: MUX = IO_D0  */
  PORTB->PCR[PTB29]= PORT_PCR_MUX(1);	/* Port B29: MUX = IO_D0  */

  PORTC->PCR[PTC6]|=PORT_PCR_MUX(2);	/* Port C6: MUX = ALT2, UART1 TX */
  PORTC->PCR[PTC7]|=PORT_PCR_MUX(2);	/* Port C7: MUX = ALT2, UART1 RX */

  PORTD->PCR[PTD0]= PORT_PCR_MUX(1);	/* Port D0: MUX = IO_D0  */
  PORTD->PCR[PTD1]= PORT_PCR_MUX(1);	/* Port D1: MUX = IO_D0  */
  PORTD->PCR[PTD2]= PORT_PCR_MUX(1);	/* Port D2: MUX = IO_D0  */
  PORTD->PCR[PTD3]= PORT_PCR_MUX(1);	/* Port D3: MUX = IO_D0  */
  PORTD->PCR[PTD13]= PORT_PCR_MUX(1);	/* Port D13: MUX = IO_D0  */

  PORTE->PCR[PTE21]= PORT_PCR_MUX(1);	/* Port E21: MUX = IO_D0  */
  PORTE->PCR[PTE22]= PORT_PCR_MUX(1);	/* Port E22: MUX = IO_D0  */
  PORTE->PCR[PTE23]= PORT_PCR_MUX(1);	/* Port E23: MUX = IO_D0  */

  /* Setting port data direction */
  PTA->PDDR |= 1<<PTA21;			/* Port A21:  Data Direction = output */
  PTA->PDDR |= 1<<PTA22;			/* Port A22:  Data Direction = output */
  PTA->PDDR |= 1<<PTA23;			/* Port A23:  Data Direction = output */
  PTA->PDDR |= 1<<PTA24;			/* Port A24:  Data Direction = output */

  PTB->PDDR &= ~(1<<PTB27);			/* Port B27: Data Direction = input */
  PTB->PDDR &= ~(1<<PTB28);			/* Port B28: Data Direction = input */
  PTB->PDDR &= ~(1<<PTB29);			/* Port B29: Data Direction = input */

  PTD->PDDR |= 1<<PTD0;				/* Port D0:  Data Direction = output */
  PTD->PDDR &= ~(1<<PTD1);			/* Port D1: Data Direction = input */
  PTD->PDDR |= 1<<PTD2;				/* Port D2:  Data Direction = output */
  PTD->PDDR |= 1<<PTD3;				/* Port D3:  Data Direction = output */
  PTD->PDDR &= ~(1<<PTD13);			/* Port D13: Data Direction = input */

  PTE->PDDR |= 1<<PTE21;			/* Port E21:  Data Direction = output */
  PTE->PDDR |= 1<<PTE22;			/* Port E22:  Data Direction = output */
  PTE->PDDR |= 1<<PTE23;			/* Port E23:  Data Direction = output */

}



void WDOG_disable (void) {
  WDOG->CNT=0xD928C520;     /* Unlock watchdog 		*/
  WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value 	*/
  WDOG->CS = 0x00002100;    /* Disable watchdog 		*/
}



/* Interrupt handler contains the code to be executed each time an interrupt occurs.
 * Interrupts occur at rate of 40 Hz, i.e., 40 times a second. The rate can be changed
 * by changing the interrupt count-down value 'LPIT0->TMR[0].TVAL' in LPIT0_init().
 */
void LPIT0_Ch0_IRQHandler (void) {

	/* Clear LPIT0 timer flag 0 */
	LPIT0->MSR |= LPIT_MSR_TIF0_MASK;

	/* Increment LPIT0 timeout counter */
	lpit0_ch0_flag_counter++;

	/* 10s buffer period for NVIDIA Jetson and RES system to initialise */
	if (lpit0_ch0_flag_counter > INIT_PERIOD) {

		/* Initiate 3-way handshake between NVIDIA Jetson */
		if (AS_state.present_state == AS_ready && !UART_flag 
			&& lpit0_ch0_flag_counter < HANDSHAKE_TIME_OUT) {
			inform_Jetson(initiate);
		}

		/* Send ACK upon receiving response from NVIDIA Jetson */
		else if (UART_flag && !ACK_sent) {
			inform_Jetson(acknowledge);
			Jetson_alive = true;
			ACK_sent = true;
		}

		// /* Activate EBS if handshake with Jetson is not established */
		// else if (!UART_flag && lpit0_ch0_flag_counter > HANDSHAKE_TIME_OUT)
		// 	activate_EBS(); 
		// }

		/* Check state of EBS in case activation occurs through RES button */
		check_EBS();
	}

	/* Set Jetson timer and mark flag for Jetson connection as false if no command
	 * from Jetson is received.
	 */
	if (counter == 0 && Jetson_alive && AS_state.present_state == AS_driving) {
		Jetson_alive = false;
		Jetson_timer = lpit0_ch0_flag_counter;
	}

	/* Activate EBS if time elapsed between last received Jetson command exceeds 10s */
	if (lpit0_ch0_flag_counter - Jetson_timer > COMM_THRESHOLD && !Jetson_alive 
		&& AS_state.present_state == AS_driving) {
		activate_EBS(); 
	}

	/* Check if switch for arming EBS has been pressed */
	arm_EBS();

	/* Check if car is ready-to-drive */
	R2D = check_R2D();

	/* Check if 'go' (clearance to drive) signal has been received */
	go = RES_go();

	/* Check state SBS through reed switch */
	check_SBS();

	/* Obtain state function pointer and execute function for present state */
	state_func = AS_states[AS_state.present_state];
	func_output = state_func();

	/* Update the car's present state based on return code from state function (state_func()) */
	AS_state.previous_state = AS_state.present_state;
	AS_state.present_state = lookup_transitions(AS_state.previous_state, func_output);

	/* Update ASSI */
	ASSI(AS_state.present_state, lpit0_ch0_flag_counter);

	/* Uncomment below code to use S32K LEDs for ASSI */ 
	// ASSI2(AS_state.present_state, lpit0_ch0_flag_counter); 
}
 


int main(void) {
	/*!
	 * Initialization:
	 * =======================
	 */
  WDOG_disable();        	/* Disable WDOG */
  SOSC_init_8MHz();      	/* Initialize system oscilator for 8 MHz xtal */
  SPLL_init_160MHz();    	/* Initialize SPLL to 160 MHz with 8 MHz SOSC */
  NormalRUNmode_80MHz(); 	/* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */

  state_init();				/* Initialise state and flags for pre-driving checks */
  NVIC_init_IRQs();       	/* Enable desired interrupts and priorities */
  LPIT0_init();           	/* Initialize PIT0 for 1 second timeout  */
  PORT_init();				/* Configure ports */
  LPUART1_init();        	/* Initialise Low Power UART @ 9600 bits/s */

  /* Ensure that SB are disengaged */
  SBS.desired_state = disengaged;
  PTD-> PSOR |= 1<<PTD3;
  PTD-> PCOR |= 1<<PTD2;

  /* Dynamically allocated memory for storing a single character */
  char* command = (char*) malloc(sizeof(char));


  /* Infinite for loop to read and store characters forming incoming commands from NVIDIA Jetson */
	  for(;;)
	  {

		  /* Read and register a character in incoming command */
		  char received = LPUART1_receive_char();

		  /* Store first character in command */
		  if (counter == 0 && received != *COMMAND_TERMINATION) {
			  *command = received;
			  counter++;
			  Jetson_alive = true;
		  }

		  /* Store subsequent characters in command */
		  else if (received != *COMMAND_TERMINATION && counter > 0) {
			  command++;
			  *command = received;
			  counter++;
			  Jetson_alive = true;
		  }

		  /* Processing entire command and initiating relevant action */
		  else if (received == *COMMAND_TERMINATION) {

			  /* Starting character for command, start of command (SOC) */
			  char* SOC = command-((counter-1)*(sizeof(char)));

			  /* Register reply from NVIDIA Jetson for initial 3-way handshake */
			  if (AS_state.present_state == AS_ready && lpit0_ch0_flag_counter > INIT_PERIOD) {
				  UART_response(SOC, counter);
			  }

			  update_EBS(SOC, counter);
			  if (AS_state.present_state == AS_driving) {

				  /* Actuate SBS or EBS based on command received */
				  update_EBS(SOC, counter);
				  update_SBS(SOC, counter);

				  /* Detect completion of mission command from NVIDIA Jetson */
				  check_Mission_Status(SOC, counter);
			  }


			  /* Freeing and allocating fresh memory block for storing next command */
			  free(SOC);
			  counter = 0;
			  command = (char*) malloc(sizeof(char));

			  /* Mark flag true upon registering data from Jetson */
			  Jetson_alive = true;
		  }

	  }
}


