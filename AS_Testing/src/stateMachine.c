/*
 * stateMachine.c contains all essential functions for monitoring and changing
 * the car's state, along with initiating emergency responses in the safety system.
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



/* Transition lookup table: lists respective end states given a start state and transition number */
struct transition state_transitions[] = {
    {AS_ready, 1, AS_ready},
    {AS_ready, 2, AS_driving},
    {AS_ready, 3, AS_emergency},
    {AS_driving, 2, AS_driving},
    {AS_driving, 4, AS_finished},
    {AS_driving, 3, AS_emergency},
    {AS_finished, 4, AS_finished},
    {AS_finished, 3, AS_emergency},
    {AS_emergency, 3, AS_emergency},
    {AS_emergency, 1, AS_ready}
};



/* AUXILIARY FUNCTIONS */

/* Checks if the incoming command from NVIDIA Jetson matches a predefined
 * command in setup.h
 */
bool command_check(char* command_sent, char* command_verify, int command_len) {
	int i = 0;

	/* String lengths for incoming command and reference in setup.h must match */
	if (command_len == strlen(command_verify)) {

		/* Iterate through and compare each character in commands */
		for(i = 0; i<command_len; i++) {
			if (*(command_sent + i) == (char)command_verify[i]){
				continue;
			}
			else {
				return false;
			}
		}
		return true;
	}

	return false;
}



/* Powers the LEDs for the ASSI depending on the present state of the car */
void ASSI(enum state_codes present_state, int interrupt_counter) {

	switch(present_state) {

	/* No LEDs powered */
	case(AS_off):
			break;

	/* Constant yellow and green LEDs */
	case(AS_ready):
			PTA-> PCOR |= 1<<PTA22;
			PTA-> PCOR |= 1<<PTA24;
			if (EBS.desired_state == armed) {
				PTA-> PSOR |= 1<<PTA21;
				PTA-> PSOR |= 1<<PTA23;
			}
			break;

	/* Flashing (every 0.5s) yellow and constant green LED */
	case(AS_driving):
			if(interrupt_counter % (NUM_INT_SEC/2) == 0) {
				PTA->PTOR |= 1<<PTA21;
			}

			PTA-> PSOR |= 1<<PTA23;
			break;

	/* Constant blue LED */
	case(AS_finished):
			PTA-> PCOR |= 1<<PTA21;
			PTA-> PSOR |= 1<<PTA22;
			PTA-> PCOR |= 1<<PTA23;
			break;

	/* Flashing (every 0.5s) blue LED */
	case(AS_emergency):
			PTA-> PCOR |= 1<<PTA21;
			PTA-> PCOR |= 1<<PTA23;
			PTA-> PSOR |= 1<<PTA24;

			if(interrupt_counter % (NUM_INT_SEC/2) == 0) {
				PTA-> PTOR |= 1<<PTA22;
			}
			break;
	}
}



/* Powers the S32K LEDs for the ASSI depending on the present state of the car */
void ASSI2(enum state_codes present_state, int interrupt_counter) {

	switch(present_state) {
	case(AS_off):
			break;
	case(AS_ready):
			PTE-> PSOR |= 1<<PTE21;
			PTE-> PSOR |= 1<<PTE22;
			PTE-> PCOR |= 1<<PTE23;
			break;
	case(AS_driving):
			if(interrupt_counter % (NUM_INT_SEC/2) == 0) {
				PTE->PTOR |= 1<<PTE21;
				PTE->PTOR |= 1<<PTE22;
			}
			break;
	case(AS_finished):
			PTE-> PCOR |= 1<<PTE21;
			PTE-> PCOR |= 1<<PTE22;
			PTE-> PSOR |= 1<<PTE23;
			break;
	case(AS_emergency):
			PTE-> PCOR |= 1<<PTE21;
			PTE-> PCOR |= 1<<PTE22;
			if(interrupt_counter % (NUM_INT_SEC/2) == 0) {
				PTE-> PTOR |= 1<<PTE23;
			}
			break;
	} 
}



/* Checks if intended response from Jetson is received (for 3-way handshake)
 * and sets UART_flag to true.
 */
void UART_response(char* command, int command_len) {

	if (!UART_flag && command_check(command, RESPONSE, command_len)) {
		UART_flag = true;
	}
}



/* Checks if command for completion or termination of mission has been
 * received from NVIDIA Jetson. This function is strictly for use with
 * a simulation, e.g., MUR sim.
 */
void check_Mission_Status(char* command, int command_len) {

	if (!mission_complete && command_check(command, MISSION_ENDED, command_len)) {
		mission_complete = true;
	}
}



/* Transmits message to NVIDIA Jetson (over UART) depending on specified
 * message request */
void inform_Jetson(enum Jetson_msg message_request) {

	switch(message_request) {

	/* Initiate 3-way handshake */
	case(initiate):
		LPUART1_transmit_string(INIT_CONNECTION);
		break;

	/* Acknowledge response from Jetson */
	case(acknowledge):
		LPUART1_transmit_string(ACK);
		break;

	/* Transmit 'go' signal to autonomous system, car is ready to drive */
	case(go_drive):
		LPUART1_transmit_string("GO\n");
		break;

	/* Inform Jetson of EBS activation, in case EBS is activated from RES */
	case(EBS_activated):
		LPUART1_transmit_string("EBS1\n");
		break;
	}

}



/* ACTION-ORIENTED FUNCTIONS */

/* Function to activate EBS */ 
void activate_EBS(void) {
	
	EBS.desired_state = activated;

	/* Output a digital low from pin PTD0 */ 
	PTD-> PCOR |= 1<<PTD0; 

}



/* Function processes commands regarding EBS and initiates necessary actuation */
void update_EBS(char* command, int command_len) {

	/* Received command to activate EBS */
	if (command_check(command, EBS_ACTIVATE, command_len)) {
		activate_EBS(); 
	}

	/* Temporary inclusion (back-up): arm EBS if EBS_ARMED command is
	 * received from NVIDIA Jetson. Primary way to arm EBS should be
	 * through EBS arming switch.
	 */
	else if (command_check(command, EBS_ARMED, command_len)) {
		PTD-> PSOR |= 1<<PTD0;
		EBS.desired_state = armed;
	}

}



/* Function to engage or disengage service brake depending on the received command */
void update_SBS(char* command, int command_len) {

	/* Received command to engage SBS */
	if (command_check(command, SBS_ENGAGE, command_len)) {
		SBS.desired_state = engaged;
		PTD-> PSOR |= 1<<PTD2;
		PTD-> PCOR |= 1<<PTD3;
	}

	/* Received command to disengage SBS */
	else if (command_check(command, SBS_DISENGAGE, command_len)) {
		SBS.desired_state = disengaged;
		PTD-> PSOR |= 1<<PTD3;
		PTD-> PCOR |= 1<<PTD2;
	}

}



/* STATE FUNCTIONS */

/* AS_off state not being used for demo. S32K powers off when ASMS = off */
enum state_codes AS_off_state(void) {
	return AS_off;
}



/* Checks transition conditions from AS_ready state */
enum state_codes AS_ready_state(void) {

	/* Checks for AS_driving:
	 * - EBS is armed
	 * - 3-way handshake with Jetson is successful
	 * - R2D is on
	 * - 'go' signal has been received
	 */
	if(EBS.desired_state == armed && go && /*R2D && */ UART_flag && ACK_sent) {
		inform_Jetson(go_drive);
		go = false;
		return AS_driving;
	}

	/* Checks for AS_emergency: EBS is activated */
	if(EBS.desired_state == activated || EBS.actual_state == activated) {
		inform_Jetson(EBS_activated);
		return AS_emergency;
	}

	return AS_ready;
}



/* Checks transition conditions from AS_driving state */
enum state_codes AS_driving_state(void) {

	/* Checks for AS_finished: mission is complete */
	if (mission_complete) {
		return AS_finished;
	}

	/* Checks for AS_emergency: EBS is activated */
	if(EBS.desired_state == activated || EBS.actual_state == activated) {
		inform_Jetson(EBS_activated);
		return AS_emergency;
	}

	return AS_driving;
}



/* Checks transition conditions from AS_emergency state */
enum state_codes AS_emergency_state(void) {

	/* Checks for AS_ready: EBS is manually returned to armed state */
	if(EBS.desired_state == armed) {
		UART_flag = false;
		ACK_sent = false;
		mission_complete = false;
		return AS_ready;
	}

	return AS_emergency;
}



/* Checks transition conditions from AS_finished state */
enum state_codes AS_finished_state(void) {

	/* Checks for AS_emergency: EBS is activated */
	if(EBS.desired_state == activated) {
		inform_Jetson(EBS_activated);
		return AS_emergency;
	}

	return AS_finished;
}



/* Outputs the state code for the next state based on the transition lookup table */
enum state_codes lookup_transitions(enum state_codes previous_state, int func_output) {
	int i = 0;
	enum state_codes state;
	int transition_num;

	for(i=0; i<sizeof(state_transitions); i++) {
		state = state_transitions[i].start_state;
		transition_num = state_transitions[i].transition_num;

		if (state == previous_state && transition_num == func_output) {
			return state_transitions[i].end_state;
		}
	}

	return 0;
}



/* CRITICAL INPUT FUNCTIONS */

/* Checks if switch to arm EBS has been pressed, and if so, sends signal to arm EBS */
bool arm_EBS(void) {

	if (PTD-> PDIR & (1<<PTD1)) {
		EBS.desired_state = armed;
		/* Command to arm EBS */
		PTD-> PSOR |= 1<<PTD0;
		return true;
	}

	return false;
}



/* Checks if switch for 'go' signal has been pressed */
bool RES_go(void) {

	if (PTD-> PDIR & (1<<PTD13)) {
		return true;
	}

	return false;
}



/* STATUS CHECK FUNCTIONS */

/* Conducts necessary pre-driving checks and determines if car is R2D */
enum R2D_status check_R2D(void) {

	/* Check for sufficient pressure in braking system */
	if (PTB-> PDIR & (1<<PTB28)) {
		return on;
	}

	/* If there is drop in braking pressure, activate the EBS */
//	else if (AS_state.present_state == AS_driving) {
//		EBS.desired_state = activated;
//		PTD-> PCOR |= 1<<PTD0;
//		return off;
//	}

	return off;

}



/* Determines the state of the EBS based on EBS relay */
void check_EBS(void) {

	/* Detecting digital low at EBS relay pin */
	uint32_t EBS_relay =  !(PTB-> PDIR & (1<<PTB29));

	if (EBS_relay) {
		EBS.desired_state = activated;
		EBS.actual_state = activated;
	}

}



/* Determines the state of the SBS based on readings from the reed switch */
void check_SBS(void) {

	/* Detecting digital low at SBS reed switch pin */
	uint32_t reed_switch =  !(PTB-> PDIR & (1<<PTB27));

	if (reed_switch) {
		SBS.actual_state = engaged;
	}
	else if (!reed_switch) {
		SBS.actual_state = disengaged;
	}

}
