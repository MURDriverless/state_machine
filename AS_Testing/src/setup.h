/*
 * setup.h contains constant, state, struct and function definitions. These are
 * essential for organising the state of actuators, autonomous system modules,
 * and therefore, the car.
 *
 * Author:	Vinamra Goswami
 * Last updated: 02/06/2022
 *
 * Autonomous Safety Engineer
 * Melbourne University Racing (MUR) Motorsports
 *
 * University of Melbourne
 */

/* !
 * Key abbreviations:
 * - EBS: Emergency Braking System
 * - SBS: Service Braking System
 * - SB: Service brake
 * - R2D: Ready-to-drive
 * - UART: Universal Asynchronous Receiver Transmitter
 * - FMS: Fault Management System
 */

#ifndef SETUP_H_
#define SETUP_H_


/* Libraries and header files */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "device_registers.h"
#include "clocks_and_modes.h"


/* Configured pins */
#define PTA21 (21)	/* Yellow LED for AS_ready and AS_driving states */
#define PTA22 (22)	/* Blue LED for AS_finished and AS_emergency states */
#define PTA23 (23)	/* Green LED representing tractive system */
#define PTA24 (24)	/* Red LED for EBS */

#define PTB29 (29)	/* Reed switch for checking EBS state */
#define PTB27 (27)	/* Reed switch for checking SBS state */
#define PTB28 (28)	/* Pressure switch */

#define PTC6 (6)	/* UART1 RX */
#define PTC7 (7)    /* UART1 TX */

#define PTD0 (0)	/* EBS trigger */
#define PTD1 (1)	/* Switch for arming EBS */
#define PTD2 (2)	/* Trigger pin for engaging SB */
#define PTD3 (3)	/* Trigger pin for disengaging SB */
#define PTD13 (13)	/* Switch for 'Go' signal */

#define PTE21 (21)	/* Pin for Red LED */
#define PTE22 (22)	/* Pin for Green LED */
#define PTE23 (23)	/* Pin for Blue LED */

/* Low Power Interrupt Timer (LPIT) variables */

/* Number of clock cycles in 1s */
#define NUM_CLKS_1_SEC 40000000

/* Clock cycles elapsed between consecutive interrupts */
#define INTERRUPT_CLKS 1000000		

/* Number of interrupts per second */
#define NUM_INT_SEC (NUM_CLKS_1_SEC/INTERRUPT_CLKS)	

/* Time period for NVIDIA Jetson & RES to initialise */
#define INIT_PERIOD 10*NUM_INT_SEC          

/* Time out for UART connection establishment */ 
#define HANDSHAKE_TIME_OUT 30*NUM_INT_SEC

/* Max. tolerable time for which no commands are received from NVDIA Jetson */
#define COMM_THRESHOLD 15*NUM_INT_SEC


/* Commands for validating connection with NVIDIA Jetson */
static char INIT_CONNECTION[] = "INIT\n";
static char RESPONSE[] = "REPLY";
static char ACK[] = "ACK\n";

/* Input commands from autonomous FMS */
static char EBS_ACTIVATE[] = "EBS1";
static char EBS_ARMED[] = "EBS0";
static char SBS_ENGAGE[] = "SBS1";
static char SBS_DISENGAGE[] = "SBS0";
static char MISSION_ENDED[] = "END";
static char COMMAND_TERMINATION[] = "\r";

/* Output 'go' command for NVIDIA Jetson */ 
static char GO[] = "GO";



/* Flags for tracking connection with NVIDIA Jetson */

/* Marked true once response from Jetson is received */
bool UART_flag;	

/* Marked true once ACK is sent for Jetson's response */			
bool ACK_sent;	

/* Marked true if data from Jetson is being received timely */			
bool Jetson_alive; 	

/* Go signal status */
bool go;

/* Mission complete flag */
bool mission_complete;



/* Enumerated types for defining the possible states of a variable */

/* Short hand codes for each car state */
enum state_codes {AS_off, AS_ready, AS_driving, AS_emergency, AS_finished};

/* EBS states */
enum EBS_states {unavailable, armed, activated};

/* SBS states */
enum SBS_states {disengaged, engaged};

/* Outgoing messages for NVIDIA Jetson */
enum Jetson_msg {initiate, acknowledge, go_drive, EBS_activated};

/* Ready-to-drive status */
enum R2D_status {off, on};

/* Ready-to-drive status variable */
enum R2D_status R2D;



/* Structures for organising and storing system states */

/* Car (autonomous system) state */
struct state {
	enum state_codes previous_state;
	enum state_codes present_state;
} AS_state;

/* EBS status */
struct EBS_status {
    enum EBS_states desired_state;
    enum EBS_states actual_state;
} EBS;

/* SBS status */
struct SBS_status {
	enum SBS_states desired_state;
	enum SBS_states actual_state;
} SBS;

/* Generic structure defining each state transition */
struct transition {
    enum state_codes start_state;
    int transition_num;
    enum state_codes end_state;
};



/* For a complete description of each function, refer to stateMachine.c */

/* State functions:
 * Each function checks every possible transition condition and accordingly outputs
 * a code telling the system what the next state must be.
 */
#define INITIAL_STATE AS_ready
enum state_codes AS_off_state(void);
enum state_codes AS_ready_state(void);
enum state_codes AS_driving_state(void);
enum state_codes AS_emergency_state(void);
enum state_codes AS_finished_state(void);
enum state_codes lookup_transitions(enum state_codes, int);



/* Action-oriented functions:
 * Initiate actions or changes in actuator states
 */
void activate_EBS(void); 
void update_EBS(char*, int);
void update_SBS(char*, int);



/* Critical input functions:
 * Monitor input from critical switches and initiate relevant actions
 */
bool RES_go(void);
bool arm_EBS(void);



/* Status check functions:
 * Check the status of subsystems by obtaining readings from sensors
 */
enum R2D_status check_R2D(void);
void check_EBS(void);
void check_SBS(void);



/* Auxiliary functions */
bool command_check(char*, char*, int);
void ASSI(enum state_codes, int);
void ASSI2(enum state_codes, int); 
void UART_response(char*, int);
void check_Mission_Status(char*, int);
void inform_Jetson(enum Jetson_msg);

#endif



