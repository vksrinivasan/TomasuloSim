#ifndef PROCSIM_H
#define PROCSIM_H

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static const uint64_t DEFAULT_R = 2;   
static const uint64_t DEFAULT_F = 4;    
static const uint64_t DEFAULT_J = 3;    
static const uint64_t DEFAULT_K = 2;    
static const uint64_t DEFAULT_L = 1;	

// Just create an instruction struct that procsim_driver will use
typedef struct instr_t {
	uint64_t address;
	int funcUnit;
	int destReg;
	int dest_tag;
	int source1;
	int source1_tag; 
	int source1_ready;
	
	int source2;
	int source2_tag;
	int source2_ready;
	
	// These are just variables that store what the cycle was when the 
	// instruction got moved to this stage
	int fetch;
	int disp;
	int sched;
	int exec;
	int state;
	
	// These additional fields are just to help with branches
	int branch; // is it a branch
	int taken; // is the branch taken or not
	int correct_pred; // was the branch prediction correct
	int resolved; // was the branch resolved
	
} instr;

// Create an IF list that the procsim_driver will use
typedef struct if_listnode {
	instr *theInstr;
	struct if_listnode *next;
} if_listnode;

/**
 * This is the struct that contains the dispatch queue. It is just 
 * a singly linked list since we just add them in order to the scheduling 
 * queue as long as there is space in the scheduling queue
 */
typedef struct dispatch_node_t {
	instr *theInstr;
	struct dispatch_node_t *next;
	// Flag that marks that this instruction will be moved to the scheduling 
	// queue at the start of the next cycle. Also those instructions that are 
	// marked will read from the register file at the right point in time 
	// while they are in the dispatch queue
	int mark_for_move;
} dispatch_node; 

/**
 * This is a struct that contains the scheduling queue nodes. It is a doubly 
 * linked list since I'll end up having to deal with deleting things potentially
 * out of order
 */
typedef struct scheduling_node_t {
	instr *theInstr;
	struct scheduling_node_t *next;
	struct scheduling_node_t *prev;
	int fired; // This just tells us if the instruction has been fired
	int sendToExecute; // This is a flag that tells us if we should send to exec. at start of next cycle
	int waiting;
} schedule_node;

/**
 * This is a struct of what will go into the FUs. It is essentially just the 
 * instruction struct and then additional information that indicates 
 * which of the elements should be sent off to the state update stage at the very
 * beginning of the next clock cycle
 */
typedef struct execute_node_t {
	instr *theInstr;
	int chosen; // Chosen to send to state update at the very start of the next cycle
} execute_node;

/**
 * This struct contains the final information needed for printing
 */
typedef struct final_node_t {
	int dest_tag;
	int fetch;
	int disp;
	int sched;
	int exec;
	int state;
	struct final_node_t *next;
} final_node;


/**
 * A struct for storing useful parameters of the simulation
 */
typedef struct config_t {
	int numRegs;
	int k0_size;
	int k1_size;
	int k2_size;
	int num_r_bus;
	int max_sched_queue;
	int fetch_rate;
} config;

/**
 * A struct for storing useful stats of the simulation
 */
typedef struct stats_t {
	long totalBranchInstr;
	long totalCorrectBranch;
	float predictionAcc;
	float avgDispQueue;
	long maxDispQueue;
	float avgInstIssue;
	float avgInstRet;
	long totalRuntime;
} stats;


/*
 * Functions I need to declare for procsim_driver. Declared in order of appearence
 */ 
 
// Initialization Function
void proc_init(int numRegs, int k0_size, int k1_size, int k2_size, int num_r_bus, int fetch_rate); // Initialize my globals

// Functions that help identify when to stop the simulation
int stateEmpty();
dispatch_node *getDispHead(); 
schedule_node *getScheduleHead();

// Functions to transition at the start of the cycle
void sendToFinal();
void sendToSU(int clock);
void resolveBranches();
void moveToExecute(int clock);
void dispatchToSchedule(int clock, int totalMarked);
void dispatch_Enqueue(if_listnode **fetch_head, int cycle);
int getPrediction(uint64_t address);

// Then just update some stats needed
void updateDispatchQueueSize();

// Then simulate the mid-cycle happenings
void writeToRegFile();
void setToFired();
int reserveScheduleSpots();
void readUpdateRegFile(int totalMarked);
void broadcastToSched();
void removeAllSUFromSched();

// Mark Instructions at various stages as ready to move to the next stage
void setToChosen();
void markForExecution();

// Print/Stats/Cleanup Functions Needed
void printScheduleQueue();
void printFinalQueue();
stats *getStats();
void freeFinalQueue();
 
#endif /* PROCSIM_H */
