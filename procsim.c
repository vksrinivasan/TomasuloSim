#include "procsim.h"
#include "assert.h"

#define INT_MIN -2147483648
#define INT_MAX 2147483647

/*
 * Globals I need
 */
dispatch_node *dispatch_head; // Dispatch queue
schedule_node *schedule_head; // Scheduling queue
final_node *final_head; // Final queue. Just stores completed instruction structs
final_node *final_tail; // Final queue tail so we can append faster.
instr **sup; // State update array. Of size r (number of common data buses)
int schedule_size;
int **reg_File; // Register file. It will hold ready and tag
execute_node **k_0; // functional unit k_0. Array of instructions
execute_node **k_1; // functional unit k_1. Array of instructions
execute_node **k_2; // functional unit k_2. Array of instructions
config *curr_Config; // Config structure that contains useful parameter constants
uint64_t GHR; // Our GHR register
uint64_t **GSelect; // Our GSelect apparatus. Stored as a 2D Array
int stallDispatch; // A lock for our dispatch queue
stats *myStats; // A struct for our stats to be stored in

/*
 * Function headers I need
 */ 
void proc_init(int numRegs, int k0_size, int k1_size, int k2_size, int num_r_bus, int fetch_rate);
int stateEmpty();
dispatch_node *getDispHead();
schedule_node *getScheduleHead();
void sendToFinal();
void sendToSU(int clock);
void resolveBranches();
int findNumUnresolved();
int getMinExecCycle();
int getMinTagIndex(int cycle);
void updateGSelect(uint64_t address, int taken);
void updateSmithCounter(uint64_t row, uint64_t col, int taken);
void updateGHR(int taken);
void shiftTaken();
void shiftNotTaken();
void moveToExecute(int clock);
void putInFU(instr *theInstr, int FU_num, int clock);
void dispatchToSchedule(int clock, int totalMarked);
void dispatch_Enqueue(if_listnode **fetch_head, int cycle);
int getPrediction(uint64_t address);
uint64_t getGHR();
void writeToRegFile();
void setToFired();
int reserveScheduleSpots();
void readUpdateRegFile(int totalMarked);
void broadcastToSched();
void removeAllSUFromSched();
void removeFromSched(instr *theInstr);
void setToChosen();
int getNumPossible();
execute_node *getMinNode();
int findMinCycle();
void markForExecution();
int numSpotsAvailable(char FU);
void markScheduleEntries(int openSpots, char FU);
void printScheduleQueue();
void printFinalQueue();

/*
 * Misc. Functions
 */
void updateDispatchQueueSize();
stats *getStats();
void freeFinalQueue();

/* 
 * Actual Functions written here
 */
 
/*
 * This function just initializes the globals I need to run the simulation
 */
void proc_init(int numRegs, int k0_size, int k1_size, int k2_size, int num_r_bus, int fetch_rate) {
	
	// Set my two queues to NULL for now. They'll fill up as instructions come in
	dispatch_head = NULL;
	schedule_head = NULL;
	final_head = NULL;
	final_tail = NULL;
	
	// Allocate space for my register file. It's (numRegs x 2) in dimension
	reg_File = (int **)malloc(sizeof(int *) * numRegs);
	if(reg_File == NULL)
		return;
	for(int i = 0; i < numRegs; i++) {
		reg_File[i] = (int *)malloc(sizeof(int) * 2);
		if(reg_File[i] == NULL) {
			return;
		} else {
			reg_File[i][0] = 1; // Ready Bit
			reg_File[i][1] = -5; // Tag. -5 is just going to be our default
		}
	}
	
	// Allocate space for my k_0 functional unit. It's just an array of pointers
	// to execute nodes (instructions + chosen flags)
	k_0 = (execute_node **)malloc(sizeof(execute_node *) * k0_size);
	if(k_0 == NULL) // Just allocate for now. We will fill them later
		return;
		
	k_1 = (execute_node **)malloc(sizeof(execute_node *) * k1_size);
	if(k_1 == NULL)
		return;
	
	k_2 = (execute_node **)malloc(sizeof(execute_node *) * k2_size);
	if(k_2 == NULL) 
		return;	
	
	// Allocate space for my state update array. It will just hold the instructions
	// from the execute stage
	sup = (instr **)malloc(sizeof(instr *)*num_r_bus);
	if(sup == NULL)
		return;
	
	// Allocate space for my curr_Config
	curr_Config = malloc(sizeof(config)*1);
	if(curr_Config == NULL)
		return;
	curr_Config->numRegs = numRegs;
	curr_Config->k0_size = k0_size;
	curr_Config->k1_size = k1_size;
	curr_Config->k2_size = k2_size;
	curr_Config->num_r_bus = num_r_bus;
	curr_Config->max_sched_queue = 2*(k0_size + k1_size + k2_size);
	curr_Config->fetch_rate = fetch_rate;
	
	// Set the size of my scheduling queue to 0
	schedule_size = 0;
	
	// Initialize GHR and Gselect Table
	GHR = 0x0;
	GSelect = (uint64_t **)malloc(sizeof(uint64_t *)*128);
	if(GSelect == NULL)
		return;
	for(int i = 0; i < 128; i++) {
		GSelect[i] = (uint64_t *)malloc(sizeof(uint64_t)*8);
		if(GSelect[i] == NULL)
			return;
	}
	for(int i = 0; i < 128; i++) {
		for(int j = 0; j < 8; j++) {
			GSelect[i][j] = 1; // Initialized at 1
		}
	}
	
	// Initialize our stallDispatch lock
	stallDispatch = 0; // it starts out unlocked
	
	// Initialize our stats
	myStats = (stats *)malloc(sizeof(stats)*1);
	if(myStats == NULL)
		return;
	myStats->totalBranchInstr = 0;
	myStats->totalCorrectBranch = 0;
	myStats->predictionAcc = 0.0;
	myStats->avgDispQueue = 0.0;
	myStats->maxDispQueue = 0;
	myStats->avgInstIssue = 0.0;
	myStats->avgInstRet = 0.0;
	myStats->totalRuntime = 0;
}

/*
 * Function that checks if all of the state update array is empty or not
 */
int stateEmpty() {
	int numEntries = curr_Config->num_r_bus;
	
	for(int i = 0; i < numEntries; i++) {
		if(sup[i] != NULL) {
			return 0;
		}
	}
	return 1;
}

/*
 * Just returns the address of the dispatch head so that the driver knows 
 * when to stop the simulation
 */
dispatch_node *getDispHead() {
	return dispatch_head;
}

/*
 * Just returns the address of the schedule head so the driver knows when 
 * to stop the simulation
 */
schedule_node *getScheduleHead() {
	return schedule_head;
}

/*
 * This function is a new send to final. It create a new node that deletes a lot
 * of the unnecessary data and it stores everything in sorted order rather
 * than just appending to the end of a queue
 */
void sendToFinal() {
	int i;
	for(i = 0; i < curr_Config->num_r_bus; i++) {
		if(sup[i] != NULL) {
			
			// Create a new final node that just stores useful information
			// that we need for output
			final_node *newNode = (final_node *)malloc(sizeof(final_node)*1);
			if(newNode == NULL)
				return;
			newNode->dest_tag = sup[i]->dest_tag;
			newNode->fetch = sup[i]->fetch;
			newNode->disp = sup[i]->disp;
			newNode->sched = sup[i]->sched;
			newNode->exec = sup[i]->exec;
			newNode->state = sup[i]->state;
			newNode->next = NULL;
			
			if(final_head == NULL) {
				final_head = newNode;
				final_tail = newNode;
			} else {
				final_tail->next = newNode;
				final_tail = newNode;
			}
			
			// Free the memory for the instruction struct we had
			free(sup[i]);
			sup[i] = NULL;
		}
	}
	return;
}

/*
 * This function just sends all the 'chosen' instructions from the FU to the 
 * state update array and then nulls out that FU entry
 */
void sendToSU(int clock) {
	int index = 0;
	int i;
	
	for(i = 0; i < curr_Config->k0_size; i++) {
		if(k_0[i] != NULL && k_0[i]->chosen == 1) {
			sup[index] = k_0[i]->theInstr;
			sup[index]->state = clock;
			index++;
			free(k_0[i]);
			k_0[i] = NULL;
		}
	}
	
	for(i = 0; i < curr_Config->k1_size; i++) {
		if(k_1[i] != NULL && k_1[i]->chosen == 1) {
			sup[index] = k_1[i]->theInstr;
			sup[index]->state = clock;
			index++;
			free(k_1[i]);
			k_1[i] = NULL;
		}
	}
	
	for(i = 0; i < curr_Config->k2_size; i++) {
		if(k_2[i] != NULL && k_2[i]->chosen == 1) {
			sup[index] = k_2[i]->theInstr;
			sup[index]->state = clock;
			index++;
			free(k_2[i]);
			k_2[i] = NULL;
		}
	}
	return;
}

/*
 * Look through what was just moved to state update and resolve them in tag order
 */
void resolveBranches() {
	int numUnresolved = findNumUnresolved();
	while(numUnresolved > 0) {
		int cycle = getMinExecCycle();
		int index = getMinTagIndex(cycle);
		
		// First update GSelect
		updateGSelect(sup[index]->address, sup[index]->taken);

		// Then update GHR
		updateGHR(sup[index]->taken);

		if(sup[index]->correct_pred == 0) {
			assert(stallDispatch == 1); // Has to be true
			// But now we resolved so we can set stall dispatch to 0
			stallDispatch = 0;
		}
		
		// mark as resolved
		sup[index]->resolved = 1;
		
		// decrement 
		numUnresolved--;
	}
}

/*
 * Look through the state update array and find the number of branches in it
 * that are unresolved
 */
int findNumUnresolved() {
	int i;
	int numUnresolved = 0;
	for(i = 0; i < curr_Config->num_r_bus; i++) {
		if(sup[i] == NULL)
			continue;
		if(sup[i]->resolved == 0) {
			assert(sup[i]->branch == 1); // had to be a branch
			numUnresolved++;
		}
	}
	return numUnresolved;
}

/*
 * Get the min cycle the instructions entered exec. This relative ordering is the
 * same as the order in which they left exec
 */
int getMinExecCycle() {
	int i;
	int minCycle = INT_MAX;
	for(i = 0; i < curr_Config->num_r_bus; i++) {
		if(sup[i] == NULL)
			continue;
		if((sup[i]->resolved == 0) && (sup[i]->exec < minCycle)) {
			minCycle = sup[i]->exec;
		}
	}
	return minCycle;
}

/*
 * Look through state update array and get the index of the branch instruction
 * that is unresolved with the lowest tag
 */
int getMinTagIndex(int cycle) {
	int i;
	int minTag = INT_MAX;
	int minIndex = -1;
	for(i = 0; i < curr_Config->num_r_bus; i++) {
		if(sup[i] == NULL)
			continue;
		if((sup[i]->resolved == 0) && (sup[i]->dest_tag < minTag) && (sup[i]->exec == cycle)) {
			minIndex = i;
			minTag = sup[i]->dest_tag;
		}
	}
	return minIndex;
}

/*
 * This helper function updates the GSelect Smith Counter
 */
void updateGSelect(uint64_t address, int taken) {
	uint64_t row = (address/4)%128;
	uint64_t col = getGHR();
	updateSmithCounter(row, col, taken);
	return;
}

/*
 * This helper function actually updates the smith counter
 */
void updateSmithCounter(uint64_t row, uint64_t col, int taken) {
	uint64_t counterValue = GSelect[row][col];
	switch(counterValue) {
		case 0:
			if(taken == 1) {
				GSelect[row][col] = 1;
			} else {
				GSelect[row][col] = 0;
			}
			break;
		case 1:
			if(taken == 1) {
				GSelect[row][col] = 2;
			} else {
				GSelect[row][col] = 0;
			}
			break;
		case 2:
			if(taken == 1) {
				GSelect[row][col] = 3;
			} else {
				GSelect[row][col] = 1;
			}
			break;
		case 3:
			if(taken == 1) {
				GSelect[row][col] = 3;
			} else {
				GSelect[row][col]  = 2;
			}
			break;
		default:
			break;
	}
	return;
}

/*
 * This helper function updates the GHR
 */
void updateGHR(int taken) {
	if(taken == 1) {
		shiftTaken();
		return;
	} 
	if(taken == 0) {
		shiftNotTaken();
		return;
	}
}

/*
 * This helper function shifts in a 1 in case the branch was taken
 */
void shiftTaken() {
	GHR = ((GHR << 1) | 1);
	return;
}

/*
 * This helper function shifts in a 0 in case the branch was not taken
 */
void shiftNotTaken() {
	GHR = (GHR << 1);
	return;
}

/* 
 * This function puts instructions in the scheduling queue that have been marked 
 * for execution in the corresponding FU. It also marks 'waiting' to be 1 so that 
 * on future calls to this function, we don't try to move over instructions that 
 * are already in a FU 
 */
void moveToExecute(int clock) {
	schedule_node *iterator = schedule_head;
	while(iterator != NULL) {
		if(iterator->sendToExecute == 1 && iterator->waiting == 0) {
			putInFU(iterator->theInstr, iterator->theInstr->funcUnit, clock);
			iterator->waiting = 1;
		}
		iterator = iterator->next;
	}
	return;
}

/*
 * Helper function that puts an instruction in an FU
 */
void putInFU(instr *theInstr, int FU_num, int clock) {
	execute_node **FU;
	int numSpots;
	execute_node *newNode = (execute_node *)malloc(sizeof(execute_node) * 1);
	theInstr->exec = clock;
	newNode->theInstr = theInstr;
	newNode->chosen = 0;
	
	switch(FU_num) {
		case 0:
			FU = k_0;
			numSpots = curr_Config->k0_size;
			break;
		case 1:
			FU = k_1;
			numSpots = curr_Config->k1_size;
			break;
		case -1:
			FU = k_1;
			numSpots = curr_Config->k1_size;
			break;
		case 2:
			FU = k_2;
			numSpots = curr_Config->k2_size;
			break;
		default:
			break;
	}
	
	int i;
	for(i = 0; i < numSpots; i++) {
		if(FU[i] == NULL) {
			FU[i] = newNode;
			break; // once you find an open spot, you're done
		}
	}
	
	// Just ensure that you actually found a spot if we thought there was an open one.
	assert(FU[i] == newNode); 
	return;
}

/*
 * This function just moves the dispatch queue marked for move instructions from 
 * the dispatch queue to the schedule queue. This should happen at the very start
 * of the cycle to simulate that they just moved immediately
 */ 
void dispatchToSchedule(int clock, int totalMarked) {
	dispatch_node *disp_iterator = dispatch_head;
	schedule_node *schedule_iterator = schedule_head;
	int count = 0;
	
	while(disp_iterator != NULL) {
		if(count == totalMarked)
			break; // No need to keep searching if we already found all the marked ones
		if(disp_iterator->mark_for_move == 1) {
			count++;
			instr *theInstr = disp_iterator->theInstr;
			schedule_node *newNode = (schedule_node *)malloc(sizeof(schedule_node)*1);
			newNode->theInstr = theInstr;
			newNode->theInstr->sched = clock;
			newNode->prev = NULL;
			newNode->next = NULL;
			newNode->fired = 0;
			newNode->sendToExecute = 0;
			newNode->waiting = 0;
			
			// Then just add this new node to the schedule queue
			if(schedule_iterator == NULL) {
				schedule_head = newNode;
				schedule_iterator = schedule_head;
				schedule_size++;
			} else {
				while(schedule_iterator->next != NULL)
					schedule_iterator = schedule_iterator->next;
				schedule_iterator->next = newNode;
				newNode->prev = schedule_iterator;
				schedule_size++;
				assert(schedule_size <= curr_Config->max_sched_queue);
			}
		}
		dispatch_node *temp = disp_iterator;
		disp_iterator = disp_iterator->next;
		free(temp); // Just free the old head of the dispatch queue
	}
	dispatch_head = disp_iterator; // Make the head whatever the new iterator is
	return;
}

/* 
 * This function enqueues all instructions from the fetch queue into the dispatch 
 * queue
 */
void dispatch_Enqueue(if_listnode **fetch_head, int cycle) {
	dispatch_node *dispatch_iterator = dispatch_head;
	int numAllowed = curr_Config->fetch_rate;
	
	while(fetch_head[0] != NULL && (stallDispatch == 0) && (numAllowed > 0)) { 
		// Get items from fetch queue and put the instruction in a dispatch node
		if_listnode *temp = fetch_head[0];
		
		dispatch_node *newDispatchNode = (dispatch_node *)malloc(sizeof(dispatch_node)*1);
		if(newDispatchNode == NULL)
			return;
		newDispatchNode->theInstr = temp->theInstr;
		newDispatchNode->theInstr->disp = cycle; // Set the cycle for each instruction
		newDispatchNode->next = NULL;
		newDispatchNode->mark_for_move = 0; 
		
		// Now if it's a branch we need to get the prediction and see if it's 
		// correct or not
		if(newDispatchNode->theInstr->branch == 1) {
			(myStats->totalBranchInstr)++;
			int prediction = getPrediction(newDispatchNode->theInstr->address);
			if(prediction == newDispatchNode->theInstr->taken) {
				(myStats->totalCorrectBranch)++;
				newDispatchNode->theInstr->correct_pred = 1;
			} else {
				newDispatchNode->theInstr->correct_pred = 0;
			}
		}
		
		// Just handle the fact that it's a branch
		if(newDispatchNode->theInstr->correct_pred == 0) {
			assert(newDispatchNode->theInstr->branch == 1); // has to be a branch
			stallDispatch = 1; // won't move any more until this flag is turned off
		}
		
		if(dispatch_iterator == NULL) {
			// If dispatch_queue is empty, the new node is the queue
			dispatch_head = newDispatchNode;
			dispatch_iterator = dispatch_head;
		} else {
			// otherwise iterator through the dispatch queue and add the new node 
			// to the end
			while(dispatch_iterator->next != NULL)
				dispatch_iterator = dispatch_iterator->next;
			dispatch_iterator->next = newDispatchNode;
		}
		// Move the fetch queue pointer to the next instruction in the list
		fetch_head[0] = (fetch_head[0]->next);
		// Then free the old head
		free(temp);
		
		numAllowed--;
	}
	
	return;
}

/*
 * Look at GSelect entry to get the prediction
 */
int getPrediction(uint64_t address) {
	uint64_t row = (address/4)%128;
	uint64_t col = getGHR();
	uint64_t smithValue = GSelect[row][col];
	int prediction;
	
	switch(smithValue) {
		case 0:
			prediction = 0;
			break;
		case 1:
			prediction = 0;
			break;
		case 2:
			prediction = 1;
			break;
		case 3:
			prediction = 1;
			break;
		default:
			assert(0);
			break;
	}
	return prediction;
}

/* 
 * This function just gets the GHR
 */
uint64_t getGHR() {
	return GHR & 0x7;
}

/*
 * This function writes whatever is in state update to the register file (if 
 * the destination tags match)
 */
void writeToRegFile() {
	int numSUElements = curr_Config->num_r_bus;
	for(int i = 0; i < numSUElements; i++) {
		
		if(sup[i] == NULL)
			continue;
		
		int destReg = sup[i]->destReg;
		int destTag = sup[i]->dest_tag;
		
		if(destReg == -1)
			continue; // If it's -1, there's nothing to update. Just move on
		
		if(reg_File[destReg][1] == destTag) {
			assert(reg_File[destReg][0] == 0); // Should not be currently ready
			reg_File[destReg][0] = 1; // Set to ready
			reg_File[destReg][1] = -5; // set back to default
		}
	}
	return;
}

/* 
 * This function goes through the schedule queue and marks instructions to fire 
 * if both the source registers are ready
 */
void setToFired() {
	schedule_node *schedule_iterator = schedule_head;
	while(schedule_iterator != NULL) {
		if(schedule_iterator->theInstr->source1_ready && schedule_iterator->theInstr->source2_ready &&
			schedule_iterator->fired != 1) {
			schedule_iterator->fired = 1;
		}
		schedule_iterator = schedule_iterator->next;
	}
	return;
}


/*
 * This function reserves n entries of the dispatch queue for moving to the 
 * scheduling queue at the very start of the next cycle. The nodes that get 
 * marked also get info updated from reading of register file 
 */
int reserveScheduleSpots() {
	// Num available spots is number of free spots
	int numAvailSpots = curr_Config->max_sched_queue - schedule_size;
	
	dispatch_node *iterator = dispatch_head;
	int count = 0; // This is the number of nodes we actually mark
	while((iterator != NULL) && (numAvailSpots > 0)) {
		assert(iterator->mark_for_move == 0); // if it was already 1, it shouldn't be here
		iterator->mark_for_move = 1;
		iterator = iterator->next;
		numAvailSpots--;
		count++;
	}
	return count;
}

/*
 * This function reads/updates the register file for the n marked slots in the 
 * dispatch queue that will be moved to the scheduling queue at the very start of the next cycle
 */
void readUpdateRegFile(int totalMarked) {
	dispatch_node *iterator = dispatch_head;
	int count = 0; // We can stop searching after we have found n instructions that were marked
	while(iterator != NULL) {
		if(count == totalMarked)
			break;
		if(iterator->mark_for_move == 1) {
			count++;
			int src_1_reg = iterator->theInstr->source1;
			int src_2_reg = iterator->theInstr->source2;
			int dest_reg = iterator->theInstr->destReg;
			
			// Fill the data for src 1 in the instruction struct
			if(src_1_reg == -1) {
				// In this case there is no register needed
				iterator->theInstr->source1_tag = -5; // Placeholder
				iterator->theInstr->source1_ready = 1;
			}
			else if(reg_File[src_1_reg][0] == 1) {
				// If the register file entry is ready, then
				// we can just take that value
				assert(reg_File[src_1_reg][1] == -5);
				iterator->theInstr->source1_tag = -5;
				iterator->theInstr->source1_ready = 1;
			} else {
				// If the register file entry is not ready, then 
				// we take the tag from the register file
				assert(reg_File[src_1_reg][0] != 1);
				assert(reg_File[src_1_reg][1] > -1);
				iterator->theInstr->source1_tag = reg_File[src_1_reg][1];
				iterator->theInstr->source1_ready = 0;
			}
			
			// Fill the data for src2 in the instruction struct
			if(src_2_reg == -1) {
				// In this case there is no register needed
				iterator->theInstr->source2_tag = -5; // Placeholder
				iterator->theInstr->source2_ready = 1;
			}
			else if(reg_File[src_2_reg][0] == 1) {
				// If the register file entry is ready, then
				// we can just take that value
				assert(reg_File[src_2_reg][1] == -5);
				iterator->theInstr->source2_tag = -5;
				iterator->theInstr->source2_ready = 1;
			} else {
				// If the register file entry is not ready, then 
				// we take the tag from the register file
				assert(reg_File[src_2_reg][0] != 1);
				assert(reg_File[src_2_reg][1] > -1);
				iterator->theInstr->source2_tag = reg_File[src_2_reg][1];
				iterator->theInstr->source2_ready = 0;
			}
			
			// Now update the register file for the dest reg
			if(dest_reg == -1) {
			}
			else {
				reg_File[dest_reg][0] = 0;
				reg_File[dest_reg][1] = iterator->theInstr->dest_tag;
			}
		}
		iterator = iterator->next;
	}
	return;
}

/*
 * This function just updates the scheduling queue via the result bus. So for 
 * each element in state update, you just run through the whole scheduling queue
 * and look for not fired nodes that have non-ready src1's and src2's that have 
 * matching register numbers and tags. Just mark them to ready and set the tag 
 * to the default -5
 */ 
void broadcastToSched() {
	int numSUElements = curr_Config->num_r_bus;
	schedule_node *iterator;
	
	for(int i = 0; i < numSUElements; i++) {
		if(sup[i]==NULL)
			continue;
		iterator = schedule_head;
		while(iterator != NULL) {
			if(iterator->fired ==0) {
				if(iterator->theInstr->source1 == sup[i]->destReg && iterator->theInstr->source1_ready == 0 &&
					iterator->theInstr->source1_tag == sup[i]->dest_tag) {
						iterator->theInstr->source1_ready = 1; // Set to ready
						iterator->theInstr->source1_tag = -5; // set to default
					}

				if(iterator->theInstr->source2 == sup[i]->destReg && iterator->theInstr->source2_ready == 0 &&
					iterator->theInstr->source2_tag == sup[i]->dest_tag) {
						iterator->theInstr->source2_ready = 1; // Set to ready
						iterator->theInstr->source2_tag = -5; // set to default
					}
					
				// Now, just check if both source 1 and 2 are ready for the instruction.
				// If so, then just mark the instruction as 'fired'
				if(iterator->theInstr->source1_ready && iterator->theInstr->source2_ready) {
					assert(iterator->fired == 0); // Has to be true
					iterator->fired = 1; // This instruction is eligible to move to exec next cycle
				}
			}
			iterator = iterator->next;
		}
	}
	return;
}

/*
 * This function just deletes the nodes from the scheduling queue that correspond 
 * to the instructions that are currently in state update
 */
void removeAllSUFromSched() {
	int numSUElements = curr_Config->num_r_bus;
	
	for(int i = 0; i < numSUElements; i++) {
		if(sup[i] == NULL)
			continue;
		removeFromSched(sup[i]);
	}
	return;
}

/*
 * Helper function to just remove nodes in the SU from the scheduling queue
 */
void removeFromSched(instr *theInstr) {
	schedule_node *iterator = schedule_head;
	
	// Handle case where the head node node we want to remove
	if(iterator->theInstr == theInstr) {
		assert(iterator->fired == 1);
		assert(iterator->sendToExecute == 1);
		assert(iterator->waiting == 1);
		
		// Move schedule head to next node
		schedule_head = iterator->next;
		
		// Set prev pointer to null
		if(schedule_head != NULL)
			schedule_head->prev = NULL;
		
		// free old head
		free(iterator);
		schedule_size--;
		
		return;
	}
	
	// handle the case where the node to remove is in the middle
	while(iterator->next != NULL) {
		if(iterator->theInstr == theInstr) {
			assert(iterator->fired == 1);
			assert(iterator->sendToExecute == 1);
			assert(iterator->waiting == 1);
			
			// Set previous node's next to iterator's next
			iterator->prev->next = iterator->next;
			// Set next node's prev to iterator's prev
			iterator->next->prev = iterator->prev;
			// free iterator
			free(iterator);
			schedule_size--;
			return;
		}
		iterator = iterator->next;
	}
	
	// Lastly handle the case where the node is at the end
	assert(iterator->theInstr == theInstr); // has to be here...
	assert(iterator->fired == 1);
	assert(iterator->sendToExecute == 1);
	assert(iterator->waiting == 1);
	
	// set the previous node's next to null
	iterator->prev->next = NULL;
	// Free iterator
	free(iterator);
	// lower size
	schedule_size--;
	return;
}

/*
 * This function looks through all of the instructions in all of the FUs, and
 * chooses r of them to mark to be sent to state update in the next cycle. These 
 * r should be the instructions in tag order. So just find the r lowest tagged
 * instructions, and mark them as chosen
 */ 
void setToChosen() {
	// Maybe the easiest way to do this is to just to look at all of the FU's 
	// r times, and each time just mark the min as chosen. On each subsequent 
	// search, don't consider any execution nodes that have already been marked
	// as chosen when trying to find the minimum
	int numPossible = getNumPossible(); // Number of filled FU spots
	int numDesired = curr_Config->num_r_bus; // Max entries that could be chosen
	
	execute_node *minNode;
	while(numPossible > 0 && numDesired > 0) {
		// Note, it's not just who has the lowest tag, but it's also who has
		// been waiting to move on the longest AND THEN who has the lowest tag
		minNode = getMinNode(); 
		minNode->chosen = 1;
		
		numPossible--;
		numDesired--;
	}
	return;
}

/*
 * Helper function to find the number of currently filled entries in all the FUs
 */
int getNumPossible() {
	int numPossible = 0;
	int i;
	for(i = 0; i < curr_Config->k0_size; i++) {
		if(k_0[i] != NULL)
			numPossible++;
	}
	for(i = 0; i < curr_Config->k1_size; i++) {
		if(k_1[i] != NULL)
			numPossible++;
	}
	for(i = 0; i < curr_Config->k2_size; i++) {
		if(k_2[i] != NULL)
			numPossible++;
	}
	return numPossible;
}

/*
 * Helper function to find the min execute node in all the FU's
 */
execute_node *getMinNode() {
	int minTag = INT_MAX;
	int minCycle = findMinCycle();
	execute_node *toReturn;
	int i;
	
	for(i = 0; i < curr_Config->k0_size; i++) {
		if((k_0[i] != NULL) && (k_0[i]->chosen != 1) && (k_0[i]->theInstr->exec <= minCycle)) {
			if(k_0[i]->theInstr->dest_tag < minTag) {
				minTag = k_0[i]->theInstr->dest_tag;
				minCycle = k_0[i]->theInstr->exec;
				toReturn = k_0[i];
			}
		}
	}
	
	for(i = 0; i < curr_Config->k1_size; i++) {
		if((k_1[i] != NULL) && (k_1[i]->chosen != 1) && (k_1[i]->theInstr->exec <= minCycle)) {
			if(k_1[i]->theInstr->dest_tag < minTag) {
				minTag = k_1[i]->theInstr->dest_tag;
				minCycle = k_1[i]->theInstr->exec;
				toReturn = k_1[i];
			}
		}
	}
	
	for(i = 0; i < curr_Config->k2_size; i++) {
		if((k_2[i] != NULL) && (k_2[i]->chosen != 1) && (k_2[i]->theInstr->exec <= minCycle)) {
			if(k_2[i]->theInstr->dest_tag < minTag) {
				minTag = k_2[i]->theInstr->dest_tag;
				minCycle = k_2[i]->theInstr->exec;
				toReturn = k_2[i];
			}
		}
	}
	
	return toReturn;
}

/*
 * Helper function that looks through all of the FU's and finds the min entry 
 * point into the exec stage
 */
int findMinCycle() {
	int minCycle = INT_MAX;
	int i;
	
	for(i = 0; i < curr_Config->k0_size; i++) {
		if((k_0[i] != NULL) && (k_0[i]->chosen != 1) && (k_0[i]->theInstr->exec <= minCycle)) {
			minCycle = k_0[i]->theInstr->exec;
		}
	}
	
	for(i = 0; i < curr_Config->k1_size; i++) {
		if((k_1[i] != NULL) && (k_1[i]->chosen != 1) && (k_1[i]->theInstr->exec <= minCycle)) {
			minCycle = k_1[i]->theInstr->exec;
		}
	}

	for(i = 0; i < curr_Config->k2_size; i++) {
		if((k_2[i] != NULL) && (k_2[i]->chosen != 1) && (k_2[i]->theInstr->exec <= minCycle)) {
			minCycle = k_2[i]->theInstr->exec;
		}
	}
	
	return minCycle;
}

/*
 * This function marks 'fired' instructions in the schedule queue as being eligible
 * for entry into the FU at the start of the next cycle. Essentially, for each 
 * FU it determines the number of currently empty spots as well as the number 
 * of entries that are going to be moving on to state update in the next cycle. 
 * It then assigns instructions in the schedule queue to these FU's in increasing 
 * tag order.
 */
void markForExecution() {
	int k0_spots = numSpotsAvailable('j'); // K0
	int k1_spots = numSpotsAvailable('k'); // K1
	int k2_spots = numSpotsAvailable('l'); // K2
	
	markScheduleEntries(k0_spots, 'j'); // K0
	markScheduleEntries(k1_spots, 'k'); // K1
	markScheduleEntries(k2_spots, 'l'); // K2
}

/*
 * Helper function to find the number of spots in the FU's that are available to
 * move things from the scheduling queue into at the start of the next cycle
 */
int numSpotsAvailable(char FU) {
	execute_node **funcUnit;
	int numEntries;
	int numAvailable = 0;
	
	switch (FU) {
		case 'j':
			funcUnit = k_0;
			numEntries = curr_Config->k0_size;
			break;
		case 'k':
			funcUnit = k_1;
			numEntries = curr_Config->k1_size;
			break;
		case 'l':
			funcUnit = k_2;
			numEntries = curr_Config->k2_size;
			break;
		default:
			break;
	}
	
	for(int i = 0; i < numEntries; i++) {
		if((funcUnit[i] == NULL) || (funcUnit[i]->chosen == 1))
			numAvailable++;
	}
	
	return numAvailable;		
}

/*
 * Helper function that just goes through the scheduling queue and marks certain
 * entries as ready for being sent to execution at the start of the next cycle
 */
void markScheduleEntries(int openSpots, char FU) {
	schedule_node *iterator = schedule_head;
	int FU_1;
	int FU_2;
	
	switch(FU) {
		case 'j': // k_0 (FU Type 0)
			FU_1 = 0;
			FU_2 = 0;
			break;
		case 'k': // k_1 (1 and -1 run on Type 1 FU)
			FU_1 = 1;
			FU_2 = -1; 
			break;
		case 'l': // k_2
			FU_1 = 2;
			FU_2 = 2;
			break;
		default:
			break;
	}
	
	while(iterator != NULL && openSpots > 0) {
		if((iterator->fired == 1) && (iterator->waiting == 0) && (iterator->theInstr->funcUnit == FU_1 || 
			iterator->theInstr->funcUnit == FU_2)) {
				iterator->sendToExecute = 1;
				openSpots--;
		}
		iterator = iterator->next;
	}
	return;
}

void printScheduleQueue() {
	printf("address \t fired \t sendToExecute \t waiting \n");
	schedule_node *iterator = schedule_head;
	while(iterator != NULL) {
		printf("%" PRIx64" \t %d \t %d \t %d \n", iterator->theInstr->address, iterator->fired, 
			iterator->sendToExecute, iterator->waiting);
		iterator = iterator->next;
	}
}

void printFinalQueue() {
	printf("INST\tFETCH\tDISP\tSCHED\tEXEC\tSTATE\n");
	final_node *iterator = final_head;
	int maxInst = 0;
	long maxCycle = 0;
	while(iterator != NULL) {
		if(iterator->dest_tag + 1 > maxInst)
			maxInst = iterator->dest_tag + 1;
		if(iterator->state > maxCycle)
			maxCycle = iterator->state;
		iterator = iterator->next;
	}
	
	// Now just deal with some stats stuff very quickly
	myStats->totalRuntime = maxCycle;
	myStats->predictionAcc = ((float)myStats->totalCorrectBranch)/((float)myStats->totalBranchInstr);
	myStats->avgDispQueue = (myStats->avgDispQueue)/((float)maxCycle);
	myStats->avgInstIssue = ((float)maxInst)/((float)maxCycle);
	myStats->avgInstRet = ((float)maxInst)/((float)maxCycle);
	
	// Back to printing out the results
	final_node **finalArray = (final_node **)malloc(sizeof(final_node *) * maxInst);
	iterator = final_head;
	
	while(iterator != NULL) {
		finalArray[iterator->dest_tag] = iterator;
		iterator = iterator->next;
	}
	
	for(int i = 0; i < maxInst; i++) {
		if(finalArray[i] == NULL)
			continue;
		printf("%d\t%d\t%d\t%d\t%d\t%d\t\n",finalArray[i]->dest_tag+1, 
			finalArray[i]->fetch, finalArray[i]->disp, finalArray[i]->sched,
			finalArray[i]->exec, finalArray[i]->state);
	}
	free(finalArray);
	
	return;
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////// Misc. Functions //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/*
 * This helper function just gets the length of the dispatch queue
 */
void updateDispatchQueueSize() {
	long size = 0;
	dispatch_node *iterator = dispatch_head;
	while(iterator != NULL) {
		size++;
		iterator = iterator->next;
	}
	myStats->avgDispQueue = myStats->avgDispQueue + ((float)size);
	
	if(size > myStats->maxDispQueue)
		myStats->maxDispQueue = size;
	
	return;
}

/*
 * This helper function just returns the stats struct
 */
stats *getStats() {
	return myStats;
}

/*
 * This function just frees our final queue 
 */
void freeFinalQueue() {
	final_node *iterator = final_head;
	while(iterator != NULL) {
		final_node *temp = iterator;
		iterator = iterator->next;
		free(temp);
	}
}




