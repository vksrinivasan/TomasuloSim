#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include "procsim.h"
#include "assert.h"

void print_help_and_exit(void) {
    printf("procsim [OPTIONS] < traces/file.trace\n");
    printf("  -r R\t\tNumber of result buses");
    printf("  -f F\t\tFetch Rate");
    printf("  -j J\t\tNumber of k_0 fu's\n");
    printf("  -k K\t\tNumber of k_1 fu's\n");
	printf("  -l L\t\tNumber of k_2 fu's\n");
    printf("  -i I\t\t tracefileName\n");
    exit(0);
}

// Create a struct of the instruction data from what was just read by the file
instr *createInstruction(uint64_t address, int fu, int dest, int src1, int src2, 
	int src1_tag, int src2_tag, int tag, int clock, int branch, int taken, int correct, 
	int resolved);
// add the instruction data to a list that will be moved to dispatch in the next
// cycle
if_listnode *addToFetchQueue(if_listnode **fetchQueue, if_listnode *fetchQueueTail, instr *currInstr);
// Just print the stats struct
void printStats();


int main(int argc, char* argv[]) {
    int opt;
    int r = DEFAULT_R;
    int f = DEFAULT_F;
    int k_0 = DEFAULT_J;
	int k_1 = DEFAULT_K;
	int k_2 = DEFAULT_L;
    FILE* fin  = stdin;

    /* Read arguments */ 
    while(-1 != (opt = getopt(argc, argv, "r:f:j:k:l:i:h"))) {
        switch(opt) {
            case 'r':
                r = atoi(optarg);
                break;
            case 'f':
                f = atoi(optarg);
                break;
            case 'j':
                k_0 = atoi(optarg);
                break;
            case 'k':
                k_1 = atoi(optarg);
                break;
            case 'l':
                k_2 = atoi(optarg);
                break;
            case 'i':
                fin = fopen(optarg, "r");
                break;
            case 'h':
            default:
                print_help_and_exit();
                break;
        }
    }

	// Just print out the processor settings
    printf("Processor Settings\n");
    printf("R: %d\n", r);
    printf("k0: %d\n", k_0);
    printf("k1: %d\n", k_1);
    printf("k2: %d\n", k_2);
    printf("F: %d\n", f);
    printf("\n");
	
	// Setup the processor
	proc_init(128, k_0, k_1, k_2, r, f); // Assume 128 registers [0,...,127]
	
	// Now just create an instruction fetch stage list. All the other stages 
	// are taken care of by procsim.c, but since we read the file here, it 
	// makes sense to just have a structure here with which we can just store 
	// the fetched instructions in
	if_listnode **fetchQueue;
	if_listnode *fetchQueueTail = NULL;
	fetchQueue = (if_listnode **)malloc(sizeof(if_listnode *) * 1);
	if(fetchQueue == NULL)
		return -1;

    /* Begin reading the file */ 
    uint64_t address;
	int fu_type;
	int dest_reg;
	int src_1;
	int src_2;
	int tag = 0;
	int clock = 1;
	int branch = 0;
	int taken = -1;
	int correct = -1;
	int resolved = -1;
	
	int totalMarked = 0; // This is used by the dispatch queue functions
	dispatch_node *dispatch_head;
	schedule_node *schedule_head;
	
	
    while (1) { 

		////////////////////////////////////////////////////////////////////////
		/*
		 * Check if this simulation should even continue or if all the instructions
		 * are done processing. If all queues are empty, we don't have to keep going
		 */
		////////////////////////////////////////////////////////////////////////
		int isStateArrayEmpty = stateEmpty();
		dispatch_head = getDispHead();
		schedule_head = getScheduleHead();
		if((fetchQueue[0] == NULL) && (dispatch_head == NULL) && (schedule_head == NULL) && 
			(clock > 1) && (isStateArrayEmpty == 1)) {
			goto finish;
		}
	
		////////////////////////////////////////////////////////////////////////
		/*
		 * First move everything from one stage to another
		 */
		////////////////////////////////////////////////////////////////////////
		sendToFinal(); // State update to Final Queue
		sendToSU(clock); // Exec to State Update
		resolveBranches(); // Check the instructions that were just moved and resolve in tag order
		moveToExecute(clock); // Scheduling Queue to Execute
		dispatchToSchedule(clock, totalMarked); // Dispatch Queue to Schedule Queue
		dispatch_Enqueue(fetchQueue, clock); // Fetch Queue to Dispatch Queue
		// Then file trace to fetch queue
        for(int i = 0; i < f; i++) {
			if(!feof(fin)) {
				char a[50];
				int b[50];
				int j = 0;
				fgets(a, sizeof(a), fin);
				char *p = strtok(a, " ");
				int ret = 0;
				while(p != NULL)  {
					ret++;
					int d;
					if(ret == 1 || ret == 6) {
						d = strtol(p, NULL, 16);
					} else {
						d = atoi(p);
					}
					b[j++] = d;
					p = strtok(NULL," ");
				}
				
				address = (uint64_t)b[0];
				fu_type = b[1];
				dest_reg = b[2];
				src_1 = b[3];
				src_2 = b[4];
				
				if(ret == 5) {
					branch = 0;
					taken  = -1;
					correct = -1;
					resolved = -1;
				} else if(ret == 7) {
					branch = 1;
					taken = b[6];
					correct = -1; // Will be determined when it goes to dispatch
					resolved = 0; // Will be 0 since it's resolved later
				} else {
					continue;
				}
					
				// First create/pop ulate an instruction struct
				instr *tempInstr = createInstruction(address, fu_type, dest_reg, 
					src_1, src_2, -5, -5, tag, clock, branch, taken, correct, resolved);
					
				// then add the instruction to an 'instruction queue'. Just a 
				// holding cell for instructions before the next cycle when 
				// they can go to dispatch
				fetchQueueTail = addToFetchQueue(fetchQueue, fetchQueueTail, tempInstr);
				tag++;
				
			}
		}
		
		////////////////////////////////////////////////////////////////////////
		/*
		 * Then just update our stats for the dispatch queue
		 */ 
		////////////////////////////////////////////////////////////////////////
		updateDispatchQueueSize();
		
		////////////////////////////////////////////////////////////////////////
		/*
		 * Then we do what needs to happen during the clock cycle
		 */
		////////////////////////////////////////////////////////////////////////
		writeToRegFile(); // Write whatever is in state update to register file
		setToFired(); // Independent Instructions are marked to fire
		totalMarked = reserveScheduleSpots(); // Dispatch queue reserve spots in scheduling queue
		readUpdateRegFile(totalMarked); // Dispatch queue reads register file to instr that will be sent at start of next cycle
		broadcastToSched(); // Update waiting schedule queue nodes via broadcast from state update
		removeAllSUFromSched(); // State update deletes finished nodes from schedule queue
		
		
		
		////////////////////////////////////////////////////////////////////////
		/*
		 * Mark instructions in the FU's and Scheduling queue's as ready
		 * to be moved at the start of the next cycle
		 */
		////////////////////////////////////////////////////////////////////////
		setToChosen(); // Mark instructions in FUs as ready to move to SU
		markForExecution(); // Mark instructions in scheduling queue to move to Exec
		
		// Lastly Update Clock
		clock++;
    }
    finish: fclose(fin);
	printFinalQueue();
	freeFinalQueue();
	printf("\n");
	printStats();
	
    return 0;
}

instr *createInstruction(uint64_t address, int fu, int dest, int src1, int src2, 
	int src1_tag, int src2_tag, int tag, int clock, int branch, int taken, int correct, 
	int resolved) {
	
	instr *tempInstr = malloc(sizeof(instr)*1);
	if(tempInstr == NULL) 
		return NULL;
	tempInstr->address = address;
	tempInstr->funcUnit = fu;
	tempInstr->destReg = dest;
	tempInstr->dest_tag = tag;
	
	tempInstr->source1 = src1;
	tempInstr->source1_tag = src1_tag; // -5 is just a placeholder
	tempInstr->source1_ready = 0; // We'll find out when dispatch reads from reg file
	
	tempInstr->source2 = src2;
	tempInstr->source2_tag = src2_tag; // -5 is just a placeholder
	tempInstr->source2_ready = 0; // We'll find out when dispatch reads from reg file
	
	tempInstr->branch = branch;
	tempInstr->taken = taken;
	tempInstr->correct_pred = correct;
	tempInstr->resolved = resolved;
	
	// These are just the clock cycles where things happen. We can set fetch to 
	// the current cycle
	tempInstr->fetch = clock;
	tempInstr->disp = 0;
	tempInstr->sched = 0;
	tempInstr->exec = 0;
	tempInstr->state = 0;

	return tempInstr;
}

if_listnode *addToFetchQueue(if_listnode **fetchQueue, if_listnode *fetchQueueTail, instr *currInstr) {
	if_listnode *newNode = malloc(sizeof(if_listnode)*1);
	if(newNode == NULL)
		return NULL;
	
	newNode->theInstr = currInstr;
	newNode->next = NULL;
	
	if(fetchQueue[0] == NULL) {
		fetchQueue[0] = newNode;
		return newNode;
	} else {
		fetchQueueTail->next = newNode;
		return newNode;
	}
	
}

void printStats() {
	stats *myStats = getStats();
	//printf("%f\n", myStats->avgInstRet); -- For experiments
	//printf("%f\n", myStats->predictionAcc); -- For experiments
 	printf("Processor stats:\n");
	printf("Total branch instructions: %lu\n", myStats->totalBranchInstr);
	printf("Total correct predicted branch instructions: %lu\n", myStats->totalCorrectBranch);
	printf("prediction accuracy: %f\n", myStats->predictionAcc);
	printf("Avg Dispatch queue size: %f\n", myStats->avgDispQueue);
	printf("Maximum Dispatch queue size: %lu\n", myStats->maxDispQueue);
	printf("Avg inst Issue per cycle: %f\n", myStats->avgInstIssue);
	printf("Avg inst retired per cycle: %f\n", myStats->avgInstRet);
	printf("Total run time (cycles): %lu\n", myStats->totalRuntime); 
	free(myStats);
}