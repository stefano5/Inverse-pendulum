#include "Scheduler.hpp"

Scheduler* Scheduler::ref = nullptr;

Scheduler::Scheduler() {
	FATAL_ERROR_IF_TRUE(ref != nullptr);	// There can be only one instance of this class.

	ref = this;
    counter_et = 0u;
    n_requested_task = 0u;
    endTR = 0u;
    inTR = 0u;
}

void Scheduler::setup() {
    ///// Called at startup, just before Scheduler::runTasks().
    
    Task::schedulerPtr = this;

    // Inserting tasks with high priority.
    for (uint8_t i=0; i<Task::getRunningTask(); ++i) {
        if (Task::getPointer(i)->getPriority() == Task::Priority_t::high) {
            taskInPriorityOrder[n_requested_task++] = Task::getPointer(i);
        }
    }

    // Inserting tasks with medium priority.
    for (uint8_t i=0; i<Task::getRunningTask(); ++i) {
        if (Task::getPointer(i)->getPriority() == Task::Priority_t::medium) {
            taskInPriorityOrder[n_requested_task++] = Task::getPointer(i);
        }
    }

    // Inserting tasks with low priority.
    for (uint8_t i=0; i<Task::getRunningTask(); ++i) {
        if (Task::getPointer(i)->getPriority() == Task::Priority_t::low) {
            taskInPriorityOrder[n_requested_task++] = Task::getPointer(i);
        }
    }

    // Call the task initialization functions
    for (uint8_t i=0; i<n_requested_task; ++i) {

        taskInPriorityOrder[i]->initialize();
        taskToBeRun[i] = nullptr;
    }
    
    registerFaults();
}


void Scheduler::runTasks(uint8_t *flag_one_ms_elapsed) {
    ///// It is continuously called by the microcontroller immediately after Scheduler::setup().
    if (*flag_one_ms_elapsed == 1u) {
        *flag_one_ms_elapsed = 0u;
        counter_et++;

        if ((counter_et % (uint16_t)Task::Period_t::T_10ms) == 0) {
            getHighestPriorityTask((uint16_t)Task::Period_t::T_10ms);
        }
        
        if ((counter_et >= (uint16_t)Task::Period_t::T_20ms) && ((counter_et % (uint16_t)Task::Period_t::T_20ms) == 0)) {
            getHighestPriorityTask((uint16_t)Task::Period_t::T_20ms);
        }
        
        if ((counter_et >= (uint16_t)Task::Period_t::T_30ms) && ((counter_et % (uint16_t)Task::Period_t::T_30ms) == 0)) {
            getHighestPriorityTask((uint16_t)Task::Period_t::T_30ms);
        }  
        
        if ((counter_et >= (uint16_t)Task::Period_t::T_50ms) && ((counter_et % (uint16_t)Task::Period_t::T_50ms) == 0)) {
            getHighestPriorityTask((uint16_t)Task::Period_t::T_50ms);
        }  
        
        if ((counter_et >= (uint16_t)Task::Period_t::T_100ms) && ((counter_et % (uint16_t)Task::Period_t::T_100ms) == 0)) {
            getHighestPriorityTask((uint16_t)Task::Period_t::T_100ms);
        }
        
        if ((counter_et >= (uint16_t)Task::Period_t::T_200ms) && ((counter_et % (uint16_t)Task::Period_t::T_200ms) == 0)) {
            getHighestPriorityTask((uint16_t)Task::Period_t::T_200ms);
        }  
        
        if ((counter_et >= (uint16_t)Task::Period_t::T_300ms) && ((counter_et % (uint16_t)Task::Period_t::T_300ms) == 0)) {
            getHighestPriorityTask((uint16_t)Task::Period_t::T_300ms);
        }  
        
        if ((counter_et >= (uint16_t)Task::Period_t::T_500ms) && ((counter_et % (uint16_t)Task::Period_t::T_500ms) == 0)) {
            getHighestPriorityTask((uint16_t)Task::Period_t::T_500ms);
        }
        
        if ((counter_et >= (uint16_t)Task::Period_t::T_1s) && ((counter_et % (uint16_t)Task::Period_t::T_1s) == 0)) {
        	getHighestPriorityTask((uint16_t)Task::Period_t::T_1s);
            counter_et = 0;
        }
    }

    if (!runQueueTasks()) {
        // When the scheduler has executed all the requested tasks, it manages messages during the idle time.
        handleMessages();
    }
}

void Scheduler::getHighestPriorityTask(uint16_t prefixedPerod) {
    for (uint8_t i=0; i<n_requested_task; ++i) {
        // Searches for the highest priority task to execute.
        uint16_t taskT = (uint16_t)taskInPriorityOrder[i]->getPeriod();
        uint16_t paramT = (uint16_t)prefixedPerod;

        if (paramT == taskT) {
        	FATAL_ERROR_IF_TRUE(endTR == TOT_TASKS);	// If the scheduler cannot empty the task queue, there might be a task slowing down the process, but not enough to trigger the Watchdog Timer (wwdg).

        	// Insert the i-th task into the queue.
        	if (!taskInPriorityOrder[i]->isRunEnable()) continue;	// Skip the task if it is disabled.

            taskToBeRun[endTR] = taskInPriorityOrder[i];	// Insert the task into the execution queue.

            if (endTR + 1 > TOT_TASKS) {
                endTR=0;
            } else {
                endTR++;
            }
        }
    }
}

bool Scheduler::runQueueTasks() {
    if (inTR + 1 > endTR) {
        // end
        inTR = endTR = 0;
        // Execution queue completed.
        return FALSE;
    } else {
        // Empty the task execution queue.
        ASSERT_PTR(taskToBeRun[inTR]);
        taskToBeRun[inTR]->run();
        taskToBeRun[inTR] = nullptr;
        inTR++;
        return TRUE;
    }
}



void Scheduler::handleMessages() {
    if (newMessageArrived()) {
        uint8_t idSender=0u;
        uint8_t idReceiver=0u;
        uint8_t messageType=0u;
        uint32_t messageBody=0u;

        getLastMessage(&idSender, &idReceiver, &messageType, &messageBody);
        Task* receiverTask = Task::getPointerFromNameTask((NameTask)idReceiver);
        ASSERT_PTR(receiverTask);	// Trying to send a message to a non-existent task.

        receiverTask->handleMessage(idSender, messageType, messageBody);
    }
}


