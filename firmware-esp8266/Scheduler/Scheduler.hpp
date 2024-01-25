#ifndef SCHEDULER_HPP__
#define SCHEDULER_HPP__

#include "Task.hpp"
#include "TaskCommunication.hpp"
#include "FaultManager.cpp"


class Task;

/*
 * @brief   Manages all objects implementing the Task class.
 * @details The scheduler assumes that no task is preemptive (so it doesn't make sense to talk about preemption).
 *          In scheduling, it considers (in this order): task period, task priority.
 *          If two tasks are defined as follows:
 *              Task A: T=10ms; P=low
 *              Task B: T=100ms; P=high
 *          The scheduling order will be:
 *              A   A   A   A   A   A   A   A   A   A,B
 *           0  10  20  30  40  50  60  70  80  90  100     time in ms
 *                                                  ^= in this case, the scheduler gives priority to task A, and task B will be executed in the next clock cycle.
 *
 *          Instantiate the class as a static object.
 * */
class Scheduler : public TaskCommunication, protected FaultsManager {
public:

    /*
     *  @breif  Instantiate the object as a global entity.
     * */
    Scheduler();

    /*
     *  @brief  Method called only once at startup, after the initialization of peripherals.
     *
     * */
    void setup();

    /*
     *  @brief  Method called continuously after the setup() method.
     *  @param  This method needs to know when 1 ms has elapsed. The parameter should be 1 when 1 ms has passed, and 0 otherwise.
     *          The method itself will reset the flag to 0.
     * */
    void runTasks(uint8_t *flag_one_ms_elapsed);

protected:
private:

    /*
     *  @brief  Method called when all periodic tasks have completed.
     *          Message handling has the lowest priority compared to all other tasks.
     *
     * */
    void handleMessages();

    /*
     *  @brief  Enqueues tasks in the order in which they should be executed.
     *
     * */
    void getHighestPriorityTask(uint16_t prefixedPerod);


    /*
     *
     * @brief  Empties the task queue ordered by the "getHighestPriorityTask()" method by executing one task per clock cycle.
     *
     * */
    bool runQueueTasks();


    Task* taskInPriorityOrder[TOT_TASKS];   // Array in which user-defined tasks are inserted, ordered by increasing priority.
    Task* taskToBeRun[TOT_TASKS];           // Array in which upcoming tasks to be executed are inserted in order.

    uint16_t n_requested_task;      // Number of tasks declared by the user (a number less than TOT_TASKS).
    uint16_t counter_et;            // Counter in milliseconds.
    uint8_t inTR;                   // Indices of the task queue to be executed, this indicates the first element.
    uint8_t endTR;                  // Indices of the task queue to be executed, this indicates the last element.
    static Scheduler* ref;
};


#endif

