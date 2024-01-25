#ifndef TASK_HPP__
#define TASK_HPP__

#include "../ErrorsHandle/ProgrammingErrors.hpp"
#include <string.h>
#include <stdint.h>
#include "../GlobalDependacies.hpp"
#include "Scheduler.hpp"
#include <string.h>

class Scheduler;

#define NAME_PRIORITY(A) ( A==Priority_t::high?"high":(A==Priority_t::medium?"medium":"low") )

/*
 * @brief   Structure of a task, to be extended to create a task.
 * @details Extend the class by implementing the two virtual methods. The task is automatically started by the scheduler.
 *          The task is automatically registered with the scheduler when the instance is created in the main function.
 *              Note: Instantiate the derived class as a static class.
 * */
class Task {
public:
    enum class Priority_t : uint8_t {
        low=0,
        medium,
        high
    };

    enum class Period_t : uint16_t {
        T_10ms = 10,
        T_20ms = 20,
        T_30ms = 30,
        T_50ms = 50,
        T_100ms = 100,
        T_200ms = 200,
        T_300ms = 300,
        T_500ms = 500,
        T_1s = 1000,
    };

    Task(const char* name, uint8_t taskId, Period_t period, Priority_t priority);

    /*
     * @brief	Method of a task called only once at the system startup.
     * 			Each task implements it according to its needs.
     * */
    virtual void initialize() = 0;    // Method to be implemented in derived classes; this method will be called once after the initialization of peripherals.

    /*
     * @brief	Cyclic method of a task, called at the desired period by the scheduler as long as the microcontroller is powered.
     * 			Each task implements it according to its needs.
     * @warning	This method can be disabled by calling disableRunTask(). In this case, the scheduler will no longer consider that specific task.
     * 			To re-enable the task, call enableRunTask().
     * 			Note: A task with the run() method disabled can still receive messages.
     */
    virtual void run() = 0;           // Method to be implemented in derived classes; this method will be called according to the chosen period, after all initialization methods have been called.

    /*
     * @brief 	Method called by the scheduler to deliver a message from one task to another.
     * 			Each task implements it according to its needs, if necessary.
     * @param[taskSender]	The ID of the task that sent the message using sendTask().
     * @param[messageType]	The type of the message, used, for example, to differentiate between various messages a task can receive.
     * @param[messageBody]	The body of the message; once the message type is determined, an additional parameter may be needed.
     *
     * @warning		It is recommended to implement messageType as enum classes declared within tasks that need to receive that type of message.
     */
    virtual void handleMessage(uint8_t taskSender, uint8_t messageType, uint32_t messageBody);

    /*
     * @brief	Method that a task calls when it wants to send a message to another task.
     * @param[idReceiver]	ID of the task to which the message should be sent.
     * @param[messageType]	The type of the message.
     * @param[messageBody]	The body of the message.
     * */
    void sendMessage(uint8_t idReceiver, uint8_t messageType, uint32_t messageBody=0);

    /*
     * @brief	Returns the information about the task; it's a kind of toString() for the task.
     * */
    const char* getInfoTask();

    inline Period_t getPeriod() { return period; }
    inline Priority_t getPriority() { return priority; }
    inline const char* getTaskName() { return name; }
    inline Task* getTask(uint8_t id) { return task[id]; }

    /*
     * @brief Returns the address of the task with the id-th index, in the order of instantiation in the main function.
     */
    inline static Task* getPointer(uint8_t id) { return task[id]; }

    /*
     * @brief Returns the pointer of the task with the specified id if it has been instantiated.
     * @return nullptr if the task has not been instantiated.
     * */
    static Task* getPointerFromNameTask(NameTask id);

    /*
     * @return Returns the numerical identifier of the task (unique).
     * */
    inline uint8_t getTaskId() { return taskId; }

    /*
     * @return 	TRUE if the run() method will be called by the scheduler,
     * 			FALSE otherwise.
     * */
    inline bool isRunEnable() { return isRunActive; }

    /*
     * @brief	Activates the scheduling of the task, allowing the scheduler to execute the run() method.
     * @warning	The initialize() method will always be called regardless.
     * */
    inline void enableRunTask() { isRunActive = TRUE; }

    /*
     * @brief	Deactivates the scheduling of the task, preventing the scheduler from executing the run() method.
     * @warning	The initialize() method will always be called regardless.
     * */
    inline void disableRunTask() { isRunActive = FALSE; }

    /*
     * @brief	Asks the SerialManager to send a message to the Task calling this function when the user sends the 'cmd' character via serial.
     * 			When the user presses that character, the task that requested registration will receive a message of type Messages_t::INCOMING_CMD_FROM_SERIAL
     * 			with the body set to uniqueIdMsg, if provided; otherwise, it will be set to cmd.
     * @param[cmd] 	Character sent by the user via serial.
     * @param[uniqueIdMsg]	Body of the message that the task will receive in handleMessage(..,..,..,messageBody=uniqueIdMsg).
     * 							If not provided, it will be set to cmd.
     *
     * Note: This method uses the virtual method registerTask() implemented by SerialManager through the CmdNotification class. 
     *          If SerialManager is not present, this functionality cannot be used.
     *
     * */
    void requestMessageIfCmdIsReceived(char cmd, uint32_t mapCmd);
    void requestMessageIfCmdIsReceived(char cmd);

	virtual void registerTask(uint8_t idRegTask, char cmd, uint32_t uniqueIdMsg);

    ~Task() { FATAL_ERROR; }	// Tasks cannot be deleted.

protected:

private:

    friend class Scheduler;

    constexpr static uint8_t LEN_NAME = 32u;   // Size of the string containing the task name
    constexpr static uint8_t LEN_INFO_BUF = 100;	// Size of the string where task information is stored

    Task() { FATAL_ERROR; } // Do not use, the public constructor must be used to enable automatic task management by the scheduler

    // Returns the address of this task
    Task* getPointer(){ return this; }


    // Returns the number of created tasks
    static uint8_t getRunningTask() { return uid; }

    char name[LEN_NAME];    // Name of the task, if needed
    uint8_t taskId;         // Numerical identifier of the task (unique)
    Priority_t priority;    // Task priority
    Period_t period;        // Task period
    bool isRunActive;
    char infoBuf[LEN_INFO_BUF];

    static uint8_t uid;             // Number of tasks in execution
    static Task* task[TOT_TASKS];   // Array where task pointers are stored, and the tasks stored here are the ones to be executed
    static Scheduler* schedulerPtr; // Pointer to the Scheduler class
};



#endif
