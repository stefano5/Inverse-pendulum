#ifndef TASK_COMMUNICATION_HPP
#define TASK_COMMUNICATION_HPP

#include "../ErrorsHandle/ProgrammingErrors.hpp"
#include "Message.hpp"
#include <stdint.h>

class TaskCommunication {
public:

    constexpr static uint8_t TOT_MEX = 30;  // Maximum available number of messages
    
    TaskCommunication();
    
    void sendMessage(uint8_t idSender, uint8_t idReceiver, uint8_t messageType, uint32_t messageBody);
    void sendMessage(Message newMessage);
   

protected:
    void getLastMessage(uint8_t *idSender, uint8_t *idReceiver, uint8_t *messageType, uint32_t *messageBody);
    virtual bool newMessageArrived();

private:
    
    Message messages[TOT_MEX];
    uint8_t firstMsg;
    uint8_t lastMsg;
};


#endif
