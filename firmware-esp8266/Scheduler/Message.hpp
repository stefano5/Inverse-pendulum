#ifndef TASK_MESSAGE_HPP
#define TASK_MESSAGE_HPP

#include <stdint.h>



class Message {
public:
    Message() : idSender(0xff), idReceiver(0xff), messageType(0xff), messageBody(0xffff) {}

    void make(uint8_t idSender, uint8_t idReceiver, uint8_t messageType, uint32_t messageBody);
    void erase();

    bool isErased();
    
    inline uint8_t getIdSender() { return idSender; }
    inline uint8_t getIdReceiver() { return idReceiver; }
    inline uint8_t getMessageType() { return messageType; }
    inline uint32_t getMessageBody() { return messageBody; }


private:
    uint8_t idSender;
    uint8_t idReceiver;
    uint8_t messageType;
    uint32_t messageBody;
};


#endif
