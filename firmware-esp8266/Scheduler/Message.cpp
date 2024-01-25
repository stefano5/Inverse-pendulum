#include "Message.hpp"



void Message::make(uint8_t idSender, uint8_t idReceiver, uint8_t messageType, uint32_t messageBody) {
    erase();
    this->idSender = idSender;
    this->idReceiver = idReceiver;
    this->messageType = messageType;
    this->messageBody = messageBody;
}

void Message::erase() {
    this->idSender = this->idReceiver = this->messageType = 0xff;
    this->messageBody = 0xffff;
}


bool Message::isErased() {
    // If true, it means that the message has been read, and therefore its content has been cleared.
    return idSender == 0xff && idReceiver == 0xff && messageType == 0xff && messageBody == 0xffff;
}


