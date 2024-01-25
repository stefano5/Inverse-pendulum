#include "TaskCommunication.hpp"


TaskCommunication::TaskCommunication() {
    for (uint8_t i=0; i<TOT_MEX; ++i) {
        messages[i].erase();
    }
    firstMsg = lastMsg = 0u;
}

void TaskCommunication::sendMessage(Message newMessage) {
    sendMessage(
            newMessage.getIdSender(),
            newMessage.getIdReceiver(),
            newMessage.getMessageType(),
            newMessage.getMessageBody()
        );
}

void TaskCommunication::sendMessage(uint8_t idSender, uint8_t idReceiver, uint8_t messageType, uint32_t messageBody) {
    // ASSERT(firstMsg <= lastMsg);            // Reached the maximum limit of available messages; the scheduler cannot handle it all in time.
    ASSERT(messages[lastMsg].isErased());   // The scheduler did not have time to send the previous messages and is overlapping them.
    
    messages[lastMsg].make(idSender, idReceiver, messageType, messageBody);

    lastMsg = (lastMsg + 1) % TOT_MEX;
}

bool TaskCommunication::newMessageArrived() {
    return lastMsg != firstMsg; // If they are misaligned, it's because a new message has arrived.
}


void TaskCommunication::getLastMessage(uint8_t *idSender, uint8_t *idReceiver, uint8_t *messageType, uint32_t *messageBody) {
    if (!newMessageArrived()) return;

    if (idSender != nullptr) {
        *idSender = messages[firstMsg].getIdSender();
    }

    if (idReceiver != nullptr) {
        *idReceiver = messages[firstMsg].getIdReceiver();
    }

    if (messageType != nullptr) {
        *messageType = messages[firstMsg].getMessageType();
    }

    if (messageBody != nullptr) {
        *messageBody = messages[firstMsg].getMessageBody();
    }

    messages[firstMsg].erase();

    firstMsg = (firstMsg + 1) % TOT_MEX;
}


