/*
 * GlobalDependacies.hpp
 *
 *  Created on: Apr 18, 2023
 *      Author: Maugeri
 */

#ifndef GLOBALDEPENDACIES_HPP_
#define GLOBALDEPENDACIES_HPP_

#define FALSE (1u==0u)
#define TRUE (1u==1u)


////// Add here the messageType that tasks exchange, ensuring they are unique across all tasks.
enum class GlobalMessages_t : uint8_t {
	INCOMING_CMD_FROM_SERIAL=60,// Reserved message
};

enum class NameTask {
    exampleTask=0,
    SerialManager,
    Control,
    Estimation,
};

#define TOT_TASKS 8
// There must be as many nullptrs below as the maximum number of tasks.
#define SET_ALL_ITEMS_TO_NULL {nullptr, nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr}




#endif /* GLOBALDEPENDACIES_HPP_ */
