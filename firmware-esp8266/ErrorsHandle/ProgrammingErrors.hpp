/*
 * ProgrammingErrors.hpp
 *
 *  Created on: Apr 18, 2023
 *      Author: Maugeri
 */

#ifndef ERRORSHANDLE_PROGRAMMINGERRORS_HPP_
#define ERRORSHANDLE_PROGRAMMINGERRORS_HPP_

#include <string.h>
#include "../Serial/TerminalManipulator.hpp"
#include "../GlobalDependacies.hpp"

#define RESET_IF_FATAL_ERROR false

#define FATAL_ERROR_D(errorDescriptionStr) fatalError(errorDescriptionStr, __PRETTY_FUNCTION__, __FILE__, __LINE__)
#define FATAL_ERROR fatalError(nullptr, __PRETTY_FUNCTION__, __FILE__, __LINE__)
#define ASSERT(A) if ((A) == FALSE) FATAL_ERROR
#define ASSERT_PTR(A) if ((A) == nullptr) FATAL_ERROR
#define ASSERT_EQUAL(A, B) if ((A) == (B)) FATAL_ERROR
#define ASSERT_NOT_EQUAL(A, B) if ((A) != (B)) FATAL_ERROR
#define ASSERT_GREATER(A, B) if ((A) <= (B)) FATAL_ERROR
#define ASSERT_GREATER_EQUALS(A, B) if ((A) < (B)) FATAL_ERROR
#define ASSERT_STR(str, itsSize) if (strlen(str) > (itsSize)) FATAL_ERROR

#define FATAL_ERROR_IF_TRUE(A) if ((A) == TRUE) fatalError("Condition failed", __PRETTY_FUNCTION__, __FILE__, __LINE__)
#define FATAL_ERROR_IF_NOT_TRUE(A) if ((A) == FALSE) fatalError("Condition failed", __PRETTY_FUNCTION__, __FILE__, __LINE__)
#define FATAL_ERROR_IF_FALSE(A) if ((A) == FALSE) fatalError("Condition failed", __PRETTY_FUNCTION__, __FILE__, __LINE__)

#define NOT_USED(A) (void)A

extern void fatalError(const char* errorType, const char* funcName, const char* fileName, uint32_t lineNumber);
extern void forceReInitSerial(void);
extern void __fastPrintStr(const char* str);
extern uint16_t __itoa(uint32_t value, char *ptr);
extern void __fastPrintInt(uint32_t val);
extern void hardResetBoard();




#endif /* ERRORSHANDLE_PROGRAMMINGERRORS_HPP_ */
