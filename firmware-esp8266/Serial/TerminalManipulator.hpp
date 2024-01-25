/*
 * TerminalManipulator.hpp
 *
 *  Created on: Apr 19, 2023
 *      Author: Maugeri
 */

#ifndef SERIAL_TERMINALMANIPULATOR_HPP_
#define SERIAL_TERMINALMANIPULATOR_HPP_


#define move_to(x,y) 	"0\33["<<y<<";"<<x<<"H "
#define move_to_home 	"\033[;H"
#define clear_screen 	"\033[2J "
#define endl			"\033[B \r\n"
//#define endl			"\033[B \n"
#define bold			"\033[1m"		// aumenta dim carattere
#define faint			"\033[2m"		// riduci dim carattere
#define underline		"\033[4m"
#define italic			"\033[3m"
#define reset_att		"\033[0m"


#define red				"187;0;0"
#define green			"0;187;0"
#define blue			"0;0;187"
#define black			"0;0;0"
#define yellow			"187;187;0"
#define magenta			"187;0;187"
#define cyan 			"0;187;187"
#define white			"187:187:187"

#define set_foreground(_DES_COLOR_)	(const char*)"\033[38;2;" _DES_COLOR_ "m"
#define set_background(_DES_COLOR_)	(const char*)"\033[48;2;"_DES_COLOR_"m"




#endif /* SERIAL_TERMINALMANIPULATOR_HPP_ */
