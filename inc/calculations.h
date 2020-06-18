/*******************************************************************/ /*!
*    \file       calculations.h
*    \date       Created: 2020-06-17
*    \author     Karina Avalos <avaloskarina04@gmail.com>
***********************************************************************/

#ifndef CALCULATIONS_H
#define CALCULATIONS_H

//=======================================================================
//=  MODULE CONFIGURATION INCLUDE FILE
//=    This may include any project/hardware-specific configuration files

//=======================================================================
//=  LIBRARY/OTHER INCLUDE FILES

//=======================================================================
//=  SYSTEM/STANDARD INCLUDE FILES
//=    The list of included files should be restricted to the minimum set of files
//=    needed to resolve all symbols contained in this file (i.e., include files
//=    should be "free standing" and if compiled by themselves should be errorless
//=    and warning free).
//=    E.g. #include <stdbool.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

//=======================================================================
//=  PUBLIC OBJECT-LIKE/FUNCTION-LIKE MACROS
//=    E.g. #define BUFFER_SIZE 1024
//=    E.g. #define SQR(s)  ((s) * (s))
#define MAX_ARRAY_LEN 128 // max size of buffer
//=======================================================================
//=  PUBLIC TYPE DEFINITIONS
//=    E.g. Typedef accessible for all .c files that include this header file
//=    typedef struct
//=    {
//=       uint16_t count;
//=    } timer_t;

//=======================================================================
//=  PUBLIC DECLARATIONS
//=    Declaration of public global constants, variables, enums and types using g_ prefix
//=    E.g. extern volatile int8_t g_error_variable;

//=======================================================================
//=  PUBLIC PROTOTYPES
//=    Declaration of public function prototypes with Doxygen-style comments (//!)

//! Calculates the min of an array
uint16_t find_min(uint32_t * const p_array);

//! Calculates the max of an arry
uint16_t find_max(uint32_t * const p_array);

//! Returns the index at which max occurs in an array
uint16_t find_max_index(uint32_t * const p_array);

//! Returns the average of an array
uint16_t calc_average(uint32_t * const p_array);

//! Prints the average value of the array in hexadecimal format
void print_avg_hex(uint32_t * const p_array);