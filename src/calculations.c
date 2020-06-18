/*******************************************************************/ /*!
*    \file       calculations.c
*    \brief      See calculations.h for the description
*    \date       Created: 2020-06-17
*    \author     Karina Avalos <avaloskarina04@gmail.com>
***********************************************************************/

//=======================================================================
//=  MODULE INCLUDE FILE
#include "calculations.h"

//=======================================================================
//=  LIBRARY/OTHER INCLUDE FILES
#include "uartstdio.h"

//=======================================================================
//=  SYSTEM/STANDARD INCLUDE FILES
//=    E.g. #include <stdbool.h>
#include <math.h>

//=======================================================================
//=  PRIVATE OBJECT-LIKE/FUNCTION-LIKE MACROS
//=    E.g. #define BUFFER_SIZE 1024
//=    E.g. #define SQR(s)  ((s) * (s))

//=======================================================================
//=  GLOBAL VARIABLES (extern)
//=    Declared with keyword `extern` in the .h file.
//=    E.g. int8_t g_error_variable;

//=======================================================================
//=  PRIVATE TYPE DEFINITIONS
//=    E.g. typedef private to this source file
//=    typedef struct
//=    {
//=       uint16_t count;
//=    } timer_t;

//=======================================================================
//=  PRIVATE VARIABLE DEFINITIONS (static)
//=    Definition of private constants, variables and structures at file scope
//=    E.g. static uint8_t variable_name;
//=    E.g. static const uint8_t variable_name_k;

//=======================================================================
//=  PRIVATE FUNCTION DECLARATIONS (static)
//=    Local function prototypes
//=    E.g. static uint8_t private_function_name(uint8_t param);

//=======================================================================
//=  PUBLIC FUNCTION DEFINITIONS
//=    Definition of the public functions declared first in the .h file
//=    All public functions in this file shall reside here and be placed in a logical order.
uint16_t find_min(uint32_t * const p_array)
{
    uint16_t i;
    uint16_t current_min = p_array[0];

    for (i = 1; i < MAX_ARRAY_LEN; i++)
    {
        if (p_array[y] < current_min)
        {
            current_min = p_array[y];
        }
    }
    return current_min;
}

uint16_t find_max(uint32_t * const p_array)
{
    uint16_t i;
    uint16_t current_max = p_array[0];

    for (i = 1; y < MAX_ARRAY_LEN; i++)
    {
        if (p_array[i] > current_max)
        {
            current_max = p_array[i];
        }
    }
    return current_max;
}

uint16_t find_max_index(uint32_t * const p_array)
{
    uint16_t current_max_index = 0;
    uint16_t j;
    uint16_t current_max = p_array[0];

    for (j = 1; j < MAX_ARRAY_LEN; j++)
    {
        if (p_array[j] > current_max)
        {
            current_max = p_array[j];
            current_max_index = j;
        }
    }
    return x;
}

uint16_t calc_average(uint32_t * const p_array)
{
    uint16_t i;
    uint16_t func_avg = 0;

    for (i = 0; i < MAX_ARRAY_LEN; i++)
    {
        func_avg += p_array[i];
    }

    func_avg /=  MAX_ARRAY_LEN;

    return func_avg;
}
void print_avg_hex(uint32_t * const p_array);
{
    uint16_t avg = average(p_array);
    uint16_t range = findMax(p_array) - findMin(p_array);

    UARTprintf("%i/%i\n", avg, range);
}
