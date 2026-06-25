/*
 * fuction.c
 *
 *  Created on: 2026Äê6ÔÂ21ÈÕ
 *      Author: jred
 */

#include "zf_common_headfile.h"


float Limit_init(float min,float value,float max)
{
    if (value <min)
    {
        return min;

    }
    if (value > max)
    {
        return max;
    }
    return value;
}


