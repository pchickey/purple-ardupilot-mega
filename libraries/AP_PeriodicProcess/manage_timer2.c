
#include <stdio.h>
#include <avr/interrupt.h>

#include "manage_timer2.h"

static void (*registered_proc)(void) = NULL;

void register_timer2_cb( void (*proc)(void) )
{
    registered_proc = proc;
}

ISR (TIMER2_OVF_vect)
{
    if (registered_proc)
        registered_proc();
}

