
#ifndef __MANAGE_TIMER2_H__
#define __MANAGE_TIMER2_H__

#ifdef __cplusplus
extern "C" {
#endif

/*  
 *  manage_timer2 exists to provide a registration facility for the timer2 isr.
 *
 *  manage_timer2.c implements the actual TIMER2_OVF_vect ISR, which calls a
 *  single callback registered through register_timer2_cb().
 *
 *  This makes up for a limitation of the Arduino IDE's link stage.
 *  Many classes in your Arduino libraries/ folder might assume exclusive use 
 *  of the timer2 ISR.  Although only one of these classes should be 
 *  instantiated in your sketch, all classes will be compiled and linked. 
 *  Therefore, the only way to ensure only one ISR is defined is to do so 
 *  seperately and register against it in runtime.
 *
 *  It is still left up to the sketch programmer to make sure only one library
 *  assumes exclusive use of the timer2 ISR. Only the most recently registered
 *  procedure will be called.
 */


void register_timer2_cb( void (*proc)(void) );

#ifdef __cplusplus
}
#endif

#endif // __MANAGE_TIMER2_H__

