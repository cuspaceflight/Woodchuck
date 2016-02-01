/**
 * Woodchuck by CU Spaceflight
 *
 * This file is part of the Woodchuck project by Cambridge University
 * Spaceflight.
 *
 * Jamie Wood 2016
 */

#ifndef __INTERRUPTS_H__
#define __INTERRUPTS_H__

inline void enable_interrupts(void)
{
	asm volatile ("cpsie i");
}

inline void disable_interrupts(void)
{
	asm volatile ("cpsid i");
}

#endif /* __INTERRUPTS_H__ */
