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
	__asm__ __volatile__ ("cpsie i");
}

inline void disable_interrupts(void)
{
	__asm__ __volatile__ ("cpsid i");
}

#endif /* __INTERRUPTS_H__ */
