// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2002
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 */

/* Replacement (=dummy) for GNU/Linux division-by zero handler */
#ifdef CONFIG_ITOP4412
void __div0 (void)
{
	extern void hang (void);

	hang();
}
#else
void __div0 (void)
{
	for (;;)
		;
}
#endif
