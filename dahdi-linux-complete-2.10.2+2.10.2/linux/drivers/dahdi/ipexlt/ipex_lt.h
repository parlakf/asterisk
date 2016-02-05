/*
 * Teknikom IPEX FXS/FXO Interface Driver for Zapata Telephony interface
 *
 * Written by FiratParlak<firat@teknikom.com>
 * $Id: ipex_lt.c 2015-10-27 istanbul $
 
 * Copyright (C) 2015-2020 Teknikom Elektronik Ltd,
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/* Rev histroy
 *
 * Rev 0.10 initial version
 *
 */
 
#ifndef IPEX_LT

#define	DEBUGGING	/* turns on verbose debugging: undef in production */

/* our debug level and default facility */
#define IPEX_LT_DBGNONE			0
#define IPEX_LT_DBGTERSE		1
#define IPEX_LT_DBGVERBOSE		2
#define IPEX_LT_DBGDEBUGGING	3 /* may clog console/syslog, use at own risk */
#define IPEX_LT_FACILITY		KERN_INFO



/* our vendor and product ids (note: sub-licensed from ST) */
#define IPEXLT_VENDOR_ID		0x0483
#define IPEXLT_PRODUCT_ID		0x5740

#ifdef __KERNEL__
#include <linux/usb.h>
#include <linux/ioctl.h>

/* max # of boards supported per system */
#define IPEX_LT_MAX_BOARDS		128
#define MAX_DUMMYCHANS			1024

#if 0
#define IPEX_LT_MAXURB			8	/* maximum number of isoc URBs */
#define IPEX_LT_INFLIGHT		4	/* number of isoc URBs in flight */
#define IPEX_LT_MAXINFLIGHT		4	/* max isoc URBs in flight */
#define IPEX_LT_MAXPCKPERURB	32	/* max number of packets per isoc URB */
#else
#define IPEX_LT_MAXURB			16	/* maximum number of isoc URBs */
#define IPEX_LT_INFLIGHT		16	/* number of isoc URBs in flight */
#define IPEX_LT_MAXINFLIGHT		16	/* max isoc URBs in flight */
#define IPEX_LT_MAXPCKPERURB	16	/* max number of packets per isoc URB */
#endif

#if (IPEX_LT_MAXURB * IPEX_LT_MAXPCKPERURB > 256)
#error "The product of IPEX_LT_MAXURB and IPEX_LT_MAXPCKPERURB must be <= 256"
#endif


/* debug macros for userland and kernel */
#ifdef DEBUGGING
#define IPEX_LT_DEBUG(level, fmt, args...)	\
    if (debuglevel >= (level)) \
      printk (IPEX_LT_FACILITY KBUILD_MODNAME ": " fmt "\n", ## args)
#define IPEX_LT_ERR(fmt, args...) \
    printk (KERN_ERR KBUILD_MODNAME ": " fmt "\n", ## args)
#define IPEX_LT_WARN(fmt, args...) \
    printk (KERN_WARNING KBUILD_MODNAME ": " fmt "\n", ## args)
#define IPEX_LT_INFO(fmt, args...) \
    printk (KERN_INFO KBUILD_MODNAME ": " fmt "\n", ## args)
#else /* DEBUGGING */
#define IPEX_LT_DEBUG(level, fmt, args...)
#endif /* DEBUGGING */

#else /* __KERNEL__ */
#include <sys/ioctl.h>

#ifdef IPEX_LT_DEBUG_SYSLOG
#define IPEX_LT_DEBUG(level, fmt, args...)	/* TODO: write a syslog macro */
#else /* IPEX_LT_DEBUG_SYSLOG */
#define IPEX_LT_DEBUG(level, fmt, args...)	\
    if (debuglevel >= (level)) fprintf (stderr, fmt, ## args)
#endif /* IPEX_LT_DEBUG_SYSLOG */


#endif /* __KERNEL__ */


#define WC_MAX_IFACES 128

/* various states a board may be in	*/
#define IPEX_LT_STATE_IDLE		0	/* not initialized		*/
#define IPEX_LT_STATE_INIT		1	/* initializing			*/
#define IPEX_LT_STATE_WAIT		2	/* waiting for device to respond*/
#define IPEX_LT_STATE_OK		3	/* initialized			*/
#define IPEX_LT_STATE_ERROR		4	/* in error			*/
#define IPEX_LT_STATE_UNLOAD 	5	/* driver unloading		*/ 
 
 
/* ipex_lt magic ioctl number */
#define IPEX_LT_IOC_MAGIC	'X'

/* reset and reinitialize board */
#define IPEX_LT_IOCRESET	_IO(IPEX_LT_IOC_MAGIC, 0)
/* cause a register dump (requires compilation with DEBUG set to VERBOSE) */
#define IPEX_LT_IOCREGDMP	_IO(IPEX_LT_IOC_MAGIC, 1)
/* set ringing on/off: zero sets ring off, non-zero sets ring on */
#define IPEX_LT_IOCSRING	_IO(IPEX_LT_IOC_MAGIC, 2)
/* set linefeed mode: zero is 'open' mode; non-zero is 'forward active' mode */
#define IPEX_LT_IOCSLMODE	_IO(IPEX_LT_IOC_MAGIC, 3)
/* get hook state: returns zero if on-hook, one if off-hook */
#define IPEX_LT_IOCGHOOK	_IOR(IPEX_LT_IOC_MAGIC, 4, int)
/* generic event (hook state or DTMF): 0 is no event, others ??? */
#define IPEX_LT_IOCGDTMF	_IOR(IPEX_LT_IOC_MAGIC, 5, int)
/* get and reset statistics on errors etc. */
#define IPEX_LT_IOCGERRSTATS	_IOR(IPEX_LT_IOC_MAGIC, 6, struct ipex_lt_errstats)
/* burn a serial number */
#define IPEX_LT_IOCBURNSN	_IO(IPEX_LT_IOC_MAGIC, 7)
/* reboot in bootloader mode */
#define IPEX_LT_IOCBOOTLOAD _IO(IPEX_LT_IOC_MAGIC, 8)
/* get the firmware version */
#define IPEX_LT_IOCGVER	_IOR(IPEX_LT_IOC_MAGIC, 9, int)
/* get the serial number */
#define IPEX_LT_IOCGSN	_IOR(IPEX_LT_IOC_MAGIC, 10, int)
#define IPEX_LT_MAX_IOCTL	10 


enum EIPEX_LT_cmd{
	ECmd_Read_Version 		=0x00,
	ECmd_Read_LT_Version 	= 0x01,
	
	ECmd_Last
};

union ipex_lt_data{
	struct{
		__u8 Data1;	//Debug
		__u8 Data2;
		__u8 Data3;
		__u8 Data4;
		__u8 Data5;
		__u8 Data6;
		__u8 Data7;
		__u8 Data8;
		__u8 Data9;
		__u8 Data10;
		__u8 Data11;
		__u8 Data12;
		__u8 Data13;
		__u8 Data14;
		__u8 Data15;
		__u8 Data16;
		__u8 Data17;
		__u8 Data18;
		__u8 Data19;
		__u8 Data20;
	}Leds;
};
#define IPEX_LT_DPACK_SIZE	sizeof(union ipex_lt_data)
 
 #endif	/*IPEX_LT*/
 
