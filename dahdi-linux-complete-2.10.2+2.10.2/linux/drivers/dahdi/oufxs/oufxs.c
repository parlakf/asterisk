/*
 *  oufxs.c: DAHDI-compatible Linux kernel driver for the Open USB FXS board
 *  Copyright (C) 2010  Angelos Varvitsiotis
 *  Parts of code (dahdi channel # persistence)
 *    Copyright (C) 2010  Angelos Varvitsiotis & Rockbochs, Inc.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Acknowledgement:
 *  The author wishes to thank Rockbochs, Inc., for their support and
 *  for partially funding the development of this driver
 *
 */

// TODO: function documentation
static char *driverversion = "0.2.3-dahdi";

/* includes */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/usb.h>
#include <linux/workqueue.h>
#include <linux/kfifo.h>
#include <linux/wait.h>
#include <linux/ioctl.h>
#include <linux/version.h>
#include <asm/uaccess.h>
#include <linux/ctype.h>

/* Open USB FXS - specific includes */
#include "oufxs.h"
#include "../proslic.h"
#include "oufxs_dvsn.h"

/* Do elementary environment verification */
#if !defined (DAHDI_VERSION_MAJOR) || !defined (DAHDI_VERSION_MINOR)
#error "DAHDI_VERSION_MAJOR or DAHDI_VERSION_MINOR is not defined!"
#elif (DAHDI_VERSION_MAJOR!=2) ||  \
       ((DAHDI_VERSION_MINOR!=2)&& \
        (DAHDI_VERSION_MINOR!=3)&& \
	(DAHDI_VERSION_MINOR!=4)&& \
	(DAHDI_VERSION_MINOR!=5)&& \
	(DAHDI_VERSION_MINOR!=10))
#error "This code compiles only against Dahdi releases 2.2, 2.3, 2.4 and 2.5"
#endif


/* dahdi-specific includes */
#include <dahdi/kernel.h>
/* normally, we would include here something like a "oufxs_user.h"
 * header, where we would define our own structures and ioctls for
 * oufxs; however, it is much more convenient to have dahdi-tools
 * programs like "fxstest.c" talk to our board without having to
 * reinvent the wheel (especially given that the differences
 * between the wctdm.c and our module from a user's perspective
 * are few - mostly vmwi capabilities, which we are lacking)
 */
#include <dahdi/wctdm_user.h>

/* local #defines */

/* configuration defines; adjust to your needs (must know what you are doing) */
	/* define to get HW-based DTMF detection in dev->dtmf (old) */
#undef HWDTMF

#ifndef HWDTMF
	/* define to use dahdi-specific hardware DTMF detection */
#define DAHDI_HWDTMF
#endif /* HWDTMF */

	/* define to get instantaneous hook state in dev->hook */
#undef HWHOOK

	/* define to print debugging messages related to seq#->buf mapping */
#undef DEBUGSEQMAP

	/* define to have the ProSLIC reset at driver loading / plug-in time */
#undef DO_RESET_PROSLIC

	/* define to make board reset functionality available through ioctl */
#define DO_RESET_BOARD

/* end of user-fiddleable configuration defines */

/* include (config-define-dependent) USB command packet definitions */
#include "cmd_packet.h"

/* initialization macro; assumes target 's' is a char array memset to 0 */
#define safeprintf(s, args...)	snprintf (s, sizeof(s) - 1, ## args)

/* immediately return on failure, leaving the board in its current state */
#ifdef DEBUGGING
#define DEFDEBUGLEVEL OUFXS_DBGVERBOSE
#define RETONFAIL 1
#else /* DEBUGGING */
#define DEFDEBUGLEVEL OUFXS_NONE
#define RETONFAIL 0
#endif /* DEBUGGING */

/* maximum buffer lengths */
#define OUFXS_MAXIBUFLEN	(rpacksperurb * OUFXS_DPACK_SIZE)
#define OUFXS_MAXOBUFLEN	(wpacksperurb * OUFXS_DPACK_SIZE)

/* default values for urb-related module parameters */
#ifndef WPACKSPERURB	/* any value from 2 to OUFXS_MAXPCKPERURB */
#define WPACKSPERURB	4
#endif /* WPACKSPERURB */

#ifndef RPACKSPERURB	/* any value from 2 to OUFXS_MAXPCKPERURB */
#define RPACKSPERURB	4
#endif /* RPACKSPERURB */

#ifndef WURBSINFLIGHT	/* any value from 2 to OUFXS_MAXINFLIGHT */
#define WURBSINFLIGHT	OUFXS_INFLIGHT
#endif /* WURBSINFLIGHT */

#ifndef RURBSINFLIGHT	/* any value from 2 to OUFXS_MAXINFLIGHT */
#define RURBSINFLIGHT	OUFXS_INFLIGHT
#endif /* RURBSINFLIGHT */

/* OHT_TIMER specifies how long to maintain on-hook-transfer after ring */
#define OHT_TIMER	6000	/* in milliseconds */

/* module parameters */

/* generic */
static int debuglevel	= DEFDEBUGLEVEL;	/* actual debug level */
/* initialization failure handling */
static int retoncnvfail	= RETONFAIL;		/* quit on dc-dc conv fail */
static int retonadcfail	= RETONFAIL;		/* quit on adc calbr. fail */
static int retonq56fail	= RETONFAIL;		/* quit on q56 calbr. fail */
/* urb etc. dimensioning */
static int wpacksperurb	= WPACKSPERURB;		/* # of write packets per urb */
static int wurbsinflight= WURBSINFLIGHT;	/* # of write urbs in-flight */
static int rpacksperurb	= RPACKSPERURB;		/* # of read packets per urb */
static int rurbsinflight= RURBSINFLIGHT;	/* # of read urbs in-flight */
static int sofprofile = 0;			/* SOF profiling mode */
static char *rsvserials="";			/* channels rsvd for serial#*/
/* telephony-related */
static int alawoverride	= 0;			/* use a-law instead of mu-law*/
static int reversepolarity = 0;			/* use reversed polarity */
static int loopcurrent = 20;			/* loop current */
static int lowpower = 0;			/* set on-hook voltage to 24V */
static int hifreq = 1;				/* ~80kHz, for L1=100uH */
static int hwdtmf = 1;				/* detect DTMF in hardware */
static int fxstxgain = 0;			/* analog TX gain, see code */
static int fxsrxgain = 0;			/* analog RX gain, see code */
#ifdef DEBUGSEQMAP
static int complaintimes = 0;			/* how many times to complain */
#endif	/* DEBUGSEQMAP */
static int availinerror = 0;			/* make available even when
						 * in error condition */

module_param(debuglevel, int, S_IWUSR|S_IRUGO);
module_param(retoncnvfail, bool, S_IWUSR|S_IRUGO);
module_param(retonadcfail, bool, S_IWUSR|S_IRUGO);
module_param(retonq56fail, bool, S_IWUSR|S_IRUGO);
module_param(wpacksperurb, int, S_IRUGO);
module_param(wurbsinflight, int, S_IRUGO);
module_param(rpacksperurb, int, S_IRUGO);
module_param(rurbsinflight, int, S_IRUGO);
module_param(sofprofile, int, S_IWUSR|S_IRUGO);
module_param(rsvserials, charp, S_IRUGO);
module_param(alawoverride, int, S_IWUSR|S_IRUGO);
module_param(reversepolarity, int, S_IWUSR|S_IRUGO);
module_param(loopcurrent, int, S_IWUSR|S_IRUGO);
module_param(lowpower, int, S_IWUSR|S_IRUGO);
module_param(hifreq, int, S_IWUSR|S_IRUGO);
module_param(hwdtmf, int, S_IWUSR|S_IRUGO);
module_param(fxstxgain, int, S_IWUSR|S_IRUGO);
module_param(fxsrxgain, int, S_IWUSR|S_IRUGO);
#ifdef DEBUGSEQMAP
module_param(complaintimes, int, S_IWUSR|S_IRUGO);
#endif	/* DEBUGSEQMAP */
module_param(availinerror, bool, S_IWUSR|S_IRUGO);

/* our device structure */


