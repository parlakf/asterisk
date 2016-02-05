/*
 * Teknikom IPEX FXS/FXO Interface Driver for DAHDI
 *
 * Written by FiratParlak<firat@teknikom.com>
 * $Id: ipex_lt.c 2016-01-01 istanbul $
 
 * Copyright (C) 2016-2021 Teknikom Elektronik Ltd,
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
 * Rev 1.0 initial version
 *
 */

// TODO: function documentation
static char *driverversion = "2.10.2-dahdi";	// ??? What will be the driver version of dahdi?

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/slab.h>
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

/* IPEX LT - specific includes */
#include "ipex_lt.h" 

/* dahdi-specific includes */
#include <dahdi/kernel.h>
#include <dahdi/wctdm_user.h> 
#include "../fxo_modes.h"


/* immediately return on failure, leaving the board in its current state */
#ifdef DEBUGGING
#define DEFDEBUGLEVEL IPEX_LT_DBGVERBOSE
#define RETONFAIL 1
#else /* DEBUGGING */
#define DEFDEBUGLEVEL IPEX_LT_NONE
#define RETONFAIL 0
#endif /* DEBUGGING */

/* maximum buffer lengths */
#define IPEX_LT_MAXIBUFLEN	(rpacksperurb * IPEX_LT_DPACK_SIZE)
#define IPEX_LT_MAXOBUFLEN	(wpacksperurb * IPEX_LT_DPACK_SIZE)

/* default values for urb-related module parameters */
#ifndef WPACKSPERURB	/* any value from 2 to IPEX_LT_MAXPCKPERURB */
#define WPACKSPERURB	4
#endif /* WPACKSPERURB */

#ifndef RPACKSPERURB	/* any value from 2 to IPEX_LT_MAXPCKPERURB */
#define RPACKSPERURB	4
#endif /* RPACKSPERURB */

#ifndef WURBSINFLIGHT	/* any value from 2 to IPEX_LT_MAXINFLIGHT */
#define WURBSINFLIGHT	IPEX_LT_INFLIGHT
#endif /* WURBSINFLIGHT */

#ifndef RURBSINFLIGHT	/* any value from 2 to IPEX_LT_MAXINFLIGHT */
#define RURBSINFLIGHT	IPEX_LT_INFLIGHT
#endif /* RURBSINFLIGHT */


//Definitions
#define NUM_CARDS 4

#define MAX_ALARMS 10

#define MOD_TYPE_FXS	0
#define MOD_TYPE_FXO	1

#define DEFAULT_RING_DEBOUNCE	64		/* Ringer Debounce (64 ms) */

#define POLARITY_DEBOUNCE 	64		/* Polarity debounce (64 ms) */

#define OHT_TIMER		6000	/* How long after RING to retain OHT */


//FUNCTIONS
static int ipex_lt_hooksig(struct dahdi_chan *chan, enum dahdi_txsig txsig);

/*Module Parameters*/

/* generic */
static int debuglevel	= DEFDEBUGLEVEL;	/* actual debug level */
/* initialization failure handling */
// static int retoncnvfail	= RETONFAIL;		/* quit on dc-dc conv fail */
// static int retonadcfail	= RETONFAIL;		/* quit on adc calbr. fail */
// static int retonq56fail	= RETONFAIL;		/* quit on q56 calbr. fail */
/* urb etc. dimensioning */
static int wpacksperurb	= WPACKSPERURB;		/* # of write packets per urb */
//static int wurbsinflight= WURBSINFLIGHT;	/* # of write urbs in-flight */
static int rpacksperurb	= RPACKSPERURB;		/* # of read packets per urb */
//static int rurbsinflight= RURBSINFLIGHT;	/* # of read urbs in-flight */
// static int sofprofile = 0;			/* SOF profiling mode */
// static char *rsvserials="";			/* channels rsvd for serial#*/
/* telephony-related */

#ifdef DEBUGSEQMAP
static int complaintimes = 0;			/* how many times to complain */
#endif	/* DEBUGSEQMAP */
static int availinerror = 0;			/* make available even when * in error condition */

/* table of devices that this driver handles */
static const struct usb_device_id ipex_lt_dev_table[] = { 
	{ USB_DEVICE(IPEXLT_VENDOR_ID, IPEXLT_PRODUCT_ID) }, 
	{ } /* terminator entry */
};

//static __u8 trash8; /* variable to dump useless values	*/

/*Device Structures*/

struct ipex_lt_dahdi {

	struct usb_device *udev;	     	/* the usb device for this device */
	struct usb_interface *interface;  	/* the interface for this device */
	unsigned char *bulk_in_buffer;    	/* the buffer to receive data */
	size_t bulk_in_size;		     	/* the size of the receive buffer */
	__u8	bulk_in_endpointAddr;	    /* the address of the bulk in endpoint */
	__u8	bulk_out_endpointAddr;      /* the address of the bulk out endpoint */
	struct kref kref;					/* kernel/usb reference counts	*/

	int slot; /* slot in boards[] below */
	/* USB core stuff */
	char *variety;
	struct dahdi_span span;
	struct dahdi_device *ddev;
	unsigned char ios;
	int usecount;
	unsigned int intcount;
	int dead;
	int pos;
	int flags[NUM_CARDS];
	int freeregion;
	int alt;
	int curcard;
	int cardflag;		/* Bit-map of present cards */
//	enum proslic_power_warn proslic_power;
	spinlock_t lock;

	/* usb endpoint descriptors and respective packet sizes */
	__u8 ep_isoc_in; /* ISOC IN  EP address	*/
	__u8 ep_isoc_out; /* ISOC OUT EP address	*/


	/* isochronous buffer/urb structures */
  //   struct isocbuf {
		// spinlock_t		lock;		/* protect this buffer	*/
  //       struct urb		*urb;		/* urb to be submitted	*/
		// char			*buf;		/* actual buffer	*/
		// enum state_enum 
		// {
		// 	st_free	= 0,	 free to be submitted	
		// 	st_subm	= 1	/* submitted to usb core*/
		// }state;		/* current buffer state	*/
		// char inconsistent;	/* bad state was found	*/
		// // int			len;		/* buffer length	*/
		// struct oufxs_dahdi	*dev;		/* back-pointer to us	*/
  //   }outbufs[IPEX_LT_MAXURB];

    	/* bulk buffer/urb structures */
    struct isocbuf {
		spinlock_t		lock;		/* protect this buffer	*/
        struct urb		*urb;		/* urb to be submitted	*/
		char			*buf;		/* actual buffer	*/
		enum state_enum 
		{
			st_free	= 0,	/* free to be submitted	*/
			st_subm	= 1	/* submitted to usb core*/
		}state;		/* current buffer state	*/
		char inconsistent;	/* bad state was found	*/
		// int			len;		/* buffer length	*/
		struct oufxs_dahdi	*dev;		/* back-pointer to us	*/
    }outbufs[IPEX_LT_MAXURB];
	
    int			outsubmit;	/* next out buffer to submit */
    spinlock_t	outbuflock;	/* short-term lock for above */
    struct 		isocbuf in_bufs[IPEX_LT_MAXURB];
    int			in_submit;	/* next in buffer to submit */
    spinlock_t	in_buflock;	/* short-term lock for above */
    char		*prevrchunk;	/* previous read chunk	*/
    __u8		*seq2chunk[256];/* seqno->sample map	*/
	
	// TODO: remove/bypass urb anchoring for kernels earlier than 2.6.23
    /* anchor for submitted/pending urbs */
    struct usb_anchor		submitted;
	
	__u8 rsrvd; /* non-zero if a pre-reserved slot */
	
    /* initialization worker thread pointer and queue */
    struct workqueue_struct	*iniwq;	/* initialization workqueue	*/
    struct work_struct		iniwt;	/* initialization worker thread	*/

    /* board state */
    unsigned char state;	/* device state - see ipex_lt.h	*/
	
	/* locks and references */
    int				opencnt;/* number of openers		*/
    spinlock_t			statelck; /* locked during state changes*/
    struct mutex		iomutex;/* serializes I/O operations	*/
	
	

	union {
		struct fxo 
		{
#ifdef AUDIO_RINGCHECK
			unsigned int pegtimer;
			int pegcount;
			int peg;
			int ring;
#else			
			int wasringing;
			int lastrdtx;
#endif			
			int ringdebounce;
			int offhook;
			unsigned int battdebounce;
			unsigned int battalarm;
//			enum battery_state battery;
		    int lastpol;
		    int polarity;
		    int polaritydebounce;
		} fxo;
		struct fxs
		{
			int oldrxhook;
			int debouncehook;
			int lastrxhook;
			int debounce;
			int ohttimer;
			int idletxhookstate;		/* IDLE changing hook state */
			int lasttxhook;
			int palarms;
			int reversepolarity;		/* Reverse Line */
			int mwisendtype;
			struct dahdi_vmwi_info vmwisetting;
			int vmwi_active_messages;
			u32 vmwi_lrev:1; /*MWI Line Reversal*/
			u32 vmwi_hvdc:1; /*MWI High Voltage DC Idle line*/
			u32 vmwi_hvac:1; /*MWI Neon High Voltage AC Idle line*/
			u32 neonringing:1; /*Ring Generator is set for NEON*/
//			struct calregs calregs;
		} fxs;
	} mod[NUM_CARDS];

	/* Receive hook state and debouncing */
	int modtype[NUM_CARDS];
	unsigned char reg0shadow[NUM_CARDS];
	unsigned char reg1shadow[NUM_CARDS];

	unsigned long ioaddr;
	dma_addr_t 	readdma;
	dma_addr_t	writedma;
	volatile unsigned int *writechunk;				/* Double-word aligned write memory */
	volatile unsigned int *readchunk;				/* Double-word aligned read memory */
	struct dahdi_chan _chans[NUM_CARDS];
	struct dahdi_chan *chans[NUM_CARDS];
};

static struct ipex_lt_dahdi *ifaces[WC_MAX_IFACES];

struct ipex_lt_desc {
	char *name;
	int flags;
};

/* telephony-related */
//static unsigned int fxovoltage;
static unsigned int battdebounce;
static unsigned int battalarm;
static unsigned int battthresh;
//static int ringdebounce = DEFAULT_RING_DEBOUNCE;
/* times 4, because must be a multiple of 4ms: */
// static int dialdebounce = 8 * 8;
// static int fwringdetect = 0;
// static int debug = 0;
// static int robust = 0;
// static int timingonly = 0;
// static int lowpower = 0;
static int boostringer = 0;
// static int fastringer = 0;
static int _opermode = 0;
static char *opermode = "FCC";
static int fxshonormode = 0;
// static int alawoverride = 0;
// static int fastpickup = 0;
//static int fxotxgain = 0;
//static int fxorxgain = 0;
//static int fxstxgain = 0;
//static int fxsrxgain = 0;


/* active device structs */
static struct ipex_lt_dahdi *boards[IPEX_LT_MAX_BOARDS];	/* active dev structs */
/* lock to be held while manipulating boards */
static spinlock_t boardslock;	/* lock while manipulating boards 	*/
/* dummy spans/channels used for reserving channel numbers to specific boards */
struct sn_to_chan {
    char serial[10];
    int  channo;
};

static struct sn_to_chan  *rsvsn2chan = NULL;
//static int numsn2chan = 0;
//static int numdummies = 0;

/* usb driver info and forward definitions of our function pointers therein */
static int ipex_lt_probe (struct usb_interface *, const struct usb_device_id *);
static void ipex_lt_disconnect (struct usb_interface *);


static struct usb_driver ipex_lt_driver = {
    .name		= "ipexlt",				//The driver name should be unique among USB drivers, and should normally be the same as the module name.
    .id_table	= ipex_lt_dev_table,	//USB drivers use ID table to support hotplugging. Export this with MODULE_DEVICE_TABLE(usb,...). This must be set or your driver's probe function will never get called.
    .probe		= ipex_lt_probe,		//Called to see if the driver is willing to manage a particular interface on a device. If it is, probe returns zero and uses usb_set_intfdata to associate driver-specific data with the interface
    .disconnect	= ipex_lt_disconnect,	//Called when the interface is no longer accessible, usually because its device has been (or is being) disconnected or the driver module is being unloaded.
};

static int ipex_lt_open (struct dahdi_chan *);
//static int ipex_lt_open_dummy (struct dahdi_chan *);
static int ipex_lt_hooksig (struct dahdi_chan *, enum dahdi_txsig);
static int ipex_lt_close (struct dahdi_chan *);
static int ipex_lt_ioctl (struct dahdi_chan *, unsigned int , unsigned long);
static int ipex_lt_watchdog(struct dahdi_span *span, int event);

static const struct dahdi_span_ops ipex_lt_span_ops = {
    .owner 		= THIS_MODULE,
    .hooksig 	= ipex_lt_hooksig,
    .open 		= ipex_lt_open,
    .close 		= ipex_lt_close,
    .ioctl 		= ipex_lt_ioctl,
    .watchdog 	= ipex_lt_watchdog,
};


/***********************************************************************/
/*
	Function		: ipex_lt_delete()
	Author			: Firat Parlak
	Description		: delete method
					  frees dev and associated memory, resets boards[] slot
	Date Created	: 05.02.2016
	Modifications	:
*/
/***********************************************************************/
static void ipex_lt_delete(struct kref *kr) 
{	
//   struct usb_skel *dev = to_skel_dev(kref);
	struct ipex_lt_dahdi *dev = container_of(kr, struct ipex_lt_dahdi, kref);
	unsigned long flags;	/* irqsave flags */

	IPEX_LT_DEBUG (IPEX_LT_DBGVERBOSE, "%s(): starting", __func__);

	spin_lock_irqsave (&dev->statelck, flags);
	dev->state = IPEX_LT_STATE_UNLOAD;
	spin_unlock_irqrestore (&dev->statelck, flags);

	/* destroy board initialization thread workqueue, killing worker thread */
    if (dev->iniwq) {
		destroy_workqueue (dev->iniwq);
    }
    dev->iniwq = NULL;

//	dahdi_unregister_device(dev->ddev);

	/* kill anchored urbs */
    usb_kill_anchored_urbs (&dev->submitted);

   	usb_put_dev(dev->udev);
	kfree (dev->bulk_in_buffer);
	kfree (dev);

	spin_lock_irqsave (&dev->statelck, flags);
	dev->state = IPEX_LT_STATE_UNLOAD;
	spin_unlock_irqrestore (&dev->statelck, flags);

	IPEX_LT_DEBUG (IPEX_LT_DBGDEBUGGING, "%s: finished", __func__);
}

/***********************************************************************/
/*
	Function		: ipex_lt_probe()
	Author			: Firat Parlak
	Description		: probe method
						Initializes the device,
						Mapping,
						I/O Memory
						Registering the interrupt handlers
	Date Created	: 08.01.2016
	Modifications	:
*/
/***********************************************************************/
static int ipex_lt_probe (struct usb_interface *intf, const struct usb_device_id *ent)
{
	int retval = -ENODEV;	//default : ENODEV
	struct ipex_lt_dahdi *dev;	//wctdm *wc;
	//struct ipex_lt_desc *d = (struct ipex_lt_desc *)ent->driver_data;
	struct usb_host_interface *intf_desc;
	struct usb_endpoint_descriptor *ep_desc;
	size_t buffer_size;
	int x;
//	int y;
	int cardcount = 0;
	int i;
	
	IPEX_LT_DEBUG (IPEX_LT_DBGVERBOSE, "%s(): starting", __func__);
	
	for (x=0;x<WC_MAX_IFACES;x++)
		if (!ifaces[x]) break;
	if (x >= WC_MAX_IFACES) {
		printk(KERN_NOTICE "ipex_lt: Too many interfaces\n");
		return -EIO;
	}
	
	// if (pci_enable_device(pdev)) {
		// retval = -EIO;
	// } else {
		dev = kmalloc(sizeof(struct ipex_lt_dahdi), GFP_KERNEL);
		if (dev) {
			IPEX_LT_INFO ("ipex_lt: dev = kmalloc is successfull. \n");
	
			ifaces[x] = dev;
			memset(dev, 0, sizeof(struct ipex_lt_dahdi));
			
			for (x=0; x < sizeof(dev->chans)/sizeof(dev->chans[0]); ++x) {
				dev->chans[x] = &dev->_chans[x];
			}
			
			memset(dev, 0x00, sizeof (*dev));
   			kref_init(&dev->kref);		/* initialize device kernel ref count */

			//spin_lock_init(&dev->lock);	//Initializes device lock.

			spin_lock_init (&dev->statelck); 	/* initialize other locks */	
			dev->state = IPEX_LT_STATE_IDLE;	/* initialize board's state variable */
			
			/* initialize usb */
			// TODO: remove/bypass urb anchoring for kernels earlier than 2.6.23
			init_usb_anchor (&dev->submitted);
			dev->udev = usb_get_dev (interface_to_usbdev (intf));
			dev->interface = intf;
			
			// /* set up the endpoint information */
			// /* use only the first bulk-in and bulk-out endpoints */
			intf_desc = intf->cur_altsetting;	//iface_desc = interface->cur_altsetting;
			
			for (i = 0; i < intf_desc->desc.bNumEndpoints; ++i) {
				ep_desc = &intf_desc->endpoint[i].desc;
				if (!dev->bulk_in_endpointAddr &&
				(ep_desc->bEndpointAddress & USB_DIR_IN) &&
				((ep_desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
				== USB_ENDPOINT_XFER_BULK)) {
					/* we found a bulk in endpoint */
					buffer_size = ep_desc->wMaxPacketSize;
					dev->bulk_in_size = buffer_size;
					dev->bulk_in_endpointAddr = ep_desc->bEndpointAddress;
					dev->bulk_in_buffer = kmalloc(buffer_size, GFP_KERNEL);
					IPEX_LT_DEBUG (IPEX_LT_DBGVERBOSE, "bulk IN  endpoint found, EP#%d, size:%d", dev->bulk_in_endpointAddr, dev->bulk_in_size);
					if (!dev->bulk_in_buffer) {
						IPEX_LT_ERR("ipex_lt: Could not allocate bulk_in_buffer");
						goto probe_error;
					}
				}

				if (!dev->bulk_out_endpointAddr &&
				!(ep_desc->bEndpointAddress & USB_DIR_IN) &&
				((ep_desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
				== USB_ENDPOINT_XFER_BULK)) {
					/* we found a bulk out endpoint */
					dev->bulk_out_endpointAddr = ep_desc->bEndpointAddress;
					IPEX_LT_DEBUG (IPEX_LT_DBGVERBOSE, "bulk OUT  endpoint found, EP#%d", dev->bulk_out_endpointAddr);
				}
			}
			if (!(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr)) {
				IPEX_LT_ERR("ipex_lt: Could not find both bulk-in and bulk-out endpoints");
				retval = -EIO;
				goto probe_error;
			}
			
	   		/* save back-pointer to our ipex_lt_dahdi structure within usb interface */
   			usb_set_intfdata (intf, dev);




			IPEX_LT_INFO ("IPEX LT device %d starting", dev->slot);
			retval = 0;
			return retval;
		}else{
			IPEX_LT_ERR ("ipex_lt: Probe method: Not available memory \n");
			return -ENOMEM;	//retval = -ENOMEM;
		}
		
		
	// }
probe_error:
	IPEX_LT_ERR("ipex_lt: An error happened in probe method");
	if (dev) {
       /* decrement kernel ref count (& indirectly, call destructor function) */
       kref_put (&dev->kref, ipex_lt_delete);
    }

	return retval;
}
/***********************************************************************/
/*
	Function		: ipex_lt_disconnect()
	Author			: Firat Parlak
	Description		: disconnect method
					  usb_interface has been removed from the system or 
				      the driver is unloaded from the USB core.
	Date Created	: 28.01.2016
	Modifications	:
*/
/***********************************************************************/
static void ipex_lt_disconnect (struct usb_interface *intf)
{
	struct ipex_lt_dahdi *dev;
	// unsigned long flags;		/* irqsave flags */
    int slot;
	
	IPEX_LT_DEBUG (IPEX_LT_DBGVERBOSE, "%s(): starting", __func__);
	
	/* get us a pointer to dev, then clear it from the USB interface */
    dev = usb_get_intfdata (intf);
    usb_set_intfdata (intf, NULL);

	kref_put (&dev->kref, ipex_lt_delete);/* decrease the kref count to dev, eventually calling ipex_lt_delete */

	/*set endpoints to zero to avoid warnings on re-plug*/
    dev->bulk_in_endpointAddr 	= 0;
    dev->bulk_out_endpointAddr 	= 0;

    IPEX_LT_INFO ("ipex_lt device %d is now disconnected", slot + 1);
}

static int ipex_lt_hooksig(struct dahdi_chan *chan, enum dahdi_txsig txsig)
{
	struct ipex_lt_dahdi *dev = chan->pvt;
	int chan_entry = chan->chanpos - 1;
	
	if (dev->modtype[chan_entry] == MOD_TYPE_FXO) 
	{
		/* XXX Enable hooksig for FXO XXX */
		switch(txsig)
		{
			case DAHDI_TXSIG_START:
			case DAHDI_TXSIG_OFFHOOK:
				break;
			case DAHDI_TXSIG_ONHOOK:
				break;
			default:
				printk(KERN_NOTICE "ipex_lt: Can't set tx state to %d\n", txsig);
				break;
		}
	}
	else 
	{

	}
	return 0;
}


static int ipex_lt_open (struct dahdi_chan *chan)
{
    struct ipex_lt_dahdi *dev = chan->pvt;

    IPEX_LT_DEBUG (IPEX_LT_DBGDEBUGGING, "open()");

    /* don't proceed unless channel points to a device */
    if(!dev) 
	{	/* can this happen during oufxs_disconnect()? */
        IPEX_LT_ERR ("%s: no device data for chan %d", __func__, chan->chanpos);
		return -ENODEV;
    }

    // /* don't proceed if the device state is not OK yet (or is unloading) */
    if(dev->state != IPEX_LT_STATE_OK) 
	{
		if(dev->state == IPEX_LT_STATE_INIT) 
			return -EAGAIN;
		if(!availinerror)
		{
			return -EIO;
		}
    }

    /* increment driver-side usage count for device */
    kref_get (&dev->kref);

    /* we CANNOT use mutexes in here; dahdi_base calls us with the channel
     * lock held, so we risk going to sleep with a spinlock held and locking
     * up the kernel
    // mutex_lock (&dev->iomutex);
     */

    /* increment open count (safe, since we are called with the lock held) */
    dev->opencnt++;

    /* tell kernel our use count has increased, so as to prevent unloading us */
    try_module_get (THIS_MODULE);

    // TODO: reset usb I/O statistics

    return 0;
}


static int ipex_lt_close(struct dahdi_chan *chan)
{

	return 0;
}

static int ipex_lt_ioctl(struct dahdi_chan *chan, unsigned int cmd, unsigned long data)
{
	// struct wctdm_stats stats;
	// struct wctdm_regs regs;
	// struct ipex_lt_dahdi *dev = chan->pvt;
	
	
	switch (cmd)
	{
		case DAHDI_ONHOOKTRANSFER:
			break;
		case DAHDI_SETPOLARITY:
			break;
		case DAHDI_VMWI_CONFIG:
			break;
		case DAHDI_VMWI:
			break;
		case WCTDM_GET_STATS:
			break;
		case WCTDM_GET_REGS:
			break;
		case WCTDM_SET_REG:
			break;
		case WCTDM_SET_ECHOTUNE:
			break;
		case DAHDI_SET_HWGAIN:
			break;
		default:
			return -ENOTTY;
	}
	return 0;

}

static int ipex_lt_watchdog(struct dahdi_span *span, int event)
{
//	printk(KERN_INFO "IPEXLT: Restarting DMA\n");
//	wctdm_restart_dma(wctdm_from_span(span));
	return 0;
}

/***********************************************************************/
/*
	Function		: ipex_lt_init()
	Author			: Firat Parlak
	Description		: Initializes the card driver. Registers the USB interface driver.
	Date Created	: 08.01.2016
	Modifications	:
*/
/***********************************************************************/
static int __init ipex_lt_init(void)
{
	int res;
	int x;

    IPEX_LT_INFO ("ipex_lt driver v%s loading\n", driverversion);    //printk (KERN_INFO KBUILD_MODNAME ": ipex_lt driver v %s loading\n", driverversion);

	IPEX_LT_DEBUG (IPEX_LT_DBGTERSE, "debug level is %d", debuglevel); 

	for (x=0;x<(sizeof(fxo_modes) / sizeof(fxo_modes[0])); x++) {
		if (!strcmp(fxo_modes[x].name, opermode)) {
			break;
		}
	}
	if (x < sizeof(fxo_modes) / sizeof(fxo_modes[0])) {
		_opermode = x;
	} else {
		printk(KERN_NOTICE "Invalid/unknown operating mode '%s' specified.  Please choose one of:\n", opermode);
		for (x=0;x<sizeof(fxo_modes) / sizeof(fxo_modes[0]); x++) {
			printk(KERN_INFO "  %s\n", fxo_modes[x].name);
		}
		printk(KERN_INFO "Note this option is CASE SENSITIVE!\n");
		return -ENODEV;
	}
	if (!strcmp(fxo_modes[_opermode].name, "AUSTRALIA")) {
		boostringer=1;
		fxshonormode=1;
	}
	/* for the voicedaa_check_hook defaults, if the user has not overridden
		   them by specifying them as module parameters, then get the values
		   from the selected operating mode
		*/
	if (battdebounce == 0) {
		battdebounce = fxo_modes[_opermode].battdebounce;
	}
	if (battalarm == 0) {
		battalarm = fxo_modes[_opermode].battalarm;
	}
	if (battthresh == 0) {
		battthresh = fxo_modes[_opermode].battthresh;
	}
	
	res = usb_register (&ipex_lt_driver);
	if (res) {
		IPEX_LT_ERR ("usb_register failed, error=%d", res);
		return -ENODEV;
	}
	// return values other than 0 are errors, see <linux/errno.h>
	return 0;
}

static void __exit ipex_lt_exit(void)
{
    usb_deregister (&ipex_lt_driver);

    IPEX_LT_INFO ("ipex_lt driver unloaded\n");
}

//by wctdm.c
// module_param(debug, int, 0600);
// module_param(fxovoltage, int, 0600);
// module_param(loopcurrent, int, 0600);
// module_param(reversepolarity, int, 0600);
// module_param(robust, int, 0600);
module_param(opermode, charp, 0600);
// module_param(timingonly, int, 0600);
// module_param(lowpower, int, 0600);
// module_param(boostringer, int, 0600);
// module_param(fastringer, int, 0600);
// module_param(fxshonormode, int, 0600);
module_param(battdebounce, uint, 0600);
module_param(battalarm, uint, 0600);
module_param(battthresh, uint, 0600);
//module_param(ringdebounce, int, 0600);
// module_param(dialdebounce, int, 0600);
// module_param(fwringdetect, int, 0600);
// module_param(alawoverride, int, 0600);
// module_param(fastpickup, int, 0600);
// module_param(fxotxgain, int, 0600);
// module_param(fxorxgain, int, 0600);
// module_param(fxstxgain, int, 0600);
// module_param(fxsrxgain, int, 0600);

//by oufxs.c
module_param(debuglevel, int, S_IWUSR|S_IRUGO);
// module_param(retoncnvfail, bool, S_IWUSR|S_IRUGO);
// module_param(retonadcfail, bool, S_IWUSR|S_IRUGO);
// module_param(retonq56fail, bool, S_IWUSR|S_IRUGO);
module_param(wpacksperurb, int, S_IRUGO);
//module_param(wurbsinflight, int, S_IRUGO);
module_param(rpacksperurb, int, S_IRUGO);
// module_param(rurbsinflight, int, S_IRUGO);
// module_param(sofprofile, int, S_IWUSR|S_IRUGO);
// module_param(rsvserials, charp, S_IRUGO);
// module_param(alawoverride, int, S_IWUSR|S_IRUGO);
// module_param(reversepolarity, int, S_IWUSR|S_IRUGO);
// module_param(loopcurrent, int, S_IWUSR|S_IRUGO);
// module_param(lowpower, int, S_IWUSR|S_IRUGO);
// module_param(hifreq, int, S_IWUSR|S_IRUGO);
// module_param(hwdtmf, int, S_IWUSR|S_IRUGO);
// module_param(fxstxgain, int, S_IWUSR|S_IRUGO);
// module_param(fxsrxgain, int, S_IWUSR|S_IRUGO);
// #ifdef DEBUGSEQMAP
// module_param(complaintimes, int, S_IWUSR|S_IRUGO);
// #endif	/* DEBUGSEQMAP */
// module_param(availinerror, bool, S_IWUSR|S_IRUGO);

MODULE_DESCRIPTION("IPEX LT driver for dahdi telephony interface");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Firat Parlak <firat-at-teknikom-dot-com>");

//MODULE_ALIAS
MODULE_DEVICE_TABLE (usb, ipex_lt_dev_table);


module_init(ipex_lt_init);	//modprobe ipex_lt
module_exit(ipex_lt_exit);	//rmmod ipex_lt
