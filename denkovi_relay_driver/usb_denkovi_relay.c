/*
 * Copyright (C) 2013 Vinicius Tinti (viniciustinti@gmail.com)
 *
 * Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <linux/types.h>
#include <linux/leds.h>
#include <linux/workqueue.h>

#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>

#define USB_DKVR_VENDOR_ID	0x0403
#define USB_DKVR_PRODUCT_ID	0x6001

static const struct usb_device_id dkvr_table[] = {
	{ USB_DEVICE(USB_DKVR_VENDOR_ID, USB_DKVR_PRODUCT_ID) },
	{ }					/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, dkvr_table);

/* Get a minor range for your devices from the usb maintainer */
#define USB_DKVR_MINOR_BASE	192
#define MAX_TRANSFER		(PAGE_SIZE - 512)
#define WRITES_IN_FLIGHT	8

struct usb_dkvr {
	struct usb_device		*udev;				/* the usb device for this device */
	struct usb_interface		*interface;			/* the interface for this device */
	struct semaphore		limit_sem;			/* limiting the number of writes in progress */

	__u8				bulk_out_endpointAddr;		/* the address of the bulk out endpoint */
	struct usb_anchor               submitted;

	int				errors;				/* the last request tanked */
	spinlock_t			err_lock;			/* lock for errors */
	struct kref			kref;
	struct mutex			io_mutex;			/* synchronize I/O with disconnect */
};
#define to_dkvr_dev(d) container_of(d, struct usb_dkvr, kref)

struct usb_dkvr_classdev {
	/* Use leds interface as output only pin */
	struct led_classdev led;
};

struct usb_dkvr_devdata {
	struct usb_dkvr *usb_dev;
	struct usb_dkvr_classdev relay_00;
	struct usb_dkvr_classdev relay_01;
	struct usb_dkvr_classdev relay_02;
	struct usb_dkvr_classdev relay_03;
};

static struct usb_driver dkvr_driver;
static void dkvr_draw_down(struct usb_dkvr *dev);

static void dkvr_delete(struct kref *kref)
{
	struct usb_dkvr *dev = to_dkvr_dev(kref);

	usb_put_dev(dev->udev);

	kfree(dev);
}

static struct usb_dkvr_devdata usb_dkvr_data;

static void usb_dkvr_led_brightness_set(struct led_classdev *led_dev,
	enum led_brightness brightness);

static void usb_dkvr_writebyte_bulk_callback(struct urb *urb);
static ssize_t usb_dkvr_writebyte(struct usb_dkvr *dev, unsigned char byte);

static int usb_dkvr_class_register(struct usb_interface *interface)
{
	int err = 0;
	struct device *dev = &interface->dev;

	usb_dkvr_data.usb_dev = usb_get_intfdata(interface);

	usb_dkvr_data.relay_00.led.name = "denkovi0.0";
	usb_dkvr_data.relay_00.led.default_trigger = "denkovi0";
	usb_dkvr_data.relay_00.led.brightness_set = usb_dkvr_led_brightness_set;

	usb_dkvr_data.relay_01.led.name = "denkovi0.1";
	usb_dkvr_data.relay_01.led.default_trigger = "denkovi0";
	usb_dkvr_data.relay_01.led.brightness_set = usb_dkvr_led_brightness_set;

	usb_dkvr_data.relay_02.led.name = "denkovi0.2";
	usb_dkvr_data.relay_02.led.default_trigger = "denkovi0";
	usb_dkvr_data.relay_02.led.brightness_set = usb_dkvr_led_brightness_set;

	usb_dkvr_data.relay_03.led.name = "denkovi0.3";
	usb_dkvr_data.relay_03.led.default_trigger = "denkovi0";
	usb_dkvr_data.relay_03.led.brightness_set = usb_dkvr_led_brightness_set;

	err = led_classdev_register(dev, &usb_dkvr_data.relay_00.led);
	if (err) {
		printk("LEDs: Failed to register %s\n", usb_dkvr_data.relay_00.led.name);
		return err;
	}

	err = led_classdev_register(dev, &usb_dkvr_data.relay_01.led);
	if (err) {
		printk("LEDs: Failed to register %s\n", usb_dkvr_data.relay_01.led.name);
		return err;
	}

	err = led_classdev_register(dev, &usb_dkvr_data.relay_02.led);
	if (err) {
		printk("LEDs: Failed to register %s\n", usb_dkvr_data.relay_02.led.name);
		return err;
	}

	err = led_classdev_register(dev, &usb_dkvr_data.relay_03.led);
	if (err) {
		printk("LEDs: Failed to register %s\n", usb_dkvr_data.relay_03.led.name);
		return err;
	}

	return 0;
}

static void usb_dkvr_class_unregister(void)
{
	led_classdev_unregister(&usb_dkvr_data.relay_00.led);
	led_classdev_unregister(&usb_dkvr_data.relay_01.led);
	led_classdev_unregister(&usb_dkvr_data.relay_02.led);
	led_classdev_unregister(&usb_dkvr_data.relay_03.led);
}

static void usb_dkvr_led_brightness_set(struct led_classdev *led_dev,
	enum led_brightness brightness)
{
	printk(KERN_INFO "Device %s has been triggered with %d | %ld\n", led_dev->name, brightness, in_interrupt());
	usb_dkvr_writebyte(usb_dkvr_data.usb_dev, brightness);
}

static void usb_dkvr_writebyte_bulk_callback(struct urb *urb)
{
	struct usb_dkvr *dev;

	dev = urb->context;

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			dev_err(&dev->interface->dev,
				"%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);

		spin_lock(&dev->err_lock);
		dev->errors = urb->status;
		spin_unlock(&dev->err_lock);
	}

	/* free up our allocated buffer */
	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			  urb->transfer_buffer, urb->transfer_dma);
	up(&dev->limit_sem);
}

static ssize_t usb_dkvr_writebyte(struct usb_dkvr *dev, unsigned char byte)
{
	int retval = 0;
	struct urb *urb = NULL;
	char *buf = NULL;
	size_t writesize = sizeof(byte);

	/*
	 * limit the number of URBs in flight to stop a user from using up all
	 * RAM
	 */
	if (down_trylock(&dev->limit_sem)) {
		retval = -EAGAIN;
		goto exit;
	}

	spin_lock_irq(&dev->err_lock);
	retval = dev->errors;
	if (retval < 0) {
		/* any error is reported once */
		dev->errors = 0;
		/* to preserve notifications about reset */
		retval = (retval == -EPIPE) ? retval : -EIO;
	}
	spin_unlock_irq(&dev->err_lock);
	if (retval < 0)
		goto error;

	/* create a urb, and a buffer for it, and copy the data to the urb */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		retval = -ENOMEM;
		goto error;
	}

	buf = usb_alloc_coherent(dev->udev, writesize, GFP_KERNEL,
				 &urb->transfer_dma);
	if (!buf) {
		retval = -ENOMEM;
		goto error;
	}

	/* write value on buffer */
	buf[0] = byte;

	/* this lock makes sure we don't submit URBs to gone devices */
	mutex_lock(&dev->io_mutex);
	if (!dev->interface) {		/* disconnect() was called */
		mutex_unlock(&dev->io_mutex);
		retval = -ENODEV;
		goto error;
	}

	/* initialize the urb properly */
	usb_fill_bulk_urb(urb, dev->udev,
			  usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
			  buf, writesize, usb_dkvr_writebyte_bulk_callback, dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &dev->submitted);

	/* send the data out the bulk port */
	retval = usb_submit_urb(urb, GFP_KERNEL);
	mutex_unlock(&dev->io_mutex);
	if (retval) {
		dev_err(&dev->interface->dev,
			"%s - failed submitting write urb, error %d\n",
			__func__, retval);
		goto error_unanchor;
	}

	/*
	 * release our reference to this urb, the USB core will eventually free
	 * it entirely
	 */
	usb_free_urb(urb);


	return writesize;

error_unanchor:
	usb_unanchor_urb(urb);
error:
	if (urb) {
		usb_free_coherent(dev->udev, writesize, buf, urb->transfer_dma);
		usb_free_urb(urb);
	}
	up(&dev->limit_sem);

exit:
	return retval;
}

/*
 *  Common interface
 */
static int dkvr_probe(struct usb_interface *interface,
		      const struct usb_device_id *id)
{
	struct usb_dkvr *dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	int i;
	int retval = -ENOMEM;
	unsigned char ctrl_input = '\0';

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&interface->dev, "Out of memory\n");
		goto error;
	}
	kref_init(&dev->kref);
	sema_init(&dev->limit_sem, WRITES_IN_FLIGHT);
	mutex_init(&dev->io_mutex);
	spin_lock_init(&dev->err_lock);
	init_usb_anchor(&dev->submitted);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	/* set up the endpoint information */
	iface_desc = interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (!dev->bulk_out_endpointAddr &&
		    usb_endpoint_is_bulk_out(endpoint)) {
			/* we found a bulk out endpoint */
			dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
		}
	}
	if (!dev->bulk_out_endpointAddr) {
		dev_err(&interface->dev,
			"Could not find both bulk-in and bulk-out endpoints\n");
		goto error;
	}

	retval = usb_control_msg(dev->udev,
			usb_sndctrlpipe(dev->udev, 0),
			0,
			USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0x0000, 0,
			0, 0,
			250);

	retval = usb_control_msg(dev->udev,
			usb_sndctrlpipe(dev->udev, 0),
			3,
			USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0x4138, 0,
			0, 0,
			250);

	retval = usb_control_msg(dev->udev,
			usb_sndctrlpipe(dev->udev, 0),
			11,
			USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0, 0,
			0, 0,
			250);

	retval = usb_control_msg(dev->udev,
			usb_sndctrlpipe(dev->udev, 0),
			11,
			USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0x01ff, 0,
			0, 0,
			250);

        retval = usb_control_msg(dev->udev,
                        usb_sndctrlpipe(dev->udev, 0),
                        12,
                        USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                        0x0000, 0,
                        &ctrl_input, 1,
                        250);

        retval = usb_control_msg(dev->udev,
                        usb_sndctrlpipe(dev->udev, 0),
                        0,
                        USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                        0x0002, 0,
                        0, 0,
                        250);

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	usb_dkvr_class_register(interface);

	return 0;

error:
	if (dev)
		/* this frees allocated memory */
		kref_put(&dev->kref, dkvr_delete);
	return retval;
}

static void dkvr_disconnect(struct usb_interface *interface)
{
	struct usb_dkvr *dev;
	int minor = interface->minor;

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	/* unregister leds */
	usb_dkvr_class_unregister();

	/* prevent more I/O from starting */
	mutex_lock(&dev->io_mutex);
	dev->interface = NULL;
	mutex_unlock(&dev->io_mutex);

	usb_kill_anchored_urbs(&dev->submitted);

	/* decrement our usage count */
	kref_put(&dev->kref, dkvr_delete);

	dev_info(&interface->dev, "USB Skeleton #%d now disconnected", minor);
}

static void dkvr_draw_down(struct usb_dkvr *dev)
{
	int time;

	time = usb_wait_anchor_empty_timeout(&dev->submitted, 1000);
	if (!time)
		usb_kill_anchored_urbs(&dev->submitted);
}

static int dkvr_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_dkvr *dev = usb_get_intfdata(intf);

	if (!dev)
		return 0;
	dkvr_draw_down(dev);
	return 0;
}

static int dkvr_resume(struct usb_interface *intf)
{
	return 0;
}

static int dkvr_pre_reset(struct usb_interface *intf)
{
	struct usb_dkvr *dev = usb_get_intfdata(intf);

	mutex_lock(&dev->io_mutex);
	dkvr_draw_down(dev);

	return 0;
}

static int dkvr_post_reset(struct usb_interface *intf)
{
	struct usb_dkvr *dev = usb_get_intfdata(intf);

	/* we are sure no URBs are active - no locking needed */
	dev->errors = -EPIPE;
	mutex_unlock(&dev->io_mutex);

	return 0;
}

static struct usb_driver dkvr_driver = {
	.name =		"usbdkvr",
	.probe =	dkvr_probe,
	.disconnect =	dkvr_disconnect,
	.suspend =	dkvr_suspend,
	.resume =	dkvr_resume,
	.pre_reset =	dkvr_pre_reset,
	.post_reset =	dkvr_post_reset,
	.id_table =	dkvr_table,
	.supports_autosuspend = 1,
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
static int dkvr_init(void)
{
	return usb_register(&dkvr_driver);
}

static void dkvr_exit(void)
{
	usb_deregister(&dkvr_driver);
}

module_init(dkvr_init);
module_exit(dkvr_exit);
#else
module_usb_driver(dkvr_driver);
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vinicius Tinti <viniciustinti@gmail.com>");
MODULE_DESCRIPTION("USB Denkovi relay module");
