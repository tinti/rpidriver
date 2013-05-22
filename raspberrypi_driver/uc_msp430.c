#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/gpio.h>
#include <linux/interrupt.h>

#define UC_MSP430_INPUT_GPIO 22
#define UC_MSP430_RED_GPIO 23
#define UC_MSP430_GREEN_GPIO 24
#define UC_MSP430_IRQ_GPIO 25

#define UC_MSP430_NAME "uc_msp430"

#define D(level, ...)   printk(level UC_MSP430_NAME ": " __VA_ARGS__)
//#define UC_MSP430_P(level, ...)   ((void)0)

static irqreturn_t uc_msp430_handler(int irq, void *dev);
static void uc_msp430_tasklet_handler(unsigned long data);

static struct gpio uc_msp430_gpio_array[] = {
	{ UC_MSP430_INPUT_GPIO, GPIOF_IN,  "uc_msp430_input"   },
	{ UC_MSP430_RED_GPIO, GPIOF_OUT_INIT_LOW, "msp430_red" },
	{ UC_MSP430_GREEN_GPIO, GPIOF_OUT_INIT_HIGH,  "uc_msp430_green" },
	{ UC_MSP430_IRQ_GPIO, GPIOF_IN,  "uc_msp430_irq"  },
};

unsigned int uc_msp430_irq_addr = 0;
char uc_msp430_driver[] = UC_MSP430_NAME;

DECLARE_TASKLET(uc_msp430_tasklet, uc_msp430_tasklet_handler, 0);

static void uc_msp430_tasklet_handler(unsigned long data)
{
	D(KERN_INFO, "tasklet called.\n");

        gpio_set_value(UC_MSP430_RED_GPIO,
			gpio_get_value(UC_MSP430_INPUT_GPIO));
}

static irqreturn_t uc_msp430_handler(int irq, void *dev)
{
	tasklet_schedule(&uc_msp430_tasklet);

	return IRQ_HANDLED;
}

static int uc_msp430_init(void)
{
	int ecode = 0;
        D(KERN_INFO, "init.\n");

	ecode = gpio_request_array(uc_msp430_gpio_array,
			ARRAY_SIZE(uc_msp430_gpio_array));
	if (ecode) {
		D(KERN_ERR, "fail to request gpio pins with error %d.\n",
				ecode);
		goto out2;
	}

	uc_msp430_irq_addr = gpio_to_irq(UC_MSP430_IRQ_GPIO);

	if (uc_msp430_irq_addr == 0) {
		D(KERN_ERR, "fail to convert gpio to irq with error %d.\n",
				ecode);
		goto out1;
	}

	ecode = request_irq(uc_msp430_irq_addr, uc_msp430_handler,
			IRQF_TRIGGER_FALLING, uc_msp430_driver,
			(void*) uc_msp430_driver);
	if (ecode) {
		D(KERN_ERR, "fail to request irq with error %d.\n", ecode);
		goto out1;
	}

        return 0;

out1:
	gpio_free_array(uc_msp430_gpio_array, ARRAY_SIZE(uc_msp430_gpio_array));
out2:
	return ecode;
}

static void uc_msp430_exit(void)
{
        D(KERN_INFO, "exit.\n");

	gpio_set_value(UC_MSP430_RED_GPIO, 1);
	gpio_set_value(UC_MSP430_GREEN_GPIO, 0);

	free_irq(uc_msp430_irq_addr, (void*) uc_msp430_driver);

	gpio_free_array(uc_msp430_gpio_array, ARRAY_SIZE(uc_msp430_gpio_array));

        D(KERN_INFO "exited.\n");
}

module_init(uc_msp430_init);
module_exit(uc_msp430_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vinicius Tinti <viniciustinti@gmail.com>");
MODULE_DESCRIPTION("Export MSP430 functionalities to Linux");
