#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>
#include <linux/string.h>

#include <asm/irq.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/completion.h>

//--------------------- define constants --------------------

#define DALI_drv				"dali"

#define DALI_OUT_PORT			4
#define DALI_IN_PORT			5

#define DALI_BIT_HALF_PERIOD	416666
#define DALI_BIT_QUARTER_PERIOD	208333
#define DALI_22TE				9166652
#define DALI_STOPBIT_VAL		3

#define DALI_STATUS_IDLE		0
#define DALI_STATUS_STARTBIT	1
#define DALI_STATUS_DATA		2
#define DALI_STATUS_STOPBIT		3
#define DALI_STATUS_RECEIVE		4
#define	DALI_STATUS_TRANSMIT	5
#define DALI_STATUS_ERROR		6

#define PARITY_FALSE			0
#define PARITY_TRUE				1

//--------------------- define structures -------------------

typedef struct manchesterCodeList_t {
    struct manchesterCodeList_t *pNext;
    unsigned char 		bitVal;
} manchesterCodeList_t;

typedef struct dali_bus_t {
    uint8_t				dali_status;
    uint8_t				stop_bits;
    uint8_t				parity;
    uint8_t				prev_bit;
    uint8_t				receive_bits;
    uint32_t			receive_data;
    uint8_t				seq;
} dali_bus_t;

typedef struct {
    struct work_struct	work;
    uint8_t				seq;
    uint16_t			data;
} work_t;

struct fifo_item {
    uint64_t 			data;
    struct list_head 	list;
};

struct fifo {
    struct list_head 	headlist;
    int16_t 			length;
};

//--------------------- variables ---------------------------

static wait_queue_head_t my_wait_queue;

static dev_t			first_dev;
static struct cdev 		kernel_cdev;
static struct class 	*cl_pointer;

static struct workqueue_struct  *qDaliSend;

struct completion 		data_read_done;
static dali_bus_t		dali_bus;
static manchesterCodeList_t* 	mRoot;

static struct hrtimer	read_timer;
static struct hrtimer	write_timer;
static struct hrtimer	wait_backward_timer;

static ktime_t 			dali_bit_half_period_time;
static ktime_t			last_irq;

static struct fifo 		reply_queue;

int						irq;
static DECLARE_WAIT_QUEUE_HEAD(dev_wait);

//--------------------- predefined functions ----------------

void* reply_queue_create(char *_data, int len);
void reply_queue_free(struct fifo_item* item);

uint64_t hex2int(char *hex);

static void manchesterListAddByte(char byte);
static void manchesterListAddVal(uint8_t val);

int fifo_init(struct fifo * fifo);
int fifo_exit(struct fifo * fifo);

int fifo_push(struct fifo * fifo, uint64_t data);
int64_t fifo_pop(struct fifo * fifo);

//--------------------- function ----------------------------


// функция обработки очереди на отправку
static void qDaliSend_worker( struct work_struct *work) {
    work_t *my_work = (work_t *)work;
    uint16_t number; 
    unsigned char  daliCommand[2];

    number = my_work->data;

    daliCommand[0] = number >> 8;
    daliCommand[1] = number & 0xff; 

    manchesterListAddVal(DALI_STOPBIT_VAL);
    manchesterListAddVal(DALI_STOPBIT_VAL);

    manchesterListAddByte(daliCommand[1]);
    manchesterListAddByte(daliCommand[0]);

    manchesterListAddVal(1);

    dali_bus.dali_status = DALI_STATUS_TRANSMIT;

    hrtimer_start(&write_timer, dali_bit_half_period_time, HRTIMER_MODE_REL);		// запускаем таймер на период Te

// тут ждем окончания отправки + backframe
    wait_for_completion(&data_read_done);											// ждем окончания отправки

    dali_bus.dali_status 	= DALI_STATUS_IDLE;

    if (my_work->seq == 0xff) my_work->seq = 0;

    dali_bus.seq 			= my_work->seq;

	hrtimer_start(&wait_backward_timer, DALI_22TE, HRTIMER_MODE_REL);				// запускаем таймер на 22 Te

    printk("Transmit dali packet: %02x%04x\n", my_work->seq, number);

    kfree((void *) work);

  return;
}

// функция обработки прерывания по изменению уровня шины
static irqreturn_t
rx_isr(int irq, void *ptr) {

    switch (dali_bus.dali_status) {
		case DALI_STATUS_TRANSMIT:
		    	return IRQ_HANDLED;
		    	break;
		case DALI_STATUS_IDLE:
		    	dali_bus.dali_status	= DALI_STATUS_STARTBIT;
		    	dali_bus.parity			= PARITY_FALSE;
		    	dali_bus.stop_bits		= 0;
		    	dali_bus.receive_bits	= 0;
		    	last_irq				= ktime_get();
		    	break;
		case DALI_STATUS_STARTBIT:
		    	dali_bus.dali_status	= DALI_STATUS_DATA;
		    	dali_bus.stop_bits		= 0;
		    	dali_bus.receive_bits	= 0;
		    	dali_bus.receive_data	= 0;
		    	break;
		default:
		    	break;
    }

    hrtimer_start(&read_timer, DALI_BIT_QUARTER_PERIOD, HRTIMER_MODE_REL);				// запускаем таймер на 1/2 Te

    return IRQ_HANDLED;
}

// функция чтения данных с шины dali
static enum hrtimer_restart 
read_timer_func (struct hrtimer * hrtimer) {
	ktime_t now, n, delta;
	int val;

	val 	= gpio_get_value(DALI_IN_PORT);

	if (val) val = 0;else val = 1;			// testing hack

	n 		= ktime_get();
	delta 	= n - last_irq;

	if (!dali_bus.parity) {
	    	dali_bus.prev_bit				= val;
	    	dali_bus.parity					= PARITY_TRUE;
	}else{
	    if (dali_bus.prev_bit != val) {
			dali_bus.parity 				= PARITY_FALSE;
			if (dali_bus.receive_bits != 0) {
		    	dali_bus.receive_data 		= dali_bus.receive_data << 1;
		    	dali_bus.receive_data 		= dali_bus.receive_data | val;
			}
			dali_bus.receive_bits++;
	    }
	}
	if (val == 0) dali_bus.stop_bits = 0; else dali_bus.stop_bits++;

	if (dali_bus.stop_bits < 5) {
	    now = hrtimer_cb_get_time(&read_timer);
	    hrtimer_forward(&read_timer, now, DALI_BIT_HALF_PERIOD);

		return HRTIMER_RESTART;
	}
	dali_bus.receive_data = dali_bus.receive_data | (dali_bus.seq << 16);
	printk("Receive dali packet (%d): %06x\n", dali_bus.receive_bits, dali_bus.receive_data);

	fifo_push(&reply_queue, dali_bus.receive_data);									// Сохраняем принятые данные в очереди

	dali_bus.dali_status	= DALI_STATUS_IDLE;
	dali_bus.stop_bits		= 0;
	dali_bus.receive_bits	= 0;
	dali_bus.receive_data	= 0;
	dali_bus.seq			= 0xff;

	return HRTIMER_NORESTART;
}

// функция таймера для отправки пакета через полупериод
static enum hrtimer_restart 
write_timer_func (struct hrtimer * hrtimer) {
	manchesterCodeList_t* pTemp = NULL;
	ktime_t now;

	if(mRoot != NULL) {
	    gpio_set_value(DALI_OUT_PORT, mRoot->bitVal);

	    if(mRoot->pNext != NULL) {
    		pTemp = mRoot;
    		mRoot = pTemp->pNext;
    		kfree(pTemp);
    		pTemp = NULL;
    		now = hrtimer_cb_get_time(&write_timer);

    		hrtimer_forward(&write_timer, now, dali_bit_half_period_time);
    		return HRTIMER_RESTART;
	    } else {
    		kfree(mRoot);
    		mRoot = NULL;
	    }
	}

	if(!completion_done (&data_read_done)) {
            complete (&data_read_done);
    }

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart
wait_backward_timer_func(struct hrtimer * hrtimer) {
	ktime_t now;

	if (dali_bus.dali_status != DALI_STATUS_IDLE) {
		now = hrtimer_cb_get_time(&wait_backward_timer);
		hrtimer_forward(&wait_backward_timer, now, DALI_22TE);

		return HRTIMER_RESTART;
	}
	dali_bus.seq = 0xff;

	return HRTIMER_NORESTART;
}

// функция преобразования бита в манчестерский код
static void
manchesterListAddVal(uint8_t val) {
	manchesterCodeList_t* pTemp = kmalloc(sizeof(manchesterCodeList_t), GFP_KERNEL);
	pTemp->pNext 				= kmalloc(sizeof(manchesterCodeList_t), GFP_KERNEL);
	pTemp->pNext->pNext 		= NULL;

	switch(val) {
    	    case 0:
    			pTemp->bitVal 			= 1;
    			pTemp->pNext->bitVal 	= 0;
    			break;

    	    case DALI_STOPBIT_VAL:
    			pTemp->bitVal 			= 1;
    			pTemp->pNext->bitVal 	= 1;
    			break;

    	    default:
    			pTemp->bitVal 			= 0;
    			pTemp->pNext->bitVal 	= 1;
    			break;
	}

	if(mRoot != NULL) pTemp->pNext->pNext = mRoot;
	mRoot = pTemp;
}

// функция преобразования байта данных в манчестерский код
static void
manchesterListAddByte(char byte) {
	int i;

	for (i = 0; i < 8; i++) manchesterListAddVal(byte & (0x1 << i));
}

uint64_t 
hex2int(char *hex) {
    uint64_t val = 0;

    while (*hex) {
        uint8_t byte = *hex++; 
        if (byte >= '0' && byte <= '9') byte = byte - '0';
        else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
        else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;
        val = (val << 4) | (byte & 0xF);
    }
    return val;
}

//------------------------------------------------------------------------------------------------------------------
int 
fifo_init(struct fifo *fifo) {
    INIT_LIST_HEAD(&(fifo->headlist));

    fifo->length = 0;

    return 0;
}

int 
fifo_exit(struct fifo* fifo) {
    struct list_head *pos, *q;
    struct fifo_item* item;
	int res = 0;

    list_for_each_safe(pos, q, &(fifo->headlist)) {
        item = list_entry(pos, struct fifo_item, list);
        list_del(pos);
        kfree(item);
    }

    return res;
}

int 
fifo_push(struct fifo* fifo, uint64_t data) {
    struct fifo_item* item;
    int res = -1;

    item = (struct fifo_item*)kmalloc(sizeof(struct fifo_item), GFP_KERNEL);

    if(NULL != item) {
        item->data = data;
        list_add_tail(&(item->list), &(fifo->headlist));
        fifo->length++;
        wake_up_interruptible(&my_wait_queue);
    }

    return res;
}

int64_t 
fifo_pop(struct fifo* fifo) {
    int64_t res = -1;
	struct fifo_item* item;

    item = list_entry(fifo->headlist.next, struct fifo_item, list);

    if(!list_empty(&(fifo->headlist))) {
        res = item->data;
        list_del(&(item->list));
        kfree(item);
        fifo->length--;
        wake_up_interruptible(&my_wait_queue);
    }
    return res;
}

//------------------------------------------------------------------------------------------------------------------
static int 
dev_open(struct inode *inode, struct file *file) {
    return 0;
}
 
static int 
dev_close(struct inode *inode, struct file *file) {
    return 0;
}

static ssize_t 
dev_read(struct file* f, const char *buf, size_t count, loff_t *f_pos) {
    uint64_t _data;
    char data[7];
    long ret;

    if (count != 6) {
		printk(KERN_ALERT "Command length incorrect\n");
		return -EFAULT;
    }

    if (qDaliSend) {
		work_t * work = (work_t *)kmalloc(sizeof(work_t), GFP_KERNEL);
		if (work) {
	    	ret = copy_from_user(&data[0], buf, count);
	    	data[count] = '\0';
	    	_data = hex2int(&data[0]);
	    	work->seq  = (uint8_t) ((_data & 0xff0000) >> 16);
	    	work->data = (uint16_t) _data & 0xffff;

	    	INIT_WORK((struct work_struct *)work, qDaliSend_worker);
    	    ret = queue_work(qDaliSend, (struct work_struct *)work);
		}
    }
    return count;
}

static ssize_t 
dev_write(struct file* f, char *buf, size_t count, loff_t *f_pos) {
	int64_t data;

	if(f->f_flags & O_NONBLOCK){
		return -EAGAIN;
	}

	wait_event_interruptible(my_wait_queue, reply_queue.length != 0);

	data = fifo_pop(&reply_queue);
	if (data != -1){
	    sprintf(buf, "%06x\n", (unsigned int) data);
	    return strlen(buf);
	}
    return 0;
}

static unsigned int
dev_poll(struct file *f, poll_table *wait) {
     poll_wait(f, &dev_wait, wait);
     if (reply_queue.length > 0) return POLLIN | POLLRDNORM;

     return 0;
 }
//------------------------------------------------------------------------------------------------------------------

 
static struct file_operations 
FileOps = {
    .owner        = THIS_MODULE,
    .open         = dev_open,
    .read         = dev_write,
    .write        = dev_read,
    .release      = dev_close,
    .poll         = dev_poll
};

static int 
init_dali_driver(void) { 
	int ret;

	printk(KERN_INFO "DALI Driver Kernel Module initialisation v.1\n");

	if (!gpio_is_valid(DALI_OUT_PORT)) {
		printk(KERN_INFO "Invalid output GPIO\n");
		return -ENODEV;
	}
	gpio_request(DALI_OUT_PORT, "sysfs");
	gpio_direction_output(DALI_OUT_PORT, 1);
	gpio_export(DALI_OUT_PORT, false);

	gpio_request(DALI_IN_PORT, "sysfs");
	gpio_direction_input(DALI_IN_PORT);
	gpio_export(DALI_IN_PORT, false);

	if (alloc_chrdev_region(&first_dev, 0, 1, DALI_drv) < 0) {
	    printk(KERN_ALERT "Device Registration failed\n");
	    return -1;
	}

	if ((cl_pointer = class_create(THIS_MODULE, "Dali_drv")) == NULL) {
		printk(KERN_ALERT "Class creation failed\n");
		unregister_chrdev_region(first_dev, 1);
		return -1;
	}
	 
	if (device_create(cl_pointer, NULL, first_dev, NULL, DALI_drv) == NULL) {
		printk(KERN_ALERT "Device creation failed\n");
		class_destroy(cl_pointer);
		unregister_chrdev_region(first_dev, 1);
		return -1;
	}

	cdev_init(&kernel_cdev, &FileOps);
	if (cdev_add( &kernel_cdev, first_dev, 1) == -1) {
		printk(KERN_ALERT "Device addition failed\n");
		device_destroy(cl_pointer, first_dev);
		class_destroy(cl_pointer);
		unregister_chrdev_region(first_dev, 1);
		return -1;
	}

	init_waitqueue_head(&my_wait_queue);

	dali_bus.dali_status			= DALI_STATUS_IDLE;
	dali_bus.parity					= PARITY_FALSE;
	dali_bus.stop_bits				= 0;
	dali_bus.prev_bit				= 0;
	dali_bus.receive_bits			= 0;
	dali_bus.receive_data			= 0;
	dali_bus.seq					= 0xff;

	dali_bit_half_period_time		= ktime_set(0, DALI_BIT_HALF_PERIOD);

	hrtimer_init(&read_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	read_timer.function				= read_timer_func;

	hrtimer_init(&write_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	write_timer.function			= write_timer_func;

	hrtimer_init(&wait_backward_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	wait_backward_timer.function	= wait_backward_timer_func;

	irq = gpio_to_irq(DALI_IN_PORT);

	if (irq < 0) {
	    printk("Unable to get irq number for GPIO %d, error %d\n", DALI_IN_PORT, irq);
	    device_destroy(cl_pointer, first_dev);
	    class_destroy(cl_pointer);
	    unregister_chrdev_region(first_dev, 1);
	    return -1;

	}

	if (request_irq(irq, rx_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING , "dali-driver", NULL)) 
	    printk(KERN_ERR "Unable to request IRQ: %d\n", ret);

	qDaliSend = create_workqueue("dali_queue");

	init_completion(&data_read_done);

	fifo_init(&reply_queue);
	return 0;
}


void
exit_dali_driver(void) {
	printk(KERN_INFO "DALI Driver Kernel Module exit\n");

	fifo_exit(&reply_queue);

	if(!completion_done (&data_read_done)) {
    	complete (&data_read_done);
    }

	flush_workqueue(qDaliSend);
	destroy_workqueue(qDaliSend);

	free_irq(irq, NULL);

	gpio_free(DALI_OUT_PORT);
	gpio_free(DALI_IN_PORT);
  
	if (hrtimer_cancel(&write_timer))	printk(KERN_ALERT "Write timer still running!\n");
	else								printk(KERN_ALERT "Write timer cancelled!\n");

	if (hrtimer_cancel(&read_timer))	printk(KERN_ALERT "Read timer still running!\n");
	else								printk(KERN_ALERT "Read timer cancelled!\n");

	cdev_del(&kernel_cdev);
	device_destroy(cl_pointer, first_dev);
	class_destroy(cl_pointer); 
	unregister_chrdev_region(first_dev, 1);
	 
	printk(KERN_ALERT "Device unregistered\n");
}
 
 
module_init(init_dali_driver);
module_exit(exit_dali_driver);
 
MODULE_AUTHOR("Alice Trade Ltd <info@alice-trade.ru>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DALI Driver for Raspberry");
