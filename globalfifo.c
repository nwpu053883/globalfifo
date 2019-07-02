/*********************************************************************************
 *      Copyright:  (C) 2014 zhouguangfeng<zhouguangfeng91@gmail.com>
 + plat_globalfifo.c
/*********************************************************************************
 *      Copyright:  (C) 2014 zhouguangfeng<zhouguangfeng91@gmail.com>
 *                  All rights reserved.
 *
 *       Filename:  plat_globalfifo.c
 *    Description:  This file is a commom platform driver
 *
 *        Version:  1.0.0(08/19/2014)
 *         Author:  zhouguangfeng <zhouguangfeng91@gmail.com>
 *      ChangeLog:  1, Release initial version on "08/19/2014 02:31:17 PM"
 *
 ********************************************************************************/
#include <linux/fs.h>//struct file_operations
#include <linux/types.h>//special type definition,like dev_t off_t defined by typedef
#include <linux/init.h> // init and exit
#include <linux/module.h>//support module load and unload
#include <linux/errno.h>
#include <linux/mm.h> //memory mannage ,include kmalloc.kfree and so on
#include <linux/sched.h>
#include <linux/cdev.h> //char device structure definition
#include <asm/io.h>     //io operation function ,like ioremap,iowrite
// #include <asm/system.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>   //for ioctl command
#include <asm/uaccess.h>
#include <linux/platform_device.h> //platform support
#include <linux/kernel.h>
#include <linux/device.h> //class_create() and device_create()


#define GLOBALFIFO_SIZE       0x1000  /* 4K */
#define NAME                  "globalfifo"
#define KELNEL_OLD             0   /* decsion ioctl() */

#ifndef GLOBALFIFO_MAJOR
#define GLOBALFIFO_MAJOR       0
#endif


//#define GLOBALFIFO_CLEAR      0x17
//#define MEM_CLEAR           __IO (GLOBALFIFO_CLEAR, 0x20)
#define MEM_CLEAR              0x20


static int globalfifo_major = GLOBALFIFO_MAJOR;
static int globalfifo_minor = 0;


/*  ============================ Platform Device part =============================== */

struct globalfifo_dev
{
    struct cdev cdev;
    unsigned int current_len;
    unsigned char mem[GLOBALFIFO_SIZE];
    struct class   *class;


    //struct semaphrore sem;
   // wait_queue_t r_wait;
    //wait_queue_t r_wait;
} globalfifo_dev;

static void plat_release(struct device * dev)
{
    return;
}

static struct platform_device globalfifo_device = {
        .name = "globalfifo",
        .id   = 1,
        .dev  = {
            .release = plat_release,
        },
};



/*  ===================== globalfifo driver part ===========================*/

int globalfifo_open(struct inode *inode, struct file *filp)
{
    struct globalfifo_dev *dev;

    dev = container_of(inode->i_cdev, struct globalfifo_dev, cdev);
    filp->private_data = dev;

    return 0;
}

int globalfifo_release(struct inode *inode, struct file *filp)
{
    return 0;
}


#if KELNEL_OLD
static ssize_t globalfifo_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct globalfifo_dev *dev = filp->private_data;
    switch(cmd)
    {
         case  MEM_CLEAR:
            memset(dev->mem, 0, GLOBALFIFO_SIZE);
            printk(KERN_INFO "globalfifo is set to zero\n");
            break;

         default:
            return -EINVAL;
    }

    return 0;
}
#endif

static ssize_t globalfifo_read(struct file *filp, char __user *buf, size_t size, loff_t *opps)
{
    unsigned long p = *opps;
    unsigned int count = size;
    int ret = 0;

    struct globalfifo_dev *dev = filp->private_data;

    if(p >= GLOBALFIFO_SIZE)
    {
         return count ? -ENXIO : 0;
    }
    if(count > GLOBALFIFO_SIZE - p)
    {
         count = GLOBALFIFO_SIZE - p;
    }

    if(copy_to_user(buf, (void *)((dev->mem)+p), count))
    {
         ret = -EFAULT;
    }
    else
    {
        *opps += count;
         ret = count;
         printk(KERN_INFO"read %u bytes(s) from %lu\n", count, p);
    }

    return ret;
}

static ssize_t globalfifo_write(struct file *filp, const char __user *buf, size_t size, loff_t *opps)
{
    unsigned long  p = *opps;
    unsigned int count = size;
    int ret;

    struct globalfifo_dev *dev = filp->private_data;

    if(p >= GLOBALFIFO_SIZE)
    {
        return count ? -ENXIO : 0;
    }

    if(count > GLOBALFIFO_SIZE - p)
    {
        count = GLOBALFIFO_SIZE - p;
    }

    if(copy_from_user(((dev->mem)+p), buf, count))
    {
        ret = -EFAULT;
    }
    else
    {
       *opps =+ count;
        ret = count;
        printk(KERN_INFO "written %u bytes(s) from %lu\n", count, p);
    }
    return ret;
}


#if 1
static loff_t globalfifo_llseek(struct file *filp, loff_t offset, int orig)
{
    loff_t ret = 0;

    switch(orig)
    {
        case 0:
            if(offset <0 )
            {
                ret = -EINVAL;
                break;
            }

            if((unsigned int )offset > GLOBALFIFO_SIZE)
            {
                ret = -EINVAL;
                break;
            }
            filp->f_pos = (unsigned int)offset;
            ret = filp->f_pos;
            break;

        case 1:
            if((filp->f_pos + offset) > GLOBALFIFO_SIZE)
            { ret = -EINVAL;
               break;
            }

            if((filp->f_pos + offset) < 0)
            {
               ret = -EINVAL;
               break;
            }
            filp->f_pos += offset;
            ret = filp->f_pos;
            break;

        default:
            ret = -EINVAL;
               break;
    }

    return ret;
}
#endif

static const struct file_operations globalfifo_fops ={
    .owner = THIS_MODULE,
    .read = globalfifo_read,
    .write = globalfifo_write,
    .open = globalfifo_open,
    .release = globalfifo_release,
    .llseek = globalfifo_llseek,

#if KELNEL_OLD
    .unlocked_ioctl = globalfifo_ioctl,
#endif
};

static int  globalfifo_probe(struct platform_device *dev)
{
    int ret;
    dev_t devno;

    /* Alloc for device major */
    if(globalfifo_major)
    {
        devno = MKDEV(globalfifo_major, globalfifo_minor);
        ret = register_chrdev_region(devno, 1, NAME);
    }
    else
    {
        ret = alloc_chrdev_region(&devno, 0, 1, NAME);
        globalfifo_major= MAJOR(devno);
    }

    /*  Alloc for device major failure */
    if (ret < 0)
    {
        printk("%s driver can't get major %d\n", NAME, globalfifo_major);
        return ret;
    }

/* Initialize globalfifo structure and register cdev*/
    memset(&globalfifo_dev, 0, sizeof(struct globalfifo_dev));
    cdev_init (&(globalfifo_dev.cdev), &globalfifo_fops);
    globalfifo_dev.cdev.owner  = THIS_MODULE;

    ret = cdev_add (&(globalfifo_dev.cdev), devno , 1);
    if (ret)
    {
         printk (KERN_NOTICE "error %d add %s device", ret, NAME);
         goto fail_cdev_add;
    }

    globalfifo_dev.class = class_create(THIS_MODULE, NAME);
    if(IS_ERR(globalfifo_dev.class))
    {
        printk("%s driver create class failure\n", NAME);
        goto fail_class;
    }

    device_create(globalfifo_dev.class, NULL, devno, NULL, NAME);

    return 0;
fail_class:
    cdev_del(&(globalfifo_dev.cdev));

fail_cdev_add:
    unregister_chrdev_region(devno, 1);
    printk("failure to insmod!\n");
    return ret;
}


static int  globalfifo_remove(struct platform_device *pdev)
{
    dev_t devno = MKDEV(globalfifo_major, globalfifo_minor);

    cdev_del(&globalfifo_dev.cdev);
    device_destroy(globalfifo_dev.class, devno);
    class_destroy(globalfifo_dev.class);

    unregister_chrdev_region(devno, 1);
    printk("s3c %s driver removed\n", NAME);

    return 0;
}


static struct platform_driver globalfifo_driver = {
    .probe  = globalfifo_probe,
    .remove = globalfifo_remove,
    .driver = {
        .name = "globalfifo",
        .owner = THIS_MODULE,
    },
};


static int __init globalfifo_init(void)
{
    int       ret = 0;

    ret = platform_device_register(&globalfifo_device);
    if(ret)
    {
        printk(KERN_ERR "%s:%d: Can't register platform device %d\n", __FUNCTION__, __LINE__ ,ret);
        goto fail_reg_plat_dev;
    }
    printk("Register S3C %s Platform Device successfully.\n", NAME);

    ret = platform_driver_register(&globalfifo_driver);
    if(ret)
    {
        printk(KERN_ERR "%s:%d: Can't register platform driver %d\n", __FUNCTION__, __LINE__, ret);
        goto fail_reg_plat_drv;
    }
    printk("Register S3C %s Platform Driver successfully.\n", NAME);

    return 0;

fail_reg_plat_drv:
    platform_device_unregister(&globalfifo_device);
fail_reg_plat_dev:
    return ret;
}

static void  __exit globalfifo_exit(void)
{
    printk("%s():%s remove %d platform drvier\n", __FUNCTION__, NAME, __LINE__);
    platform_driver_unregister(&globalfifo_driver);

    printk("%s():%s remove %d platform device\n", __FUNCTION__, NAME, __LINE__);
    platform_device_unregister(&globalfifo_device);
}

module_init(globalfifo_init);
module_exit(globalfifo_exit);
MODULE_ALIAS("platform: globalfifo");
MODULE_LICENSE("GPL");
