// SPDX-License-Identifier: GPL-2.0
/*
 * Arduino TTY Mouse Driver - Direct TTY Access Version
 * Compatible with WSL2 and systems without device tree
 *
 * This version bypasses serdev and accesses /dev/ttyACM0 directly
 * More compatible with virtualized environments like WSL2
 *
 * Author: Arduino TTY Mouse Driver Developer
 * Version: 4.1 (WSL2 Compatible - Fixed for modern kernels)
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/tty.h>
#include <linux/serial.h>

#define DRIVER_NAME "arduino-tty-mouse"
#define DEVICE_NAME "Arduino TTY Mouse"
#define TTY_DEVICE_PATH "/dev/ttyACM0"
#define SERIAL_BUFFER_SIZE 512
#define MAX_PACKET_SIZE 16
#define CONNECTION_TIMEOUT_MS 5000
#define SERIAL_BAUD_RATE 9600

/* Communication protocol with Arduino */
#define PROTOCOL_START_BYTE    0xAA
#define PROTOCOL_END_BYTE      0x55
#define BUTTON_LEFT_MASK       0x01
#define BUTTON_RIGHT_MASK      0x02
#define HAS_MOVEMENT_MASK      0x80

/* Mouse movement constants */
#define MAX_MOUSE_MOVEMENT     127
#define MIN_MOUSE_MOVEMENT     -128
#define MOVEMENT_SCALE         2

/* Timing constants */
#define HEARTBEAT_INTERVAL_SEC     3
#define TIMEOUT_CHECK_INTERVAL_SEC 2
#define READ_INTERVAL_MS          10

struct arduino_packet {
    u8 start;           // 0xAA
    u8 buttons;         // bit0=left, bit1=right, bit7=has_movement
    s8 delta_x;         // X movement (-128 to 127)
    s8 delta_y;         // Y movement (-128 to 127)
    u8 checksum;        // Simple checksum
    u8 end;             // 0x55
} __packed;

struct arduino_tty_ctx {
    struct input_dev *input;
    struct task_struct *read_thread;

    /* TTY file access */
    struct file *tty_file;

    /* Button and movement states */
    bool left_pressed;
    bool right_pressed;
    bool device_connected;
    bool supports_movement;
    bool driver_active;

    /* Timer for timeout detection and heartbeat */
    struct timer_list timeout_timer;
    struct timer_list heartbeat_timer;
    unsigned long last_data_time;

    /* Statistics */
    unsigned long packets_received;
    unsigned long packets_invalid;
    unsigned long button_events;
    unsigned long movement_events;
    unsigned long checksum_errors;
    unsigned long read_errors;

    /* Synchronization */
    struct mutex ctx_mutex;
    wait_queue_head_t read_wait;
};

static struct arduino_tty_ctx *global_ctx = NULL;
static struct class *arduino_class = NULL;
static struct device *arduino_device = NULL;

/*
 * Calculate simple checksum for packet validation
 */
static u8 calculate_checksum(const struct arduino_packet *pkt)
{
    return (pkt->start + pkt->buttons + pkt->delta_x + pkt->delta_y) & 0xFF;
}

/*
 * Process packet received from Arduino
 */
static void process_arduino_packet(struct arduino_tty_ctx *ctx,
                                   const struct arduino_packet *pkt)
{
    bool left_state, right_state, has_movement;
    bool left_changed, right_changed;
    s8 delta_x, delta_y;

    if (!ctx || !pkt)
        return;

    /* Validate packet structure */
    if (pkt->start != PROTOCOL_START_BYTE || pkt->end != PROTOCOL_END_BYTE) {
        ctx->packets_invalid++;
        return;
    }

    /* Validate checksum */
    if (pkt->checksum != calculate_checksum(pkt)) {
        ctx->checksum_errors++;
        ctx->packets_invalid++;
        return;
    }

    /* Extract packet data */
    left_state = !!(pkt->buttons & BUTTON_LEFT_MASK);
    right_state = !!(pkt->buttons & BUTTON_RIGHT_MASK);
    has_movement = !!(pkt->buttons & HAS_MOVEMENT_MASK);
    delta_x = pkt->delta_x;
    delta_y = pkt->delta_y;

    if (!mutex_trylock(&ctx->ctx_mutex))
        return;

    if (!ctx->driver_active) {
        mutex_unlock(&ctx->ctx_mutex);
        return;
    }

    /* Detect button changes */
    left_changed = (left_state != ctx->left_pressed);
    right_changed = (right_state != ctx->right_pressed);

    /* Update states */
    ctx->left_pressed = left_state;
    ctx->right_pressed = right_state;
    ctx->last_data_time = jiffies;
    ctx->supports_movement = has_movement;
    ctx->device_connected = true;

    /* Send input events */
    if (ctx->input) {
        /* Button events */
        if (left_changed) {
            input_report_key(ctx->input, BTN_LEFT, left_state);
            ctx->button_events++;
        }

        if (right_changed) {
            input_report_key(ctx->input, BTN_RIGHT, right_state);
            ctx->button_events++;
        }

        /* Movement events */
        if (has_movement && (delta_x != 0 || delta_y != 0)) {
            /* Scale movement for better sensitivity */
            delta_x = (delta_x * MOVEMENT_SCALE);
            delta_y = (delta_y * MOVEMENT_SCALE);

            input_report_rel(ctx->input, REL_X, delta_x);
            input_report_rel(ctx->input, REL_Y, delta_y);
            ctx->movement_events++;
        }

        input_sync(ctx->input);
    }

    ctx->packets_received++;

    pr_debug("arduino_tty: Packet: L=%d R=%d X=%d Y=%d (buttons=0x%02x)\n",
             left_state, right_state, delta_x, delta_y, pkt->buttons);

    mutex_unlock(&ctx->ctx_mutex);
}

/*
 * Read data from TTY device - Fixed for modern kernels
 */
static ssize_t read_from_tty(struct file *file, char *buffer, size_t count)
{
    ssize_t result;
    loff_t pos = 0;

    if (!file || !buffer)
        return -EINVAL;

    result = kernel_read(file, buffer, count, &pos);

    return result;
}

/*
 * Write data to TTY device - Fixed for modern kernels
 */
static ssize_t write_to_tty(struct file *file, const char *buffer, size_t count)
{
    ssize_t result;
    loff_t pos = 0;

    if (!file || !buffer)
        return -EINVAL;

    result = kernel_write(file, buffer, count, &pos);

    return result;
}

/*
 * Main reading thread
 */
static int arduino_read_thread(void *data)
{
    struct arduino_tty_ctx *ctx = (struct arduino_tty_ctx *)data;
    char buffer[SERIAL_BUFFER_SIZE];
    int buffer_pos = 0;
    struct arduino_packet packet;
    int i;
    ssize_t bytes_read;

    pr_info("arduino_tty: Reading thread started\n");

    while (!kthread_should_stop() && ctx->driver_active) {
        if (!ctx->tty_file) {
            msleep(1000);
            continue;
        }

        /* Read data from TTY */
        bytes_read = read_from_tty(ctx->tty_file, buffer + buffer_pos,
                                   sizeof(buffer) - buffer_pos);

        if (bytes_read > 0) {
            buffer_pos += bytes_read;

            /* Process complete packets */
            while (buffer_pos >= sizeof(struct arduino_packet)) {
                /* Find start byte */
                for (i = 0; i < buffer_pos - sizeof(struct arduino_packet) + 1; i++) {
                    if (buffer[i] == PROTOCOL_START_BYTE) {
                        /* Copy potential packet */
                        memcpy(&packet, &buffer[i], sizeof(struct arduino_packet));

                        /* Process packet */
                        process_arduino_packet(ctx, &packet);

                        /* Remove processed data from buffer */
                        memmove(buffer, &buffer[i + sizeof(struct arduino_packet)],
                                buffer_pos - i - sizeof(struct arduino_packet));
                        buffer_pos -= (i + sizeof(struct arduino_packet));
                        break;
                    }
                }

                /* If no start byte found, remove first byte */
                if (i >= buffer_pos - sizeof(struct arduino_packet) + 1) {
                    memmove(buffer, buffer + 1, buffer_pos - 1);
                    buffer_pos--;
                    break;
                }
            }
        } else if (bytes_read < 0) {
            ctx->read_errors++;
            pr_debug("arduino_tty: Read error: %zd\n", bytes_read);
        }

        /* Sleep briefly to avoid consuming too much CPU */
        msleep(READ_INTERVAL_MS);
    }

    pr_info("arduino_tty: Reading thread stopped\n");
    return 0;
}

/*
 * Open TTY device
 */
static int open_tty_device(struct arduino_tty_ctx *ctx)
{
    if (ctx->tty_file) {
        pr_warn("arduino_tty: TTY device already open\n");
        return 0;
    }

    ctx->tty_file = filp_open(TTY_DEVICE_PATH, O_RDWR | O_NOCTTY | O_NONBLOCK, 0);
    if (IS_ERR(ctx->tty_file)) {
        pr_err("arduino_tty: Failed to open %s: %ld\n",
               TTY_DEVICE_PATH, PTR_ERR(ctx->tty_file));
        ctx->tty_file = NULL;
        return PTR_ERR(ctx->tty_file);
    }

    pr_info("arduino_tty: Opened %s successfully\n", TTY_DEVICE_PATH);
    return 0;
}

/*
 * Close TTY device
 */
static void close_tty_device(struct arduino_tty_ctx *ctx)
{
    if (ctx->tty_file) {
        filp_close(ctx->tty_file, NULL);
        ctx->tty_file = NULL;
        pr_info("arduino_tty: Closed TTY device\n");
    }
}

/*
 * Send heartbeat command to Arduino
 */
static void send_heartbeat(struct arduino_tty_ctx *ctx)
{
    const char heartbeat_cmd[] = "PING\n";
    ssize_t result;

    if (ctx && ctx->tty_file && ctx->driver_active) {
        result = write_to_tty(ctx->tty_file, heartbeat_cmd, sizeof(heartbeat_cmd) - 1);
        if (result < 0) {
            pr_debug("arduino_tty: Failed to send heartbeat: %zd\n", result);
        }
    }
}

/*
 * Timeout timer - detect disconnection
 */
static void arduino_timeout_timer(struct timer_list *timer)
{
    struct arduino_tty_ctx *ctx =
    container_of(timer, struct arduino_tty_ctx, timeout_timer);

    if (!ctx || !ctx->driver_active)
        return;

    if (mutex_trylock(&ctx->ctx_mutex)) {
        if (time_after(jiffies, ctx->last_data_time + msecs_to_jiffies(CONNECTION_TIMEOUT_MS))) {
            if (ctx->device_connected) {
                ctx->device_connected = false;
                pr_warn("arduino_tty: Arduino disconnected (timeout)\n");

                /* Clear button states */
                if (ctx->input) {
                    if (ctx->left_pressed) {
                        input_report_key(ctx->input, BTN_LEFT, false);
                        ctx->left_pressed = false;
                    }
                    if (ctx->right_pressed) {
                        input_report_key(ctx->input, BTN_RIGHT, false);
                        ctx->right_pressed = false;
                    }
                    input_sync(ctx->input);
                }
            }
        }

        if (ctx->driver_active) {
            mod_timer(&ctx->timeout_timer, jiffies + TIMEOUT_CHECK_INTERVAL_SEC * HZ);
        }

        mutex_unlock(&ctx->ctx_mutex);
    }
}

/*
 * Heartbeat timer - send periodic ping to Arduino
 */
static void arduino_heartbeat_timer(struct timer_list *timer)
{
    struct arduino_tty_ctx *ctx =
    container_of(timer, struct arduino_tty_ctx, heartbeat_timer);

    if (!ctx || !ctx->driver_active)
        return;

    send_heartbeat(ctx);

    if (ctx->driver_active) {
        mod_timer(&ctx->heartbeat_timer, jiffies + HEARTBEAT_INTERVAL_SEC * HZ);
    }
}

/*
 * Show statistics via sysfs
 */
static ssize_t stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct arduino_tty_ctx *ctx = global_ctx;
    ssize_t result;

    if (!ctx)
        return -ENODEV;

    if (!mutex_trylock(&ctx->ctx_mutex))
        return -EBUSY;

    result = scnprintf(buf, PAGE_SIZE,
                       "Packets received: %lu\n"
                       "Packets invalid: %lu\n"
                       "Checksum errors: %lu\n"
                       "Button events: %lu\n"
                       "Movement events: %lu\n"
                       "Read errors: %lu\n"
                       "Connected: %s\n"
                       "Supports movement: %s\n"
                       "Left pressed: %s\n"
                       "Right pressed: %s\n"
                       "Driver active: %s\n"
                       "TTY device: %s\n"
                       "TTY open: %s\n",
                       ctx->packets_received,
                       ctx->packets_invalid,
                       ctx->checksum_errors,
                       ctx->button_events,
                       ctx->movement_events,
                       ctx->read_errors,
                       ctx->device_connected ? "yes" : "no",
                       ctx->supports_movement ? "yes" : "no",
                       ctx->left_pressed ? "yes" : "no",
                       ctx->right_pressed ? "yes" : "no",
                       ctx->driver_active ? "yes" : "no",
                       TTY_DEVICE_PATH,
                       ctx->tty_file ? "yes" : "no");

    mutex_unlock(&ctx->ctx_mutex);

    return result;
}
static DEVICE_ATTR_RO(stats);

/*
 * Send configuration command to Arduino
 */
static ssize_t config_store(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t count)
{
    struct arduino_tty_ctx *ctx = global_ctx;
    char command[64];
    size_t cmd_len;
    ssize_t result;

    if (!ctx || !ctx->driver_active || count >= sizeof(command) - 2)
        return -EINVAL;

    if (!ctx->tty_file)
        return -ENODEV;

    /* Safe copy with bounds checking */
    cmd_len = min(count, sizeof(command) - 2);
    memcpy(command, buf, cmd_len);
    command[cmd_len] = '\0';

    /* Remove trailing newline if present */
    if (cmd_len > 0 && command[cmd_len-1] == '\n') {
        command[cmd_len-1] = '\0';
        cmd_len--;
    }

    /* Add newline for Arduino communication */
    if (cmd_len < sizeof(command) - 2) {
        command[cmd_len] = '\n';
        command[cmd_len + 1] = '\0';
        cmd_len++;
    }

    result = write_to_tty(ctx->tty_file, command, cmd_len);
    if (result < 0)
        return result;

    return count;
}

static ssize_t config_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE,
                     "Available commands:\n"
                     "  RESET     - Reset Arduino\n"
                     "  PING      - Send heartbeat\n"
                     "  ENABLE    - Enable mouse reporting\n"
                     "  DISABLE   - Disable mouse reporting\n"
                     "  STATUS    - Request status\n");
}
static DEVICE_ATTR_RW(config);

/*
 * Module initialization
 */
static int __init arduino_tty_mouse_init(void)
{
    struct arduino_tty_ctx *ctx;
    int result;

    pr_info("arduino_tty: Arduino TTY Mouse Driver v4.1 loading...\n");

    /* Allocate context */
    ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
    if (!ctx)
        return -ENOMEM;

    global_ctx = ctx;

    /* Initialize context */
    mutex_init(&ctx->ctx_mutex);
    init_waitqueue_head(&ctx->read_wait);
    ctx->left_pressed = false;
    ctx->right_pressed = false;
    ctx->device_connected = false;
    ctx->supports_movement = false;
    ctx->driver_active = true;
    ctx->last_data_time = jiffies;
    ctx->tty_file = NULL;

    /* Initialize timers */
    timer_setup(&ctx->timeout_timer, arduino_timeout_timer, 0);
    timer_setup(&ctx->heartbeat_timer, arduino_heartbeat_timer, 0);

    /* Create class - Fixed for modern kernels */
    arduino_class = class_create(DRIVER_NAME);
    if (IS_ERR(arduino_class)) {
        result = PTR_ERR(arduino_class);
        pr_err("arduino_tty: Failed to create class: %d\n", result);
        goto err_free_ctx;
    }

    /* Create device */
    arduino_device = device_create(arduino_class, NULL, 0, NULL, DRIVER_NAME);
    if (IS_ERR(arduino_device)) {
        result = PTR_ERR(arduino_device);
        pr_err("arduino_tty: Failed to create device: %d\n", result);
        goto err_destroy_class;
    }

    /* Create input device */
    ctx->input = input_allocate_device();
    if (!ctx->input) {
        result = -ENOMEM;
        pr_err("arduino_tty: Failed to allocate input device\n");
        goto err_destroy_device;
    }

    ctx->input->name = DEVICE_NAME;
    ctx->input->phys = "arduino-tty/input0";
    ctx->input->id.bustype = BUS_HOST;
    ctx->input->id.vendor = 0x2341;  // Arduino VID
    ctx->input->id.product = 0x0043; // Arduino Uno PID
    ctx->input->id.version = 4;

    /* Set capabilities */
    input_set_capability(ctx->input, EV_KEY, BTN_LEFT);
    input_set_capability(ctx->input, EV_KEY, BTN_RIGHT);
    input_set_capability(ctx->input, EV_REL, REL_X);
    input_set_capability(ctx->input, EV_REL, REL_Y);

    result = input_register_device(ctx->input);
    if (result) {
        pr_err("arduino_tty: Failed to register input device: %d\n", result);
        goto err_free_input;
    }

    /* Create sysfs files */
    result = device_create_file(arduino_device, &dev_attr_stats);
    if (result) {
        pr_err("arduino_tty: Failed to create stats sysfs file: %d\n", result);
        goto err_unregister_input;
    }

    result = device_create_file(arduino_device, &dev_attr_config);
    if (result) {
        pr_err("arduino_tty: Failed to create config sysfs file: %d\n", result);
        goto err_remove_stats;
    }

    /* Try to open TTY device */
    result = open_tty_device(ctx);
    if (result) {
        pr_warn("arduino_tty: Could not open TTY device at init (will retry): %d\n", result);
        /* Continue without TTY for now */
    }

    /* Start reading thread */
    ctx->read_thread = kthread_run(arduino_read_thread, ctx, "arduino_tty_reader");
    if (IS_ERR(ctx->read_thread)) {
        result = PTR_ERR(ctx->read_thread);
        pr_err("arduino_tty: Failed to create reading thread: %d\n", result);
        goto err_close_tty;
    }

    /* Start timers */
    mod_timer(&ctx->timeout_timer, jiffies + TIMEOUT_CHECK_INTERVAL_SEC * HZ);
    mod_timer(&ctx->heartbeat_timer, jiffies + HEARTBEAT_INTERVAL_SEC * HZ);

    /* Send initial enable command if TTY is open */
    if (ctx->tty_file) {
        msleep(500);  // Wait for Arduino boot
        write_to_tty(ctx->tty_file, "ENABLE\n", 7);
    }

    pr_info("arduino_tty: Arduino TTY Mouse Driver loaded successfully\n");
    return 0;

    err_close_tty:
    close_tty_device(ctx);
    device_remove_file(arduino_device, &dev_attr_config);
    err_remove_stats:
    device_remove_file(arduino_device, &dev_attr_stats);
    err_unregister_input:
    input_unregister_device(ctx->input);
    ctx->input = NULL;
    err_free_input:
    if (ctx->input)
        input_free_device(ctx->input);
    err_destroy_device:
    device_destroy(arduino_class, 0);
    err_destroy_class:
    class_destroy(arduino_class);
    err_free_ctx:
    kfree(ctx);
    global_ctx = NULL;
    return result;
}

/*
 * Module cleanup
 */
static void __exit arduino_tty_mouse_exit(void)
{
    struct arduino_tty_ctx *ctx = global_ctx;

    if (!ctx)
        return;

    pr_info("arduino_tty: Arduino TTY Mouse Driver unloading...\n");

    /* Mark driver as inactive */
    mutex_lock(&ctx->ctx_mutex);
    ctx->driver_active = false;
    mutex_unlock(&ctx->ctx_mutex);

    /* Stop timers */
    del_timer_sync(&ctx->timeout_timer);
    del_timer_sync(&ctx->heartbeat_timer);

    /* Stop reading thread */
    if (ctx->read_thread) {
        kthread_stop(ctx->read_thread);
        ctx->read_thread = NULL;
    }

    /* Remove sysfs files */
    device_remove_file(arduino_device, &dev_attr_config);
    device_remove_file(arduino_device, &dev_attr_stats);

    /* Close TTY */
    close_tty_device(ctx);

    /* Cleanup input device */
    if (ctx->input) {
        input_unregister_device(ctx->input);
        ctx->input = NULL;
    }

    /* Cleanup device and class */
    device_destroy(arduino_class, 0);
    class_destroy(arduino_class);

    /* Free context */
    kfree(ctx);
    global_ctx = NULL;

    pr_info("arduino_tty: Arduino TTY Mouse Driver unloaded\n");
}

module_init(arduino_tty_mouse_init);
module_exit(arduino_tty_mouse_exit);

MODULE_AUTHOR("Arduino TTY Mouse Driver Developer");
MODULE_DESCRIPTION("Direct TTY-based mouse driver for Arduino (WSL2 Compatible)");
MODULE_LICENSE("GPL");
MODULE_VERSION("4.1");
MODULE_ALIAS("char-major-4-*");