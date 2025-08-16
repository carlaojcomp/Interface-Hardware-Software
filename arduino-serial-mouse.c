// SPDX-License-Identifier: GPL-2.0
/*
 * Arduino Uno Serial Mouse Driver - Corrected Serial Communication
 * Uses serdev framework for proper serial communication
 *
 * Arduino Uno Connections:
 * - Pin 2 -> Left Button -> GND
 * - Pin 3 -> Right Button -> GND
 * - Pin A0 -> X-axis potentiometer (optional)
 * - Pin A1 -> Y-axis potentiometer (optional)
 * - Pin 13 -> LED indicator (internal)
 * - USB -> Serial communication with PC (/dev/ttyACM0)
 *
 * Device Tree binding required:
 * &uart1 {
 *     arduino-mouse {
 *         compatible = "arduino,serial-mouse";
 *     };
 * };
 *
 * Author: Arduino Serial Mouse Driver Developer
 * Version: 3.1 (Fixed C89 compatibility and race conditions)
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/serdev.h>
#include <linux/of.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/mod_devicetable.h>

#define DRIVER_NAME "arduino-serial-mouse"
#define DEVICE_NAME "Arduino Uno Serial Mouse"
#define SERIAL_BUFFER_SIZE 512
#define ARDUINO_VID 0x2341
#define ARDUINO_UNO_PID 0x0043
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
#define ARDUINO_BOOT_DELAY_MS      500
#define HEARTBEAT_INTERVAL_SEC     2
#define TIMEOUT_CHECK_INTERVAL_SEC 1
#define MAX_PACKETS_PER_WORK       10

struct arduino_packet {
    u8 start;           // 0xAA
    u8 buttons;         // bit0=left, bit1=right, bit7=has_movement
    s8 delta_x;         // X movement (-128 to 127)
    s8 delta_y;         // Y movement (-128 to 127)
    u8 checksum;        // Simple checksum
    u8 end;             // 0x55
} __packed;

struct arduino_serial_ctx {
    struct serdev_device *serdev;
    struct input_dev *input;

    /* Serial data buffer */
    unsigned char rx_buffer[SERIAL_BUFFER_SIZE];
    int rx_head, rx_tail;
    spinlock_t rx_lock;

    /* Button and movement states */
    bool left_pressed;
    bool right_pressed;
    bool device_connected;
    bool supports_movement;
    bool driver_active;

    /* Work for processing data */
    struct work_struct process_work;
    struct workqueue_struct *workqueue;

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

    /* Synchronization */
    struct mutex ctx_mutex;
};
#if IS_ENABLED(CONFIG_SERIAL_DEV_BUS)
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

static void process_arduino_packet(struct arduino_serial_ctx *ctx,
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

    dev_dbg(&ctx->serdev->dev, "Packet: L=%d R=%d X=%d Y=%d (buttons=0x%02x)\n",
            left_state, right_state, delta_x, delta_y, pkt->buttons);

    mutex_unlock(&ctx->ctx_mutex);
}

/*
 * Worker to process received data - C89 compliant
 */

static void arduino_process_work(struct work_struct *work)
{
    struct arduino_serial_ctx *ctx =
    container_of(work, struct arduino_serial_ctx, process_work);
    struct arduino_packet packet;
    unsigned long flags;
    int bytes_available;
    int processed = 0;
    int i;

    if (!ctx)
        return;

    spin_lock_irqsave(&ctx->rx_lock, flags);

    /* Process packets with controlled latency */
    while ((bytes_available = (ctx->rx_head - ctx->rx_tail + SERIAL_BUFFER_SIZE) % SERIAL_BUFFER_SIZE) >= sizeof(struct arduino_packet)) {

        /* Limit time with interrupts disabled */
        if (processed >= MAX_PACKETS_PER_WORK) {
            spin_unlock_irqrestore(&ctx->rx_lock, flags);
            /* Reschedule work if more data available */
            if (ctx->workqueue && ctx->driver_active)
                queue_work(ctx->workqueue, &ctx->process_work);
            return;
        }

        /* Look for start byte */
        while (ctx->rx_head != ctx->rx_tail &&
               ctx->rx_buffer[ctx->rx_tail] != PROTOCOL_START_BYTE) {
            ctx->rx_tail = (ctx->rx_tail + 1) % SERIAL_BUFFER_SIZE;
        }

        /* Check if we have enough bytes for a complete packet */
        bytes_available = (ctx->rx_head - ctx->rx_tail + SERIAL_BUFFER_SIZE) % SERIAL_BUFFER_SIZE;
        if (bytes_available < sizeof(struct arduino_packet))
            break;

        /* Copy packet from circular buffer - C89 compliant */
        for (i = 0; i < sizeof(struct arduino_packet); i++) {
            ((u8*)&packet)[i] = ctx->rx_buffer[(ctx->rx_tail + i) % SERIAL_BUFFER_SIZE];
        }

        /* Advance tail */
        ctx->rx_tail = (ctx->rx_tail + sizeof(struct arduino_packet)) % SERIAL_BUFFER_SIZE;
        processed++;

        spin_unlock_irqrestore(&ctx->rx_lock, flags);

        /* Process packet outside of spinlock */
        process_arduino_packet(ctx, &packet);

        spin_lock_irqsave(&ctx->rx_lock, flags);
    }

    spin_unlock_irqrestore(&ctx->rx_lock, flags);
}

/*
 * Serial device receive callback - FIXED RETURN TYPE
 */
static int arduino_serial_receive_buf(struct serdev_device *serdev,
                                      const unsigned char *data, size_t count)
{
    struct arduino_serial_ctx *ctx = serdev_device_get_drvdata(serdev);
    unsigned long flags;
    size_t i;

    if (!ctx || !ctx->driver_active)
        return 0;

    spin_lock_irqsave(&ctx->rx_lock, flags);

    /* Add data to circular buffer */
    for (i = 0; i < count; i++) {
        int next_head = (ctx->rx_head + 1) % SERIAL_BUFFER_SIZE;

        /* Check for buffer overflow */
        if (next_head == ctx->rx_tail) {
            /* Buffer full - drop oldest data */
            ctx->rx_tail = (ctx->rx_tail + 1) % SERIAL_BUFFER_SIZE;
        }

        ctx->rx_buffer[ctx->rx_head] = data[i];
        ctx->rx_head = next_head;
    }

    spin_unlock_irqrestore(&ctx->rx_lock, flags);

    /* Mark as connected and schedule processing */
    if (mutex_trylock(&ctx->ctx_mutex)) {
        ctx->device_connected = true;
        ctx->last_data_time = jiffies;
        mutex_unlock(&ctx->ctx_mutex);
    }

    /* Schedule work to process data */
    if (ctx->workqueue && ctx->driver_active)
        queue_work(ctx->workqueue, &ctx->process_work);

    return (int)count;  /* Return int as expected by kernel */
}

/*
 * Send heartbeat command to Arduino
 */
static void send_heartbeat(struct arduino_serial_ctx *ctx)
{
    const char heartbeat_cmd[] = "PING\n";

    if (ctx && ctx->serdev && ctx->driver_active) {
        serdev_device_write(ctx->serdev, heartbeat_cmd, sizeof(heartbeat_cmd) - 1, HZ);
    }
}

/*
 * Timeout timer - detect disconnection with safe context check
 */
static void arduino_timeout_timer(struct timer_list *timer)
{
    struct arduino_serial_ctx *ctx =
    container_of(timer, struct arduino_serial_ctx, timeout_timer);

    if (!ctx || !ctx->serdev || !ctx->driver_active)
        return;

    if (mutex_trylock(&ctx->ctx_mutex)) {
        if (time_after(jiffies, ctx->last_data_time + msecs_to_jiffies(CONNECTION_TIMEOUT_MS))) {
            /* No data for timeout period - mark as disconnected */
            if (ctx->device_connected) {
                ctx->device_connected = false;
                dev_warn(&ctx->serdev->dev, "Arduino disconnected (timeout)\n");

                /* Clear button states safely */
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

        /* Reschedule timer only if driver still active */
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
    struct arduino_serial_ctx *ctx =
    container_of(timer, struct arduino_serial_ctx, heartbeat_timer);

    if (!ctx || !ctx->driver_active)
        return;

    send_heartbeat(ctx);

    /* Reschedule every 2 seconds if driver active */
    if (ctx->driver_active) {
        mod_timer(&ctx->heartbeat_timer, jiffies + HEARTBEAT_INTERVAL_SEC * HZ);
    }
}

/*
 * Serial device operations
 */
static const struct serdev_device_ops arduino_serial_ops = {
        .receive_buf = arduino_serial_receive_buf,
};

/*
 * Show statistics via sysfs - C89 compliant
 */
static ssize_t stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct arduino_serial_ctx *ctx = dev_get_drvdata(dev);
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
                       "Connected: %s\n"
                       "Supports movement: %s\n"
                       "Left pressed: %s\n"
                       "Right pressed: %s\n"
                       "Driver active: %s\n"
                       "Serial device: %s\n",
                       ctx->packets_received,
                       ctx->packets_invalid,
                       ctx->checksum_errors,
                       ctx->button_events,
                       ctx->movement_events,
                       ctx->device_connected ? "yes" : "no",
                       ctx->supports_movement ? "yes" : "no",
                       ctx->left_pressed ? "yes" : "no",
                       ctx->right_pressed ? "yes" : "no",
                       ctx->driver_active ? "yes" : "no",
                       dev_name(&ctx->serdev->dev));

    mutex_unlock(&ctx->ctx_mutex);

    return result;
}
static DEVICE_ATTR_RO(stats);

/*
 * Send configuration command to Arduino - Safe buffer handling
 */
static ssize_t config_store(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t count)
{
    struct arduino_serial_ctx *ctx = dev_get_drvdata(dev);
    char command[64];
    size_t cmd_len;

    if (!ctx || !ctx->driver_active || count >= sizeof(command) - 2)
        return -EINVAL;

    /* Safe copy with bounds checking */
    cmd_len = min(count, sizeof(command) - 2);
    memcpy(command, buf, cmd_len);
    command[cmd_len] = '\0';

    /* Remove trailing newline if present */
    if (cmd_len > 0 && command[cmd_len-1] == '\n') {
        command[cmd_len-1] = '\0';
        cmd_len--;
    }

    /* Add newline for Arduino communication - safe concatenation */
    if (cmd_len < sizeof(command) - 2) {
        command[cmd_len] = '\n';
        command[cmd_len + 1] = '\0';
        cmd_len++;
    }

    serdev_device_write(ctx->serdev, command, cmd_len, HZ);

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
 * Probe function - called when device is found - Fixed error handling
 */


/* __maybe_unused é redundante com o #if, mas ajuda em builds exóticos */
static int __maybe_unused arduino_serial_probe(struct serdev_device *serdev)
{
    struct arduino_serial_ctx *ctx;
    int result;

    dev_info(&serdev->dev, "Arduino Serial Mouse probing device\n");

    /* Allocate context */
    ctx = devm_kzalloc(&serdev->dev, sizeof(*ctx), GFP_KERNEL);
    if (!ctx)
        return -ENOMEM;

    ctx->serdev = serdev;
    serdev_device_set_drvdata(serdev, ctx);

    /* Initialize context */
    spin_lock_init(&ctx->rx_lock);
    mutex_init(&ctx->ctx_mutex);
    ctx->rx_head = 0;
    ctx->rx_tail = 0;
    ctx->left_pressed = false;
    ctx->right_pressed = false;
    ctx->device_connected = false;
    ctx->supports_movement = false;
    ctx->driver_active = true;
    ctx->last_data_time = jiffies;

    /* Create workqueue (nome inclui o dev_name p/ facilitar debug) */
    ctx->workqueue = create_singlethread_workqueue(devm_kasprintf(&serdev->dev, GFP_KERNEL,
                                                                  "arduino_serial/%s",
                                                                  dev_name(&serdev->dev)));
    if (!ctx->workqueue) {
        dev_err(&serdev->dev, "Failed to create workqueue\n");
        return -ENOMEM;
    }
    INIT_WORK(&ctx->process_work, arduino_process_work);

    /* Initialize timers (ainda não armados) */
    timer_setup(&ctx->timeout_timer, arduino_timeout_timer, 0);
    timer_setup(&ctx->heartbeat_timer, arduino_heartbeat_timer, 0);

    /* Configure serial device */
    serdev_device_set_baudrate(serdev, SERIAL_BAUD_RATE);
    serdev_device_set_flow_control(serdev, false);
    serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
    serdev_device_set_client_ops(serdev, &arduino_serial_ops);

    /* Open serial device */
    result = serdev_device_open(serdev);
    if (result) {
        dev_err(&serdev->dev, "Failed to open serial device: %d\n", result);
        goto err_cleanup_timers;
    }

    /* Create input device */
    ctx->input = devm_input_allocate_device(&serdev->dev);
    if (!ctx->input) {
        result = -ENOMEM;
        dev_err(&serdev->dev, "Failed to allocate input device\n");
        goto err_close_serial;
    }

    ctx->input->name = DEVICE_NAME;
    ctx->input->phys = dev_name(&serdev->dev);
    ctx->input->id.bustype = BUS_HOST;
    ctx->input->id.vendor = ARDUINO_VID;
    ctx->input->id.product = ARDUINO_UNO_PID;
    ctx->input->id.version = 3;
    ctx->input->dev.parent = &serdev->dev;

    /* Capabilities */
    input_set_capability(ctx->input, EV_KEY, BTN_LEFT);
    input_set_capability(ctx->input, EV_KEY, BTN_RIGHT);
    input_set_capability(ctx->input, EV_REL, REL_X);
    input_set_capability(ctx->input, EV_REL, REL_Y);

    result = input_register_device(ctx->input);
    if (result) {
        dev_err(&serdev->dev, "Failed to register input device: %d\n", result);
        goto err_close_serial;
    }

    /* Sysfs */
    result = device_create_file(&serdev->dev, &dev_attr_stats);
    if (result) {
        dev_err(&serdev->dev, "Failed to create stats sysfs file: %d\n", result);
        goto err_close_serial;
    }

    result = device_create_file(&serdev->dev, &dev_attr_config);
    if (result) {
        dev_err(&serdev->dev, "Failed to create config sysfs file: %d\n", result);
        goto err_remove_stats;
    }

    /* Start timers */
    mod_timer(&ctx->timeout_timer, jiffies + TIMEOUT_CHECK_INTERVAL_SEC * HZ);
    mod_timer(&ctx->heartbeat_timer, jiffies + HEARTBEAT_INTERVAL_SEC * HZ);

    /* Initial configuration (após pequeno atraso p/ boot do Arduino) */
    msleep(ARDUINO_BOOT_DELAY_MS);
    result = serdev_device_write(serdev, "ENABLE\n", 7, HZ);
    if (result < 0)
        dev_warn(&serdev->dev, "Initial ENABLE write failed: %d\n", result);

    dev_info(&serdev->dev, "Arduino Serial Mouse initialized successfully\n");
    return 0;

err_remove_stats:
    device_remove_file(&serdev->dev, &dev_attr_stats);
err_close_serial:
    serdev_device_close(serdev);
err_cleanup_timers:
    ctx->driver_active = false;
    del_timer_sync(&ctx->timeout_timer);
    del_timer_sync(&ctx->heartbeat_timer);
    if (ctx->workqueue) {
        cancel_work_sync(&ctx->process_work);
        destroy_workqueue(ctx->workqueue);
    }
    return result;
}

/*
 * Remove function - called when device is removed - Improved cleanup
 */
static void __maybe_unused arduino_serial_remove(struct serdev_device *serdev)
{
    struct arduino_serial_ctx *ctx = serdev_device_get_drvdata(serdev);

    if (!ctx)
        return;

    dev_info(&serdev->dev, "Arduino Serial Mouse removing device\n");

    /* Mark driver as inactive first */
    mutex_lock(&ctx->ctx_mutex);
    ctx->driver_active = false;
    mutex_unlock(&ctx->ctx_mutex);

    /* Stop timers synchronously */
    del_timer_sync(&ctx->timeout_timer);
    del_timer_sync(&ctx->heartbeat_timer);

    /* Remove sysfs files */
    device_remove_file(&serdev->dev, &dev_attr_config);
    device_remove_file(&serdev->dev, &dev_attr_stats);

    /* Stop workqueue */
    if (ctx->workqueue) {
        cancel_work_sync(&ctx->process_work);
        destroy_workqueue(ctx->workqueue);
    }

    /* Close serial device */
    serdev_device_close(serdev);

    /* Input device é devm_*: liberado automaticamente */

    dev_info(&serdev->dev, "Arduino Serial Mouse removed\n");
}

#endif /* IS_ENABLED(CONFIG_SERIAL_DEV_BUS) */

/*
 * Device tree match table
 */
/*
 * Device tree match table
 */
/*
 * Device tree match table
 */
static const struct of_device_id arduino_serial_of_match[] = {
        { .compatible = "arduino,serial-mouse" },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, arduino_serial_of_match);

/*
 * Serial device driver structure
 */
#if IS_ENABLED(CONFIG_SERIAL_DEV_BUS)
static struct serdev_device_driver arduino_serial_driver = {
        .probe  = arduino_serial_probe,
        .remove = arduino_serial_remove,
        .driver = {
                .name           = DRIVER_NAME,
                .of_match_table = of_match_ptr(arduino_serial_of_match),
        },
};
#endif

/*
 * Registro do driver
 * - Quando SERDEV estiver habilitado: usa module_driver() com as funcs do serdev.
 * - Quando SERDEV estiver desabilitado: init retorna -ENODEV e não registra nada,
 *   evitando 'defined but not used' e 'return sem valor'.
 */
#if IS_ENABLED(CONFIG_SERIAL_DEV_BUS)

module_driver(arduino_serial_driver,
              serdev_device_driver_register,
              serdev_device_driver_unregister);

#else /* !CONFIG_SERIAL_DEV_BUS */

static int __init arduino_serial_mouse_init(void)
{
    pr_warn(DRIVER_NAME ": built without CONFIG_SERIAL_DEV_BUS; not registering\n");
    return -ENODEV;
}

static void __exit arduino_serial_mouse_exit(void)
{
    /* nada a fazer */
}

module_init(arduino_serial_mouse_init);
module_exit(arduino_serial_mouse_exit);

#endif /* CONFIG_SERIAL_DEV_BUS */

MODULE_AUTHOR("Arduino Serial Mouse Driver Developer");
MODULE_DESCRIPTION("Serial-based mouse driver for Arduino Uno using serdev framework");
MODULE_LICENSE("GPL");
MODULE_VERSION("3.1");
MODULE_ALIAS("of:N*T*Carduino,serial-mouse");

