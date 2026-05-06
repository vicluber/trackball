#define DT_DRV_COMPAT zmk_bb_trackball

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bb_trackball, CONFIG_INPUT_LOG_LEVEL);

/* Accumulate pulses for this many ms before reporting.
 * Filters cross-axis noise: rolling left fires mostly LEFT edges but
 * occasionally a stray UP/DOWN edge from magnetic leakage. We report
 * the net delta, so a few stray counts get washed out by the dominant axis. */
#define REPORT_INTERVAL_MS 8

/* Each physical "tick" of the ball generates one falling edge per sensor.
 * Scale factor converts raw pulse count to HID units. */
#define PULSE_SCALE 5

struct bb_trackball_config {
    struct gpio_dt_spec up;
    struct gpio_dt_spec down;
    struct gpio_dt_spec left;
    struct gpio_dt_spec right;
};

struct bb_trackball_data {
    const struct device *dev;
    struct gpio_callback up_cb;
    struct gpio_callback down_cb;
    struct gpio_callback left_cb;
    struct gpio_callback right_cb;
    struct k_work_delayable report_work;
    atomic_t acc_x;
    atomic_t acc_y;
};

static void report_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct bb_trackball_data *data = CONTAINER_OF(dwork, struct bb_trackball_data, report_work);

    int x = (int)atomic_set(&data->acc_x, 0);
    int y = (int)atomic_set(&data->acc_y, 0);

    if (x != 0) {
        input_report_rel(data->dev, INPUT_REL_X, x * PULSE_SCALE, y == 0, K_NO_WAIT);
    }
    if (y != 0) {
        input_report_rel(data->dev, INPUT_REL_Y, y * PULSE_SCALE, true, K_NO_WAIT);
    }
}

static void schedule_report(struct bb_trackball_data *data) {
    /* Reschedule resets the timer if already pending — keeps the window
     * anchored to the last pulse so a fast roll doesn't cut off early. */
    k_work_reschedule(&data->report_work, K_MSEC(REPORT_INTERVAL_MS));
}

static void up_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct bb_trackball_data *data = CONTAINER_OF(cb, struct bb_trackball_data, up_cb);
    atomic_dec(&data->acc_y);
    schedule_report(data);
}

static void down_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct bb_trackball_data *data = CONTAINER_OF(cb, struct bb_trackball_data, down_cb);
    atomic_inc(&data->acc_y);
    schedule_report(data);
}

static void left_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct bb_trackball_data *data = CONTAINER_OF(cb, struct bb_trackball_data, left_cb);
    atomic_dec(&data->acc_x);
    schedule_report(data);
}

static void right_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct bb_trackball_data *data = CONTAINER_OF(cb, struct bb_trackball_data, right_cb);
    atomic_inc(&data->acc_x);
    schedule_report(data);
}

static int setup_pin(const struct gpio_dt_spec *pin, struct gpio_callback *cb,
                     gpio_callback_handler_t handler) {
    int ret;

    if (!gpio_is_ready_dt(pin)) {
        LOG_ERR("GPIO port not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(pin, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) return ret;

    ret = gpio_pin_interrupt_configure_dt(pin, GPIO_INT_EDGE_FALLING);
    if (ret < 0) return ret;

    gpio_init_callback(cb, handler, BIT(pin->pin));
    return gpio_add_callback(pin->port, cb);
}

static int bb_trackball_init(const struct device *dev) {
    const struct bb_trackball_config *cfg = dev->config;
    struct bb_trackball_data *data = dev->data;
    int ret;

    data->dev = dev;
    atomic_set(&data->acc_x, 0);
    atomic_set(&data->acc_y, 0);
    k_work_init_delayable(&data->report_work, report_work_handler);

    ret = setup_pin(&cfg->up,    &data->up_cb,    up_handler);    if (ret < 0) return ret;
    ret = setup_pin(&cfg->down,  &data->down_cb,  down_handler);  if (ret < 0) return ret;
    ret = setup_pin(&cfg->left,  &data->left_cb,  left_handler);  if (ret < 0) return ret;
    ret = setup_pin(&cfg->right, &data->right_cb, right_handler); if (ret < 0) return ret;

    LOG_INF("Blackberry trackball initialized");
    return 0;
}

#define BB_TRACKBALL_INIT(n)                                                    \
    static struct bb_trackball_data bb_trackball_data_##n;                      \
    static const struct bb_trackball_config bb_trackball_config_##n = {         \
        .up    = GPIO_DT_SPEC_INST_GET(n, up_gpios),                            \
        .down  = GPIO_DT_SPEC_INST_GET(n, down_gpios),                          \
        .left  = GPIO_DT_SPEC_INST_GET(n, left_gpios),                          \
        .right = GPIO_DT_SPEC_INST_GET(n, right_gpios),                         \
    };                                                                          \
    DEVICE_DT_INST_DEFINE(n, bb_trackball_init, NULL,                           \
                          &bb_trackball_data_##n,                               \
                          &bb_trackball_config_##n,                             \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(BB_TRACKBALL_INIT)
