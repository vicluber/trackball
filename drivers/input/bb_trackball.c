#define DT_DRV_COMPAT zmk_bb_trackball

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bb_trackball, CONFIG_INPUT_LOG_LEVEL);

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
};

static void up_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct bb_trackball_data *data = CONTAINER_OF(cb, struct bb_trackball_data, up_cb);
    input_report_rel(data->dev, INPUT_REL_Y, -1, true, K_NO_WAIT);
}

static void down_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct bb_trackball_data *data = CONTAINER_OF(cb, struct bb_trackball_data, down_cb);
    input_report_rel(data->dev, INPUT_REL_Y, 1, true, K_NO_WAIT);
}

static void left_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct bb_trackball_data *data = CONTAINER_OF(cb, struct bb_trackball_data, left_cb);
    input_report_rel(data->dev, INPUT_REL_X, -1, true, K_NO_WAIT);
}

static void right_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    struct bb_trackball_data *data = CONTAINER_OF(cb, struct bb_trackball_data, right_cb);
    input_report_rel(data->dev, INPUT_REL_X, 1, true, K_NO_WAIT);
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
