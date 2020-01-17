// SPDX-License-Identifier: GPL-2.0+
/*
 * teensyloopctrl.c - hwmon driver for the Teensy Fan Controller
 *
 * Originally:
 * liquidctl.c - hwmon driver for the NZXT Smart Device and Grid+ V3
 *
 * Copyright 2019  Jonas Malaco <jonas@protocubo.io>
 */

#include <linux/hid.h>
#include <linux/hwmon.h>
#include <linux/kernel.h>
#include <linux/module.h>

#define DRVNAME        "teensyloopctrl"
#define MAX_REPORT_SIZE 65

struct tloopctrl_device_data {
    struct hid_device *hid_dev;
    struct device *hwmon_dev;

    int temp_count;
    int fan_count;
    int pwm_count;
    const char *const *temp_label;
    const char *const *fan_label;
//    const char *const *pwm_label;
    u64 *temp_input;
    long *fan_input;
    u64 *pwm_input;
    u8 *buf;
};

static umode_t tloopctrl_is_visible(const void *data,
                                    enum hwmon_sensor_types type,
                                    u32 attr, int channel) {
    struct tloopctrl_device_data *ldata;
    struct hid_device *hdev;

    ldata = (struct tloopctrl_device_data *)data;
    hdev = ldata->hid_dev;

    if (hdev->dev_rsize < 10 || hdev->dev_rdesc[1] != 0xAB || hdev->dev_rdesc[2] != 0xFF || hdev->dev_rdesc[4] != 0x00 || hdev->dev_rdesc[5] != 0x02) {
//        printk(KERN_WARNING "tloopctrl_is_visible() return: 0");
        return 0;
    }
    else {
        return S_IRUGO;
    }
}

static int tloopctrl_read(struct device *dev, enum hwmon_sensor_types type,
                          u32 attr, int channel, long *val) {
    struct tloopctrl_device_data *ldata = dev_get_drvdata(dev);

    switch (type) {
        case hwmon_temp:
            if (attr != hwmon_temp_input || channel >= ldata->temp_count)
                return -EINVAL;
            *val = (int)ldata->temp_input[channel];
            break;
        case hwmon_fan:
            if (attr != hwmon_fan_input || channel >= ldata->fan_count)
                return -EINVAL;
            *val = ldata->fan_input[channel];
            break;
        case hwmon_pwm:
            if (attr != hwmon_pwm_input || channel >= ldata->pwm_count)
                return -EINVAL;
            *val = (int)ldata->pwm_input[channel];
            break;
        default:
            return -EINVAL;
    }
    return 0;
}

static int tloopctrl_read_string(struct device *dev,
                                 enum hwmon_sensor_types type, u32 attr,
                                 int channel, const char **str) {
    struct tloopctrl_device_data *ldata = dev_get_drvdata(dev);

    switch (type) {
        case hwmon_temp:
            if (attr != hwmon_temp_label || channel >= ldata->temp_count ||
                !ldata->temp_label[channel])
                return -EINVAL;
            *str = ldata->temp_label[channel];
            break;
        case hwmon_fan:
            if (attr != hwmon_fan_label || channel >= ldata->fan_count ||
                !ldata->fan_label[channel])
                return -EINVAL;
            *str = ldata->fan_label[channel];
            break;
//    case hwmon_pwm:
//        if (attr != hwmon_pwm_label || channel >= ldata->pwm_count ||
//            !ldata->pwm_label[channel])
//            return -EINVAL;
//        *str = ldata->pwm_label[channel];
//        break;
        default:
            return -EINVAL;
    }
    return 0;
}

static const struct hwmon_ops tloopctrl_hwmon_ops = {
        .is_visible = tloopctrl_is_visible,
        .read = tloopctrl_read,
        .read_string = tloopctrl_read_string,
};

#define DEVNAME_TLOOPCTRL_DEVICE "teensyloopctrl"
#define TLOOPCTRL_DEVICE_TEMP_COUNT         7
#define TLOOPCTRL_DEVICE_FAN_COUNT          6
#define TLOOPCTRL_DEVICE_PWM_COUNT          1

static const u32 tloopctrl_device_fan_config[] = {
        HWMON_F_INPUT,
        HWMON_F_INPUT,
        HWMON_F_INPUT,
        HWMON_F_INPUT,
        HWMON_F_INPUT,
        HWMON_F_INPUT,
        0
};

static const struct hwmon_channel_info tloopctrl_device_fan = {
        .type = hwmon_fan,
        .config = tloopctrl_device_fan_config,
};

static const u32 tloopctrl_device_temp_config[] = {
        HWMON_T_INPUT,
        HWMON_T_INPUT,
        HWMON_T_INPUT,
        HWMON_T_INPUT,
        HWMON_T_INPUT,
        HWMON_T_INPUT,
        HWMON_T_INPUT,
        0
};

static const struct hwmon_channel_info tloopctrl_device_temp = {
        .type = hwmon_temp,
        .config = tloopctrl_device_temp_config,
};

static const u32 tloopctrl_device_pwm_config[] = {
        HWMON_PWM_INPUT,
        0
};

static const struct hwmon_channel_info tloopctrl_device_pwm = {
        .type = hwmon_pwm,
        .config = tloopctrl_device_pwm_config,
};

static const struct hwmon_channel_info *tloopctrl_device_info[] = {
        &tloopctrl_device_fan,
        &tloopctrl_device_temp,
        &tloopctrl_device_pwm,
        NULL
};

static const struct hwmon_chip_info tloopctrl_device_chip_info = {
        .ops = &tloopctrl_hwmon_ops,
        .info = tloopctrl_device_info,
};

#define USB_VENDOR_ID_ARDUINO               0x16C0
#define USB_DEVICE_ID_TLOOPCTRL_DEVICE      0x0486

// TODO
#define STATUS_REPORT_ID        4
#define STATUS_MIN_BYTES        16

#define show_ctx() \
    printk(KERN_DEBUG "%s:%d: irq: %lu, serving_softirq: %lu, nmi: %lu, task: %u\n", \
           __FUNCTION__, __LINE__, in_irq(), in_serving_softirq(), in_nmi(), in_task());

static int tloopctrl_raw_event(struct hid_device *hdev,
                               struct hid_report *report, u8 *data, int size) {
    struct tloopctrl_device_data *ldata;

    /* TODO show_ctx(): in hard irq, how much should we do here? */

    if (size <= 0 || report->maxfield != 1 || report->field[0]->application != 0xFFAB0200)
        return 0;

    ldata = hid_get_drvdata(hdev);

    /* TODO reads don't need the latest data, but each store must be atomic */
    switch (hdev->product) {
        case USB_DEVICE_ID_TLOOPCTRL_DEVICE:
            if (data[0] == 0xDA) {
                ldata->temp_input[0] = le64_to_cpup((__le64 *) (data + 2));  // supplyTemp
                ldata->temp_input[1] = le64_to_cpup((__le64 *) (data + 6));  // returnTemp
                ldata->temp_input[2] = le64_to_cpup((__le64 *) (data + 10));  // caseTemp
                ldata->temp_input[3] = le64_to_cpup((__le64 *) (data + 14));  // aux1Temp
                ldata->temp_input[4] = le64_to_cpup((__le64 *) (data + 18));  // aux2Temp
                ldata->temp_input[5] = le64_to_cpup((__le64 *) (data + 22));  // deltaT
//                ldata->temp_input[5] = le64_to_cpup((__le64 *) (data + 26));  //
                ldata->temp_input[6] = le64_to_cpup((__le64 *) (data + 30));  // setpoint

                ldata->fan_input[0] = le16_to_cpup((__le16 *) (data + 34));  // rpm1
                ldata->fan_input[1] = le16_to_cpup((__le16 *) (data + 36));  // rpm2
                ldata->fan_input[2] = le16_to_cpup((__le16 *) (data + 38));  // rpm3
                ldata->fan_input[3] = le16_to_cpup((__le16 *) (data + 40));  // rpm4
                ldata->fan_input[4] = le16_to_cpup((__le16 *) (data + 42));  // rpm5
                ldata->fan_input[5] = le16_to_cpup((__le16 *) (data + 44));  // rpm6

                ldata->pwm_input[0] = 0;
//                ldata->pwm_input[0] = le64_to_cpup((__le64 *) (data + 26));  // fan % (PID)
//                ldata->pwm_input[0] = le64_to_cpup((__le64 *) (data + 26));  // fan % (%-tbl)
            }
            break;
        default:
            return 0;
    }
    return 0;
}

static int tloopctrl_initialize(struct tloopctrl_device_data *ldata) {
    // TODO
    int ret;
	memset(ldata->buf, '\0', MAX_REPORT_SIZE);
    *(ldata->buf) = 0;  // report id ?
    *(ldata->buf + 1) = 0;  //0xC0;
    *(ldata->buf + 2) = 0;  //0xFF;
    ret = hid_hw_output_report(ldata->hid_dev, ldata->buf, MAX_REPORT_SIZE);
    //printk(KERN_WARNING "tloopctrl_initialize() ret: %d", ret);
    if (ret < 0)
        return ret;
    if (ret != MAX_REPORT_SIZE)
        return -EINVAL;

    return 0;
}

static const struct hid_device_id tloopctrl_table[] = {
        {HID_USB_DEVICE(USB_VENDOR_ID_ARDUINO, USB_DEVICE_ID_TLOOPCTRL_DEVICE)},
        {}
};

MODULE_DEVICE_TABLE(hid, tloopctrl_table
);

static int tloopctrl_probe(struct hid_device *hdev,
                           const struct hid_device_id *id) {
    struct tloopctrl_device_data *ldata;
    struct device *hwmon_dev;
    const struct hwmon_chip_info *chip_info;
    char *chip_name;
    int ret;

    ldata = devm_kzalloc(&hdev->dev, sizeof(*ldata), GFP_KERNEL);
    if (!ldata)
        return -ENOMEM;

    switch (hdev->product) {
        case USB_DEVICE_ID_TLOOPCTRL_DEVICE:
            chip_name = DEVNAME_TLOOPCTRL_DEVICE;
            ldata->temp_count = TLOOPCTRL_DEVICE_TEMP_COUNT;
            ldata->fan_count = TLOOPCTRL_DEVICE_FAN_COUNT;
            ldata->pwm_count = TLOOPCTRL_DEVICE_PWM_COUNT;
            ldata->temp_label = NULL;
            ldata->fan_label = NULL;
//            ldata->pwm_label = NULL;
            chip_info = &tloopctrl_device_chip_info;
            break;
        default:
            return -EINVAL;
    }
    hid_info(hdev, "device: %s\n", chip_name);

    ldata->buf = devm_kmalloc(&hdev->dev, MAX_REPORT_SIZE, GFP_KERNEL);
    if (!ldata->buf)
        return -ENOMEM;

    ldata->temp_input = devm_kcalloc(&hdev->dev, ldata->temp_count,
                                     sizeof(*ldata->temp_input), GFP_KERNEL);
    if (!ldata->temp_input)
        return -ENOMEM;

    ldata->fan_input = devm_kcalloc(&hdev->dev, ldata->fan_count,
                                    sizeof(*ldata->fan_input), GFP_KERNEL);
    if (!ldata->fan_input)
        return -ENOMEM;

    ldata->pwm_input = devm_kcalloc(&hdev->dev, ldata->pwm_count,
                                    sizeof(*ldata->pwm_input), GFP_KERNEL);
    if (!ldata->pwm_input)
        return -ENOMEM;

    ldata->hid_dev = hdev;
    hid_set_drvdata(hdev, ldata);

    ret = hid_parse(hdev);
    if (ret) {
        hid_err(hdev, "hid_parse failed with %d\n", ret);
        return ret;
    }

    /* keep hidraw so user-space can (easily) take care of the other
     * features of the device (e.g. LEDs) */
    ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
    if (ret) {
        hid_err(hdev, "hid_hw_start failed with %d\n", ret);
        goto rec_stop_hid;
    }

    ret = hid_hw_open(hdev);
    if (ret) {
        hid_err(hdev, "hid_hw_open failed with %d\n", ret);
        goto rec_close_hid;
    }

    hwmon_dev = devm_hwmon_device_register_with_info(&hdev->dev, chip_name,
                                                     ldata, chip_info,
                                                     NULL);
    if (IS_ERR(hwmon_dev)) {
        hid_err(hdev, "failed to register hwmon device\n");
        ret = PTR_ERR(hwmon_dev);
        goto rec_close_hid;
    }
    ldata->hwmon_dev = hwmon_dev;

    ret = tloopctrl_initialize(ldata);
    if (ret) {
        hid_err(hdev, "failed to initialize device");
        goto rec_close_hid;
    }

    hid_info(hdev, "probing successful\n");
    return 0;

    rec_close_hid:
    hid_hw_close(hdev);
    rec_stop_hid:
    hid_hw_stop(hdev);
    return ret;
}

static void tloopctrl_remove(struct hid_device *hdev) {
    hid_hw_close(hdev);
    hid_hw_stop(hdev);
}

static struct hid_driver tloopctrl_driver = {
        .name = DRVNAME,
        .id_table = tloopctrl_table,
        .probe = tloopctrl_probe,
        .remove = tloopctrl_remove,
        .raw_event = tloopctrl_raw_event,
};

module_hid_driver(tloopctrl_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jonas Malaco <jonas@protocubo.io>");
MODULE_AUTHOR("Jack Davis <c@jtd.me>");
MODULE_DESCRIPTION("Hwmon driver for Teensy Fan Controller");
