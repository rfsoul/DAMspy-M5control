#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"

#include "driver/i2c_master.h"

#include "bsp/esp-bsp.h"
#include "bsp/m5stack_core_s3.h"

#include "lvgl.h"

#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"

#include "espnow_echo.h"


/* ============================================================
 * IDs
 * ============================================================ */

#define RODE_VID               0x19F7
#define WIRELESS_PRO_RX_PID    0x0058

#define AXP2101_ADDR           0x34
#define AW9523_ADDR            0x58


/* ============================================================
 * AXP2101
 * ============================================================ */

#define AXP_REG_STATUS0        0x00
#define AXP_REG_STATUS1        0x01
#define AXP_REG_ADC_ENABLE     0x30
#define AXP_REG_BAT_VOLTAGE    0x34
#define AXP_REG_BAT_PERCENT    0xA4

#define AXP_VBUS_GOOD_MASK     0x20


/* ============================================================
 * AW9523
 * ============================================================ */

#define AW_REG_OUTPUT_P0       0x02
#define AW_REG_OUTPUT_P1       0x03
#define AW_REG_CONFIG_P0       0x04
#define AW_REG_CONFIG_P1       0x05
#define AW_REG_ID              0x10
#define AW_REG_GCR             0x11
#define AW_REG_LED_MODE_P0     0x12
#define AW_REG_LED_MODE_P1     0x13

#define BUS_OUT_EN_MASK        (1u << 1)
#define USB_OTG_EN_MASK        (1u << 5)
#define BOOST_EN_MASK          (1u << 7)

#define AW_GCR_PUSH_PULL       (1u << 4)


/* ============================================================
 * Wireless PRO battery HID command
 *
 * Existing damspy-rpicontrol:
 *
 * request report ID = 0x01
 * command           = 0x61
 * request length    = 17
 *
 * expected reply:
 * 02 61 41 ...
 * ============================================================ */

#define BATTERY_REPORT_ID              0x01
#define BATTERY_RESPONSE_REPORT_ID     0x02
#define BATTERY_COMMAND                0x61
#define BATTERY_REQUEST_LENGTH         17
#define BATTERY_RESPONSE_MIN_LENGTH    12

static const uint8_t wireless_pro_battery_request[BATTERY_REQUEST_LENGTH] = {
    0x01, 0x61,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00
};


/* ============================================================
 * Globals
 * ============================================================ */

static const char *TAG = "DAMspy";

static lv_obj_t *status_label = NULL;

static i2c_master_dev_handle_t axp_dev = NULL;
static i2c_master_dev_handle_t aw_dev = NULL;

static usb_host_client_handle_t client_hdl = NULL;
static usb_device_handle_t device_hdl = NULL;

static volatile int pending_device_address = -1;
static volatile bool device_gone = false;

static int claimed_hid_interface = -1;

static uint8_t hid_in_endpoint = 0;
static uint16_t hid_in_mps = 0;

static usb_transfer_t *battery_control_transfer = NULL;
static usb_transfer_t *battery_input_transfer = NULL;

static bool battery_transaction_active = false;
static size_t active_request_length = 0;
static uint32_t active_transaction_id = 0;
static uint8_t active_requester[ESP_NOW_ETH_ALEN] = {0};
static bool active_tunnel_transaction = false;


typedef struct {
    uint8_t destination[ESP_NOW_ETH_ALEN];
    uint32_t transaction_id;
    int length;
    bool tunnel_transaction;
    uint8_t data[HID_TUNNEL_MAX_PAYLOAD];
} hid_response_t;


static QueueHandle_t hid_response_queue = NULL;


/* ============================================================
 * Power information
 * ============================================================ */

typedef struct {
    bool vbus_present;
    bool charging;
    int battery_percent;
    float battery_voltage;
} core_power_info_t;


/* ============================================================
 * HID information
 * ============================================================ */

typedef struct {
    int interface_number;
    int alternate_setting;
    int subclass;
    int protocol;
    int endpoint_count;

    uint8_t in_endpoint;
    uint16_t in_mps;
    uint8_t in_interval;
} hid_interface_info_t;


/* ============================================================
 * Screen
 * ============================================================ */

static void screen_printf(const char *format, ...)
{
    char buffer[1000];

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    bsp_display_lock(0);

    if (status_label == NULL) {
        status_label = lv_label_create(lv_screen_active());

        lv_obj_set_width(status_label, 300);

        lv_obj_set_style_text_font(
            status_label,
            &lv_font_montserrat_14,
            0
        );

        lv_obj_align(
            status_label,
            LV_ALIGN_TOP_LEFT,
            10,
            10
        );
    }

    lv_label_set_text(status_label, buffer);

    bsp_display_unlock();
}


/* ============================================================
 * Fatal error
 * ============================================================ */

static void fatal_error(
    const char *where,
    esp_err_t err
)
{
    screen_printf(
        "DAMspy M5 Control\n\n"
        "FATAL ERROR\n\n"
        "%s\n\n"
        "%s",
        where,
        esp_err_to_name(err)
    );

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


/* ============================================================
 * I2C helpers
 * ============================================================ */

static esp_err_t i2c_read_reg(
    i2c_master_dev_handle_t dev,
    uint8_t reg,
    uint8_t *value
)
{
    return i2c_master_transmit_receive(
        dev,
        &reg,
        1,
        value,
        1,
        1000
    );
}


static esp_err_t i2c_read_pair(
    i2c_master_dev_handle_t dev,
    uint8_t reg,
    uint8_t *value0,
    uint8_t *value1
)
{
    uint8_t data[2];

    esp_err_t err = i2c_master_transmit_receive(
        dev,
        &reg,
        1,
        data,
        2,
        1000
    );

    if (err == ESP_OK) {
        *value0 = data[0];
        *value1 = data[1];
    }

    return err;
}


static esp_err_t i2c_write_reg(
    i2c_master_dev_handle_t dev,
    uint8_t reg,
    uint8_t value
)
{
    uint8_t data[2] = {
        reg,
        value
    };

    return i2c_master_transmit(
        dev,
        data,
        sizeof(data),
        1000
    );
}


static esp_err_t i2c_write_pair(
    i2c_master_dev_handle_t dev,
    uint8_t reg,
    uint8_t value0,
    uint8_t value1
)
{
    uint8_t data[3] = {
        reg,
        value0,
        value1
    };

    return i2c_master_transmit(
        dev,
        data,
        sizeof(data),
        1000
    );
}


/* ============================================================
 * Power IC setup
 * ============================================================ */

static void init_power_devices(void)
{
    esp_err_t err;

    err = bsp_i2c_init();

    if (err != ESP_OK) {
        fatal_error("bsp_i2c_init", err);
    }

    i2c_master_bus_handle_t bus =
        bsp_i2c_get_handle();

    if (bus == NULL) {
        screen_printf(
            "POWER ERROR\n\n"
            "No BSP I2C bus"
        );

        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }


    i2c_device_config_t axp_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AXP2101_ADDR,
        .scl_speed_hz = 100000
    };

    err = i2c_master_bus_add_device(
        bus,
        &axp_cfg,
        &axp_dev
    );

    if (err != ESP_OK) {
        fatal_error("add AXP2101", err);
    }


    i2c_device_config_t aw_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AW9523_ADDR,
        .scl_speed_hz = 100000
    };

    err = i2c_master_bus_add_device(
        bus,
        &aw_cfg,
        &aw_dev
    );

    if (err != ESP_OK) {
        fatal_error("add AW9523", err);
    }


    err = i2c_write_reg(
        axp_dev,
        AXP_REG_ADC_ENABLE,
        0x0F
    );

    if (err != ESP_OK) {
        fatal_error("AXP ADC enable", err);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}


/* ============================================================
 * Core battery
 * ============================================================ */

static esp_err_t read_core_power(
    core_power_info_t *info
)
{
    esp_err_t err;

    uint8_t status0 = 0;
    uint8_t status1 = 0;
    uint8_t percent = 0;

    uint8_t bat_hi = 0;
    uint8_t bat_lo = 0;


    err = i2c_read_reg(
        axp_dev,
        AXP_REG_STATUS0,
        &status0
    );

    if (err != ESP_OK) {
        return err;
    }


    err = i2c_read_reg(
        axp_dev,
        AXP_REG_STATUS1,
        &status1
    );

    if (err != ESP_OK) {
        return err;
    }


    err = i2c_read_reg(
        axp_dev,
        AXP_REG_BAT_PERCENT,
        &percent
    );

    if (err != ESP_OK) {
        return err;
    }


    err = i2c_read_pair(
        axp_dev,
        AXP_REG_BAT_VOLTAGE,
        &bat_hi,
        &bat_lo
    );

    if (err != ESP_OK) {
        return err;
    }


    uint16_t raw_voltage =
        ((uint16_t)(bat_hi & 0x3F) << 8) |
        bat_lo;


    info->vbus_present =
        (status0 & AXP_VBUS_GOOD_MASK) != 0;


    info->charging =
        (status1 & 0x60) == 0x20;


    info->battery_percent =
        percent <= 100
        ? percent
        : -1;


    info->battery_voltage =
        raw_voltage / 1000.0f;


    return ESP_OK;
}


/* ============================================================
 * CoreS3 USB power
 * ============================================================ */

static void cores3_usb_power(bool enable)
{
    esp_err_t err;

    uint8_t id = 0;

    err = i2c_read_reg(
        aw_dev,
        AW_REG_ID,
        &id
    );

    if (err != ESP_OK) {
        fatal_error("AW9523 ID read", err);
    }

    if (id != 0x23) {
        screen_printf(
            "POWER ERROR\n\n"
            "AW9523 ID = %02X\n"
            "Expected 23",
            id
        );

        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }


    uint8_t out0 = 0;
    uint8_t out1 = 0;

    uint8_t cfg0 = 0;
    uint8_t cfg1 = 0;

    uint8_t led0 = 0;
    uint8_t led1 = 0;

    uint8_t gcr = 0;


    err = i2c_read_pair(
        aw_dev,
        AW_REG_OUTPUT_P0,
        &out0,
        &out1
    );

    if (err != ESP_OK) {
        fatal_error("read AW outputs", err);
    }


    i2c_read_reg(
        aw_dev,
        AW_REG_CONFIG_P0,
        &cfg0
    );

    i2c_read_reg(
        aw_dev,
        AW_REG_CONFIG_P1,
        &cfg1
    );

    i2c_read_reg(
        aw_dev,
        AW_REG_LED_MODE_P0,
        &led0
    );

    i2c_read_reg(
        aw_dev,
        AW_REG_LED_MODE_P1,
        &led1
    );

    i2c_read_reg(
        aw_dev,
        AW_REG_GCR,
        &gcr
    );


    led0 |=
        BUS_OUT_EN_MASK |
        USB_OTG_EN_MASK;

    led1 |=
        BOOST_EN_MASK;


    err = i2c_write_reg(
        aw_dev,
        AW_REG_LED_MODE_P0,
        led0
    );

    if (err != ESP_OK) {
        fatal_error("AW LED0", err);
    }


    err = i2c_write_reg(
        aw_dev,
        AW_REG_LED_MODE_P1,
        led1
    );

    if (err != ESP_OK) {
        fatal_error("AW LED1", err);
    }


    /*
     * 0 = output
     */

    cfg0 &=
        ~(BUS_OUT_EN_MASK |
          USB_OTG_EN_MASK);

    cfg1 &=
        ~BOOST_EN_MASK;


    err = i2c_write_reg(
        aw_dev,
        AW_REG_CONFIG_P0,
        cfg0
    );

    if (err != ESP_OK) {
        fatal_error("AW CFG0", err);
    }


    err = i2c_write_reg(
        aw_dev,
        AW_REG_CONFIG_P1,
        cfg1
    );

    if (err != ESP_OK) {
        fatal_error("AW CFG1", err);
    }


    gcr |= AW_GCR_PUSH_PULL;

    err = i2c_write_reg(
        aw_dev,
        AW_REG_GCR,
        gcr
    );

    if (err != ESP_OK) {
        fatal_error("AW GCR", err);
    }


    if (enable) {

        /*
         * Proven sequence:
         *
         * BOOST
         * wait
         * BUS_OUT + USB_OTG
         */

        out1 |= BOOST_EN_MASK;

        err = i2c_write_pair(
            aw_dev,
            AW_REG_OUTPUT_P0,
            out0,
            out1
        );

        if (err != ESP_OK) {
            fatal_error("BOOST ON", err);
        }

        vTaskDelay(pdMS_TO_TICKS(200));


        out0 |=
            BUS_OUT_EN_MASK |
            USB_OTG_EN_MASK;


        err = i2c_write_pair(
            aw_dev,
            AW_REG_OUTPUT_P0,
            out0,
            out1
        );

        if (err != ESP_OK) {
            fatal_error("VBUS ON", err);
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }
    else {

        /*
         * Kill downstream VBUS first.
         */

        out0 &=
            ~(BUS_OUT_EN_MASK |
              USB_OTG_EN_MASK);


        err = i2c_write_pair(
            aw_dev,
            AW_REG_OUTPUT_P0,
            out0,
            out1
        );

        if (err != ESP_OK) {
            fatal_error("VBUS OFF", err);
        }

        vTaskDelay(pdMS_TO_TICKS(50));


        /*
         * Then kill boost.
         */

        out1 &=
            ~BOOST_EN_MASK;


        err = i2c_write_pair(
            aw_dev,
            AW_REG_OUTPUT_P0,
            out0,
            out1
        );

        if (err != ESP_OK) {
            fatal_error("BOOST OFF", err);
        }
    }
}


/* ============================================================
 * USB strings
 * ============================================================ */

static void usb_string_to_ascii(
    const usb_str_desc_t *desc,
    char *output,
    size_t output_size
)
{
    if (output_size == 0) {
        return;
    }

    output[0] = '\0';

    if (
        desc == NULL ||
        desc->bLength < 2
    ) {
        snprintf(
            output,
            output_size,
            "-"
        );
        return;
    }


    size_t chars =
        (desc->bLength - 2) / 2;


    if (chars >= output_size) {
        chars = output_size - 1;
    }


    for (size_t i = 0; i < chars; i++) {

        uint16_t c = desc->wData[i];

        if (
            c >= 32 &&
            c <= 126
        ) {
            output[i] = (char)c;
        }
        else {
            output[i] = '?';
        }
    }

    output[chars] = '\0';
}


/* ============================================================
 * Parse HID interface and interrupt IN endpoint
 *
 * Deliberately parses raw descriptor bytes so we can see exactly
 * what Wireless PRO presents.
 * ============================================================ */

static bool find_hid_interface(
    const usb_config_desc_t *config,
    hid_interface_info_t *info
)
{
    memset(
        info,
        0,
        sizeof(*info)
    );

    info->interface_number = -1;


    const uint8_t *raw =
        (const uint8_t *)config;

    int offset = 0;
    int total = config->wTotalLength;

    bool inside_hid = false;


    while (offset + 2 <= total) {

        uint8_t length =
            raw[offset];

        uint8_t type =
            raw[offset + 1];


        if (
            length < 2 ||
            offset + length > total
        ) {
            break;
        }


        /*
         * INTERFACE descriptor = type 4
         */

        if (
            type == 4 &&
            length >= 9
        ) {

            /*
             * If we already found a HID interface with an
             * interrupt IN endpoint, we're finished.
             */

            if (
                info->interface_number >= 0 &&
                info->in_endpoint != 0
            ) {
                return true;
            }


            uint8_t interface_class =
                raw[offset + 5];


            inside_hid =
                interface_class == 0x03;


            if (inside_hid) {

                info->interface_number =
                    raw[offset + 2];

                info->alternate_setting =
                    raw[offset + 3];

                info->endpoint_count =
                    raw[offset + 4];

                info->subclass =
                    raw[offset + 6];

                info->protocol =
                    raw[offset + 7];

                info->in_endpoint = 0;
                info->in_mps = 0;
                info->in_interval = 0;
            }
        }


        /*
         * ENDPOINT descriptor = type 5
         */

        else if (
            type == 5 &&
            length >= 7 &&
            inside_hid
        ) {

            uint8_t endpoint_address =
                raw[offset + 2];

            uint8_t attributes =
                raw[offset + 3];

            uint16_t mps =
                (uint16_t)raw[offset + 4] |
                ((uint16_t)raw[offset + 5] << 8);

            mps &= 0x07FF;


            bool is_in =
                (endpoint_address & 0x80) != 0;

            bool is_interrupt =
                (attributes & 0x03) == 0x03;


            if (
                is_in &&
                is_interrupt
            ) {

                info->in_endpoint =
                    endpoint_address;

                info->in_mps =
                    mps;

                info->in_interval =
                    raw[offset + 6];
            }
        }


        offset += length;
    }


    return
        info->interface_number >= 0 &&
        info->in_endpoint != 0;
}


/* ============================================================
 * Battery response display
 * ============================================================ */

static void show_battery_response(
    const uint8_t *data,
    int length
)
{
    char hex[200];

    int pos = 0;

    int print_length = length;

    if (print_length > 32) {
        print_length = 32;
    }


    for (
        int i = 0;
        i < print_length &&
        pos < (int)sizeof(hex) - 4;
        i++
    ) {
        pos += snprintf(
            &hex[pos],
            sizeof(hex) - pos,
            "%02X ",
            data[i]
        );
    }

    if (pos > 0) {
        hex[pos - 1] = '\0';
    }
    else {
        hex[0] = '\0';
    }


    core_power_info_t core;

    int core_percent = -1;
    float core_voltage = 0.0f;

    if (
        read_core_power(&core) ==
        ESP_OK
    ) {
        core_percent =
            core.battery_percent;

        core_voltage =
            core.battery_voltage;
    }


    if (
        length >= BATTERY_RESPONSE_MIN_LENGTH &&
        data[0] ==
            BATTERY_RESPONSE_REPORT_ID &&
        data[1] ==
            BATTERY_COMMAND &&
        data[2] ==
            0x41
    ) {

        uint16_t battery_mv =
            (uint16_t)data[3] |
            ((uint16_t)data[4] << 8);

        uint16_t temperature_c =
            (uint16_t)data[7] |
            ((uint16_t)data[8] << 8);

        uint8_t charge_state =
            data[9];

        uint16_t charge_current_ma =
            (uint16_t)data[10] |
            ((uint16_t)data[11] << 8);


        screen_printf(
            "WIRELESS PRO BATTERY\n\n"
            "RODE: %u mV\n"
            "Temp: %u C\n"
            "State: 0x%02X\n"
            "Current: %u mA\n\n"
            "Core: %d%%  %.3f V\n\n"
            "RX %d bytes:\n"
            "%s",
            battery_mv,
            temperature_c,
            charge_state,
            charge_current_ma,
            core_percent,
            core_voltage,
            length,
            hex
        );
    }
    else {

        screen_printf(
            "BATTERY RESPONSE\n\n"
            "Unexpected reply\n\n"
            "RX %d bytes:\n"
            "%s\n\n"
            "Core: %d%%  %.3f V",
            length,
            hex,
            core_percent,
            core_voltage
        );
    }
}


/* ============================================================
 * HID interrupt-IN callback
 *
 * USB client callback context: copy the raw response to a queue only.
 * ============================================================ */

static void battery_input_callback(
    usb_transfer_t *transfer
)
{
    if (
        transfer->status == USB_TRANSFER_STATUS_COMPLETED &&
        transfer->actual_num_bytes > 0 &&
        hid_response_queue != NULL
    ) {
        hid_response_t response = {0};

        response.length = transfer->actual_num_bytes;

        if (response.length > HID_TUNNEL_MAX_PAYLOAD) {
            response.length = HID_TUNNEL_MAX_PAYLOAD;
        }

        memcpy(
            response.destination,
            active_requester,
            ESP_NOW_ETH_ALEN
        );
        response.transaction_id = active_transaction_id;
        response.tunnel_transaction = active_tunnel_transaction;
        memcpy(
            response.data,
            transfer->data_buffer,
            response.length
        );

        (void)xQueueSend(
            hid_response_queue,
            &response,
            0
        );
    }

    battery_transaction_active = false;
}


/* ============================================================
 * HID SET_REPORT callback
 *
 * Once the opaque report has been written successfully, submit the
 * already-proven interrupt-IN transfer.
 * ============================================================ */

static void battery_control_callback(
    usb_transfer_t *transfer
)
{
    if (
        transfer->status != USB_TRANSFER_STATUS_COMPLETED ||
        battery_input_transfer == NULL ||
        hid_in_endpoint == 0 ||
        hid_in_mps == 0
    ) {
        battery_transaction_active = false;
        return;
    }

    battery_input_transfer->device_handle = device_hdl;
    battery_input_transfer->bEndpointAddress = hid_in_endpoint;
    battery_input_transfer->callback = battery_input_callback;
    battery_input_transfer->context = NULL;

    int receive_length = hid_in_mps;

    while (receive_length < active_request_length) {
        receive_length += hid_in_mps;
    }

    battery_input_transfer->num_bytes = receive_length;

    esp_err_t err = usb_host_transfer_submit(
        battery_input_transfer
    );

    if (err != ESP_OK) {
        battery_transaction_active = false;
    }
}


/* ============================================================
 * Start one opaque HID SET_REPORT followed by interrupt-IN.
 * ============================================================ */

static esp_err_t hid_set_report_transaction(
    const uint8_t *request,
    size_t request_length,
    const uint8_t requester[ESP_NOW_ETH_ALEN],
    uint32_t transaction_id,
    bool tunnel_transaction
)
{
    if (
        device_hdl == NULL ||
        claimed_hid_interface < 0 ||
        hid_in_endpoint == 0 ||
        request == NULL ||
        request_length == 0 ||
        request_length > HID_TUNNEL_MAX_PAYLOAD
    ) {
        return ESP_ERR_INVALID_ARG;
    }

    if (battery_transaction_active) {
        return ESP_ERR_INVALID_STATE;
    }

    if (battery_control_transfer == NULL) {
        esp_err_t err = usb_host_transfer_alloc(
            8 + HID_TUNNEL_MAX_PAYLOAD,
            0,
            &battery_control_transfer
        );

        if (err != ESP_OK) {
            return err;
        }
    }

    int receive_capacity = hid_in_mps;

    while (receive_capacity < HID_TUNNEL_MAX_PAYLOAD) {
        receive_capacity += hid_in_mps;
    }

    if (battery_input_transfer == NULL) {
        esp_err_t err = usb_host_transfer_alloc(
            receive_capacity,
            0,
            &battery_input_transfer
        );

        if (err != ESP_OK) {
            return err;
        }
    }

    uint8_t *buf = battery_control_transfer->data_buffer;

    buf[0] = 0x21;
    buf[1] = 0x09;
    buf[2] = request[0];
    buf[3] = 0x02;
    buf[4] = claimed_hid_interface & 0xFF;
    buf[5] = 0x00;
    buf[6] = request_length & 0xFF;
    buf[7] = (request_length >> 8) & 0xFF;

    memcpy(&buf[8], request, request_length);

    battery_control_transfer->device_handle = device_hdl;
    battery_control_transfer->bEndpointAddress = 0x00;
    battery_control_transfer->callback = battery_control_callback;
    battery_control_transfer->context = NULL;
    battery_control_transfer->num_bytes = 8 + request_length;

    active_request_length = request_length;
    active_transaction_id = transaction_id;
    active_tunnel_transaction = tunnel_transaction;

    if (requester != NULL) {
        memcpy(active_requester, requester, ESP_NOW_ETH_ALEN);
    }
    else {
        memset(active_requester, 0, ESP_NOW_ETH_ALEN);
    }

    battery_transaction_active = true;

    esp_err_t err = usb_host_transfer_submit_control(
        client_hdl,
        battery_control_transfer
    );

    if (err != ESP_OK) {
        battery_transaction_active = false;
    }

    return err;
}


/*
 * Preserved known-good diagnostic entry point. It is not triggered
 * automatically; the tunnel supplies this same payload for testing.
 */
static esp_err_t wireless_pro_read_battery(void)
{
    return hid_set_report_transaction(
        wireless_pro_battery_request,
        sizeof(wireless_pro_battery_request),
        NULL,
        0,
        false
    );
}


/* ============================================================
 * USB host daemon
 * ============================================================ */

static void usb_lib_task(void *arg)
{
    while (1) {

        uint32_t event_flags = 0;

        esp_err_t err =
            usb_host_lib_handle_events(
                portMAX_DELAY,
                &event_flags
            );


        if (err != ESP_OK) {

            ESP_LOGE(
                TAG,
                "USB library: %s",
                esp_err_to_name(err)
            );

            vTaskDelay(
                pdMS_TO_TICKS(100)
            );
        }
    }
}


/* ============================================================
 * USB client event callback
 * ============================================================ */

static void usb_client_event_cb(
    const usb_host_client_event_msg_t *event_msg,
    void *arg
)
{
    switch (event_msg->event) {

        case USB_HOST_CLIENT_EVENT_NEW_DEV:

            pending_device_address =
                event_msg->new_dev.address;

            break;


        case USB_HOST_CLIENT_EVENT_DEV_GONE:

            if (
                device_hdl != NULL &&
                event_msg->dev_gone.dev_hdl ==
                    device_hdl
            ) {
                device_gone = true;
            }

            break;


        default:
            break;
    }
}


/* ============================================================
 * Inspect device and prepare the HID interface
 * ============================================================ */

static void inspect_usb_device(
    uint8_t address
)
{
    esp_err_t err;


    screen_printf(
        "DAMspy M5 Control\n\n"
        "USB DEVICE FOUND\n\n"
        "Address %d\n\n"
        "Opening...",
        address
    );


    err = usb_host_device_open(
        client_hdl,
        address,
        &device_hdl
    );


    if (err != ESP_OK) {

        screen_printf(
            "USB OPEN ERROR\n\n"
            "%s",
            esp_err_to_name(err)
        );

        return;
    }


    const usb_device_desc_t *dev_desc =
        NULL;


    err = usb_host_get_device_descriptor(
        device_hdl,
        &dev_desc
    );


    if (
        err != ESP_OK ||
        dev_desc == NULL
    ) {

        screen_printf(
            "DESCRIPTOR ERROR\n\n"
            "%s",
            esp_err_to_name(err)
        );

        return;
    }


    usb_device_info_t info = {0};


    err = usb_host_device_info(
        device_hdl,
        &info
    );


    if (err != ESP_OK) {

        screen_printf(
            "DEVICE INFO ERROR\n\n"
            "%s",
            esp_err_to_name(err)
        );

        return;
    }


    char manufacturer[80];
    char product[100];
    char serial[100];


    usb_string_to_ascii(
        info.str_desc_manufacturer,
        manufacturer,
        sizeof(manufacturer)
    );

    usb_string_to_ascii(
        info.str_desc_product,
        product,
        sizeof(product)
    );

    usb_string_to_ascii(
        info.str_desc_serial_num,
        serial,
        sizeof(serial)
    );


    const usb_config_desc_t *config_desc =
        NULL;


    err =
        usb_host_get_active_config_descriptor(
            device_hdl,
            &config_desc
        );


    if (
        err != ESP_OK ||
        config_desc == NULL
    ) {

        screen_printf(
            "CONFIG ERROR\n\n"
            "%s",
            esp_err_to_name(err)
        );

        return;
    }


    hid_interface_info_t hid;


    bool has_hid =
        find_hid_interface(
            config_desc,
            &hid
        );


    core_power_info_t core;

    int core_percent = -1;
    float core_voltage = 0.0f;


    if (
        read_core_power(&core) ==
        ESP_OK
    ) {

        core_percent =
            core.battery_percent;

        core_voltage =
            core.battery_voltage;
    }


    if (!has_hid) {

        screen_printf(
            "USB DEVICE\n\n"
            "VID %04X PID %04X\n"
            "%s\n"
            "SN: %s\n\n"
            "NO HID IN ENDPOINT",
            dev_desc->idVendor,
            dev_desc->idProduct,
            product,
            serial
        );

        return;
    }


    /*
     * Claim HID interface before using its endpoint.
     */

    err = usb_host_interface_claim(
        client_hdl,
        device_hdl,
        hid.interface_number,
        hid.alternate_setting
    );


    if (err != ESP_OK) {

        screen_printf(
            "HID CLAIM FAILED\n\n"
            "Interface %d\n"
            "%s",
            hid.interface_number,
            esp_err_to_name(err)
        );

        return;
    }


    claimed_hid_interface =
        hid.interface_number;

    hid_in_endpoint =
        hid.in_endpoint;

    hid_in_mps =
        hid.in_mps;


    screen_printf(
        "RODE USB DEVICE\n\n"
        "VID %04X PID %04X\n"
        "%s\n"
        "SN: %s\n\n"
        "HID IF %d\n"
        "IN EP 0x%02X MPS %u\n\n"
        "Core: %d%% %.3fV\n\n"
        "HID ready",
        dev_desc->idVendor,
        dev_desc->idProduct,
        product,
        serial,
        hid.interface_number,
        hid.in_endpoint,
        hid.in_mps,
        core_percent,
        core_voltage
    );


    /*
     * Keep the known-good Wireless PRO battery request and parser above
     * as diagnostic code, but do not trigger the request automatically.
     */

    if (
        dev_desc->idVendor ==
            RODE_VID &&
        dev_desc->idProduct ==
            WIRELESS_PRO_RX_PID
    ) {

        screen_printf(
            "WIRELESS PRO READY\n\n"
            "VID %04X PID %04X\n"
            "%s\n"
            "SN: %s\n\n"
            "HID IF %d\n"
            "IN EP 0x%02X MPS %u\n\n"
            "Waiting for tunnel test",
            dev_desc->idVendor,
            dev_desc->idProduct,
            product,
            serial,
            hid.interface_number,
            hid.in_endpoint,
            hid.in_mps
        );
    }
    else {

        screen_printf(
            "USB HID DEVICE\n\n"
            "VID %04X PID %04X\n"
            "%s\n\n"
            "Not Wireless PRO RX\n"
            "No command sent.",
            dev_desc->idVendor,
            dev_desc->idProduct,
            product
        );
    }
}


/* ============================================================
 * Main
 * ============================================================ */

void app_main(void)
{
    esp_err_t err;


    /* --------------------------------------------------------
     * Display
     * -------------------------------------------------------- */

    bsp_display_start();
    bsp_display_backlight_on();


    screen_printf(
        "DAMspy M5 Control\n\n"
        "Checking power..."
    );


    /* --------------------------------------------------------
     * Power
     * -------------------------------------------------------- */

    init_power_devices();


    core_power_info_t power;


    err = read_core_power(
        &power
    );


    if (err != ESP_OK) {
        fatal_error(
            "read core power",
            err
        );
    }


    /* ========================================================
     * CHARGE MODE
     * ======================================================== */

    if (power.vbus_present) {

        cores3_usb_power(false);


        while (1) {

            err =
                read_core_power(
                    &power
                );


            if (err != ESP_OK) {

                screen_printf(
                    "DAMspy M5 Control\n\n"
                    "CHARGE MODE\n\n"
                    "Power read error"
                );

                vTaskDelay(
                    pdMS_TO_TICKS(2000)
                );

                continue;
            }


            if (power.vbus_present) {

                screen_printf(
                    "DAMspy M5 Control\n\n"
                    "CHARGE MODE\n\n"
                    "External VBUS: YES\n\n"
                    "Core battery: %d%%\n"
                    "Battery: %.3f V\n\n"
                    "%s\n\n"
                    "Unplug + RESET\n"
                    "for HOST mode",
                    power.battery_percent,
                    power.battery_voltage,
                    power.charging
                        ? "CHARGING"
                        : "Powered / standby"
                );
            }
            else {

                screen_printf(
                    "DAMspy M5 Control\n\n"
                    "CHARGE MODE\n\n"
                    "External VBUS removed\n\n"
                    "Core battery: %d%%\n"
                    "Battery: %.3f V\n\n"
                    "Press RESET\n"
                    "for HOST mode",
                    power.battery_percent,
                    power.battery_voltage
                );
            }


            vTaskDelay(
                pdMS_TO_TICKS(2000)
            );
        }
    }


    /* ========================================================
     * HOST MODE
     * ======================================================== */

    for (int i = 5; i > 0; i--) {

        screen_printf(
            "DAMspy M5 Control\n\n"
            "HOST MODE\n\n"
            "Core battery: %d%%\n"
            "Battery: %.3f V\n\n"
            "USB host in %d...",
            power.battery_percent,
            power.battery_voltage,
            i
        );


        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );
    }


    screen_printf(
        "DAMspy M5 Control\n\n"
        "HOST MODE\n\n"
        "Core: %d%% %.3fV\n\n"
        "Enabling DUT VBUS...",
        power.battery_percent,
        power.battery_voltage
    );


    cores3_usb_power(true);


    /* --------------------------------------------------------
     * ESP-NOW opaque HID transport
     * -------------------------------------------------------- */

    hid_response_queue = xQueueCreate(
        2,
        sizeof(hid_response_t)
    );

    if (hid_response_queue == NULL) {
        fatal_error("HID response queue", ESP_ERR_NO_MEM);
    }

    err = espnow_transport_start();

    if (err != ESP_OK) {
        fatal_error(
            "ESP-NOW transport",
            err
        );
    }


    /* --------------------------------------------------------
     * USB host
     * -------------------------------------------------------- */

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED
    };


    err = usb_host_install(
        &host_config
    );


    if (err != ESP_OK) {
        fatal_error(
            "usb_host_install",
            err
        );
    }


    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,

        .async = {
            .client_event_callback =
                usb_client_event_cb,

            .callback_arg =
                NULL
        }
    };


    err = usb_host_client_register(
        &client_config,
        &client_hdl
    );


    if (err != ESP_OK) {
        fatal_error(
            "USB client",
            err
        );
    }


    BaseType_t task_ok =
        xTaskCreate(
            usb_lib_task,
            "usb_lib",
            4096,
            NULL,
            10,
            NULL
        );


    if (task_ok != pdPASS) {

        screen_printf(
            "USB ERROR\n\n"
            "Cannot start\n"
            "USB library task"
        );

        while (1) {
            vTaskDelay(
                pdMS_TO_TICKS(1000)
            );
        }
    }


    screen_printf(
        "DAMspy M5 Control\n\n"
        "USB HOST READY\n\n"
        "Core: %d%% %.3fV\n\n"
        "Waiting for RODE...",
        power.battery_percent,
        power.battery_voltage
    );


    /* --------------------------------------------------------
     * Event loop
     * -------------------------------------------------------- */

    while (1) {

        err =
            usb_host_client_handle_events(
                client_hdl,
                pdMS_TO_TICKS(100)
            );


        if (
            err != ESP_OK &&
            err != ESP_ERR_TIMEOUT
        ) {

            ESP_LOGW(
                TAG,
                "client events: %s",
                esp_err_to_name(err)
            );
        }


        hid_response_t hid_response;

        while (
            xQueueReceive(
                hid_response_queue,
                &hid_response,
                0
            ) == pdTRUE
        ) {
            show_battery_response(
                hid_response.data,
                hid_response.length
            );

            if (hid_response.tunnel_transaction) {
                esp_err_t send_err =
                    espnow_transport_send_response(
                        hid_response.destination,
                        hid_response.transaction_id,
                        hid_response.data,
                        hid_response.length
                    );

                if (send_err != ESP_OK) {
                    ESP_LOGE(
                        TAG,
                        "ESP-NOW response: %s",
                        esp_err_to_name(send_err)
                    );
                }
            }
        }


        if (
            !battery_transaction_active &&
            device_hdl != NULL &&
            claimed_hid_interface >= 0
        ) {
            espnow_hid_request_t request;

            if (
                espnow_transport_receive_request(
                    &request,
                    0
                )
            ) {
                esp_err_t hid_err =
                    hid_set_report_transaction(
                        request.payload,
                        request.payload_length,
                        request.source,
                        request.transaction_id,
                        true
                    );

                if (hid_err != ESP_OK) {
                    ESP_LOGE(
                        TAG,
                        "HID tunnel request: %s",
                        esp_err_to_name(hid_err)
                    );
                }
            }
        }


        /*
         * New USB device
         */

        if (
            pending_device_address >= 0 &&
            device_hdl == NULL
        ) {

            int address =
                pending_device_address;

            pending_device_address = -1;


            inspect_usb_device(
                (uint8_t)address
            );
        }


        /*
         * Device removed
         */

        if (
            device_gone &&
            device_hdl != NULL
        ) {

            device_gone = false;
            battery_transaction_active = false;
            xQueueReset(hid_response_queue);


            if (
                claimed_hid_interface >= 0
            ) {

                usb_host_interface_release(
                    client_hdl,
                    device_hdl,
                    claimed_hid_interface
                );
            }


            usb_host_device_close(
                client_hdl,
                device_hdl
            );


            device_hdl = NULL;

            claimed_hid_interface = -1;

            hid_in_endpoint = 0;
            hid_in_mps = 0;


            read_core_power(
                &power
            );


            screen_printf(
                "DAMspy M5 Control\n\n"
                "USB HOST READY\n\n"
                "Device removed\n\n"
                "Core: %d%% %.3fV\n\n"
                "Waiting for RODE...",
                power.battery_percent,
                power.battery_voltage
            );
        }
    }
}
