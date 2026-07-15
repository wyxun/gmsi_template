#include "at32f403a_407.h"
#include "usbd_core.h"
#include "cdc_class.h"
#include "cdc_desc.h"
#include "usbd_int.h"
#include "perf_counter.h"

usbd_core_type usb_core_dev;

void *get_usb_core_dev(void)
{
    return &usb_core_dev;
}

void usb_delay_ms(uint32_t ms)
{
    delay_ms(ms);
}

void usb_delay_us(uint32_t us)
{
    delay_us(us);
}

static void usb_clock48m_select(void)
{
    /* Select HICK as USB clock source */
    crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_HICK);

    /* Enable ACC peripheral clock */
    crm_periph_clock_enable(CRM_ACC_PERIPH_CLOCK, TRUE);

    /* Write target values for HICK calibration */
    acc_write_c1(7980);
    acc_write_c2(8000);
    acc_write_c3(8020);

    /* Enable ACC self-calibration */
    acc_calibration_mode_enable(ACC_CAL_HICKTRIM, TRUE);
}

void port_usb_Init(void)
{
    /* Configure NVIC for USBFS */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

    /* Configure USB clock source */
    usb_clock48m_select();

    /* Enable USB peripheral clock */
    crm_periph_clock_enable(CRM_USB_PERIPH_CLOCK, TRUE);

    /* Enable USB interrupt with priority 5 (below step timer priority 0) */
    nvic_irq_enable(USBFS_L_CAN1_RX0_IRQn, 5, 0);

    /* Initialize USB core as CDC Device */
    usbd_core_init(&usb_core_dev, USB, &cdc_class_handler, &cdc_desc_handler, 0);

    /* Connect USB pull-up to signal host */
    usbd_connect(&usb_core_dev);
}
