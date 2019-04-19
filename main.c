/* Copyright (c) 2019, EmbeddedCentric.com
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//////////////////////////////////////////////////////////////////////////////////
// Company: EmbeddedCentric.com
// Create Date: 19 April 2019
// Title: Touch Screen Demo 1
//
// Description: This demo is part of Nordic nRF5x BLE In-Depth Training Course -Intermediate Level ( Optional extracurricular activity after lesson 12)
// A simple demo program that utilizes the nRF52840 DK, and gen4-uLCD-24PT touch screen by 4D Systems.
// The demo supports the following: 
// 1. Control LED4 on the nRF52840 DK through both the touch screen and BLE.
// 2. Get the status of Button 4 on the nRF52840 DK through both the touch screen and BLE.
// 3. Read the nRF52840 on-chip tempreture sensor through both the touch screen and BLE.
// The demo comes with a custom made Android App to interact with the board. nRF Connect could also be used.
// The demo creates a NUS BLE service. NUS stands for  Nordic UART service.
// NUS sets up one "RX" characteristic with "write" properties, and one "TX" characteristic with "notify" properties datachannel, to fit basic communication needs.
// The LED4 is controlled through the RX characteristics writing  (0x46 == ON) and (0x4F == OFF).
// When notification is enabled on the TX characteristics, pressing Button4 will send the string "Button 4 Pressed". In addition the tempreture of the chip is sent over the TX characteristics every TEMP_RADING_INTERVAL time

// Interface to the touch screen is serial following the Genie Standard Protocol.
// LCD RX -> p0.27
// LCD TX -> p0.26

// Important notes:
// *Segger Embedded Studio V4.12 is used to build this demo. nRF5 SDK 15.2.0 and above is needed.
// *Double check that the nRF5SDK global macro is pointing to the right location where the nRF5 SDK is downloaded on your local disk. This can be done by -
//  checking  Tools Menu -> Options -> Building -> Global Macros (nRF5SDK) and compare it to the root directory where the nRF5 SDK is downloaded
//////////////////////////////////////////////////////////////////////////////////


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "app_uart.h"
#include "ble_nus.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_gpio.h"     
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "ECDemo_Extra1"                             /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */
#define MESSAGE_BUFFER_SIZE             32                                          /**< Lenght of the temperature reading sent over NUS */
#define TEMP_RADING_INTERVAL            4000                                        /**< How Frequent we read the tempreture. */
#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//Screen Commands
#define LCD_RX                          NRF_GPIO_PIN_MAP(0,27)
#define LCD_TX                          NRF_GPIO_PIN_MAP(0,26)
#define THERMOMETER_OFFSET              20
#define THERMOMETER_MAX                 70
#define THERMOMETER_CMD_WIDTH           6
#define THERMOMETER_VALUE               4
#define THERMOMETER_CHECKSUM            5
#define LED_CMD_WIDTH                   6

unsigned char const led_on_cmd[]  =     {0x7, 0x1E, 0x0, 0x0, 0x1,0x18};
unsigned char const led_off_cmd[] =     {0x7, 0x1E, 0x0, 0x0, 0x0,0x19};
unsigned char const button_press_mesg[]  = {0x1, 0x13, 0x0, 0x0, 0x1,0x13}; 
unsigned char const button_depress_mesg[]  = {0x1, 0x13, 0x0, 0x0, 0x0,0x12}; 

/*Step-1 
Macro for defining a ble_nus instance.
A-Name of the instance.
B-Maximum number of NUS clients connected at a time.
*/
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
/*Step-2
define a nrf_ble_gatt instance.
*/
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
/*Step-3
define a nrf_ble_qwr instance
*/
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
/*Step-4
define a ble_advertising instance.
*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

/**< Handler for repeated timer used read the tempreture sensor . */
APP_TIMER_DEF(temp_reading_timer_id); 
    
static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

//Timer callback for reading the chip temperature
static void tempreading_timer_handler(void * p_context)
{
    int32_t temp; //Temperature reading
    int return_status;
    uint32_t err_code;
    char message_buffer [MESSAGE_BUFFER_SIZE]; //Message sent over UART buffer
          if (sd_temp_get(&temp)== NRF_SUCCESS){
            temp *= 0.25;
            return_status = snprintf ( message_buffer, MESSAGE_BUFFER_SIZE, "Temperature=%d\r\n\0",(int)temp);
            if(return_status>=0 && return_status < MESSAGE_BUFFER_SIZE){
              uint16_t length = strlen(message_buffer);
              if(m_conn_handle !=BLE_CONN_HANDLE_INVALID)
              ble_nus_data_send(&m_nus,message_buffer,&length,m_conn_handle);
                          
              //Send Data to the screen
              //Add 20 offset to temp
              temp += THERMOMETER_OFFSET; 
                unsigned char thermo_cmd[THERMOMETER_CMD_WIDTH] ={0x1,0x12,};
                if((temp >=0) && (temp <=THERMOMETER_MAX)){
                  thermo_cmd[THERMOMETER_VALUE] = (char)temp;
                  for(int i =0;i<THERMOMETER_CHECKSUM;i++)
                  thermo_cmd[THERMOMETER_CHECKSUM] ^=  thermo_cmd[i];

                  for( int i =0;i<THERMOMETER_CMD_WIDTH;i++)
                  UNUSED_VARIABLE(app_uart_put(thermo_cmd[i]));
                  }
             
            }
          }
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&temp_reading_timer_id,
                            APP_TIMER_MODE_REPEATED,
                            tempreading_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(temp_reading_timer_id, APP_TIMER_TICKS(TEMP_RADING_INTERVAL), NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.

 The Bluetooth Device Address is set here. It is similar to a Media Access Control address (MAC address), but not necessarily universally unique.
 There are four types of addresse:
 -Public address.

This is a global, fixed address. It must be registered with the IEEE Registration Authority and will never change during the lifetime of the device.

-Random Static address.

A random static address is simply a random number that can either be generated every time the device boots up or can stay the same for the lifetime of the device. But it cannot be changed within a single power cycle of the device.

-Private Resolvable address.

Those are generated from an identity resolving key (IRK) and a random number, and they can be changed often (even during the lifetime of a connection) to avoid the device being identified and tracked by an unknown scanning device. Only devices that possess the IRK distributed by the device using a private resolvable address can actually resolve that address, allowing them to identify the device.

-Private Non-Resolvable address.

Not very commonly used, a random number that you can change anytime.

The Address used here is the default: Random Static. 
 */


static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */

/*
Available Evenets:
    BLE_NUS_EVT_RX_DATA,      /**< Data received. 
    BLE_NUS_EVT_TX_RDY,       /**< Service is ready to accept new data to be transmitted.
    BLE_NUS_EVT_COMM_STARTED, /**< Notification has been enabled. 
    BLE_NUS_EVT_COMM_STOPPED, /**< Notification has been disabled. */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_INFO("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        if (*(p_evt->params.rx_data.p_data) == (uint8_t)'O'){
            nrf_gpio_pin_write(BSP_LED_3,1); 
        }

        if (*(p_evt->params.rx_data.p_data) == (uint8_t)'F'){
            nrf_gpio_pin_write(BSP_LED_3,0); 
        }
/*
        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {

        }
        */
    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

 /*
Function for initializing the Nordic UART Service. 
ble_nus_t *  	p_nus,
Nordic UART Service structure. This structure must be supplied by the application. 
It is initialized by this function and will later be used to identify this particular service instance. 
ble_nus_init_t const *  	p_nus_init
Information needed to initialize the service. 
*/ 
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

//Connection Parameters Negotiation
/**@brief Function for initializing the Connection Parameters module.
Parameters:
p_conn_params;                    //!< Pointer to the connection parameters desired by the application. When calling ble_conn_params_init, if this parameter is set to NULL, the connection parameters will be fetched from host.
first_conn_params_update_delay;   //!< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (in number of timer ticks).
next_conn_params_update_delay;    //!< Time between each call to sd_ble_gap_conn_param_update after the first (in number of timer ticks). Recommended value 30 seconds as per BLUETOOTH SPECIFICATION Version 4.0.
max_conn_params_update_count;     //!< Number of attempts before giving up the negotiation.
start_on_notify_cccd_handle;      //!< If procedure is to be started when notification is started, set this to the handle of the corresponding CCCD. Set to BLE_GATT_HANDLE_INVALID if procedure is to be started on connect event.
disconnect_on_fail;               //!< Set to TRUE if a failed connection parameters update shall cause an automatic disconnection, set to FALSE otherwise.
evt_handler;                      //!< Event handler to be called for handling events in the Connection Parameters.
error_handler;                    //!< Function to be called in case of an error.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
  uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;	 
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);															   
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    static bool toggle_effect = false;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;
        case BSP_EVENT_KEY_3:
             NRF_LOG_INFO("Button 4 was pressed");
             uint8_t data_to_send[] = "Button 4 Pressed";
             uint16_t length = sizeof(data_to_send);
             if(m_conn_handle !=BLE_CONN_HANDLE_INVALID)
             ble_nus_data_send(&m_nus,data_to_send,&length,m_conn_handle);

             //Send to the screen that Button 4 is pressed 
             if(!toggle_effect){
              for(int i=0;i<sizeof(button_press_mesg);i++)
              UNUSED_VARIABLE(app_uart_put(button_press_mesg[i]));
              toggle_effect = true; 
             }
             else{
              for(int i=0;i<sizeof(button_depress_mesg);i++)
              UNUSED_VARIABLE(app_uart_put(button_depress_mesg[i]));
              toggle_effect = false; 
             }

          break;
        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
  // Advertising Module Setup

 */
static void advertising_init(void)
{
    uint32_t               err_code;
    /*
    ble_advertising_init_t       
    advdata;         Advertising data: name, appearance, discovery flags, and more. 
    srdata;          Scan response data: Supplement to advertising data. 
    config;          Select which advertising modes and intervals will be utilized.
    evt_handler;     Event handler that will be called upon advertising events.
    error_handler;  Error handler that will propogate internal errors to the main applications.
        */

    /*
    config
    bool     ble_adv_on_disconnect_disabled;     /**< Enable or disable automatic return to advertising upon disconnecting.
    bool     ble_adv_whitelist_enabled;          /**< Enable or disable use of the whitelist. 
    bool     ble_adv_directed_high_duty_enabled; /**< Enable or disable high duty direct advertising mode. Can not be used together with extended advertising. 
    bool     ble_adv_directed_enabled;           /**< Enable or disable direct advertising mode. 
    bool     ble_adv_fast_enabled;               /**< Enable or disable fast advertising mode. 
    bool     ble_adv_slow_enabled;               /**< Enable or disable slow advertising mode. 
    uint32_t ble_adv_directed_interval;          /**< Advertising interval for directed advertising. 
    uint32_t ble_adv_directed_timeout;           /**< Time-out (number of tries) for direct advertising. 
    uint32_t ble_adv_fast_interval;              /**< Advertising interval for fast advertising. 
    uint32_t ble_adv_fast_timeout;               /**< Time-out (in units of 10ms) for fast advertising. 
    uint32_t ble_adv_slow_interval;              /**< Advertising interval for slow advertising.
    uint32_t ble_adv_slow_timeout;               /**< Time-out (in units of 10ms) for slow advertising.
    bool     ble_adv_extended_enabled;           /**< Enable or disable extended advertising.
    uint32_t ble_adv_secondary_phy;              /**< PHY for the secondary (extended) advertising @ref BLE_GAP_PHYS (BLE_GAP_PHY_1MBPS, BLE_GAP_PHY_2MBPS or BLE_GAP_PHY_CODED).
    uint32_t ble_adv_primary_phy;                /**< PHY for the primary advertising. @ref BLE_GAP_PHYS (BLE_GAP_PHY_1MBPS, BLE_GAP_PHY_2MBPS or BLE_GAP_PHY_CODED).
    */
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[10];
    static uint8_t index = 0;
    uint32_t       err_code;
    uint8_t        temp_reading;
    static bool    read_flag=false;
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&temp_reading));
            if(temp_reading == led_on_cmd[0] || read_flag == true){
              data_array[index] =temp_reading;
              index++;
              read_flag=true;
            }
            //NRF_LOG_HEXDUMP_DEBUG(data_array, index);           
            if (index == LED_CMD_WIDTH){
              if(memcmp(data_array,led_on_cmd,LED_CMD_WIDTH) == 0){
              //Turn LED4 On
              nrf_gpio_pin_write(BSP_LED_3,0);
              }
            
           
              else if(memcmp(data_array,led_off_cmd,LED_CMD_WIDTH) == 0){
              // Turn LED4 Off
              nrf_gpio_pin_write(BSP_LED_3,1);
              }

              index = 0;
              read_flag=false;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART - used to interface with the touch screen] */

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void touchscreen_uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = LCD_RX,
        .tx_pin_no    = LCD_TX,
        .rts_pin_no   = RTS_PIN_NUMBER, // Not Used
        .cts_pin_no   = CTS_PIN_NUMBER, // Not Used
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud9600
#else
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud9600
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    //start advertising in any of the advertising modes that you enabled during initialization.
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;
    nrf_gpio_cfg_output(BSP_LED_3); //BSP_LED_3 is a symbolic constant defined in pca10056.h for pin 16(LED4) in the nRF52840-DK


    // Initialize.
    touchscreen_uart_init();
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);					  
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    NRF_LOG_INFO("EmbeddedCentric.com");
    NRF_LOG_INFO("Nordic nRF5x BLE In-Depth Training Course -Intermediate Level");
    advertising_start();

    // Enter main loop.
    for (;;)
    {
      
      idle_state_handle(); 

    }
}

