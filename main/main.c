#include <stdio.h> 
#include <stdlib.h> 
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h" 
#include "driver/gpio.h" 
#include "driver/adc.h" 
#include "esp_adc_cal.h"
#include "ssd1306.h"
  
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "sdkconfig.h"
#include "soc/soc_caps.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/rtc_io.h"
#include "soc/rtc.h"
#include "ulp.h"
#include "string.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "ds18b20.h"

#include "http.h"


#define TAG "SSD1306"
const int DS_PIN = 14; //GPIO where you connected ds18b20

#if SOC_TOUCH_SENSOR_NUM > 0
#include "soc/sens_periph.h"
#include "driver/touch_pad.h"
#endif

#define ADC_CHAN       ADC1_CHANNEL_6
#define NUMBER_PORTS   5

#define I2C_PORT_DATA  GPIO_NUM_21
#define I2C_PORT_CLOCK GPIO_NUM_22
#define SLAVE_ADDRS    0x3c
#define FREQ_CLOCK     100000

SSD1306_t ssd_conf;

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate 

#define NO_OF_SAMPLES   64

static RTC_DATA_ATTR struct timeval sleep_enter_time;

#ifdef CONFIG_EXAMPLE_ULP_TEMPERATURE_WAKEUP
#if CONFIG_IDF_TARGET_ESP32

#define ULP_DATA_OFFSET     36

_Static_assert(ULP_DATA_OFFSET < CONFIG_ESP32_ULP_COPROC_RESERVE_MEM/4 - 6, "ULP_DATA_OFFSET is set too high, or CONFIG_ESP32_ULP_COPROC_RESERVE_MEM is not sufficient");

static inline uint16_t ulp_data_read(size_t offset)
{
    return RTC_SLOW_MEM[ULP_DATA_OFFSET + offset] & 0xffff;
}

static inline void ulp_data_write(size_t offset, uint16_t value)
{
    RTC_SLOW_MEM[ULP_DATA_OFFSET + offset] = value;
}

#endif // CONFIG_IDF_TARGET_ESP32
#endif // CONFIG_EXAMPLE_ULP_TEMPERATURE_WAKEUP

#ifdef CONFIG_EXAMPLE_TOUCH_WAKEUP
#if CONFIG_IDF_TARGET_ESP32
#define TOUCH_THRESH_NO_USE 0
#endif
#endif

static esp_adc_cal_characteristics_t *adc_chars; 

#if CONFIG_IDF_TARGET_ESP32 
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2 
static const adc_bits_width_t width = ADC_WIDTH_BIT_12; 

#elif CONFIG_IDF_TARGET_ESP32S2 
static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2 
static const adc_bits_width_t width = ADC_WIDTH_BIT_13; 

#endif 

static const adc_atten_t atten = ADC_ATTEN_DB_11; 
static const adc_unit_t unit = ADC_UNIT_1; 

void temp(void)
{
    ds18b20_init(DS_PIN);

    char temperatura[50];
    int t = ds18b20_get_temp();
    snprintf(temperatura, sizeof(temperatura), " Temp:%d gradosC", t);
    printf("%s\n",temperatura);
    ssd1306_display_text(&ssd_conf, 0, temperatura, strlen(temperatura), false);
    struct url_data temperatura_data = {"1AzBxk0YBXay11GJBDkL/telemetry", "temperature", "", t};
    http_post(&temperatura_data);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void hum(void)
{
    if (unit == ADC_UNIT_1) { 

        adc1_config_width(width); 
        adc1_config_channel_atten(channel, atten); 

    } else { 

        adc2_config_channel_atten((adc2_channel_t)channel, atten); 

    } 

    //Characterize ADC 

    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t)); 
    //esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars); 

             
    int humedad=0;
    int adc_reading = 0; 

    //Multisampling 

    for (int i = 0; i < NO_OF_SAMPLES; i++) { 

        if (unit == ADC_UNIT_1) { 

            adc_reading += adc1_get_raw((adc1_channel_t)channel); 

        }       
        } 

    adc_reading /= NO_OF_SAMPLES; 

    char moisture[50];
    humedad = adc_reading * 100 / 4095;
	humedad = 100 - ((humedad - 25) * (100 / (62 - 25)));
    snprintf(moisture, sizeof(moisture), " Moisture:%d %%", humedad);
    ssd1306_display_text(&ssd_conf, 2, moisture, strlen(moisture), false);
    printf("%s\n", moisture); 
    struct url_data humedad_data = {"1AzBxk0YBXay11GJBDkL/telemetry", "humidity", "", humedad};
    http_post(&humedad_data);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}  




void initialize_oled(void);
void config_I2C_protocol(void);
void run_I2C(void);

void app_main(void)
{

    initialize_oled();
    nvs_flash_init();
    init_http();

    /*struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {
    #if CONFIG_EXAMPLE_EXT0_WAKEUP
            case ESP_SLEEP_WAKEUP_EXT0: {
                printf("Wake up from ext0\n");
                break;
            }
    #endif // CONFIG_EXAMPLE_EXT0_WAKEUP
    #ifdef CONFIG_EXAMPLE_EXT1_WAKEUP
            case ESP_SLEEP_WAKEUP_EXT1: {
                uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
                if (wakeup_pin_mask != 0) {
                    int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                    printf("Wake up from GPIO %d\n", pin);
                } else {
                    printf("Wake up from GPIO\n");
                }
                break;
            }
    #endif // CONFIG_EXAMPLE_EXT1_WAKEUP
    #if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
            case ESP_SLEEP_WAKEUP_GPIO: {
                uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
                if (wakeup_pin_mask != 0) {
                    int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                    printf("Wake up from GPIO %d\n", pin);
                } else {
                    printf("Wake up from GPIO\n");
                }
                break;
            }
    #endif //SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
            case ESP_SLEEP_WAKEUP_TIMER: {
                printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
                break;
            }
    #ifdef CONFIG_EXAMPLE_TOUCH_WAKEUP
            case ESP_SLEEP_WAKEUP_TOUCHPAD: {
                printf("Wake up from touch on pad %d\n", esp_sleep_get_touchpad_wakeup_status());
                break;
            }
    #endif // CONFIG_EXAMPLE_TOUCH_WAKEUP

            case ESP_SLEEP_WAKEUP_UNDEFINED:
            default:
                printf("Not a deep sleep reset\n");
        }
    
    */
   while (1)
   {
    temp();

    hum();


    vTaskDelay(1000 / portTICK_PERIOD_MS);
   }
   
    
   
   
    /*const int wakeup_time_sec = 10;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

    #if CONFIG_EXAMPLE_EXT0_WAKEUP
        const int ext_wakeup_pin_0 = 15;

        printf("Enabling EXT0 wakeup on pin GPIO%d\n", ext_wakeup_pin_0);
        esp_sleep_enable_ext0_wakeup(ext_wakeup_pin_0, 1);
        rtc_gpio_pullup_dis(ext_wakeup_pin_0);
        rtc_gpio_pulldown_en(ext_wakeup_pin_0);
    #endif // CONFIG_EXAMPLE_EXT0_WAKEUP
    #ifdef CONFIG_EXAMPLE_EXT1_WAKEUP
        const int ext_wakeup_pin_1 = 2;
        const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
        const int ext_wakeup_pin_2 = 4;
        const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;

        printf("Enabling EXT1 wakeup on pins GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2);
        esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    #  if CONFIG_EXAMPLE_EXT1_USE_INTERNAL_PULLUPS
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
        rtc_gpio_pullup_dis(ext_wakeup_pin_1);
        rtc_gpio_pulldown_en(ext_wakeup_pin_1);
        rtc_gpio_pullup_dis(ext_wakeup_pin_2);
        rtc_gpio_pulldown_en(ext_wakeup_pin_2);
    #  endif //CONFIG_EXAMPLE_EXT1_USE_INTERNAL_PULLUPS
    #endif // CONFIG_EXAMPLE_EXT1_WAKEUP

    #ifdef CONFIG_EXAMPLE_GPIO_WAKEUP
        const gpio_config_t config = {
            .pin_bit_mask = BIT(DEFAULT_WAKEUP_PIN),
            .mode = GPIO_MODE_INPUT,
        };
        ESP_ERROR_CHECK(gpio_config(&config));
        ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(BIT(DEFAULT_WAKEUP_PIN), DEFAULT_WAKEUP_LEVEL));
        printf("Enabling GPIO wakeup on pins GPIO%d\n", DEFAULT_WAKEUP_PIN);
    #endif


    #if CONFIG_IDF_TARGET_ESP32
        rtc_gpio_isolate(GPIO_NUM_12);
    #endif

        printf("Entering deep sleep\n");
        gettimeofday(&sleep_enter_time, NULL);

        esp_deep_sleep_start();
    */
}

void initialize_oled(void)
{
    run_I2C();
}



void config_I2C_protocol (void)
{
    i2c_master_init(&ssd_conf,I2C_PORT_DATA,I2C_PORT_CLOCK,CONFIG_RESET_GPIO);
    ssd1306_init(&ssd_conf,128,64);
    ssd_conf._flip = false;
    ssd1306_contrast(&ssd_conf,0xff);
    ssd1306_clear_screen(&ssd_conf,false);
}

void run_I2C (void)
{
    config_I2C_protocol();
}