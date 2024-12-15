/* File:   ADC_1_1
 *
 * Author: Victor Adolfo Palacio Bastidas
 *
 * El código configura el ADC y modifica el Duty Cycle en tres canales (salida
 * por tres pines) diferentes, para esto se usa un timer para que cambie el Duty
 * Cycle cada vez que el timer se desborda, el código también muestra por
 * consola el valor del duty cycle que se está aplicando en cada pin.
 *
 * Created on 14 de Septiembre de 2024, 14:34
 */

#include <stdio.h>
#include "driver/gpio.h"       //Encontrado en: C:\ESP32_ESPRESSIF\v5.2.1\esp-idf\components\driver\gpio\include\driver
#include "freertos/FreeRTOS.h" //Encontrado en: C:\ESP32_ESPRESSIF\v5.2.1\esp-idf\components\freertos\FreeRTOS-Kernel\include\freertos
#include "freertos/task.h"     //Encontrado en: C:\ESP32_ESPRESSIF\v5.2.1\esp-idf\components\freertos\FreeRTOS-Kernel\include\freertos
#include "esp_log.h"           //Encontrado en: C:\ESP32_ESPRESSIF\v5.2.1\esp-idf\components\log\include\esp_log.h
#include "freertos/timers.h"   //Encontrado en: C:\ESP32_ESPRESSIF\v5.2.1\esp-idf\components\freertos\FreeRTOS-Kernel\include\freertos
#include "driver/adc.h"        //Encontrado en: C:\ESP32_ESPRESSIF\v5.2.1\esp-idf\components\driver\deprecated\driver\adc.h

#define ledRed 25
#define ledGreen 26
#define ledBlue 27

/*Variables globales*/
uint8_t led_level = 0;
static const char *tag = "Main"; // Main es el módulo donde se está usando la librería, en este caso el main
TimerHandle_t xTimers;           // Este tipo de variable existe dentro del timers.h
int interval = 240;              // Intervalo de tiempo para el timer, o el evento
int timerId = 1;                 // Identificador del timer
int adc_valor = 0;

/*Funciones creadas*/
esp_err_t init_led(void);  // Debería tener otro nombre esta función, se utiliza para configurar los pines a usar
esp_err_t set_timer(void); // Configura el Timer
esp_err_t set_adc(void);   // Configura el ADC

/* vTimerCallback
* timers.h: línea 161: Each timer calls the same callback when it expires.
Define una función de devolución de llamada que será utilizada por varias
instancias del temporizador.
La función de devolución de llamada no hace nada más que contar la cantidad de
veces que el temporizador asociado expira y lo detiene una vez que ha expirado
10 veces, este control solo aplica cuando se usa la variable
xMaxExpiryCountBeforeStopping, en esta aplicación NO se está usando
*/
void vTimerCallback(TimerHandle_t pxTimer)
{
    adc_valor = adc1_get_raw(ADC1_CHANNEL_0); // Revisar el canal donde está conectado el adc !!!!!!!
    int adc_case = adc_valor / 1000;          // max = 4094, resolución de 12bits, debe ser 4095
    ESP_LOGI(tag, "ADC VALOR: %i", adc_valor);
    switch (adc_case)
    {
    case 0:
        gpio_set_level(ledRed, 0);
        gpio_set_level(ledGreen, 0);
        gpio_set_level(ledBlue, 0);
        break;
    case 1:
        gpio_set_level(ledRed, 1);
        gpio_set_level(ledGreen, 0);
        gpio_set_level(ledBlue, 0);
        break;
    case 2:
        gpio_set_level(ledRed, 0);
        gpio_set_level(ledGreen, 1);
        gpio_set_level(ledBlue, 0);
        break;
    case 3:
        gpio_set_level(ledRed, 0);
        gpio_set_level(ledGreen, 0);
        gpio_set_level(ledBlue, 1);
        break;
    case 4:
        gpio_set_level(ledRed, 1);
        gpio_set_level(ledGreen, 1);
        gpio_set_level(ledBlue, 1);
        break;
    default:
        break;
    }
}

/*Revisar, el main falta llamar la función de inicialización del led (pines a usar)*/
void app_main(void)
{
    init_led();
    set_adc();
    set_timer();
}
/* set_timer
/// @brief Configuura el timer así:
* Funcion xTimerCreate crea y configura el timer
* Timer: es el nombre, no es usado por el Kernel
* (pdMS_TO_TICKS(interval)): función que pasa de ms a ticks para definir
el PERIODO de duración del timer.
* vTimerCallback: función definida en la libreria "timers.h"
* pdTRUE: Este parametro afirma que se debe recargar el timer
/// @param No recibe nada
/// @return ok
*/

esp_err_t set_timer(void)
{
    ESP_LOGW(tag, "Timer init configuration");
    xTimers = xTimerCreate("Timer",                   // Solo es un nombre, no es usado por el kernel.
                           (pdMS_TO_TICKS(interval)), // Conversión de ms a Ticks.
                           pdTRUE,                    // El timer se autorecargará cuando expire. linea nueva 10082024
                           (void *)timerId,           // Asigna un unico ID para igual para indice del array
                           vTimerCallback);           // Cada timer llama la misma función callback cuando termina
    // Aquí sería muy interesante saber que valor recibe xTimers
    if (xTimers == NULL)
    {
        ESP_LOGE(tag, "The timer was not created.");
    }
    else
    {
        if (xTimerStart(xTimers, 0) != pdPASS)
        {
            ESP_LOGE(tag, "The timer could not be set into the active state");
        }
    }
    return ESP_OK;
}

esp_err_t init_led(void)
{
    gpio_reset_pin(ledRed);
    gpio_set_direction(ledRed, GPIO_MODE_OUTPUT);
    gpio_reset_pin(ledGreen);
    gpio_set_direction(ledGreen, GPIO_MODE_OUTPUT);
    gpio_reset_pin(ledBlue);
    gpio_set_direction(ledBlue, GPIO_MODE_OUTPUT);
    return ESP_OK;
}

esp_err_t set_adc(void)
{
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12);
    adc1_config_width(ADC_WIDTH_BIT_12);
    return ESP_OK;
}