#ifndef LED_H
#define LED_H

#include <Arduino.h>

#define BLINK_GPIO (gpio_num_t)CONFIG_BLINK_GPIO

#define ON 255
#define OFF 0

class LED {
public:
    LED(const gpio_num_t pin = BLINK_GPIO) : _pin(pin){

    }

    void setup(){
        gpio_reset_pin(_pin);
        /* Set the GPIO as a push/pull output */
        gpio_set_direction(_pin, GPIO_MODE_OUTPUT);

        ledcAttachPin(_pin, 0); // Assign ledpin to PWM channel
        // Initialize channels
        // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
        // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
        ledcSetup(0, 4000, 8); // 12 kHz PWM, 8-bit resolution
    }
        
//  void blink_task(void *pvParameter)
//  {
//      // Configure the IOMUX register for pad BLINK_GPIO (some pads are
//      // muxed to GPIO on reset already, but some default to other
//      // functions and need to be switched to GPIO. Consult the
//      // Technical Reference for a list of pads and their default
//      // functions.)
//      //
//      gpio_pad_select_gpio(BLINK_GPIO);
//      // Set the GPIO as a push/pull output 
//      gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
//      while(1) {
//          // Blink off (output low) 
//          gpio_set_level(BLINK_GPIO, 0);
//          vTaskDelay(1000 / portTICK_PERIOD_MS);
//          // Blink on (output high) 
//          gpio_set_level(BLINK_GPIO, 1);
//          vTaskDelay(1000 / portTICK_PERIOD_MS);
//      }
//  }

    void toggle() {
        // Toggle the LED state 
        _led_state = (_led_state ? 0 : 255);

        ledcWrite(0, _led_state); // set the brightness of the LED
    }
    
    void set(uint8_t level) {
        //gpio_set_level(_pin, level);

        ledcWrite(0, level); // set the brightness of the LED
        _led_state = level;
    }

private:
    uint8_t _led_state = 0;
    const gpio_num_t _pin;
};

extern LED led;

#endif // LED_H