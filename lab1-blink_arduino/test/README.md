# Lab 1: Filip Voch

### Morse code

1. Listing of C code which repeats one "dot" and one "comma" (BTW, in Morse code it is letter `A`) on a LED. Always use syntax highlighting, meaningful comments, and follow C guidelines:

```c
int main(void)
{
    // Set pin where on-board LED is connected as output
    pinMode(LED_GREEN, OUTPUT);

    // Infinite loop
    while (1)
    {
        led_value = LOW;

        _delay_ms(SHORT_DELAY);
        digitalWrite(LED_GREEN, led_value);

        led_value = HIGH;

        _delay_ms(SHORT_DELAY);
        digitalWrite(LED_GREEN, led_value);

        led_value = LOW;

        _delay_ms(LONG_DELAY);
        digitalWrite(LED_GREEN, led_value);

        led_value = HIGH;

        _delay_ms(SHORT_DELAY);
        digitalWrite(LED_GREEN, led_value);
    }

    // Will never reach this
    return 0;
}
```

2. Scheme of Morse code application, i.e. connection of AVR device, LED, resistor, and supply voltage. The image can be drawn on a computer or by hand. Always name all components and their values!

   ![your figure]()
