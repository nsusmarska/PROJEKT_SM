#include "stm32f7xx_hal.h"

void lcd_init(void); //Ustawienie wyświetlacza

void lcd_send_cmd(char cmd); //Wysłanie komendy do wyświetlacza

void lcd_send_data(char data); //Wysłanie znaku do wyświetlenia na wyświetlacz

void lcd_send_string(char *str); //Wysłanie stringu do wyświetlenia na wyświetlacz

void lcd_put_cur(int row, int col); // Ustawienie kursora

void lcd_clear(void); // Wyczyszczenie ekranu
