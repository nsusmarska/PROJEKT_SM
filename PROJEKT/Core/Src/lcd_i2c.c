#include <lcd_i2c.h>
extern I2C_HandleTypeDef hi2c4;

#define SLAVE_ADDRESS_LCD (0x27 << 1)

void lcd_send_cmd(char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0); //nałożenie maski
	data_l = (cmd<<4);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c4, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data(char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0); //nałożenie maski
	data_l = (data<<4); //nałożenie maski
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit (&hi2c4, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_clear(void)
{
	lcd_send_cmd(0x80);
	for(int i = 0; i<70;i++)
	{
		lcd_send_data(' ');
	}
}

void lcd_put_cur(int row, int col)
{
	switch(row)
	{
		case 0:
			col |= 0x80;
			break;
		case 1:
			col |= 0xC0;
			break;
	}
	lcd_send_cmd(col);
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

void lcd_init (void)
{
	// Włączenie trybu 4-bitowego
	HAL_Delay(50);  // Odczekanie 40ms
	lcd_send_cmd (0x30); //Włączenie trybu 8-bitowego
	HAL_Delay(5);  // Odczekanie 4.1ms
	lcd_send_cmd (0x30);//Włączenie trybu 8-bitowego
	HAL_Delay(1);  // Odczekanie 100us
	lcd_send_cmd (0x30);//Włączenie trybu 8-bitowego
	HAL_Delay(10);
	lcd_send_cmd (0x20);  //init 4-bit mode
	HAL_Delay(10);

    //Inicjalizacja wyświetlacza
	lcd_send_cmd (0x28); // Ustawienie funkcji --> DL=0 (Tryb 4-bitowy), N = 1 (Włączenie 2 linii) F = 0 (5x8 znaków)
	HAL_Delay(1);
	lcd_send_cmd (0x08); //Włączenie/wyłączenie wyświetlacza --> D=0,C=0, B=0  ---> Wyświetlacz wyłączony
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // Wyczyszczenie wyświetlacza
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Ustawienie trybu wejściowego --> I/D = 1 (Kursor inkrementujący) & S = 0 (Bez przesunięcia/spacji)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Włączenie/wyłączenie wyświetlacza --> D = 1, C and B = 0. (Mruganie kursora)
}
