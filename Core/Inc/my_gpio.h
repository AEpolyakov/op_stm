void buffer_direction_out();
void buffer_direction_in();
void write_in();
void write_out();


void write_address(uint16_t address);
void write_data(uint16_t data);

uint16_t gpio_data_read();
void make_ti();

void change_gpio_direction(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin, uint8_t Mode);
