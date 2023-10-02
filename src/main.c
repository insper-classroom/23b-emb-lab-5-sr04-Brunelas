#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1u << BUT_PIO_PIN)

/* Trigger */
#define TRI_PIO     PIOC
#define TRI_PIO_ID  ID_PIOC
#define TRI_PIO_PIN 19
#define TRI_PIO_PIN_MASK (1 << ECO_PIO_PIN)

/* Echo */
#define ECO_PIO     PIOA
#define ECO_PIO_ID  ID_PIOA
#define ECO_PIO_PIN 3
#define ECO_PIO_PIN_MASK (1u << ECO_PIO_PIN)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_SENSOR_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_SENSOR_STACK_PRIORITY            (tskIDLE_PRIORITY)


extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);


SemaphoreHandle_t xSemaphoreBUT1;



/************************************************************************/	


QueueHandle_t xQueueDados;
QueueHandle_t xQueueDistancia;



/** prototypes declarando as funcoes que vou usar */
void eco_callback();
void but1_callback(void);
static void BUT_init();
void TRI_init();
void ECO_init();
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/


void eco_callback(void) {
	if (pio_get(ECO_PIO,ECO_PIO_ID,ECO_PIO_PIN_MASK)) {
	printf("3");
	
		RTT_init(32000,0,0);
		} else {
	int tempo = rtt_read_timer_value(RTT); // tempo em ms
	//printf("4");
		xQueueSend(xQueueDados, &tempo, 0);
	}
}

void but1_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreBUT1, &xHigherPriorityTaskWoken);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	
	double distancia;

	char display[128];

	for (;;)  {
		xQueueReceive(xQueueDistancia,&distancia,0);
		sprintf(display,"%f",distancia);
		gfx_mono_draw_string(display,0,20,&sysfont);

	}
}



void task_but () {
	
	
	double distancia;
	int tempo;

	for (;;)  {
		/* code */
		if (xSemaphoreTake(xSemaphoreBUT1, 1000)) {
			pio_set(TRI_PIO,TRI_PIO_PIN_MASK);
	    	delay_us(10);
			pio_clear(TRI_PIO,TRI_PIO_PIN_MASK);
			
			
			if (xQueueReceive(xQueueDados, &tempo, portMAX_DELAY)) {
				// para calcular a distacia eu faco delta t * velocidade do som/2 pq vai e volta
				double distancia = (tempo*320000)*340/2;
				printf("distancia: %f\n", distancia);
				xQueueSend(xQueueDistancia,&distancia,0);

			}
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}


static void BUT_init(void) {
	/* conf bot�o como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but1_callback);
	
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

}

 void TRI_init(void) {
	
	pmc_enable_periph_clk(TRI_PIO_ID);
	pio_configure(TRI_PIO, PIO_OUTPUT_0, TRI_PIO_PIN_MASK, PIO_DEFAULT);

}

 void ECO_init(void) {
	pio_configure(ECO_PIO, PIO_INPUT, ECO_PIO_PIN_MASK, PIO_DEFAULT);
	pio_enable_interrupt(ECO_PIO, ECO_PIO_PIN_MASK);
	pio_handler_set(ECO_PIO, ECO_PIO_ID, ECO_PIO_PIN_MASK, PIO_IT_EDGE , eco_callback);
	NVIC_EnableIRQ(ECO_PIO_ID);
	NVIC_SetPriority(ECO_PIO_ID, 4);
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}


/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	/* criando uma fila para mandar os dados de distancia do sensor*/
	xQueueDados = xQueueCreate(32, sizeof(int));
	xQueueDistancia = xQueueCreate(32, sizeof(double));
	xSemaphoreBUT1 = xSemaphoreCreateBinary();

	if (xSemaphoreBUT1 == NULL) {
		printf("Nao foi possivel criar o semaforo\n");
	}

	if (xQueueDados == NULL) {
		printf("Nao foi possivel criar a fila de dados\n");
	}

	if (xQueueDistancia == NULL) {
		printf("Nao foi possivel criar a fila de distancia\n");
	}
	/* Create task to control oled */
	

	/* Create task to control sensor */
	if (xTaskCreate(task_but, "but", TASK_SENSOR_STACK_SIZE, NULL, TASK_SENSOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create sensor task\r\n");
	}

	
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}

	/* Start the scheduler. */
	BUT_init();
	TRI_init();
	ECO_init();
	vTaskStartScheduler();

	/* RTOS n�o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
