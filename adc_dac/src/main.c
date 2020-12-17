#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/dac.h>
#include <drivers/adc.h>
#include <stdio.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <sys/__assert.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <drivers/uart.h>
#include <console/console.h>
#include <console/tty.h>
#include <string.h>
#include <stdarg.h>

//DAC
#if defined(CONFIG_BOARD_NUCLEO_F767ZI)
#define DAC_DEVICE_NAME		DT_LABEL(DT_NODELABEL(dac1))
#define DAC_CHANNEL_ID		1
#define DAC_RESOLUTION		12
#else
#error "Unsupported board (DAC)"
#endif

static const struct dac_channel_cfg dac_ch_cfg = {
	.channel_id  = DAC_CHANNEL_ID,
	.resolution  = DAC_RESOLUTION
};

int iDACValue = 0;

//LED
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif

bool led_is_on = true;
int retb;

//UART
/* 1000 msec = 1 sec */
#define UART_DEVICE_NAME CONFIG_UART_CONSOLE_ON_DEV_NAME
//#define UART_DEVICE_NAME "UART_1"

const struct device *uart_dev;
static volatile bool data_transmitted;
static int char_sent;
static const char fifo_data[] = "Prueba Tx.\r\n";
static int tx_data_idx;
static volatile bool data_transmitted;
const struct device *devb;

#define DATA_SIZE	(sizeof(fifo_data) - 1)

//HILOS
#define STACK_SIZE (768 + CONFIG_TEST_EXTRA_STACKSIZE)
//static K_THREAD_STACK_ARRAY_DEFINE(stacks, 2, STACK_SIZE);
//static struct k_thread threads[2];
static K_THREAD_STACK_ARRAY_DEFINE(stacks, 4, STACK_SIZE);
static struct k_thread threads[4];
K_MUTEX_DEFINE( cliblock );

#define SLEEP_TIME  100
#define SLEEP_TIME_MS  10

//ADC
#if defined(CONFIG_BOARD_NUCLEO_F767ZI)
#define ADC_DEVICE_NAME         DT_LABEL(DT_INST(0, st_stm32_adc))
#define ADC_RESOLUTION		12
#define ADC_GAIN		ADC_GAIN_1
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME_DEFAULT
#define ADC_1ST_CHANNEL_ID	0
#else
#error "Unsupported board (ADC)"
#endif

#define BUFFER_SIZE  6
//static ZTEST_BMEM int16_t m_sample_buffer[BUFFER_SIZE];
static int16_t m_sample_buffer[BUFFER_SIZE];

int iADCValue;

//ADC
const struct device *get_adc_device(void)
{
	return device_get_binding(ADC_DEVICE_NAME);
}

//UART
void uart_init()
{
struct uart_config uart_cfg;
	int ret;
	uart_dev = device_get_binding(UART_DEVICE_NAME);
	ret = uart_config_get(uart_dev, &uart_cfg);
	if (!ret) {
		printk("\n======== [%s] ========\n", UART_DEVICE_NAME);
		printk("[%s] uart_config.baudrate=%d\n", UART_DEVICE_NAME, uart_cfg.baudrate);
		printk("[%s] uart_config.parity=%d\n", UART_DEVICE_NAME, uart_cfg.parity);
		printk("[%s] uart_config.stop_bits=%d\n", UART_DEVICE_NAME, uart_cfg.stop_bits);
		printk("[%s] uart_config.data_bits=%d\n", UART_DEVICE_NAME, uart_cfg.data_bits);
		printk("[%s] uart_config.flow_ctrl=%d\n\n\n", UART_DEVICE_NAME, uart_cfg.flow_ctrl);
	}
}

static void uart_fifo_callback(const struct device *uart_dev, void *user_data)
{
	//printk("Callback\n");
	ARG_UNUSED(user_data);
	if (!uart_irq_update(uart_dev)) {
		printk("retval should always be 1\n");
		return;
	}
	uart_irq_tx_enable(uart_dev);
	if (uart_irq_tx_ready(uart_dev) && tx_data_idx < DATA_SIZE) {

		if (uart_fifo_fill(uart_dev,
				   (uint8_t *)&fifo_data[tx_data_idx++], 1) > 0) {
			data_transmitted = true;
			char_sent++;
		}

		if (tx_data_idx == DATA_SIZE) {
			uart_irq_tx_disable(uart_dev);
		}
	}
}

//HILOS
static int32_t get_random_delay(int id, int period_in_ms){
	/*
	 * The random delay is unit-less, and is based on the philosopher's ID
	 * and the current uptime to create some pseudo-randomness. It produces
	 * a value between 0 and 31.
	 */
	int32_t delay = (k_uptime_get_32()/100 * (id + 1)) & 0x1f;

	/* add 1 to not generate a delay of 0 */
	int32_t ms = (delay + 1) * period_in_ms;

	return ms;
}

void philosopher(void *id, void *unused1, void *unused2)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	int my_id = POINTER_TO_INT(id);

	printk("Beginning execution; thread data is %d\n", my_id);

	k_msleep(200);

	while (1) {
		int32_t delay;

		//delay = 200 + get_random_delay(my_id, 25);
		delay = 25 + get_random_delay(my_id, 25);

		//k_mutex_lock(&cliblock, K_NO_WAIT);

		k_mutex_lock(&cliblock, K_FOREVER);

		//printk("Thread %d: and now a delay of %d clock ticks\n", my_id, delay);

		if(my_id == 1){
			uart_unal();
		}
		else if(my_id == 2) {
			blink_unal();
		}
		else if(my_id == 3) {
			adc_read_unal();
		}
		else{
			dac_unal();
		}

		k_mutex_unlock(&cliblock);

		k_msleep(delay);
	}
}

void uart_unal(void)
{
	/* Verify uart_irq_callback_set() */
	//printk("UART: \n");
	//Imprimir valor DAC
	float fVoltajeDAC = iDACValue * 3.3 / 4096;
	printk("DAC: %d ", iDACValue);
	printf("Voltaje: %.2lf V\n", fVoltajeDAC);

	float fVoltajeADC = iADCValue * 3.3 / 4096;
	printk("ADC: %d ", iADCValue);
	printf("Voltaje: %.2lf V\n", fVoltajeADC);

	char_sent = 0;
	uart_irq_callback_set(uart_dev, uart_fifo_callback);
	//Enable Tx/Rx interrupt before using fifo
	//Verify uart_irq_tx_enable()
	uart_irq_tx_enable(uart_dev);
	k_msleep(SLEEP_TIME_MS);
	tx_data_idx = 0;
	//Verify uart_irq_tx_disable()
	uart_irq_tx_disable(uart_dev);
}

void blink_unal(void)
{
	printk("BLINK\n");
	gpio_pin_set(devb, PIN, (int)led_is_on);
	led_is_on = !led_is_on;
	k_msleep(SLEEP_TIME_MS);
}

void adc_read_unal(void)
{
	printk("ADC\n");

	const struct adc_sequence sequence = {
		.channels    = BIT(ADC_1ST_CHANNEL_ID),
		.buffer      = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};

	const struct device *adc_dev = device_get_binding(ADC_DEVICE_NAME);
	//const struct device *adc_dev = device_get_binding("ADC_1");
	//printk("\n======== [%s] ========\n", ADC_DEVICE_NAME);

	if (!adc_dev) {
		printk("Cannot get ADC device\n");
		return;
	}

	//iADCValue = adc_read(adc_dev, &sequence);
	adc_read(adc_dev, &sequence);

	iADCValue = m_sample_buffer[0];

	k_msleep(SLEEP_TIME_MS);
}

void dac_unal(void)
{
	printk("DAC\n");

	const struct device *dac_dev = device_get_binding(DAC_DEVICE_NAME);

	if (!dac_dev) {
		printk("Cannot get DAC device\n");
		return;
	}

	int ret = dac_channel_setup(dac_dev, &dac_ch_cfg);

	if (ret != 0) {
		printk("Setting up of DAC channel failed with code %d\n", ret);
		return;
	}

	//DAC
	dac_write_value(dac_dev, DAC_CHANNEL_ID, iDACValue);

	//Reiniciar contador
	iDACValue++;
	if(iDACValue == 4096){
		iDACValue = 0;
	}

	k_msleep(SLEEP_TIME_MS);
}

static void start_threads(void)
{
	//Crear hilo 0
	k_thread_create(&threads[0], &stacks[0][0], STACK_SIZE,
		philosopher, INT_TO_POINTER(0), NULL, NULL,
		0, K_USER, K_NO_WAIT);

	//k_object_access_grant(&cliblock, &threads[0]);
	//k_object_access_grant(&cliblock, &threads[1]);

	k_thread_start(&threads[0]);

	//Crear hilo 1
	k_thread_create(&threads[1], &stacks[1][0], STACK_SIZE,
		philosopher, INT_TO_POINTER(1), NULL, NULL,
		1, K_USER, K_NO_WAIT);

	k_thread_start(&threads[1]);

	//Crear hilo 2
	k_thread_create(&threads[2], &stacks[2][0], STACK_SIZE,
		philosopher, INT_TO_POINTER(2), NULL, NULL,
		2, K_USER, K_NO_WAIT);

	k_thread_start(&threads[2]);

	//Crear hilo 3
	k_thread_create(&threads[3], &stacks[3][0], STACK_SIZE,
		philosopher, INT_TO_POINTER(3), NULL, NULL,
		3, K_USER, K_NO_WAIT);

	k_thread_start(&threads[3]);
}

void main(void)
{
	printk("main()\n");

	//DAC
	printk("Generating sawtooth signal at DAC channel %d.\n",
		DAC_CHANNEL_ID);

	/* Number of valid DAC values, e.g. 4096 for 12-bit DAC */
	const int dac_values = 1U << DAC_RESOLUTION;

	printk("DAC Values: %d\n",  dac_values);

	//LED
	devb = device_get_binding(LED0);
	if (devb == NULL) {
		return;
	}

	retb = gpio_pin_configure(devb, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (retb < 0) {
		return;
	}

	//UART
	uart_init();

	//HILOS
	//display_demo_description();
	#if CONFIG_TIMESLICING
		k_sched_time_slice_set(5000, 0);
	#endif

		k_mutex_init(&cliblock);	//Inicializar Mutex
		start_threads();

	#ifdef CONFIG_COVERAGE
		/* Wait a few seconds before main() exit, giving the sample the
		 * opportunity to dump some output before coverage data gets emitted
		 */
		k_sleep(K_MSEC(5000));
	#endif
}
