#include <Arduino.h>
#include "can2040/can2040.h"

CAN2040 cbus;
volatile bool trig = false;
CAN2040::Message* received = nullptr;

static void can2040_cb(CAN2040* cd, CAN2040::NotificationType notify, CAN2040::Message* msg, uint32_t errorCode) {
	// Add message processing code here...
	trig = true;
	received = msg;
}

static void PIOx_IRQHandler() {
	cbus.pioIrqHandler();
}

void canbus_setup() {
	uint32_t pio_num = 0;
	uint32_t bitrate = 500000;
	uint32_t gpio_rx = 4, gpio_tx = 5;

	// Setup canbus
	cbus.setup(pio_num);
	cbus.callbackConfig(can2040_cb);

	// Enable irqs
	irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, PIOx_IRQHandler);
	NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
	NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);

	// Start canbus
	cbus.start(bitrate, gpio_rx, gpio_tx);
}

int32_t ledState = 0;
const uint LED_PIN = PICO_DEFAULT_LED_PIN;

void setup() {
	Serial.begin(9600);
	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	canbus_setup();
}

CAN2040::Message msgToSend = {.id = 0x56, .dlc = 8, .data = {0}};
unsigned long previousMillis = 0;

void loop() {
	// Serial.print("bla\n");
	gpio_put(LED_PIN, ledState);
	if (ledState == 0) {
		ledState = 1;
	} else {
		ledState = 0;
	}
	if (trig) {
		trig = false;
		Serial.printf("received: %x\n", received->id);
		for (size_t i = 0; i < received->dlc; i++) {
			Serial.print(received->data[i], HEX); // Print as hex
			Serial.print(" "); // Add a space between each byte
		}
		Serial.println();
	}

	unsigned long currentMillis = millis();
	if (currentMillis - previousMillis >= 3000) {
		previousMillis = currentMillis;
		for (int i = 0; i < 8; i++) {
			msgToSend.data[i] = random(100, 140);
		}
		if (cbus.checkTransmit()) {
			auto r = cbus.transmit(&msgToSend);
			Serial.print("sent ");
			Serial.println(r);
		} else {
			Serial.println("sent fail");
		}

	}

	// Serial.println(cbus.stats.rx_total);
	// Serial.println(cbus.stats.tx_total);
	// Serial.println(cbus.stats.tx_attempt);
	// Serial.println(cbus.stats.parse_error);
	sleep_ms(500);
}
