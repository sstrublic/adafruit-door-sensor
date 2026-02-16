// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RH_RF95.h>
#include "door_sensor.h"

/* Set to 1 to enable serial debug. */
#define DEBUG 1

/* Pins for state selection (Active low) */
#define PIN_DOOR1_CLOSED            14    // Door 1 closed - RA0
#define PIN_DOOR1_OPEN              15    // Door 1 open - RA1
#define PIN_DOOR2_CLOSED            16    // Door 2 closed - RA2
#define PIN_DOOR2_OPEN              17    // Door 3 open - RA3

/* Time in milliseconds to poll the sensors. */
#define STATUS_POLL_MILLISECONDS    200

/**
 * @brief Singleton instance of the radio driver.
 */
RH_RF95 rf95(RFM95_CS, RFM95_INT);

/**
 * @brief  Door control data.
 */
static door_control_t door_control;

#if DEBUG
/**
 * @brief Custom printf function.
 */
static void s_printf(const char *format, ...)
{
    char buffer[256];  // or smaller or static &c.
    va_list args;

    va_start(args, format);
    vsprintf(buffer, format, args);
    va_end(args);

    Serial.print(buffer);
}
#else
#define s_printf(format, args)
#endif

/**
 * @brief Print bytes to serial console.
 */
static void print_bytes(uint8_t *buf, uint8_t len)
{
#if DEBUG    
    for (int i = 0; i < len; i++)
    {
        s_printf(" %02x", buf[i]);
    }
#endif    
}

/**
 * @brief Send a door status message.
 */
static void send_door_status(uint8_t door1, uint8_t door2)
{
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    door_state_message_t *msg = (door_state_message_t *)buf;

#if DEBUG
    s_printf("Door status:");
    if (door1 & DOOR_STATE_OPEN) s_printf(" Door #1 is Open.");
    if (door1 & DOOR_STATE_CLOSED) s_printf(" Door #1 is Closed.");
    if (door2 & DOOR_STATE_OPEN) s_printf(" Door #2 is Open.");
    if (door2 & DOOR_STATE_CLOSED) s_printf(" Door #2 is Closed.");
    s_printf("\r\n");

#endif
    /* Update state. */
    door_control.door1.state = door1;
    door_control.door2.state = door2;

    /* Send the status update. */
    msg->signature = DOOR_MESSAGE_SIGNATURE;
    msg->type = DOOR_MESSAGE_TYPE_STATUS;
    msg->status.door1 = door1;
    msg->status.door2 = door2;

    door_control.messages_sent++;

    s_printf("(S: %ld) Sending status...  ", door_control.messages_sent);
    print_bytes(buf, 8);
    s_printf("\r\n");

    rf95.send((uint8_t *)msg, 8);
    delay(10);
    rf95.waitPacketSent();

    /* Indicate the status has not been acknowledged. */
    door_control.acked = false;
}

/** 
 * @brief Initialization function.
 */
void setup()
{
    /* Pin settings. */
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    memset(&door_control, 0, sizeof(door_control));

    /* Start by assuming acknowledgement so an initial state can be read. */
    door_control.acked = true;

    /* Set initial door control config and state. */
    door_control.door1 = {  .pin_open = PIN_DOOR1_OPEN,
                            .pin_closed = PIN_DOOR1_CLOSED,
                            .state = DOOR_STATE_UNKNOWN
                         };

    door_control.door2 = {  .pin_open = PIN_DOOR2_OPEN,
                            .pin_closed = PIN_DOOR2_CLOSED,
                            .state = DOOR_STATE_UNKNOWN
                         };

    /* Set sensor pins as inputs, with pullup resistors enabled. */
    pinMode(door_control.door1.pin_closed, INPUT_PULLUP);
    pinMode(door_control.door1.pin_open, INPUT_PULLUP);
    pinMode(door_control.door2.pin_closed, INPUT_PULLUP);
    pinMode(door_control.door2.pin_open, INPUT_PULLUP);


#if DEBUG
    /* Set up serial monitor output. */
    Serial.begin(115200);
    while (!Serial) delay(1);
    delay(100);

    s_printf("\r\nDoor Sensor\r\n");
#endif

    /* Toggle reset on the radio. */
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    /* Initialize the radio. */
    if (false == rf95.init())
    {
        s_printf("LoRa radio init failed\r\n");
        s_printf("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info\r\n");
        while (1);
    }

    s_printf("LoRa radio init OK!\r\n");

    /* Set frequency to 915 MHz, modulation GFSK_Rb250Fd250, +13dbM */
    if (!rf95.setFrequency(RF95_FREQ))
    {
        s_printf("setFrequency failed\r\n");
        while (1);
    }

    s_printf("Set radio frequency to: %3.2f MHz\r\n", RF95_FREQ);

    /*
     * The default transmitter power is 13dBm, using PA_BOOST.
     * If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
     * you can set transmitter powers from 5 to 23 dBm:
     */
    rf95.setTxPower(23, false);

    /* Set next poll time to 'now' to kick off reading the sensors. */
    door_control.next_poll = millis();
}

void loop()
{
    /* 50 milliseconds per iteration is fine. */
    delay(50);

    /* Get the tick time. */
    uint32_t now = millis();

    /* If it's time to check the sensor, do so. */
    if (now > door_control.next_poll)
    {
        // Read the pins.
        uint8_t door1 = digitalRead(door_control.door1.pin_open) | (digitalRead(door_control.door1.pin_closed) << 4);
        uint8_t door2 = digitalRead(door_control.door2.pin_open) | (digitalRead(door_control.door2.pin_closed) << 4);

        /* If the status has changed, or was not acknowledged, send an update. */
        if ((door1 != door_control.door1.state) || (door2 != door_control.door2.state) ||
            (false == door_control.acked))
        {
#if DEBUG            
            if (true == door_control.acked)
            {
                s_printf("Door status changed!\r\n");
            }
#endif
            /* Send status to the listener. */
            send_door_status(door1, door2);
        }

        door_control.door1.state = door1;
        door_control.door2.state = door2;

        /* When to check again. */
        door_control.next_poll = now + STATUS_POLL_MILLISECONDS;
    }

    /* See if a message was received. */
    if (true == rf95.available())
    {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        // Upon receipt of a message, process it and send an ackknowledgement.
        if (rf95.recv(buf, &len))
        {
            door_state_message_t *msg = (door_state_message_t *)buf;

            door_control.messages_received++;

            s_printf("(R: %ld) Received message...", door_control.messages_received);
            print_bytes((uint8_t *)msg, 6);
            s_printf("\r\n");

            /* Got something! */
            digitalWrite(LED_BUILTIN, HIGH);

            if (msg->signature == DOOR_MESSAGE_SIGNATURE)
            {
                switch (msg->type)
                {
                    /* Request from listener */
                    case DOOR_MESSAGE_TYPE_REQUEST:
                    {
                        s_printf("Received poll request...\r\n");

                        /* Setting the ack to false allows the next loop to send the response. */
                        door_control.acked = false;

                        /* Since a request was processed, set when to check again. */
                        door_control.next_poll = now + STATUS_POLL_MILLISECONDS;
                    }
                    break;

                    case DOOR_MESSAGE_TYPE_ACK:
                    {
                        s_printf("Status was acknowledged.\r\n");
                        door_control.acked = true;
                    }
                    break;

                    default:
                    {
                        s_printf("Received unknown message type!\r\n");
                    }
                    break;
                }
            }
            else
            {
                s_printf("Received invalid message:");
                print_bytes(buf, len);
                s_printf("\r\n");
            }

            digitalWrite(LED_BUILTIN, LOW);
        }
    }
}
