#include <SPI.h>
#include <RH_RF95.h>

#include "door_listener.h"

/* Set to 1 to enable serial debug. */
#define DEBUG 1

/* Sets the transmit power. */
#define TX_POWER_SETTING            13

/* Pins for LED selection (Active low) */
#define PIN_DOOR1_CLOSED            14    // Door 1 closed - RA0
#define PIN_DOOR1_OPEN              15    // Door 1 open - RA1
#define PIN_DOOR2_CLOSED            16    // Door 2 closed - RA2
#define PIN_DOOR2_OPEN              17    // Door 3 open - RA3

/* Time in milliseconds between subsequent polls of the sensor. */
#define POLL_TIME_MILLISECONDS      6000

/* Time in milliseconds to re-poll if a timeout occurred. */
#define POLL_TIMEOUT_MILLISECONDS   1000

/* Number of polls before indicating a failure status. */
#define POLL_RETRY_COUNT            5

/**
 * @brief Singleton instance of the radio driver.
 */
static RH_RF95 rf95(RFM95_CS, RFM95_INT);

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
 * @brief  Door control data.
 */
static door_control_t door_control;

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

static void send_message(uint16_t type)
{
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    door_state_message_t *msg = (door_state_message_t *)buf;
    
    door_control.messages_sent++;

    msg->signature = DOOR_MESSAGE_SIGNATURE;
    msg->type = type;

    switch (msg->type)
    {
        case DOOR_MESSAGE_TYPE_REQUEST:
        {
            /* Increment number of attempts. */
            door_control.retries++;
            s_printf("(S: %ld) Polling sensor (attempt #%d)...\r\n", door_control.messages_sent, door_control.retries);
        }
        break;

        case DOOR_MESSAGE_TYPE_ACK:
        {
            /* Reset retry count. */
            door_control.retries = 0;
            s_printf("(S: %ld) Sending acknowledgement...\r\n", door_control.messages_sent);
        }
        break;

        default:
        break;
    }

    /* Send the message. */
    rf95.send((uint8_t *)msg, 6);
    delay(10);
    rf95.waitPacketSent();
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

    /* Start by assuming acknowledgement so an initial poll can be issued. */
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

    /* Set LED pins as outputs. */
    pinMode(door_control.door1.pin_open, OUTPUT);
    pinMode(door_control.door1.pin_closed, OUTPUT);
    pinMode(door_control.door2.pin_open, OUTPUT);
    pinMode(door_control.door2.pin_closed, OUTPUT);

    /* Default to off (no ground). */
    digitalWrite(door_control.door1.pin_open, HIGH);
    digitalWrite(door_control.door1.pin_closed, HIGH);
    digitalWrite(door_control.door2.pin_open, HIGH);
    digitalWrite(door_control.door2.pin_closed, HIGH);

#if DEBUG
    /* Set up serial monitor output. */
    Serial.begin(115200);
    while (!Serial) delay(1);
    delay(100);

    s_printf("\r\nDoor Listener\r\n");
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
    rf95.setTxPower(TX_POWER_SETTING, false);

    /* Set next poll time to 'now' to kick off requesting state from the sensor. */
    door_control.next_poll = millis();
}

void loop()
{
    /* 50 milliseconds per iteration is fine. */
    delay(50);

    /* Get the tick time. */
    uint32_t now = millis();

    if (true == rf95.available())
    {
        /* If a message was received, process it. */
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len))
        {
            door_state_message_t *msg = (door_state_message_t *)buf;

            door_control.messages_received++;

            s_printf("(R: %ld) Received message...", door_control.messages_received);
            print_bytes((uint8_t *)msg, 8);
            s_printf("\r\n");

            /* Got something! */
            digitalWrite(LED_BUILTIN, HIGH);

            if (msg->signature == DOOR_MESSAGE_SIGNATURE)
            {
                switch (msg->type)
                {
                    case DOOR_MESSAGE_TYPE_STATUS:
                    {
                        s_printf("Received door status...");

                        /* Extract the state as read from the sensor. */
                        door_control.door1.state = msg->status.door1;
                        door_control.door2.state = msg->status.door2;

                        if (door_control.door1.state & DOOR_STATE_OPEN)
                        {
                            s_printf(" Door #1 is Open.");
                            digitalWrite(door_control.door1.pin_open, LOW);
                        }
                        else
                        {
                            digitalWrite(door_control.door1.pin_open, HIGH);
                        }
                        
                        if (door_control.door1.state & DOOR_STATE_CLOSED)
                        {
                            s_printf(" Door #1 is Closed.");
                            digitalWrite(door_control.door1.pin_closed, LOW);
                        }
                        else
                        {
                            digitalWrite(door_control.door1.pin_closed, HIGH);
                        }

                        if (door_control.door2.state & DOOR_STATE_OPEN)
                        {
                            s_printf(" Door #2 is Open.");
                            digitalWrite(door_control.door2.pin_open, LOW);
                        }
                        else
                        {
                            digitalWrite(door_control.door2.pin_open, HIGH);
                        }
                        
                        if (door_control.door2.state & DOOR_STATE_CLOSED)
                        {
                            s_printf(" Door #2 is Closed.");
                            digitalWrite(door_control.door2.pin_closed, LOW);
                        }
                        else
                        {
                            digitalWrite(door_control.door2.pin_closed, HIGH);
                        }

                        s_printf("\r\n");

                        /* Send an acknowledgement. */
                        send_message(DOOR_MESSAGE_TYPE_ACK);
                        door_control.acked = true;

                        /* Set the next polling time (slow poll). */
                        door_control.next_poll = now + POLL_TIME_MILLISECONDS;
                    }
                    break;

                    default:
                    {
                        s_printf("Received unknown message type!\r\n");

                        /* Assume acked to return to a slow poll time. */
                        door_control.next_poll = now + POLL_TIME_MILLISECONDS;
                        door_control.acked = true;
                    }
                    break;
                }
            }
            else
            {
                s_printf("Received invalid message:");
                print_bytes(buf, len);
                s_printf("\r\n");

                /* Assume acked to return to a slow poll time. */
                door_control.next_poll = now + POLL_TIME_MILLISECONDS;
                door_control.acked = true;
            }

            digitalWrite(LED_BUILTIN, LOW);
        }
    }

    /* If the poll time has elapsed, send a request. */
    if (now > door_control.next_poll)
    {
        /* On the initial poll, the ack has not occurred, so clear the flag. */
        if (true == door_control.acked)
        {
            door_control.acked = false;
        }
        else
        {
            /* If the retry count is exceeded, blink the red LEDs. */
            if (door_control.retries >= POLL_RETRY_COUNT)
            {
                s_printf("*** Door sensor is not responding!!! ***\r\n");

                digitalWrite(door_control.door1.pin_open, HIGH);
                digitalWrite(door_control.door2.pin_open, HIGH);

                if (door_control.retries & 1)
                {
                    digitalWrite(door_control.door1.pin_closed, HIGH);
                    digitalWrite(door_control.door2.pin_closed, HIGH);
                }
                else
                {
                    digitalWrite(door_control.door1.pin_closed, LOW);
                    digitalWrite(door_control.door2.pin_closed, LOW);
                }
            }
        }

        /* Fast poll to send request again until received from sensor. */
        send_message(DOOR_MESSAGE_TYPE_REQUEST);
        door_control.next_poll = now + POLL_TIMEOUT_MILLISECONDS;
    }
}
