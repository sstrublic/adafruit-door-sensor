#ifndef __DOOR_SENSOR_H__
#define __DOOR_SENSOR_H__

#include <stdint.h>
#include <stdarg.h>

/* Pin definitions. */
#define RFM95_CS            8
#define RFM95_INT           3
#define RFM95_RST           4

/* Change to 434.0 or other frequency, must match RX's freq! */
#define RF95_FREQ           915.0

/* Door states. */
#define DOOR_STATE_UNKNOWN  0x00
#define DOOR_STATE_OPEN     0x01
#define DOOR_STATE_CLOSED   0x10

/* Each message must start with this signature or it is not valid. */
#define DOOR_MESSAGE_SIGNATURE      0xaa55cc33

/**
 * @brief Door message type codes.
 */
typedef enum DOOR_MESSAGE_TYPE
{
    DOOR_MESSAGE_TYPE_STATUS = 1,
    DOOR_MESSAGE_TYPE_REQUEST = 2,
    DOOR_MESSAGE_TYPE_ACK = 3,

} DOOR_MESSAGE_TYPE;

/**
 * @brief  Door information.
 */
typedef struct door_config
{
    uint8_t pin_open;
    uint8_t pin_closed;
    uint8_t state;    

} door_config_t;

/**
 * @brief Control data.
  */
typedef struct door_control
{
    uint32_t        messages_sent;      // Number of messages sent
    uint32_t        messages_received;  // Number of messages received
    uint32_t        next_poll;          // Next poll time in system ticks

    door_config_t   door1;      // Config for door 1
    door_config_t   door2;      // Config for door 2

    bool            acked;      // Last message was acknowledged

} door_control_t;

/**
 * @brief Door state message format.
 */
typedef struct door_state_message
{
    uint32_t    signature;      // Defines the header signature identifying this as a real message
    uint16_t    type;           // Defines the message type

    union
    {
        /* Status update, from sensor */
        struct
        {
            uint8_t     door1;  // Door 1 state
            uint8_t     door2;  // Door 2 state
        } status;

        /* Other messages as needed */
    };

} door_state_message_t;

#endif // __DOOR_SENSOR_H__
