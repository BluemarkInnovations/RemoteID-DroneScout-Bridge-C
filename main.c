//
// application to receive and display the MAVLink message of DroneScout Bridge receiver dongles.
//
// (c) Bluemark Innovations BV
// MIT license
//
// Change the UART_PORT according to your serial UART interface
//

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <endian.h>
#include <sys/time.h>
#include <math.h>
#include <pthread.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

#include <common/mavlink.h>
#include <opendroneid.h>
#include <mav2odid.h>

#define MAVLINK_SYSTEM_ID       3 // used to transmit heartbeat messages to the DroneScout bridge
#define MAVLINK_COMPONENT_ID    1 // used to transmit heartbeat messages to the DroneScout bridge

#define UART_PORT 				"/dev/ttyACM0" //change to the serial interface that is connected to DroneScout Bridge
#define UART_BAUDRATE			B115200

pthread_mutex_t  serial_port_lock;
int uart0_FS_RDWR = -1;

static uint64_t current_timestamp_ms()
{   //return current timestamp in milliseconds
    struct timeval te;
    gettimeofday(&te, NULL); // get current time
    uint64_t milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds

    return milliseconds;
}

void printADSBvehicle_data(mavlink_adsb_vehicle_t* adsb_vehicle)
{

    /*
     typedef struct __mavlink_adsb_vehicle_t {
    uint32_t ICAO_address; //<  ICAO address
    int32_t lat; //< [degE7] Latitude
    int32_t lon; //< [degE7] Longitude
    int32_t altitude; //< [mm] Altitude(ASL)
    uint16_t heading; //< [cdeg] Course over ground
    uint16_t hor_velocity; //< [cm/s] The horizontal velocity
    int16_t ver_velocity; //< [cm/s] The vertical velocity. Positive is up
    uint16_t flags; //<  Bitmap to indicate various statuses including valid data fields
    uint16_t squawk; //<  Squawk code
    uint8_t altitude_type; //<  ADSB altitude type.
    char callsign[9]; //<  The callsign, 8+null
    uint8_t emitter_type; //<  ADSB emitter type.
    uint8_t tslc; //< [s] Time since last communication in seconds
    } mavlink_adsb_vehicle_t;
    */

    const char ADSBvehicle_data_format[] =
        "ICAO addres: %6.6X\ncallsign: %s\nsquawk: %i\nflags: %X\nemitter type: %i\ntslc %d\nLat/Lon: %i, %i\nvelocity H/V: %i, %i\nheading: %i\naltitude: %i\naltitude type: %i\n";
    printf(ADSBvehicle_data_format, adsb_vehicle->ICAO_address,adsb_vehicle->callsign,adsb_vehicle->squawk,adsb_vehicle->flags,
    adsb_vehicle->emitter_type,adsb_vehicle->tslc, adsb_vehicle->lat, adsb_vehicle->lon,adsb_vehicle->hor_velocity,adsb_vehicle->ver_velocity,adsb_vehicle->heading,
    adsb_vehicle->altitude,adsb_vehicle->altitude_type);
}

//thread to parse received MAVLink messages and send every second/1 Hz heartbeat messages.
void *serial_port_receive(void *ptr)
{
	uint8_t rx_buffer[MAVLINK_MAX_PACKET_LEN];
	uint64_t time_last_heartbeat_ms = 0;

	while(1)
	{
		union {
			mavlink_heartbeat_t heartbeat;
            mavlink_open_drone_id_arm_status_t arm_status;
            mavlink_open_drone_id_message_pack_t message_pack;
            mavlink_adsb_vehicle_t vehicle;
		} msg_rx;

		usleep(25000);

		memset(rx_buffer,0,MAVLINK_MAX_PACKET_LEN);
		pthread_mutex_lock(&serial_port_lock);
		int rx_length = read(uart0_FS_RDWR, (void*)rx_buffer, MAVLINK_MAX_PACKET_LEN);
		pthread_mutex_unlock(&serial_port_lock);

		if (rx_length > 0)
		{
			rx_buffer[rx_length] = '\0';
			mavlink_message_t message;
			mavlink_status_t status;
			int i = 0;
			while (i < rx_length)
			{
				mavlink_message_t message;
				mavlink_status_t status;

				if (mavlink_parse_char(MAVLINK_COMM_0, rx_buffer[i++], &message, &status))
				{
                    time_t t;   // not a primitive datatype
                    time(&t);
                    struct tm *currentTime = localtime(&t);
                    char time_str[64];
                    sprintf(time_str,"%02d:%02d:%02d", currentTime->tm_hour, currentTime->tm_min, currentTime->tm_sec);
                    printf("%s received MAVLink message: System ID %i Component ID %i Message ID %i Sequence %i\n",time_str,message.sysid,message.compid, message.msgid, message.seq);

					switch ((int) message.msgid)
					{
						case MAVLINK_MSG_ID_HEARTBEAT:
							mavlink_msg_heartbeat_decode(&message, &msg_rx.heartbeat);
							printf("heartbeat ver 0x%X status 0x%X autopilot 0x%X type 0x%X\n",msg_rx.heartbeat.mavlink_version, msg_rx.heartbeat.system_status, msg_rx.heartbeat.autopilot, msg_rx.heartbeat.type);
							fflush(stdout);
							break;
                        case MAVLINK_MSG_ID_OPEN_DRONE_ID_MESSAGE_PACK:
							mavlink_msg_open_drone_id_message_pack_decode(&message, &msg_rx.message_pack);
							printf("open drone ID message pack received: quantity %i size %i\n",msg_rx.message_pack.msg_pack_size,msg_rx.message_pack.single_message_size);
							fflush(stdout);
                            for (int k = 0; k < msg_rx.message_pack.msg_pack_size; k++)
                            {
                                uint8_t start_byte = msg_rx.message_pack.messages[25*k];
                                uint8_t RID_type = (start_byte & 0xF0) >> 4;
                                switch (RID_type)
                                {
                                    case ODID_MESSAGETYPE_BASIC_ID:
                                        printf("basic ID\n");
                                        struct ODID_BasicID_data basic_id;
                                        odid_initBasicIDData(&basic_id);
                                        decodeBasicIDMessage(&basic_id, (ODID_BasicID_encoded *) &msg_rx.message_pack.messages[25*k]);
                                        printBasicID_data(&basic_id);
                                    break;
                                    case ODID_MESSAGETYPE_LOCATION:
                                        printf("location\n");
                                        struct ODID_Location_data location;
                                        odid_initLocationData(&location);
                                        decodeLocationMessage(&location, (ODID_Location_encoded *) &msg_rx.message_pack.messages[25*k]);
                                        printLocation_data(&location);
                                    break;
                                    case ODID_MESSAGETYPE_AUTH:
                                        printf("Auth\n");
                                        struct ODID_Auth_data auth;
                                        odid_initAuthData(&auth);
                                        decodeAuthMessage(&auth, (ODID_Auth_encoded *) &msg_rx.message_pack.messages[25*k]);
                                        printAuth_data(&auth);
                                    break;
                                    case ODID_MESSAGETYPE_SELF_ID:
                                        printf("Self ID\n");
                                        struct ODID_SelfID_data self_id;
                                        odid_initSelfIDData(&self_id);
                                        decodeSelfIDMessage(&self_id, (ODID_SelfID_encoded *) &msg_rx.message_pack.messages[25*k]);
                                        printSelfID_data(&self_id);
                                    break;
                                    case ODID_MESSAGETYPE_SYSTEM:
                                        printf("system\n");
                                        struct ODID_System_data system;
                                        odid_initSystemData(&system);
                                        decodeSystemMessage(&system, (ODID_System_encoded *) &msg_rx.message_pack.messages[25*k]);
                                        printSelfID_data(&system);
                                    break;
                                    case ODID_MESSAGETYPE_OPERATOR_ID:
                                        printf("Operator ID\n");
                                        struct ODID_OperatorID_data operator_id;
                                        odid_initOperatorIDData(&operator_id);
                                        decodeOperatorIDMessage(&operator_id, (ODID_OperatorID_encoded *) &msg_rx.message_pack.messages[25*k]);
                                        printSelfID_data(&operator_id);
                                    break;
                                }
                            }
                            printf("\n");
                            fflush(stdout);
							break;
                        case MAVLINK_MSG_ID_ADSB_VEHICLE:
							mavlink_msg_adsb_vehicle_decode(&message, &msg_rx.vehicle);
                            printADSBvehicle_data(&msg_rx.vehicle);
                            printf("\n");
                            fflush(stdout);
                            break;
						default:
							printf("unknown message\n");
							break;
					}
				}
			}
		}

/*
		//transmit heart beat message at 1 Hz
		uint64_t time_ms = current_timestamp_ms();
        if ((time_ms - time_last_heartbeat_ms) > (1000 - 25)) //1Hz
        {
			time_last_heartbeat_ms = time_ms;

			//send new heart beat message
			mavlink_heartbeat_t heartbeat;
			heartbeat.type = MAV_TYPE_ODID;
			heartbeat.autopilot = MAV_AUTOPILOT_INVALID; //for not flight controll
			heartbeat.base_mode = 0;
			heartbeat.custom_mode = 0;
			heartbeat.system_status = MAV_STATE_ACTIVE;

			mavlink_message_t msg = { 0 };
			mavlink_msg_heartbeat_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg, &heartbeat);

			uint8_t tx_buffer[MAVLINK_MAX_PACKET_LEN];
			memset(tx_buffer,0,MAVLINK_MAX_PACKET_LEN);
			int message_bytes = mavlink_msg_to_send_buffer(tx_buffer, &msg);

			pthread_mutex_lock(&serial_port_lock);
			int tx_bytes = write(uart0_FS_RDWR, tx_buffer, message_bytes);
			pthread_mutex_unlock(&serial_port_lock);
			if (tx_bytes != message_bytes)
			{
				printf("ERROR: UART TX error\n");
			}
		}*/
	}
}

int main(int argc, char *argv[] )
{
	int result = pthread_mutex_init(&serial_port_lock, NULL);

	pthread_t thread_serial_port_RX;
	int  iret_serial_port_RX;
	char *message_serial_port_RX = "Thread receiving MAVLink messages";

	char interface_UART[128]; //serial interface
	strcpy(interface_UART,UART_PORT);

	if( access(interface_UART, F_OK ) != -1 )
	{
		uart0_FS_RDWR = open(interface_UART, O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
		if (uart0_FS_RDWR == -1)
		{
			printf("WARNING: Unable to open UART interface for MAVLink communication. %s\n",interface_UART);
			exit(11);
		}
		struct termios options;
		tcgetattr(uart0_FS_RDWR, &options);
		options.c_cflag = UART_BAUDRATE | CS8 | CLOCAL | CREAD;
		options.c_iflag = IGNPAR;
		options.c_oflag = 0;
		options.c_lflag = 0;

		options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
		options.c_oflag &= ~(ONLCR | OCRNL);
		options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

		tcflush(uart0_FS_RDWR, TCIFLUSH);
		tcsetattr(uart0_FS_RDWR, TCSANOW, &options);
	}

	//set timezone to application
	char buf[80];
	snprintf(buf, sizeof(buf), "TZ=UTC");
	putenv(buf);
	tzset();

	char *date2 = __DATE__;
	char *time2 = __TIME__;
	printf("DroneScout Bridge application\n2024 Bluemark Innovations\n");
	printf("version: %s %s\n", date2,time2);
    setvbuf(stdout, NULL, _IONBF, 0);
	iret_serial_port_RX = pthread_create( &thread_serial_port_RX, NULL, serial_port_receive,(void*) message_serial_port_RX);

	while (1)
	{
		sleep(10);
	}

	return 0;
}
