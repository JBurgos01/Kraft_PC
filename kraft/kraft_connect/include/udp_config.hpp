/************************************************************************************

udp_config.hpp

This file contains some definitions useful for the kraft_connect package

************************************************************************************/

#ifndef UDP_CONFIG
#define UDP_CONFIG

#include <stdio.h>

// Number of DoF of the robot. The parser in udp_receiver will expect arrays of this length.
#define DOF 6
// Legth of the complete buffer used both in udp_receiver and udp_sender.
#define BUFLEN 256

// Names of the topic and node for each executable.
#define SENDER_TOPIC_NAME	"kraft_state"
#define SENDER_NODE_NAME	"udp_sender"
#define RECEIVER_TOPIC_NAME	"kraft_state"
#define RECEIVER_NODE_NAME	"udp_receiver"

// IP and ports of the Windows machine running LabView.
#define SRV_IP "192.168.0.107"
#define SENDER_PORT	  5051
#define RECEIVER_PORT 5052

// Función para imprimir un paquete UDP byte a byte (para debug).
void print_buffer(char* buffer)
{
	for(int i = 0; i < BUFLEN; i++)
	{
		printf("%c\t", buffer[i]);
	}
	printf("\n--------------------------------------\n\n");

	return;
}

#endif /* UDP_CONFIG */
