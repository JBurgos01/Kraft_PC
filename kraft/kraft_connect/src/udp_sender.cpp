/**************************************************************************************************

udp_sender.cpp

Este programa se encarga de leer de un topic que contiene la posición en la que se quiere tener 
cada articulación del robot, y envía todo lo que lea por UDP a LabView.

**************************************************************************************************/

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <arpa/inet.h>
#include "sensor_msgs/JointState.h"
#include "udp_config.hpp"

using namespace std;

// Aquí se almacena el número de bytes que se ha desplazado el puntero del buffer, para poder 
// devolverlo a su posición original una vez se acabe de rellenar.
int desplazamiento = 0;

// Estas variables permitirán iniciar la conexión en main, 
// mientras que el envío de mensajes se hará desde send_udp.
int sock;
struct sockaddr_in addr;

// Función para convertir un int en cuatro bytes.
void int_to_char(int n, char* &buf)
{
	for (int i = 3; i >= 0; i--)
	{
	    buf[3-i] = (n >> 8 * i) & 0xFF;
	}

	// Se modifica el puntero para evitar borrar lo que se acaba de escribir.
	buf += sizeof(int);

	desplazamiento += sizeof(int);

	return;
}

// Función para descomponer un string en chars.
void string_to_char(string name, char* &buf)
{
	int_to_char(name.length(), buf);

	for(int i = 0; i < name.length(); i++)
	{
		buf[i] = name[i];
	}

	// Se modifica el puntero para evitar borrar lo que se acaba de escribir.
	buf += name.length();

	desplazamiento += name.length();

	return;
}

// Función para convertir un double en ocho bytes.
void double_to_char(double n, char* &buf)
{
	union { char mydouble[sizeof(double)]; double d; };
	d = n;

	for(int i = 0; i < sizeof(double); i++)
	{
		buf[i] = mydouble[7 - i];
	}

	// Se modifica el puntero para evitar borrar lo que se acaba de escribir.
	buf += sizeof(double);

	desplazamiento += sizeof(double);

	return;
}

// Función para inicializar una conexión.
void start_udp_connection()
{
	if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0){
		printf("ERROR opening socket\n");
        exit(1);
	}else{
    	printf("OK opening socket\n");
    }
	
	bzero(&addr,sizeof(addr));
	
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr(SRV_IP);
	addr.sin_port = htons(SENDER_PORT);
}

// Función que se llamará cada vez que se publique un nuevo mensaje en el topic.
void msg_receiver(const sensor_msgs::JointState::ConstPtr& msg)
{
	char *buffer = new char [BUFLEN];

	// Introducimos el header en el buffer.
	int_to_char(msg->header.seq, buffer);
	int_to_char(msg->header.stamp.sec, buffer);
	int_to_char(msg->header.stamp.nsec, buffer);

	// Luego el frame_id.
	string_to_char(msg->header.frame_id, buffer);

	// Luego un int que indicará el nº de elementos en el array name, 
	// y posteriormente el propio array.
	int_to_char(DOF, buffer);
	for(int i = 0; i < DOF; i++)
	{
		string_to_char(msg->name[i], buffer);
	}

	// Luego el array de posiciones.
	int_to_char(DOF, buffer);
	for(int i = 0; i < DOF; i++)
	{
		double_to_char(msg->position[i], buffer);
	}

	// Luego el array de velocidades.
	int_to_char(DOF, buffer);
	for(int i = 0; i < DOF; i++)
	{
		double_to_char(msg->velocity[i], buffer);
	}
	// Y por último el array de esfuerzos.
	int_to_char(DOF, buffer);
	for(int i = 0; i < DOF; i++)
	{
		double_to_char(msg->effort[i], buffer);
	}

	// Devolvemos el puntero del buffer al inicio del mensaje y lo enviamos.
	buffer -= desplazamiento;
	sendto(sock , buffer , desplazamiento , 0, (sockaddr*)&addr, sizeof(addr));

	// cout << "The following message has been sent: " << endl;
	// print_buffer(buffer);

	// Reiniciamos la variable desplazamiento para prepararla para un nuevo mensaje.
	desplazamiento = 0;

	return;
}

int main(int argc, char* argv[])
{
 	// Inicialización del socket
 	start_udp_connection();

	// Inicialización del nodo y del topic.
 	ros::init(argc, argv, SENDER_NODE_NAME);
 	ros::NodeHandle n;
 	ros::Subscriber subs = n.subscribe(SENDER_TOPIC_NAME, 1000, msg_receiver);
 	ros::Rate loop_rate(1);

 	// Dejamos el programa a la espera de que se publique algo en el topic.
 	ros::spin();

 	close(sock);

	return 0;
}