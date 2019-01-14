/*************************************************

udp_receiver.cpp

Programa que inicializa un nodo con nombre definido en NODE_NAME, que recibe paquetes UDP de LabView desde 
la dirección definida en SRV_IP y el puerto especificado en PORT, y los publica en un topic con nombre 
definido en TOPIC_NAME.

No se ha conseguido hacer una desserialización del paquete UDP con ros::serialization::deserializeMessage
ni con otras herramientas parecidas, por lo que se ha implementado un parseador propio. 

*************************************************/

#include <stdio.h>
#include <ros/ros.h>
#include <arpa/inet.h>
#include "sensor_msgs/JointState.h"
#include "udp_config.hpp"

#define LOOP_RATE 	1

using namespace std;

// Función para obtener un int a partir de 4 bytes.
int parse_int(char* & buffer)
{
	int seq = ((int)buffer[0]<<24) + 
			  ((int)buffer[1]<<16) + 
			  ((int)buffer[2]<<8) + 
			  ((int)buffer[3]);
	
	// Se modifica el puntero para que apunte al siguiente elemento.
	buffer += sizeof(int);

	return seq;
}

// Función para obtener un double a partir de 8 bytes.
double parse_float(char* &buf)
{
	// Debido a diferencias en la ordenación de los bytes entre 
	// LabView y Ubuntu, habrá que dar la vuelta al array.
	union { char mydouble[sizeof(double)]; double d; };
	for(int i = 0; i < sizeof(double); i++) { mydouble[i] = buf[7-i]; }

	// Se modifica el puntero para que apunte al siguietne elemento.
	buf += sizeof(double);

	return d;
}

// Función para obtener un sting de tantos bytes como se indique en el argumento "size".
string parse_string(char* & buffer, int size)
{
	string result;

	for(int i = 0; i < size; i++) { result = result + buffer[i]; }

	// Se modifica el puntero para que apunte al siguietne elemento.
	buffer += size;

	return result;
}

// Función para descomponer el paquete UDP en un mensaje del tipo sensor_msgs::JointState.
sensor_msgs::JointState parse_buffer(char* buffer)
{
	sensor_msgs::JointState msg;

	// El header consta de 3 int y un string.
	// "seq" realmente acaba siendo sustituido en Ubuntu por un número que indica 
	// cuantas veces se ha modificado el contenido del mensaje
	msg.header.seq        = parse_int(buffer);
	msg.header.stamp.sec  = parse_int(buffer);
	msg.header.stamp.nsec = parse_int(buffer);

	// Cada string va precedido por un número que incica el número de letras que contiene.
	int n_fid = parse_int(buffer);
	msg.header.frame_id = parse_string(buffer, n_fid);

	// Cada array va precedido por un número que incica el número de elementos que contiene.
	int arr_size = parse_int(buffer);
	for(int i = 0; i < arr_size; i++)
	{
		int str_size = parse_int(buffer);
		msg.name.push_back(parse_string(buffer, str_size));
	}

	arr_size = parse_int(buffer);
	for(int i = 0; i < arr_size; i++)
	{
		msg.position.push_back(parse_float(buffer));
	}

	arr_size = parse_int(buffer);
	for(int i = 0; i < arr_size; i++)
	{
		msg.velocity.push_back(parse_float(buffer));
	}

	arr_size = parse_int(buffer);
	for(int i = 0; i < arr_size; i++)
	{
		msg.effort.push_back(parse_float(buffer));
	}
	
	return msg;
}

// Función para inicializar una conexión.
void start_udp_connection(int& s, struct sockaddr_in& addr)
{
	if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0){
		printf("ERROR opening socket\n");
        exit(1);
	}else { printf("OK opening socket\n"); }

	memset(&addr, 0, sizeof(struct sockaddr_in));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(RECEIVER_PORT);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(s, (struct sockaddr *) &addr, sizeof(addr)) < 0){
		printf("ERROR binding socket\n");
        exit(1);
	}else { printf("OK binding socket\n"); }

    return;
}

int main(int argc, char *argv[]){

	int sock;
 	struct sockaddr_in addr;
 	sensor_msgs::JointState jointbuffer;
	char buffer[BUFLEN];
 	
 	// Inicialización del socket
 	start_udp_connection(sock, addr);

 	// Inicialización del nodo, del topic y de la frecuencia de muestreo.
 	ros::init(argc, argv, RECEIVER_NODE_NAME);
 	ros::NodeHandle n;
 	ros::Publisher pub = n.advertise<sensor_msgs::JointState>(RECEIVER_TOPIC_NAME, 1000);
 	ros::Rate loop_rate(LOOP_RATE);

    while(ros::ok()){

    	int n = recv(sock, buffer, sizeof(buffer), MSG_WAITALL);

    	// print_buffer(buffer);
	    
	    if (n < 0) { printf("ERROR reading from socket.\n"); exit(0); }
	    else 
	    { 
	    	jointbuffer = parse_buffer(buffer);
	    	pub.publish(jointbuffer);
	    }

	    loop_rate.sleep();
	}

	close(sock);
	return 0;
}