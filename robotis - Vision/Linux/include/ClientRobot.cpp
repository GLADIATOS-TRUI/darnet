#include <stdio.h>
#include <string.h>    //strlen
#include <stdlib.h>    //strlen
#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include <unistd.h>    //write
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h> //inet_addr

#include <iostream>

class ClientRobot{
	private:
		int client_socket;
		struct sockaddr_in server;
		int portno ;
		char* ip_server ; // sesuai ip server nya aja
		int MESSAGE_LENGTH;
		
		
	public:
		/**
		 * constructor sekaligus menyiapkan koneksi ke server
		 */
		ClientRobot(char* ip_server, int portno){
			puts("hahahahaha");
			this->ip_server = ip_server ;
			puts("yeay");
			this->portno	= portno;
			MESSAGE_LENGTH = 1024;
			
			client_socket = socket(AF_INET, SOCK_STREAM, 0);
			
			if (client_socket<0){
				perror("Error create socket");
				return;
			}
			else 
				puts("Client socket created.");
			
			server.sin_addr.s_addr = inet_addr(ip_server);
			server.sin_family = AF_INET;
			server.sin_port = htons(portno);
			
		}
		
		/**
		 * method untuk connect ke server sesuai constructor
		 * return : true jika berhasil dan false jika gagal
		 */
		bool connectToServer(){
			int stats = connect(client_socket, (struct sockaddr *) &server, sizeof server);
			if (stats < 0) perror("Connection failed.");
			else printf("Connection success.");
			
			return stats >=0;	
		}
		
		/**
		 * method untuk mengirim pesan ke server
		 * maksimum karakter yang diterima 1024
		 * return : true jika berhasil dan false jika gagal
		 */
		bool sendMessageToServer(char* msg){
			puts("sending...");
			return send(client_socket, msg, MESSAGE_LENGTH, 0)  >= 0;
		}
		
		/**
		 * method untuk menerima pesan
		 * selama belum ada pesan valid (lengthnya > 0) maka dia akan terus menunggu
		 * maksimum karakter yang diterima 1024
		 * return : pesan valid yang diterima
		 */
		char* waitForMessage(){
			char respond[MESSAGE_LENGTH];
			
			bzero(respond, sizeof respond);
			
			int n;
			
			while (strlen(respond) == 0)
				n = recv( client_socket, respond, MESSAGE_LENGTH-1, 0);
			
			if (n < 0) strcpy(respond, "-1");
			else if (n==0) strcpy(respond, "0");
			
			char* message = &respond[0]; 
			return message;
			
		}
		
		/**
		 * untukl close connection
		 */
		void closeConnection(){
			close(client_socket);
		}
};

/**
 * tester saran jika mau waitForMessage gunakan pthread sehingga tidak infinite loop
 */ 
int main(){
	ClientRobot client("127.0.0.1", 12356);
	
	bool stats;
	
	stats = client.connectToServer();
	if (stats) std::cout << "true" << std::endl;
	else std::cout << "false" << std::endl;
	
	stats = client.sendMessageToServer("Hola");
	if (stats) std::cout << "true" << std::endl;
	else std::cout << "false" << std::endl;
	
	
	std::cout << client.waitForMessage() << std::endl;
	puts("berikutnya");
	std::cout << client.waitForMessage() << std::endl;
	
	
	client.closeConnection();
	
	
	return 0;
}
