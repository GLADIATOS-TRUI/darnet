#include <stdio.h>
#include <string.h>    //strlen
#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include <unistd.h>    //write

#include <iostream>

class ServerRobot {
	private:
		int client;
		int sockfd;
		int portno;
		struct sockaddr_in serv_addr, cli_addr;
		int MESSAGE_LENGTH;
		
	
	public:
		/**
		 * bikin constructor sekaligus binding
		 */
		ServerRobot(){
			portno = 12356;
			MESSAGE_LENGTH = 1024;
			sockfd = socket(AF_INET, SOCK_STREAM, 0);
	
			if (sockfd < 0 ) {
				puts("Error create socket");
			}
			else {
				printf("new Socket created.");
				
				bzero( (char *) &serv_addr, sizeof serv_addr);

				serv_addr.sin_family = AF_INET;
				serv_addr.sin_addr.s_addr = INADDR_ANY;
				serv_addr.sin_port = htons(portno);
	
				int n = bind(sockfd, (struct sockaddr *) &serv_addr, sizeof serv_addr);
				
				if(n < 0) {
					perror("Error binding");
				}
				else
					printf("Binding completed.\n Port No : %d\n", portno);
			}
		}
		
		/**
		 * method untuk menunggu client. dia akan wait selama belum ada yang connect
		 */
		void waitForClient() {
			int cli_addr_len = sizeof cli_addr;
			listen(sockfd, 5);
			client = accept( sockfd, (struct sockaddr *) &cli_addr ,  (socklen_t *) &cli_addr_len );
		}
		
		/**
		 * method untuk mengirim pesan ke client
		 * maksimum karakter yang diterima 1024
		 * return : true jika berhasil dan false jika gagal 
		 */
		bool sendMessageToClient(const char* msg) {
			int n = write(client , msg , MESSAGE_LENGTH);
			return n >= 0; 
		}
		
		/**
		 * method untuk menerima pesan
		 * selama menerima pesan maka dia wait terus jika belum ada pesan valid (lengthnya > 0) yang diterima
		 * maksimum karakter yang diterima 1024
		 * return : pesan valid yang diterima
		 */
		char* waitForMessage(){
			char msg[MESSAGE_LENGTH];
			bzero(msg, sizeof msg);
			int n ;
			
			while (strlen(msg) == 0)
				n = read(client, msg, MESSAGE_LENGTH - 1);
			
			if (n <0) strcpy(msg, "-1");
			
			char* message = &msg[0]; 
			return message;
		}
		
		/**
		 * untuk close connection
		 */
		void closeConnection(){
			close (client);
		}	
};

/**
 * tester nya. saran jika mau menerima message maka pakai multi threading aja
 */
int main() {
	ServerRobot server;
		
	puts("wait");
	server.waitForClient();
	
	puts("send");
	server.sendMessageToClient("Hello");
	std::cout << server.waitForMessage();
	
	puts("send");
	server.sendMessageToClient("Roger That");
	
	puts("send");
	server.sendMessageToClient("Hello");
	
	puts("send");
	server.sendMessageToClient("Roger That");
	
	server.closeConnection();
	
	return 0;
}
