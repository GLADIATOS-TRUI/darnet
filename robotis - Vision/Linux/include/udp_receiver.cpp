#include <stdio.h>
# include <netdb.h>
# include <netinet/in.h>
# include <arpa/inet.h>
# include <sys/time.h>
# include <sys/types.h>
# include <sys/socket.h>
# include <unistd.h>
# include <time.h>
# include <cstring>
# include <cstdlib>
# include <iostream>

using namespace std;

// game state
	//~ public static final byte STATE_INITIAL = 0;
    //~ public static final byte STATE_READY = 1;
    //~ public static final byte STATE_SET = 2;
    //~ public static final byte STATE_PLAYING = 3;
    //~ public static final byte STATE_FINISHED = 4;

class GameController{
	private :
		int version ;
		uint16_t packetNumber;
		uint16_t numPlayers;
		uint16_t gameType;
		uint16_t gameState;
		uint16_t firstHalf;
		uint16_t kickOffTeam;
		uint16_t secGameState;
		uint16_t dropInTeam;
		uint32_t dropInTime;
		uint32_t secsRemaining;
		uint32_t secondaryTime;
		char header[4];
		
		sockaddr_in broadcast_sockaddr;
		int broadcast_addr;
		int portno;
		int broadcast_socket;
	
	public :
		
		
		GameController(char* ip_addr, int portno){
			
			if (ip_addr == NULL )
				broadcast_addr = htonl(INADDR_ANY);
			else
				broadcast_addr = inet_addr(ip_addr);
				
			this->portno = portno;
			
			int broadcast=1;
	
			if(( broadcast_socket =socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0){
				perror("socket failed.");
			}


			setsockopt( broadcast_socket , SOL_SOCKET, SO_BINDTODEVICE, &broadcast, sizeof broadcast);
			
			memset(&broadcast_sockaddr, 0, sizeof(broadcast_sockaddr));

			broadcast_sockaddr.sin_family = AF_INET;
			broadcast_sockaddr.sin_port = htons(portno);
			broadcast_sockaddr.sin_addr.s_addr = broadcast_addr;
			
			
		}
		
		
		void start() {
			if ( bind(broadcast_socket, (sockaddr *) &broadcast_sockaddr, sizeof(broadcast_sockaddr)) < 0  )
				perror("error binding.");
			puts("binding completed.");
		}
		
		int receive_broadcast() {
			
			uint8_t buf[10000];

			bzero (buf, sizeof buf);
		
			unsigned slen = sizeof (broadcast_sockaddr);
			int n = recvfrom(broadcast_socket, buf, (uint32_t) sizeof(buf)-1, 0, (sockaddr *)&broadcast_sockaddr, &slen);
		
			for (int i = 0; i<4; i++)
					header[i] = ((uint16_t)buf[i] );
			
			version = buf[4] + ((uint16_t) buf[5] << 8);
			packetNumber = buf[6];
			numPlayers = buf[7];
			gameType = buf[8];
			gameState = buf[9];
			firstHalf = buf[10];
			kickOffTeam = buf[11];
			secGameState = buf[12];
			dropInTeam = buf[13];
			dropInTime = buf[14] + ((uint16_t) buf[15] << 8);
			secsRemaining = buf[16] + ((uint16_t) buf[17] << 8);
			secondaryTime = buf[18] + ((uint16_t) buf[19] << 8);
			
			return n;
		}
		
		uint16_t getGameState() {
			return gameState;
		}
		
		
		uint16_t getSecGameState() {
			return secGameState;
		}
		
		
		uint32_t getRemainingSecs() {
			return secsRemaining;
		}
		
		
		uint32_t getSecondaryTime() {
			return secondaryTime;
		}
		
		
		bool isAllowedToMove() {
			return secsRemaining < 600;
		}
		
		uint16_t getKickOff() {
			return kickOffTeam;
		}
};
//~ 
//~ int main (int argc, char** args){
	//~ 
	//~ int portno = atoi(args[1]);
	//~ 
	//~ GameController GC(NULL, portno);
//~ 
	//~ GC.start();
//~ 
	//~ while(1)
	//~ {
		//~ GC.receive_broadcast();
		//~ cout << "rec num_of_bytes : " << GC.getKickOff() << "  ,sec_remaining : " << GC.getRemainingSecs() << " -- " << GC.getSecondaryTime() << " -- " << GC.getSecGameState() <<  " ,state " << GC.getGameState() << endl;
	//~ }
	//~ 
	//~ return 0;
//~ }
