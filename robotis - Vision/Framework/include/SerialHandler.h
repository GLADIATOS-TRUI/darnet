#ifndef _SERIAL_HANDLER_H_
#define _SERIAL_HANDLER_H_

#include <termios.h>

class SerialHandler
{
	private :
		static SerialHandler* m_UniqueInstance;
		SerialHandler();

	public :
		static SerialHandler* GetInstance() { return m_UniqueInstance; }


		//char             bufferRead[BUFFER_SIZE];
		//char             bufferWrite[BUFFER_SIZE];
		//int              fd, nser;        // File descriptor for serial port

		bool isRunning;
		int port;

		int X_MOVE_AMPLITUDE;
		int A_MOVE_AMPLITUDE;
		int Y_MOVE_AMPLITUDE;

		void Initialize(int baud);
		int SerialStart(const char* portname, speed_t baud, int data);
		void KirimChar(char* inputIn, int fd);
		void itoa(int value, int length, char output[], bool isSigned);
		void SendKinematics();
		void SendHead(int pan, int tilt);
		void SendAction(int aVal);
		bool IsRunning() {return isRunning;};
		void WalkingStop();
		void WalkingStart();

		~SerialHandler();

};

#endif
