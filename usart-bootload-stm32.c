/*
Copyright (C) 2022 kumohakase
Program loader for stm32 uart bootloader protocol (Load to RAM)
Please consider supporting me using kofi.com https://ko-fi.com/kumohakase
*/

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>

int receive_byte(int,int);
int receive_multi(int,uint8_t*,int,int);
int transmit_multi_c(int,uint8_t*,int,int);
int wait_input(int,int);

int main(int argc,char *argv[]) {
	if(argc < 2) {
		printf("Usage: %s SerialPort SRECPath\n",argv[0]);
		printf("If you want to get target information only, please omit SRECPath\n");
		return 1;
	}
	//Open serial port and configure
	struct termios tattr;
	int port = open(argv[1],O_RDWR);
	if(port < 0) {
		perror("Serial port open failed");
		return 1;
	}
	//Get current config
	if(tcgetattr(port,&tattr) != 0) {
		perror("Getting configuration failed");
		close(port);
		return 1;
	}
	cfmakeraw(&tattr); //Use raw mode
	cfsetspeed(&tattr,B115200); //Baud set to 115200
	tattr.c_cflag |= PARENB; //Even parity enabled
	//Set
	if(tcsetattr(port,TCSANOW,&tattr) != 0) {
		perror("Setting configuration failed");
		close(port);
		return 1;
	}
	//Perform auto baud. First step of STM32 UART bootloader.
	int i,r;
	printf("Autobaud\n");
	for(i = 0;i < 50;i++) {
		//Break loop if ACK ('y') received when sending 0x7f.
		r = write(port,"\x7f",1);
		if(r != 1) {
			//Write error case.
			printf("Error!\n");
			close(port);
			return 2;
		}
		if(receive_byte(port,100) == 'y') {break;}
		putchar('.');
		fflush(stdout); //Show letter without newline.
	}
	if(i < 50) {printf("OK!\n");} else {
		//Loop timeout case
		printf("Fail!\n");
		close(port);
		return 2;
	}
	//Get target CPU information.
	//Send Get version command (0x1) to get bootloader information.
	if(transmit_multi_c(port,"\x01",1,100) != 0) {
		printf("Get command (0x1) failed!\n");
		close(port);
		return 2;
	}
	//Read information we got.
	uint8_t buffer[256];
	if(receive_multi(port,buffer,4,100) != 0) {
		printf("Data receive failed!\n");
		close(port);
		return 2;
	}
	printf("Bootloader version: %x.%x\n",buffer[0] >> 4,buffer[0] & 0xf);
	//Get target CPU PID, send command GetID (0x2)
	if(transmit_multi_c(port,"\x02",1,100) != 0) {
		printf("Get PID command (0x2) failed!\n");
		close(port);
		return 2;
	}
	//Read information we got.
	if(receive_multi(port,buffer,4,100) != 0) {
		printf("Data receive failed!\n");
		close(port);
		return 2;
	}
	printf("Target PID: %x\n",buffer[1] * 0x100 + buffer[2]);
	//SREC file omitted case.
	if(argc < 3) {
		close(port);
		return 0;
	}
	//Read SREC file and write to target memory
	FILE *f = fopen(argv[2],"r");
	if(f == NULL) {
		perror("SREC file open failed");
		close(port);
		return 1;
	}
	int lineno = 0; //line number
	int startaddr = -1;
	while(fgets(buffer,sizeof(buffer),f) != NULL) {
		//Read for each line, loop until EOF
		lineno ++;
		//Decode for S3 record format: S3LLAAAAAAAADD..DDCC where LL is length of address + data + checksum, AAAAAAAA is address, DD..DD is data (non fixed length), CC is checksum.
		//Check header
		if(memcmp("S3",buffer,2) != 0) {
			printf("Only S3 record will be recognized, skipping. (line %d)\n",lineno);
			continue;
		}
		//Decode address and length.
		int recaddr,reclength;
		char buffer2[16];
		//Getting substring represents record length and address
		memcpy(buffer2,&buffer[2],10);
		buffer2[10] = '\0';
		if(sscanf(buffer2,"%02X%08X",&reclength,&recaddr) != 2) {
			//Conversion failed case.
			printf("Can not decode line %d, skipping.\n",lineno);
			continue;
		}
		reclength -= 5;
		//Length must be in 0 to 64, Address must be in range 0 to 0xffffffff.
		if(!(0 <= recaddr && recaddr <= 0xffffffff && 0 <= reclength && reclength <= 64)) {
			printf("Bad record detected on line %d, skipping.\n",lineno);
			continue;
		}
		if(startaddr == -1) { startaddr = recaddr; } //First record address will be start address of go command.
		//Decode data
		uint8_t recdata[reclength + 1];
		int e = 0;
		for(i = 0;i < reclength; i++) {
			//Get letters represents data by 2 digit hex. offset: 12
			memcpy(buffer2,&buffer[i * 2 + 12],2);
			buffer2[2] = '\0';
			if(sscanf(buffer2,"%02X",&r) != 1) {
				//Convertion failure case.
				printf("Bad record detected on line %d, data %d, skipping\n",lineno,i);
				e = 1;
				break;
			}
			if(!(0 <= r && r <= 255)) {
				//Not in range.
				printf("Bad record detected on line %d, data %d, skipping.\n",lineno,i);
				e = 1;
				break;
			}
			recdata[i + 1] = r;
		}
		if(e == 1) {continue;}
		//Write data to target device memory.
		printf("Writing %d bytes on 0x%x\n",reclength,recaddr);
		//Send memory write command (0x31)
		if(transmit_multi_c(port,"\x31",1,100) != 0) {
			printf("Memory write command (0x31) failed.\n");
			close(port);
			return 2;
		}
		//Send 4 byte start address.
		buffer2[0] = (recaddr >> 24) & 0xff;
		buffer2[1] = (recaddr >> 16) & 0xff;
		buffer2[2] = (recaddr >> 8) & 0xff;
		buffer2[3] = recaddr & 0xff;
		if(transmit_multi_c(port,buffer2,4,100) != 0) {
			printf("Memory address send failed.\n");
			close(port);
			return 2;
		}
		//Send data length - 1 (single byte) + data
		recdata[0] = reclength - 1;
		i = transmit_multi_c(port,recdata,reclength + 1,500);
		if(i != 0) {
			if(i == -2) {printf("Memory write failed (Timeout)!\n");} else {printf("Memory write failed!\n");}
			close(port);
			return 2;
		}
	}
	fclose(f);
	if(startaddr == -1) {startaddr = 0;}
	//Jump to written program address.
	//Send go command (0x21)
	printf("Jumping to loaded program, startaddress=0x%x\n",startaddr);
	if(transmit_multi_c(port,"\x21",1,100) != 0) {
		printf("Go command (0x21) sending failed.\n");
		close(port);
		return 2;
	}
	//Send address
	buffer[0] = (startaddr >> 24) & 0xff;
	buffer[1] = (startaddr >> 16) & 0xff;
	buffer[2] = (startaddr >> 8) & 0xff;
	buffer[3] = startaddr & 0xff;
	if(transmit_multi_c(port,buffer,4,100) != 0) {
		printf("Go command failed!\n");
		close(port);
		return 2;
	}
	printf("Jumped into loaded program, have a nice day!\n");
	close(port);
	return 0;
}

//Wait for input of fd.
int wait_input(int fd,int timeout) {
	struct pollfd p;
	p.fd = fd;
	p.events = POLLIN;
	return poll(&p,1,timeout);
}

//Receive one byte.
int receive_byte(int fd,int timeout) {
	int i;
	if(timeout != 0) {
		i = wait_input(fd,timeout); //Wait for input up to timeout if timeout is nonzero.
		if(i == -1) {
			//Error case
			return -1;
		} else if(i == 0) {
			//Timeout case
			return -2;
		}
	}
	uint8_t rdata; //Single byte buffer.
	i = read(fd,&rdata,1); //Read one byte.
	if(i != 1) {return -1;} //On error.
	return rdata;
}

//Receive multiple bytes.
int receive_multi(int fd,uint8_t *data,int len,int timeout) {
	int i;
	int r = 0;
	while(r < len) {
		if(timeout != 0) {
			i = wait_input(fd,timeout); //Wait for input up to timeout if timeout is nonzero.
			if(i == -1) {
				//Error case
				return -1;
			} else if(i == 0) {
				//Timeout case
				return -2;
			}
		}
		i = read(fd,&data[r],len);
		if(i < 1) {return -1;} //Error case
		r = r + i;
	}
	return 0;
}

//Send multiple byte plus checksum and get one byte response.
int transmit_multi_c(int fd,uint8_t *data,int len,int timeout) {
	int i;
	uint8_t c;
	i = write(fd,data,len);
	if(i != len) {return -1;} //Error case.
	if(len == 1) {
		//Checksum: data XOR 0xff when single byte
		c = 0xff - data[0];
	} else {
		//Checksum: XOR of each datas when multi byte
		c = data[0];
		for(i = 1;i < len; i++) {c = c ^ data[i];}
	}
	write(fd,&c,1);
	i = receive_byte(fd,timeout); //Receive one byte.
	if(i == -1) {
		return -1; //Error case.
	} else if(i == -2) {
		return -2; //Timeout case.
	} else if(i != 'y') {
		return -3; //NACK case.
	}
	return 0;
}
