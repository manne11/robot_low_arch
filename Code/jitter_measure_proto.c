/* testing the encoder input */


#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <unistd.h>
struct termios saved_attributes;
static int fd_serial_usb = -1;


int written_USB = -1;

/* difference between two timespec structures; courtesy GNU C library */

/* Use this variable to remember original terminal attributes. */



static void
reset_input_mode (void)
{
  tcsetattr (fd_serial_usb, TCSANOW, &saved_attributes);
}

void
signal_callback_handler(int signum) {
    printf("Caught SIGTERM\n");
    reset_input_mode();
    close(fd_serial_usb);
    exit(signum);
}


void
set_input_mode (int fd_serial_usb,
		struct termios tattr
		struct termios saved_attr)
{

  /* Make sure stdin is a terminal. */
  if (!isatty (fd_serial_usb))
    {
      fprintf (stderr, "Not a terminal.\n");
      exit (EXIT_FAILURE);
    }

  /* Save the terminal attributes so we can restore them later. */
  tcgetattr (fd_serial_usb, &saved_attr);
  atexit (reset_input_mode);

  /* ********set terminal modes********** */
  tcgetattr (fd_serial_usb, &tattr);

  /* set the BSD style raw mode */
  cfmakeraw (&tattr);
  tattr.c_cflag |= CREAD | CLOCAL;



  /* only need 1 stop bit */
  tattr.c_cflag &= ~CSTOPB;

  /* no hardware flowcontrol */
  tattr.c_cflag &= ~CRTSCTS;

  /* set the baud rate */
  cfsetspeed (&tattr,B115200);


  /* set the attributes for tuning read() */
  tattr.c_cc[VMIN] = 4;
  tattr.c_cc[VTIME] = 1 ;
  tcsetattr (fd_serial_usb, TCSAFLUSH, &tattr);
}

/* opens the requested serial port(tty)  */
int open_serial_port(int fd_serial_usb, const char * tty)
{
  fd_serial_usb = open (tty, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_serial_usb < 0) {
      printf ("error opening file ");
      exit (EXIT_FAILURE);
  }
  return fd_serial_usb;
}


void set_echo_off()
{
  int written_serial = 0;
  written_serial = write (fd_serial_usb, "^ECHOF 1\r", 9);
  if (written_serial < 0){
      puts ("error writing to serial port");
      exit (EXIT_FAILURE);

  }

}

void write_to_port(int fd_serial, const void * write_string, size_t write_string_size)
{
  if (write (fd_serial, write_string, write_string_size)){
    puts ("error writing to serial port");
    exit (EXIT_FAILURE);
  }

}

void read_from_port(int fd_serial, char * read_buffer_serial, size_t buffer_size, int characters_read)
{
  characters_read = read (fd_serial, read_buffer_serial, buffer_size);
  if ( characters_read < 0){
    puts ("error reading from serial port");
    exit (EXIT_FAILURE);
  } else {
   /* set the last array cell to null so it can be read by printf */
   read_buffer_serial[characters_read] = 0;
 }
}


int
main (void)
{
  char read_buffer_USB[50];
  int written_USB = -1;
  int read_USB = -1;
  int characters_read;
  const int TRUE = 1;
  const int FALSE = 0;
  int state = TRUE;
  int loop = 0;
  /* int input_buffer_size = -1; */
  

  /* in case of SIGINT */
  signal(SIGINT, signal_callback_handler);

  /* opening the file */
  fd_serial_usb = open_serial_port(fd_serial_usb,"/dev/ttyUSB0" );
  
  
  set_input_mode (fd_serial_usb);
  /* puts("starting reading and writing \n\n"); */
  /* setting the echo off */
  set_echo_off();



  written_USB = -1;

  while (state)
    {
      /* giving a speed */
      write_to_port(fd_serial_usb, "!P 1 1000\r", 10);
      read_from_port(fd_serial_usb, read_buffer_USB, 49, characters_read);



      


      printf("%s \n", read_buffer_USB);
      if (loop == 5) {
	  state = FALSE;
      }
      else {
	  loop = loop + 1;
      }

    }


  return EXIT_SUCCESS;
}


/* look at ioctl TIOCMGET FIONREAD */
/* hints exist that there needs to be a time delay before a read is performed because of the way drivers are setup */




























/* by sawdust from stack overflow https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c */








/* #include <errno.h> */
/* #include <fcntl.h>  */
/* #include <stdio.h> */
/* #include <stdlib.h> */
/* #include <string.h> */
/* #include <termios.h> */
/* #include <unistd.h> */
/* #include <signal.h> */


/* #define DISPLAY_STRING */
/* static int fd = 0; */

/* int set_interface_attribs(int fd, int speed) */
/* { */
/*     struct termios tty; */

/*     if (tcgetattr(fd, &tty) < 0) { */
/*         printf("Error from tcgetattr: %s\n", strerror(errno)); */
/*         return -1; */
/*     } */

/*     cfsetospeed(&tty, (speed_t)speed); */
/*     cfsetispeed(&tty, (speed_t)speed); */

/*     tty.c_cflag |= (CLOCAL | CREAD);    /\* ignore modem controls *\/ */
/*     tty.c_cflag &= ~CSIZE; */
/*     tty.c_cflag |= CS8;         /\* 8-bit characters *\/ */
/*     tty.c_cflag &= ~PARENB;     /\* no parity bit *\/ */
/*     tty.c_cflag &= ~CSTOPB;     /\* only need 1 stop bit *\/ */
/*     tty.c_cflag &= ~CRTSCTS;    /\* no hardware flowcontrol *\/ */

/*     /\* setup for non-canonical mode *\/ */
/*     tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON); */
/*     tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN); */
/*     tty.c_oflag &= ~OPOST; */

/*     /\* fetch bytes as they become available *\/ */
/*     tty.c_cc[VMIN] = 1; */
/*     tty.c_cc[VTIME] = 1; */

/*     if (tcsetattr(fd, TCSANOW, &tty) != 0) { */
/*         printf("Error from tcsetattr: %s\n", strerror(errno)); */
/*         return -1; */
/*     } */
/*     return 0; */
/* } */

/* void set_mincount(int fd, int mcount) */
/* { */
/*     struct termios tty; */

/*     if (tcgetattr(fd, &tty) < 0) { */
/*         printf("Error tcgetattr: %s\n", strerror(errno)); */
/*         return; */
/*     } */

/*     tty.c_cc[VMIN] = mcount ? 1 : 0; */
/*     tty.c_cc[VTIME] = 5;        /\* half second timer *\/ */

/*     if (tcsetattr(fd, TCSANOW, &tty) < 0) */
/*         printf("Error tcsetattr: %s\n", strerror(errno)); */
/* } */


/* void */
/* signal_callback_handler(int signum) { */
/*     printf("Caught SIGTERM\n"); */
/*     close(fd); */
/*     exit(signum); */
/* } */



/* int main() */
/* { */

/*     unsigned char buf[80]; */
/*     int rdlen; */

/*     char *portname = "/dev/ttyUSB0"; */
/*     int wlen; */

/*     fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC); */
/*     if (fd < 0) { */
/*         printf("Error opening %s: %s\n", portname, strerror(errno)); */
/*         return -1; */
/*     } */

/*     signal(SIGINT, signal_callback_handler); */
/*     /\*baudrate 115200, 8 bits, no parity, 1 stop bit *\/ */
/*     set_interface_attribs(fd, B115200); */
/*     //set_mincount(fd, 0);                /\* set to pure timed read *\/ */



/*     /\* simple noncanonical input *\/ */
/*     do { */






/*     /\* simple output *\/ */
/*     wlen = write(fd, "!M 50 2_\n", 5); */
/*     if (wlen != 5) { */
/*         printf("Error from write: %d, %d\n", wlen, errno); */
/*     } */
/*     tcdrain(fd);    /\* delay for output *\/ */





/*         rdlen = read(fd, buf, sizeof(buf) - 1); */
/*         if (rdlen > 0) { */
/* #ifdef DISPLAY_STRING */
/*             buf[rdlen] = 0; */
/*             printf("Read %d: \"%s\"\n", rdlen, buf); */
/* #else /\* display hex *\/ */
/*             unsigned char   *p; */
/*             printf("Read %d:", rdlen); */
/*             for (p = buf; rdlen-- > 0; p++) */
/*                 printf(" 0x%x", *p); */
/*             printf("\n"); */
/* #endif */
/*         } else if (rdlen < 0) { */
/*             printf("Error from read: %d: %s\n", rdlen, strerror(errno)); */
/*         } else {  /\* rdlen == 0 *\/ */
/*             printf("Timeout from read\n"); */
/*         }                */
/*         /\* repeat read to get full message *\/ */
/*     } while (1); */
/* 	} */
