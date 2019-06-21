#include "serial_port.h"

static int ttyfd;
static FILE *fd;

gboolean serial_port_init (gchar *port) 
{
	struct termios options;
	
	if ((ttyfd = open (port, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
	{
		perror ("Unable to open serial port: ");
		return FALSE;		
	}
		
	tcgetattr (ttyfd, &options);
	
	// set baud rate
	cfsetispeed (&options, TTY_SPEED);
	cfsetospeed (&options, TTY_SPEED);
	
	options.c_cflag |= CLOCAL | CREAD;						// enable the receiver and set local mode
	options.c_cflag &= ~CSIZE;								// mask the character size bits
	options.c_cflag |= CS8;									// select 8 data bits
	options.c_cflag &= ~PARENB;								// disable the parity check
	options.c_cflag &= ~CSTOPB;								// 1 stop bit
	options.c_cflag &= ~CRTSCTS;								// disable hardware flow control
	options.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK);		// disable software flow control
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);		// select raw input	
	options.c_oflag &= ~OPOST;								// select raw output
	
	// set the new options for the port
	tcsetattr (ttyfd, TCSAFLUSH, &options);
	
	if ((fd = fdopen (ttyfd, "rw+")) == NULL)
	{
		perror ("Unable to open file stream");
		return FALSE;
	}
	
	return TRUE;
}

void serial_port_close (void) 
{
	if (fd)
		fclose (fd);
}

gboolean read_nonblock (char *buf)
{
	int watchdog = 0, i = 0;
	char c;
	
	while (c != '\n') {
		while (fread (&c, 1, 1, fd) != 1)	{
			usleep (100);
			if (watchdog++ > 10000)
				return FALSE;
		}
		buf [i++] = c;
	}
		
	return TRUE;
}

gboolean read_value (gint32 *value)
{
	char buf [64];
	gboolean ret;
	
	ret = read_nonblock (buf);
	*value = atoi (buf);
	
	return ret;
}

gboolean serial_read_position (gint32 *value) 
{
	fprintf (fd, "L\n");
	return read_value (value);
}

gboolean serial_port_get_feedback (gint32 *buf[], guint32 *size)
{
	*size = CHART_BUF;
	guint32 fb_index;
	
	*buf = g_malloc0 ((gsize) *size * sizeof (gint32));
	
	while (fb_index != *size) {
		usleep (100);
		fprintf (fd, "F\n");		
		if (!read_value ((gint32 *) &fb_index))
			return FALSE;
	}	
		
	fprintf (fd, "G\n");

	for (guint i = 0; i < *size; i++) {
		if (!read_value (&buf[0][i]))
			return FALSE;
	} 	
	
	return TRUE;
}

void serial_port_send_setpoint (gint32 value) 
{
	fprintf (fd, "J%d\n", value);
}

void serial_port_send_parameters (gint32 vl, gint32 kf, gint32 kp, gint32 ki, gint32 kd, gint32 kg) 
{
	fprintf (fd, "P0%d\n", vl);
	fprintf (fd, "P1%d\n", kf);
	fprintf (fd, "P2%d\n", kp);
	fprintf (fd, "P3%d\n", ki);
	fprintf (fd, "P4%d\n", kd);
	fprintf (fd, "P5%d\n", kg);
}

void serial_port_send_reset (void) 
{
	fprintf (fd, "R\n");
}

void serial_port_send_new_mode (guint32 mode) 
{
	fprintf (fd, "M%d\n", mode);
}

void serial_port_send_sub_cmd (guint32 cmd)
{
	fprintf (fd, "C%d\n", cmd);
}

void serial_port_send_save_parameters (guint8 cell)
{
	fprintf (fd, "Ss%d\n", cell);
}

gboolean serial_port_load_parameters (
	guint8 cell, gint32 *vl, gint32 *kf, gint32 *kp, gint32 *ki, gint32 *kd, gint32 *kg)
{

	gchar str [256];
	
	fprintf (fd, "Sl%d\n", cell);
	
	if (!read_nonblock (str))
		return FALSE;	
	
	sscanf (str, "%d,%d,%d,%d,%d,%d", vl, kf, kp, ki, kd, kg);
	
	return TRUE;
}

