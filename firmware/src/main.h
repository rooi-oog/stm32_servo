#ifndef MAIN_HEADER_H
#define MAIN_HEADER_H

void usb_message_handler ();

extern void clock_setup (void);
extern void encoders_setup (void);
extern void pwm_setup (void);
extern void gpio_setup (void);
extern void usb_setup (void);
extern void uart_setup (void);
extern void systick_setup (void);

extern void cdcacm_write (char *, int);
extern int usb_read_nonblock (void);
extern void usb_read (char *);

#endif /* MAIN_HEADER_H */

