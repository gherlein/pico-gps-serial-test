#include "hardware/irq.h"
#include "hardware/uart.h"
#include "minmea.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "stdio.h"
#include "stdlib.h"

#define INDENT_SPACES "  "

#define UART_ID uart1
#define BAUD_RATE 9600
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

#define UART_TX_PIN 4
#define UART_RX_PIN 5

#define BUFLEN MINMEA_MAX_SENTENCE_LENGTH

static uint8_t buf[BUFLEN];
static uint8_t len = 0;

static uint8_t gga[BUFLEN];
static uint8_t gsa[BUFLEN];
static uint8_t gll[BUFLEN];
static uint8_t rmc[BUFLEN];
static uint8_t vtg[BUFLEN];

int parseGPS(uint8_t *line) {
  printf("%s", line);
  switch (minmea_sentence_id(line, false)) {
  case MINMEA_SENTENCE_RMC: {
    struct minmea_sentence_rmc frame;
    if (minmea_parse_rmc(&frame, line)) {
      printf(INDENT_SPACES
             "$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
             frame.latitude.value, frame.latitude.scale, frame.longitude.value,
             frame.longitude.scale, frame.speed.value, frame.speed.scale);
      printf(INDENT_SPACES "$xxRMC fixed-point coordinates and speed scaled "
                           "to three decimal places: (%d,%d) %d\n",
             minmea_rescale(&frame.latitude, 1000),
             minmea_rescale(&frame.longitude, 1000),
             minmea_rescale(&frame.speed, 1000));
      printf(INDENT_SPACES
             "$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
             minmea_tocoord(&frame.latitude), minmea_tocoord(&frame.longitude),
             minmea_tofloat(&frame.speed));
    } else {
      printf(INDENT_SPACES "$xxRMC sentence is not parsed\n");
    }
  } break;

  case MINMEA_SENTENCE_GGA: {
    struct minmea_sentence_gga frame;
    if (minmea_parse_gga(&frame, line)) {
      printf(INDENT_SPACES "$xxGGA: fix quality: %d\n", frame.fix_quality);
    } else {
      printf(INDENT_SPACES "$xxGGA sentence is not parsed\n");
    }
  } break;

  case MINMEA_SENTENCE_GST: {
    struct minmea_sentence_gst frame;
    if (minmea_parse_gst(&frame, line)) {
      printf(INDENT_SPACES "$xxGST: raw latitude,longitude and altitude "
                           "error deviation: (%d/%d,%d/%d,%d/%d)\n",
             frame.latitude_error_deviation.value,
             frame.latitude_error_deviation.scale,
             frame.longitude_error_deviation.value,
             frame.longitude_error_deviation.scale,
             frame.altitude_error_deviation.value,
             frame.altitude_error_deviation.scale);
      printf(
          INDENT_SPACES
          "$xxGST fixed point latitude,longitude and altitude error deviation"
          " scaled to one decimal place: (%d,%d,%d)\n",
          minmea_rescale(&frame.latitude_error_deviation, 10),
          minmea_rescale(&frame.longitude_error_deviation, 10),
          minmea_rescale(&frame.altitude_error_deviation, 10));
      printf(INDENT_SPACES "$xxGST floating point degree latitude, longitude "
                           "and altitude error deviation: (%f,%f,%f)",
             minmea_tofloat(&frame.latitude_error_deviation),
             minmea_tofloat(&frame.longitude_error_deviation),
             minmea_tofloat(&frame.altitude_error_deviation));
    } else {
      printf(INDENT_SPACES "$xxGST sentence is not parsed\n");
    }
  } break;

  case MINMEA_SENTENCE_GSV: {
    struct minmea_sentence_gsv frame;
    if (minmea_parse_gsv(&frame, line)) {
      printf(INDENT_SPACES "$xxGSV: message %d of %d\n", frame.msg_nr,
             frame.total_msgs);
      printf(INDENT_SPACES "$xxGSV: satellites in view: %d\n",
             frame.total_sats);
      for (int i = 0; i < 4; i++)
        printf(INDENT_SPACES
               "$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
               frame.sats[i].nr, frame.sats[i].elevation, frame.sats[i].azimuth,
               frame.sats[i].snr);
    } else {
      printf(INDENT_SPACES "$xxGSV sentence is not parsed\n");
    }
  } break;

  case MINMEA_SENTENCE_VTG: {
    struct minmea_sentence_vtg frame;
    if (minmea_parse_vtg(&frame, line)) {
      printf(INDENT_SPACES "$xxVTG: true track degrees = %f\n",
             minmea_tofloat(&frame.true_track_degrees));
      printf(INDENT_SPACES "        magnetic track degrees = %f\n",
             minmea_tofloat(&frame.magnetic_track_degrees));
      printf(INDENT_SPACES "        speed knots = %f\n",
             minmea_tofloat(&frame.speed_knots));
      printf(INDENT_SPACES "        speed kph = %f\n",
             minmea_tofloat(&frame.speed_kph));
    } else {
      printf(INDENT_SPACES "$xxVTG sentence is not parsed\n");
    }
  } break;

  case MINMEA_SENTENCE_ZDA: {
    struct minmea_sentence_zda frame;
    if (minmea_parse_zda(&frame, line)) {
      printf(INDENT_SPACES "$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\n",
             frame.time.hours, frame.time.minutes, frame.time.seconds,
             frame.date.day, frame.date.month, frame.date.year,
             frame.hour_offset, frame.minute_offset);
    } else {
      printf(INDENT_SPACES "$xxZDA sentence is not parsed\n");
    }
  } break;

  case MINMEA_INVALID: {
    printf(INDENT_SPACES "$xxxxx sentence is not valid\n");
  } break;

  default: {
    printf(INDENT_SPACES "$xxxxx sentence is not parsed\n");
  } break;
  }

  return 0;
}

// RX interrupt handler
void on_uart_rx() {
  uint8_t ch = 0;

  while (uart_is_readable(UART_ID)) {

    ch = uart_getc(UART_ID);
    if (ch == '$') {
      buf[0] = ch;
      len++;
      continue;
    }
    if (len > 0) {
      buf[len] = ch;
      len++;
    }
    if (ch == '\n') {
      // end of string
      for (int x = 0; x < len; x++) {
        // uart_putc(uart0, buf[x]);
      }
      parseGPS(buf);
      memset(buf, 0x00, BUFLEN);
      len = 0;
    }
  }
}

int main() {

  memset(buf, 0x00, BUFLEN);
  memset(gga, 0x00, BUFLEN);
  memset(gsa, 0x00, BUFLEN);
  memset(gll, 0x00, BUFLEN);
  memset(rmc, 0x00, BUFLEN);
  memset(vtg, 0x00, BUFLEN);
  len = 0;

  stdio_init_all();

  if (cyw43_arch_init()) {
    printf("Wi-Fi init failed");
    return -1;
  }

  // Set up our UART with a basic baud rate.
  uart_init(UART_ID, 9600);

  // Set the TX and RX pins by using the function select on the GPIO
  // Set datasheet for more information on function select
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

  // Actually, we want a different speed
  // The call will return the actual baud rate selected, which will be as
  // close as possible to that requested
  int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

  // Set UART flow control CTS/RTS, we don't want these, so turn them off
  uart_set_hw_flow(UART_ID, false, false);

  // Set our data format
  uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

  // Turn off FIFO's - we want to do this character by character
  uart_set_fifo_enabled(UART_ID, false);

  // Set up a RX interrupt
  // We need to set up the handler first
  // Select correct interrupt for the UART we are using
  int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

  // And set up and enable the interrupt handlers
  irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
  irq_set_enabled(UART_IRQ, true);

  // Now enable the UART to send interrupts - RX only
  uart_set_irq_enables(UART_ID, true, false);

  while (1) {
    // tight_loop_contents();
    sleep_ms(100);
  }
}
