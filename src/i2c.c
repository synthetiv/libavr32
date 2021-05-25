#include <avr32/io.h>
#include "gpio.h"
#include "print_funcs.h"
#include "twi.h"
#include "events.h"
#include "i2c.h"

#define RX_BUFFER_COUNT 8

uint8_t rx_buffer[RX_BUFFER_COUNT][I2C_RX_BUF_SIZE];
static uint8_t rx_pos[RX_BUFFER_COUNT];
static uint8_t rx_buffer_index;

uint8_t tx_buffer[I2C_TX_BUF_SIZE];
static uint8_t tx_pos_read;
static uint8_t tx_pos_write;

static twi_package_t packet;
// static twi_package_t packet_received;

static volatile avr32_twi_t *twi = TWI;
static uint8_t i2c_follower_address;

static volatile bool i2c_leading = false;
static volatile bool i2c_nack = false;
static volatile bool i2c_arblst = false;
static volatile bool i2c_txcomp = true;

// interrupt mask
static volatile unsigned long i2c_it_mask;
static const unsigned char *volatile i2c_tx_data = NULL;
static volatile unsigned char *volatile i2c_rx_data = NULL; // TODO: const vs. volatile? why?
static volatile int i2c_tx_nb_bytes = 0;
static volatile int i2c_rx_nb_bytes = 0;

volatile process_ii_t process_ii;

#define I2C_BUSY_COUNTER 10000
static uint16_t busy_limiter;

#define I2C_QUIET_CHECKS 4
static uint16_t quiet_countdown;

int16_t i2c_read_status(void) {
  return twi->sr;
}

int16_t i2c_read_interrupts(void) {
  return twi->imr;
}

int16_t i2c_get_it_mask(void) {
  return i2c_it_mask;
}


int i2c_leader_tx(uint8_t addr, uint8_t *data, uint8_t length) {
  int status;

  packet.chip = addr;
  packet.addr_length = 0;
  // Where to find the data to be written
  packet.buffer = data;
  // How many bytes do we want to write
  packet.length = length;

  // perform a write access
  status = twi_master_write(TWI, &packet);

  return status;
}


int i2c_leader_rx(uint8_t addr, uint8_t *data, uint8_t l) {
  int status;

  packet.chip = addr;
  packet.addr_length = 0;
  // Where to find the data to be written
  packet.buffer = data;
  // How many bytes do we want to write
  packet.length = l;

  status = twi_master_read(TWI, &packet);

  return status;
}


int i2c_cooperator_tx(uint8_t addr, uint8_t *data, uint8_t length) {

  if ( ! i2c_wait_for_txcomp()) return I2C_NO_TXCOMP;
  if ( ! i2c_wait_for_silence()) return I2C_LINES_LOW;

  // reset flags
  i2c_leading = true;
  i2c_nack = false;
  i2c_arblst = false;
  i2c_txcomp = false;

  // switch hardware to leader mode
  // NOTE: the order in which this and mmr are set may be important (??)
  twi->cr = AVR32_TWI_CR_SVDIS_MASK;
  twi->cr = AVR32_TWI_CR_MSEN_MASK;

  // set read mode and follower address (internal address unused since all II
  // addresses are 1 byte)
  twi->mmr = (addr << AVR32_TWI_MMR_DADR_OFFSET) |
    (0 << AVR32_TWI_MMR_MREAD_OFFSET);

  // get a pointer to data
  i2c_tx_data = data;

  // get a copy of the number of bytes to read
  i2c_tx_nb_bytes = length;

  // load the first byte into the Transmit Holding Register
  twi->thr = *i2c_tx_data++;

  // enable interrupts
  twi->idr = ~0UL;
  i2c_it_mask = AVR32_TWI_IER_NACK_MASK | AVR32_TWI_IER_ARBLST_MASK | AVR32_TWI_IER_TXRDY_MASK;
  twi->ier = i2c_it_mask;

  twi->sr;

  // wait until data is sent
  if ( ! i2c_wait_for_txcomp()) return I2C_NO_TXCOMP - 100;

  i2c_follow();

  if (i2c_nack) return TWI_RECEIVE_NACK - 100;
  if (i2c_arblst) return TWI_ARBITRATION_LOST - 100;

  return TWI_SUCCESS;
}


int i2c_cooperator_rx(uint8_t addr, uint8_t *data, uint8_t length) {

  if ( ! i2c_wait_for_txcomp()) return I2C_NO_TXCOMP - 200;
  if ( ! i2c_wait_for_silence()) return I2C_LINES_LOW - 200;

  // reset flags
  i2c_leading = true;
  i2c_nack = false;
  i2c_arblst = false;
  i2c_txcomp = false;

  // set read mode and follower address (internal address unused since all II
  // addresses are 1 byte)
  twi->mmr = (addr << AVR32_TWI_MMR_DADR_OFFSET) |
    (1 << AVR32_TWI_MMR_MREAD_OFFSET);

  // get a pointer to data
  i2c_rx_data = data;

  // get a copy of the number of bytes to read
  i2c_rx_nb_bytes = length;

  // switch hardware to leader mode
  twi->cr = AVR32_TWI_CR_SVDIS_MASK;
  twi->cr = AVR32_TWI_CR_MSEN_MASK;

  // send start condition
  twi->cr = AVR32_TWI_START_MASK;

  // if we're only expecting one byte to receive, set stop condition too
  if (i2c_rx_nb_bytes == 1) {
    twi->cr = AVR32_TWI_STOP_MASK;
  }

  // enable interrupts
  twi->idr = ~0UL;
  i2c_it_mask = AVR32_TWI_IER_NACK_MASK | AVR32_TWI_IER_ARBLST_MASK | AVR32_TWI_IER_RXRDY_MASK;
  twi->ier = i2c_it_mask;

  twi->sr;

  // wait until data is sent
  if ( ! i2c_wait_for_txcomp()) return I2C_NO_TXCOMP - 100;

  i2c_follow();

  if (i2c_nack) return TWI_RECEIVE_NACK - 300;
  if (i2c_arblst) return TWI_ARBITRATION_LOST - 300;

  return TWI_SUCCESS;
}



void twi_follower_rx(uint8_t u8_value)
{
  if (rx_pos[rx_buffer_index] < I2C_RX_BUF_SIZE) {
    rx_buffer[rx_buffer_index][rx_pos[rx_buffer_index]] = u8_value;
    rx_pos[rx_buffer_index]++;
  }
}

void twi_follower_stop(void)
{
  uint8_t index = rx_buffer_index;
  // rx_buffer_index must be incremented before process_ii is called
  if (++rx_buffer_index >= RX_BUFFER_COUNT) rx_buffer_index = 0;

  process_ii(rx_buffer[index], rx_pos[index]);
  rx_pos[index] = 0;
}




extern void ii_tx_queue(uint8_t data) {
  // print_dbg("\r\nii_tx_queue ");
  tx_buffer[tx_pos_write] = data;
  // print_dbg_ulong(data);

  tx_pos_write = (tx_pos_write + 1) & 7;
  if(tx_pos_write == tx_pos_read)
    print_dbg("\r\nii queue overrun");
}

uint8_t twi_follower_tx(void)
{
   // print_dbg("\r\nii_tx ");
   if(tx_pos_write == tx_pos_read)
    return 27;
  else {
    uint8_t d = tx_buffer[tx_pos_read];
    tx_pos_read = (tx_pos_read + 1) & 7;
    // print_dbg_ulong(d);
    return d;
  }
}

bool i2c_check_silence(void) {

  if ( ! gpio_get_pin_value(AVR32_TWI_SDA_0_0_PIN) ||
       ! gpio_get_pin_value(AVR32_TWI_SCL_0_0_PIN)) {
    // one or both i2c lines are low; we'd better wait to use the bus
    quiet_countdown = I2C_QUIET_CHECKS;
    return false;
  }
  else if (quiet_countdown > 0 ) {
    quiet_countdown--;
    return false;
  }

  return true;
}

bool i2c_has_txcomp(void) {
  if (i2c_txcomp) {
    return true;
  } else {
    return false;
  }
}

bool i2c_wait_for_txcomp() {

  busy_limiter = I2C_BUSY_COUNTER;
  while (busy_limiter-- > 0 && ! i2c_has_txcomp()) {
    cpu_relax();
    if (busy_limiter == 0) return false;
  }

  return true;
}

bool i2c_wait_for_silence(void) {

  busy_limiter = I2C_BUSY_COUNTER;
  while (busy_limiter-- > 0 && ! i2c_check_silence()) {
    cpu_relax();
    if (busy_limiter == 0) return false;
  }

  return true;
}

void i2c_follow(void) {

  i2c_leading = false;

  // set follower device address
  twi->smr = (i2c_follower_address << AVR32_TWI_SMR_SADR_OFFSET);

  // switch hardware to follower mode
  twi->cr = AVR32_TWI_CR_MSDIS_MASK;
  twi->cr = AVR32_TWI_CR_SVEN_MASK;

  // enable SVACC (follower access) interrupt
  i2c_it_mask = AVR32_TWI_IER_SVACC_MASK;
  twi->ier = i2c_it_mask;
}

ISR(i2c_interrupt_handler, AVR32_TWI_IRQ_GROUP, CONF_TWI_IRQ_LEVEL) {

  // get status register value
  int status = twi->sr;

  i2c_txcomp = (status & AVR32_TWI_SR_TXCOMP_MASK);

  if (i2c_leading) {

    // NOTE: in theory, if either NACK and ARBLST gets set, TXCOMP does too
    if ( (status & AVR32_TWI_SR_NACK_MASK) && (i2c_it_mask & AVR32_TWI_IER_NACK_MASK) ) {

      // received no ACK for last data sent
      i2c_nack = true;
      i2c_done();
      return;
    }
    else if ( (status & AVR32_TWI_SR_ARBLST_MASK) && (i2c_it_mask & AVR32_TWI_IER_ARBLST_MASK) ) {

      // arbitration lost
      i2c_arblst = true;
      i2c_done();
      return;
    }
    else if ( (status & AVR32_TWI_SR_RXRDY_MASK) && (i2c_it_mask & AVR32_TWI_IER_RXRDY_MASK) ) {

      // we've received data from a follower
      // get data from receive holding register
      *i2c_rx_data = twi->rhr;
      i2c_rx_data++;
      // last byte to receive
      if (--i2c_rx_nb_bytes == 1) {
        // set stop bit
        twi->cr = AVR32_TWI_STOP_MASK;
      }
      // receive complete
      if (i2c_rx_nb_bytes == 0) {
        // disable other interrupts and wait for TXCOMP
        i2c_it_mask = AVR32_TWI_IER_TXCOMP_MASK;
        twi->idr = ~0UL;
        twi->ier = i2c_it_mask;
      }

      // dummy read, no idea if this helps
      // twi->sr;

      return;
    }
    else if ( (status & AVR32_TWI_SR_TXRDY_MASK) && (i2c_it_mask & AVR32_TWI_IER_TXRDY_MASK) ) {

      // ready to transmit data to a follower
      i2c_tx_nb_bytes--;
      if (i2c_tx_nb_bytes <= 0) {
        // last byte? disable other interrupts and wait for TXCOMP
        i2c_it_mask = AVR32_TWI_IER_TXCOMP_MASK;
        twi->idr = ~0UL;
        twi->ier = i2c_it_mask;
      }
      else {
        // load next byte to transmit to follower
        twi->thr = *i2c_tx_data++;
      }

      // dummy read, no idea if this helps
      twi->sr;

      return;
    }
    else if (i2c_txcomp && (i2c_it_mask & AVR32_TWI_IER_TXCOMP_MASK)) {

      // transmission complete
      i2c_leading = false;
      i2c_done();
      return;
    }
  }
  else {
    // follower mode

    if ( (status & AVR32_TWI_SR_EOSACC_MASK) && (i2c_it_mask & AVR32_TWI_IER_EOSACC_MASK) ) {

      // end of follower access
      // complete follower response process
      // NOTE: this must be working, otherwise script would never get triggered
      i2c_done();
      twi_follower_stop();
      return;
    }
    else if ( (status & AVR32_TWI_SR_SVACC_MASK) && (i2c_it_mask & AVR32_TWI_IER_SVACC_MASK) ) {

      // begin follower access; check read/write bit
      if (status & AVR32_TWI_SR_SVREAD_MASK) {
        // enable flag to signal data transmission
        i2c_it_mask = AVR32_TWI_IER_TXRDY_MASK; // 4
        twi->ier = i2c_it_mask;
        // load first byte to transmit to leader
        twi->thr = twi_follower_tx();
      } else {
        // enable interrupt for data reception
        i2c_it_mask = AVR32_TWI_IER_RXRDY_MASK | AVR32_TWI_IER_EOSACC_MASK; // 2050
        twi->ier = i2c_it_mask;
      }

      // dummy read, no idea if this helps
      // twi->sr;

      return;
    }
    else if ( (status & AVR32_TWI_SR_RXRDY_MASK) && (i2c_it_mask & AVR32_TWI_IER_RXRDY_MASK) ) {

      // we've received transmission from a leader; process it
      twi_follower_rx(twi->rhr);

      // dummy read, no idea if this helps
      // twi->sr;
      return;
    }
    else if ( (status & AVR32_TWI_SR_TXRDY_MASK) && (i2c_it_mask & AVR32_TWI_IER_TXRDY_MASK) ) {

      // ready to transmit data to a leader
      if (status & AVR32_TWI_SR_NACK_MASK) {
        // NACK from leader means we're done transmitting
        // clear NACK flag (actually looks like this just reads the receive holding reg...???)
        twi->rhr;
        // enable end of follower access interrupt
        i2c_it_mask = AVR32_TWI_IER_EOSACC_MASK;
        twi->ier = i2c_it_mask;
      } else {
        // load next byte to transmit to leader
        twi->thr = twi_follower_tx();
      }

      twi->sr;

      return;
    }
  }
}


void i2c_cooperator_init(uint8_t addr) {

  // pause interrupts
  irqflags_t flags = sysreg_read(AVR32_SR);

  // disable any/all TWI interrupts
  cpu_irq_disable();
  TWI->idr = ~0UL;
  TWI->sr;

  // reset TWI state
  TWI->cr = AVR32_TWI_CR_SWRST_MASK;
  cpu_irq_restore(flags);

  // clear internal address register (used for long follower addresses, which
  // we don't bother with)
  TWI->iadr = 0;

  // clear all clearable status register bits
  TWI->sr;

  // register our anarchistic interrupt handler
  flags = cpu_irq_save();
  irq_register_handler(&i2c_interrupt_handler, AVR32_TWI_IRQ, CONF_TWI_IRQ_LEVEL);
  cpu_irq_restore(flags);

  // set bus speed (used only by leader mode but we can set it once here and
  // not worry about it again)
  // clock calculations ripped from `twi_set_speed()`
  unsigned int ckdiv = 0;
  unsigned int c_lh_div;

  c_lh_div = FOSC0 / (TWI_SPEED * 2) - 4;

  // cldiv must fit in 8 bits, ckdiv must fit in 3 bits
  while ((c_lh_div > 0xFF) && (ckdiv < 0x7)) {
    // increase clock divider
    ckdiv++;

    // divide cldiv value
    c_lh_div /= 2;
  }

  // set clock waveform generator register
  TWI->cwgr = ((c_lh_div << AVR32_TWI_CWGR_CLDIV_OFFSET) |
    (c_lh_div << AVR32_TWI_CWGR_CHDIV_OFFSET) |
    (ckdiv << AVR32_TWI_CWGR_CKDIV_OFFSET));

  // set address (used by i2c_follow now and on subsequent calls)
  i2c_follower_address = addr;

  // program hardware registers for follower mode
  i2c_follow();
}


void i2c_done(void) { // TODO: inline?

  // disable all interrupts
  twi->idr = ~0UL;

  // dummy read (note: not used in default follower handler, why?)
  twi->sr;

  // re-enable follower access interrupt
  if (i2c_txcomp) {
    i2c_it_mask = AVR32_TWI_IER_SVACC_MASK;
  } else {
    // make sure we still catch TXCOMP if/when it arrives
    i2c_it_mask = AVR32_TWI_IER_SVACC_MASK | AVR32_TWI_IER_TXCOMP_MASK;
  }
  twi->ier = i2c_it_mask;
}
