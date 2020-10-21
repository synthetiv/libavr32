#ifndef _I2C_H_
#define _I2C_H_

#include <stdint.h>

// PPP framing
#define STX 12
#define ETX 13
#define DLE 27

#define I2C_TX_BUF_SIZE 8
#define I2C_RX_BUF_SIZE 8

#define I2C_READY 0
#define I2C_NO_TXCOMP -10
#define I2C_LINES_LOW -11

extern void i2c_follow(void);
extern int16_t i2c_read_status(void);
extern int16_t i2c_read_interrupts(void);
extern int16_t i2c_get_it_mask(void);

extern int i2c_leader_tx(uint8_t addr, uint8_t *data, uint8_t l);
extern int i2c_leader_rx(uint8_t addr, uint8_t *data, uint8_t l);

extern int i2c_cooperator_tx(uint8_t addr, uint8_t *data, uint8_t l);
extern int i2c_cooperator_rx(uint8_t addr, uint8_t *data, uint8_t l);

extern void twi_follower_rx(uint8_t u8_value);
extern uint8_t twi_follower_tx(void);
extern void twi_follower_stop(void);

extern void ii_tx_queue(uint8_t);

typedef void (*process_ii_t)(uint8_t *d, uint8_t l);
extern volatile process_ii_t process_ii;

extern bool i2c_has_txcomp(void);
extern bool i2c_check_silence(void);
bool i2c_wait_for_txcomp(void);
bool i2c_wait_for_silence(void);

void i2c_cooperator_init(uint8_t addr);
void i2c_done(void);

#endif // header guard
