#ifndef ICM_LOGIC_H
#define ICM_LOGIC_H

/*
 * Logica pura do ICM (sem hardware e sem OS): decodificacao CAN,
 * verificacao de faixa e timeout por mensagem.
 * Compila para o STM32 (app) e para o host (Google Test).
 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint16_t rpm;        /* M1 EngineSpeed */
  uint16_t speed_x100; /* M2 VehicleSpeed * 100 (0.01 km/h) */
  uint8_t  fuel_x2;    /* M3 FuelLevel * 2 (0.5%) */
  int16_t  temp;       /* M3 CoolantTemp (offset -40) */
  uint8_t  warn;       /* M4 WarningFlags */
} ICM_Data_t;

extern volatile ICM_Data_t ICM_Data;
extern volatile uint16_t age_m1, age_m2, age_m3, age_m4;
extern volatile uint8_t  valid_m1, valid_m2, valid_m3, valid_m4;
extern volatile uint32_t range_drops;
extern volatile uint8_t  range_err;

/* Decodifica um frame CAN em ICM_Data (ICM-CAN-02/03/04/06). */
void icm_decode(uint16_t id, const uint8_t *d);

/* Um ciclo de diagnostico de 100 ms: envelhece e calcula validade (ICM-CAN-05). */
void icm_diag_tick(void);

/* Volta ao estado de power-on (usado pelos testes). */
void icm_logic_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* ICM_LOGIC_H */