#include "icm_logic.h"

/* ======== dados compartilhados (protegidos por ICM_Resource no app) ======== */
volatile ICM_Data_t ICM_Data = {0, 0, 0, 0, 0};
/* timeout por mensagem: idade desde o ultimo frame de cada ID (ciclos de 100 ms) */
volatile uint16_t age_m1 = 9999U, age_m2 = 9999U, age_m3 = 9999U, age_m4 = 9999U;
/* flags de validade (0 = timeout/invalido) */
volatile uint8_t valid_m1 = 0U, valid_m2 = 0U, valid_m3 = 0U, valid_m4 = 0U;
/* frames descartados por fora de faixa */
volatile uint32_t range_drops = 0U;
/* 1 = ultimo valor estava fora de faixa */
volatile uint8_t range_err = 0U;

void icm_logic_reset(void)
{
  ICM_Data.rpm = 0U;
  ICM_Data.speed_x100 = 0U;
  ICM_Data.fuel_x2 = 0U;
  ICM_Data.temp = 0;
  ICM_Data.warn = 0U;
  age_m1 = 9999U; age_m2 = 9999U; age_m3 = 9999U; age_m4 = 9999U;
  valid_m1 = 0U; valid_m2 = 0U; valid_m3 = 0U; valid_m4 = 0U;
  range_drops = 0U;
  range_err = 0U;
}

/* ======== decodifica frame em ICM_Data (ICM-CAN-02/03/04/06) ======== */
void icm_decode(uint16_t id, const uint8_t *d)
{
  switch (id)
  {
  case 0x100: /* EngineSpeed: 0-8000 rpm */
    {
      uint16_t v = (uint16_t)(d[0] | (d[1] << 8));
      if (v <= 8000U) { ICM_Data.rpm = v; age_m1 = 0; range_err = 0; }
      else { range_drops++; range_err = 1; }  /* fora de faixa: descarta */
    }
    break;
  case 0x200: /* VehicleSpeed: 0-250 km/h (valor*100) -> 0-25000 */
    {
      uint16_t v = (uint16_t)(d[0] | (d[1] << 8));
      if (v <= 25000U) { ICM_Data.speed_x100 = v; age_m2 = 0; range_err = 0; }
      else { range_drops++; range_err = 1; }
    }
    break;
  case 0x300: /* FuelLevel 0-100% (valor*2 -> 0-200); CoolantTemp -40..150 */
    {
      uint8_t  f = d[0];
      int16_t  t = (int16_t)d[1] - 40;
      uint8_t ok = 1U;
      if (f > 200U) ok = 0U;                 /* fuel fora */
      if ((t < -40) || (t > 150)) ok = 0U;   /* temp fora */
      if (ok) { ICM_Data.fuel_x2 = f; ICM_Data.temp = t; age_m3 = 0; range_err = 0; }
      else { range_drops++; range_err = 1; }
    }
    break;
  case 0x400: /* WarningFlags: bitfield, sem range */
    ICM_Data.warn = d[0];
    age_m4 = 0;
    break;
  default:
    break;
  }
}

/* ======== ciclo de diagnostico de 100 ms (ICM-CAN-05) ======== */
void icm_diag_tick(void)
{
  /* envelhece (cada ciclo = 100ms); timeout = 3x periodo */
  if (age_m1 < 9999U) age_m1++;
  if (age_m2 < 9999U) age_m2++;
  if (age_m3 < 9999U) age_m3++;
  if (age_m4 < 9999U) age_m4++;

  /* M1 periodo 500ms -> timeout 1500ms = 15 ciclos; M2/M3/M4 1000ms -> 30 ciclos */
  valid_m1 = (age_m1 <= 15U) ? 1U : 0U;
  valid_m2 = (age_m2 <= 30U) ? 1U : 0U;
  valid_m3 = (age_m3 <= 30U) ? 1U : 0U;
  valid_m4 = (age_m4 <= 30U) ? 1U : 0U;
}