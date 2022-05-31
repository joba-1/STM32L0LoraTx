#include <payload.h>

// checksum for payload
uint32_t payload_checksum(payload_t *p) {
  return p->magic + p->id +
#ifdef PAYLOAD_BME
      p->data.mVcc + p->data.mVbat + p->data.mpHumi + p->data.paPressure
      + p->data.ctCelsius;
#else
      p->data.mVaccu + p->data.mVbat + p->data.mVcc + p->data.dCelsius;
#endif
}

int payload_is_valid(payload_t *p) {
  return payload_checksum(p) == p->check;
}

void payload_make_valid(payload_t *p) {
#ifdef PAYLOAD_BME
  p->magic = PAYLOAD_MAGIC_BME;
#else
  p->magic = PAYLOAD_MAGIC_ADC;
#endif
  p->check = payload_checksum(p);
}
