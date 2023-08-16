#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void flow_init(void);
uint16_t flow_read_dtof(int32_t *dtof);

#ifdef __cplusplus
}
#endif