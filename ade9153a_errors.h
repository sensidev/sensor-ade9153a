#ifndef APP_ADE9153A_ERRORS_H
#define APP_ADE9153A_ERRORS_H


typedef enum ADE9153AStatus {
    ADE9153A_OK = 0x00u,
    ADE9153A_SPI_WRITE_FAIL_ERROR = 0x10u,
    ADE9153A_SPI_READ_FAIL_ERROR = 0x11u,

} ADE9153AStatus_t;

#endif
