#ifndef eas_gcm_h
#define eas_gcm_h

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
* Initialize AES-256-GCM context
*/
bool aes_gcm_init(const uint8_t* const cipherdata, const size_t cipherdata_len, const bool isExport, char* const response);

void aes_gcm_cleanup();

/**
 * Decrypt ciphertext using AES-256-GCM
 * Notes:
 *  'cipherdata' format is IV(12) + TAG(16) + DATA(cipherdata_len - 12 - 16)
 *  'plain_output' length is expected to be of DATA() size
 */
int aes_gcm_decrypt(const uint8_t* const cipherdata, const size_t cipherdata_len, uint8_t* const plain_output, char* const response);

int aes_gcm_encrypt(const uint8_t* const plaindata, const size_t plaindata_len, uint8_t* const cipher_output, char* const response);


#endif // eas_gcm_h