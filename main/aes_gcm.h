#ifndef eas_gcm_h
#define eas_gcm_h

#include <stdint.h>
#include <mbedtls/gcm.h>


bool aes_gcm_init(mbedtls_gcm_context* gcm);

/**
 * Decrypt ciphertext using AES-256-GCM
 * Notes:
 *  'cipherdata' format is IV(12) + TAG(16) + DATA(cipherdata_len - 12 - 16)
 *  'plain_output' length is expected to be of DATA() size
 */
int aes_gcm_decrypt(mbedtls_gcm_context* gcm, const uint8_t* const cipherdata, const size_t cipherdata_len, uint8_t* const plain_output);

int aes_gcm_encrypt(mbedtls_gcm_context* gcm, const uint8_t* const plaindata, const size_t plaindata_len, uint8_t* const cipher_output);


#endif // eas_gcm_h