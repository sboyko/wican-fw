#ifndef _MICRO_ECC_H_
#define _MICRO_ECC_H_

#include <stdint.h>

/* Curve selection options. */
#define secp256r1 32

#define NUM_ECC_DIGITS secp256r1

typedef struct EccPoint
{
    uint8_t x[NUM_ECC_DIGITS];
    uint8_t y[NUM_ECC_DIGITS];
} EccPoint;

/* Функция ecc_make_key().
создает пару открытый/закрытый ключи.
Вы должны использовать новое случайное число для генерации каждой новой пары ключей.

Выходные параметры:
    p_publicKey  - Будет заполнен значениями точки, представляющей открытый ключ.
    p_privateKey - Будет заполнен значением закрытого ключа.

Входные параметры:
    p_random - Случайное число, используемое для генерации пары ключей.

Возвращает 1 если пара ключей была сгенерирована успешно, 0 если произошла ошибка. Если возвращается 0, попробуйте с другим случайным числом. */
int ecc_make_key(EccPoint *p_publicKey, uint8_t p_privateKey[NUM_ECC_DIGITS], const uint8_t p_random[NUM_ECC_DIGITS]);


/* Функция ecc_valid_public_key().
Определяет принадлежит ли заданная точка выбранной эллиптической кривой (то есть, является ли корректным открытым ключом).

Входные параметры:
    p_publicKey - Точка для проверки.

Возвращает 1 если данная точка корректна, 0 если некорректна. */
int ecc_valid_public_key(const EccPoint *p_publicKey);


/* Функция ecdh_shared_secret().
Вычисляет общий секретный ключ, используя ваш закрытый и чей-либо открытый.

При желании, вы можете указать случайный множитель для устойчивости к DPA-атакам. Случайный множитель должен быть разным для каждого вызова ecdh_shared_secret().

Выходные параметры:
    p_secret - Будет заполнен значением общего секретного ключа.
    
Входные параметры:
    p_publicKey  - Открытый ключ удаленного участника.
    p_privateKey - Ваш закрытый ключ.
    p_random     - Необязательное услучайное число для устойчивости к DPA-атакам. Передайте NULL если DPA-атаки не вызывают беспокойства.

Возвращает 1 если общий секретный ключ сгенерирован успешно, 0 в противном случае.

Примечание: рекомендуется проверить хэш результата функции ecdh_shared_secret, прежде чем использовать симметричное шифрование или HMAC.
Если у вас нет хэша общего секретного ключа, то вы должны вызвать функцию ecc_valid_public_key() для проверки того, что открытый ключ удаленной стороны корректен.
Если этого не сделать, то злоумышленник может создать открытый ключ для использования с вами общего секретного ключа, что приведет к утечке информации о вашем закрытом ключе. */
int ecdh_shared_secret(uint8_t p_secret[NUM_ECC_DIGITS], const EccPoint *p_publicKey, const uint8_t p_privateKey[NUM_ECC_DIGITS], const uint8_t p_random[NUM_ECC_DIGITS]);


/* Функция ecdsa_sign().
Генерирует подпись ECDSA подпись для заданного значения хэш-функции.
Применение: вычислите хэш для данных, которые хотите подписать (рекомендуется SHA-2) и передайте в функцию вместе с закрытым ключом и случайным числом.
Вы должны использовать новое случайное число для каждой новой подписи.

Выходные параметры:
    r, s - Будут заполнены значениями подписи.

Входные параметры:
    p_privateKey - Ваш закрытый ключ.
    p_random     - Случайное число, используемое для создания подписи.
    p_hash       - Хэш сообщения для подписи.

Возвращает 1 если подпись сгенерирована успешно, 0 если произошла ошибка. Если возвращается 0, попробуйте с другим случайным числом. */
int ecdsa_sign(uint8_t r[NUM_ECC_DIGITS], uint8_t s[NUM_ECC_DIGITS], const uint8_t p_privateKey[NUM_ECC_DIGITS],
    const uint8_t p_random[NUM_ECC_DIGITS], const uint8_t p_hash[NUM_ECC_DIGITS]);


/* Функция ecdsa_verify().
Проверка подписи ECDSA.

Применение: Compute the hash of the signed data using the same hash as the signer and pass it to this function along with the signer's public key and the signature values (r and s).

Inputs:
    p_publicKey - The signer's public key
    p_hash      - The hash of the signed data.
    r, s        - The signature values.

Возвращает 1 if the signature is valid, 0 if it is invalid.
*/
int ecdsa_verify(const EccPoint *p_publicKey, const uint8_t p_hash[NUM_ECC_DIGITS], const uint8_t r[NUM_ECC_DIGITS], const uint8_t s[NUM_ECC_DIGITS]);


/* ecc_bytes2native() function.
Convert an integer in standard octet representation to the native format.

Outputs:
    p_native - Will be filled in with the native integer value.

Inputs:
    p_bytes - The standard octet representation of the integer to convert.
*/
void ecc_bytes2native(uint8_t p_native[NUM_ECC_DIGITS], const uint8_t p_bytes[NUM_ECC_DIGITS]);

/* ecc_native2bytes() function.
Convert an integer in native format to the standard octet representation.

Outputs:
    p_bytes - Will be filled in with the standard octet representation of the integer.

Inputs:
    p_native - The native integer value to convert.
*/
void ecc_native2bytes(uint8_t p_bytes[NUM_ECC_DIGITS], const uint8_t p_native[NUM_ECC_DIGITS]);


#endif /* _MICRO_ECC_H_ */