#!/bin/bash

# 1. Извлекаем публичные параметры RSA-ключа в формате DER
openssl rsa -in secure_boot_signing_key.pem -pubout -outform DER -out pubkey.der

# 2. Обрезаем заголовки ASN.1, оставляя чистый модуль RSA-3072 (384 байта)
# Примечание: Смещение зависит от формата, поэтому безопаснее использовать openssl asn1parse,
# но для стандартного RSA-3072 из openssl чистый модуль вытаскивается так:
openssl rsa -in secure_boot_signing_key.pem -pubout -text -noout | grep -A 24 "Modulus:" | grep -v "Modulus:" | tr -d ' \n:' | xxd -r -p > modulus.bin

# 3. Secure Boot V2 для ESP32-C3 требует хэш SHA-256 от структуры: 
# [Модуль RSA (384 байта)] + [Экспонента (4 байта, обычно 0x00010001)]
printf "\x00\x01\x00\x01" > exponent.bin
cat modulus.bin exponent.bin > public_key_block.bin

# 4. Хешируем полученный блок — это и есть ваш финальный digest.bin (32 байта)
openssl dgst -sha256 -binary public_key_block.bin > digest.bin

# Чистим временные файлы
rm pubkey.der modulus.bin exponent.bin public_key_block.bin
