#!/bin/bash

openssl dgst -sha256 -binary -out app.sha256 app.bin
openssl pkeyutl -sign -inkey secure_boot_signing_key.pem -keyform PEM -in app.sha256 -out app.signature
# Для склейки (если espsecure.py append_key тоже будет падать) может потребоваться исправление Python-окружения.
