// Библиотека парсинга протокола NMEA.
// Версия: 1.1.2
// Последнюю версию библиотеки Вы можете скачать по ссылке: https://iarduino.ru/file/538.html
// Подробное описание функций бибилиотеки доступно по ссылке: https://wiki.iarduino.ru/page/NMEA-protocol-parser/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include "gps_common.h"
#include "gps_nmea.h"
#include "types.h"

#include <ctype.h>
#include <string.h>


const char* const GPS_REPLY_HEADER = "gps";

bool (*nmea_response)(const char*, uint32_t, QueueHandle_t *q) = NULL;


// Example: GNRMC,094220.100,A,5952.42338,N,03020.27622,E,0.00,0.00,180526,,,A,V
typedef struct
{
	uint32_t m_utcInMs;
	double m_latitude; // decimal representation (dd.dddddddd), sign = (S: '-', N: '+')
	double m_longitude; // decimal representation (ddd.dddddddd), sign = (W: '-', E: '+')
	double m_speed; // in km/h
} Nmea_RMC;

// local
static bool _process_nmea_RMC(const uint8_t* data, const uint16_t dataLength, Nmea_RMC* msgData)
{
	int field = 0;
	for (uint16_t i = 0; i < dataLength; ++i) {
		const char* const ptr = (const char*)&data[i];

		switch (field) {
		case 0: { // (0) Время (ЧЧММСС.ССС)
			const double utc = atof(ptr);

			const uint32_t utc_sec = (uint32_t)utc;
			const uint32_t ms = (uint32_t)(utc * 1000.) - utc_sec * 1000;

			const uint32_t sec = utc_sec % 100;
			const uint32_t min = (uint32_t)(utc_sec / 100.) % 100;
			const uint32_t hour = (uint32_t)(utc_sec / 10000.) % 100;

			msgData->m_utcInMs = ((hour * 60 + min) * 60 + sec) * 1000 + ms;
			break;
		}

		case 1: { // (1) Достоверность полученных координат ('A'-данные достоверны (OK), 'V'-ошибочные данные)
			if (toupper(*ptr) != 'A') {
				return false; // sentence does not contain valid data
			}
			break;
		}

		case 2: { // (2) Координаты по широте (ГГММ.МММММ)
			const double lat = atof(ptr);
			// convert to decimal representation (dd.dddddddd)
			const double deg = (int)(lat / 100.);
			msgData->m_latitude = deg + (lat - deg * 100.) / 60.;
			break;
		}

		case 3: { // (3) Направление широты (N-север, S-юг)
			if (toupper(*ptr) == 'S') {
				msgData->m_latitude = -msgData->m_latitude;
			}
			break;
		}

		case 4: { // (4) Координаты по долготе (ГГГММ.МММММ)
			const double lon = atof(ptr);
			// convert to decimal representation (ddd.dddddddd)
			const double deg = (int)(lon / 100.);
			msgData->m_longitude = deg + (lon - deg * 100.) / 60.;
			break;
		}

		case 5: { // (5) Направление долготы (E-восток, W-запад)
			if (toupper(*ptr) == 'W') {
				msgData->m_longitude = -msgData->m_longitude;
			}
			break;
		}

		case 6: { // (6) Скорость переведя её из узлов в км/ч
			msgData->m_speed = atoi(ptr) * 1.851999999999051776; // convertion to km/h
			break;
		}

		// (7)  Курс на истинный полюс в градусах (XX.XX)
		// (8)  Дата (ДДММГГ)
		// (9)  Магнитное склонение в градусах
		// (10) Направление магнитного склонения (E-вычесть / W-прибавить к истинному курсу)
		// (11) Способ вычисления координат ('A'-автономный (OK), 'D'-дифференциальный, 'E'-аппроксимация, 'M'-фиксированные данные, 'N'-недостоверные данные)
		// (12) Статус навигации (X)
		case 7: {
			return true; // OK, got all data
		}

		} // switch (field)

		for (; i < dataLength && data[i] != '\0'; ++i) { // goes to the next field
		}
		field += 1;
	}

	return false; // unexpected
}

// local
static void _calc_checksum(const char* str, const uint16_t strLength, char checksum[3])
{
	const uint16_t len = (strLength == 0 ? (uint16_t)strlen(str) : strLength);
	uint8_t cs = 0;
	for (uint8_t i = 0; i < len && str[i] != '\0'; ++i) {
		cs ^= str[i];
	}

	checksum[0] = '*';
	checksum[1] = (cs / 16) + '0'; // 1 байт контрольной суммы
	if (checksum[1] > '9') {
		checksum[1] += 7;
	}
	checksum[2] = (cs % 16) + '0'; // 2 байт контрольной суммы
	if (checksum[2] > '9') {
		checksum[2] += 7;
	}
}


// API
void gps_nmea_init(bool (*send_to_host)(const char*, uint32_t, QueueHandle_t *q))
{
	nmea_response = send_to_host;
}

// API
void gps_nmea_read_sentence(
	const uint32_t timeoutMs,
	bool (*process_nmea_sentence)(const uint8_t* data, const uint16_t dataLength, QueueHandle_t* q),
	QueueHandle_t* (*tx_queue)(),
	const bool debugUnknowSentences
)
{
	// Максимальная длина одного сообщения (предложения) NMEA 0183 (версии 3.0 и выше) ограничена 83 символами.
	// Сообщение не должно превышать 80 байт данных, плюс символы начала ($) и конца (<CR><LF>), что в сумме дает 83 символа.
	uint8_t nmeaSentence[85];
	uint16_t sentencePos = 0;
	uint16_t checksumPos = 0;

	bool startFound = false;
	bool endFound = false;
	uint8_t nextByte = 0;

	gps_serial_flush();

	const int64_t finishTimeMks = esp_timer_get_time() + timeoutMs*1000LL;
	while (true) {
		if (esp_timer_get_time() > finishTimeMks) {
			break; // timeout
		}
		if (!gps_serial_get_byte(&nextByte)) {
			vTaskDelay(pdMS_TO_TICKS(10));
			continue;
		}
		
		if (startFound) {
			if (nextByte == '\r' || nextByte == '\n') { // '\r' or '\n' terminates NMEA sentence
				nmeaSentence[sentencePos] = '\0';
				endFound = true;
				break;
			} else if (nextByte >= 0x20 && nextByte <= 0x7E) { // Все символы тела сообщения должны быть из диапазона печатных ASCII-символов
				if (nextByte == '*') { // checksum delimiter
					checksumPos = sentencePos;
				}
				nmeaSentence[sentencePos] = nextByte;
				sentencePos += 1;

				if (sentencePos >= sizeof(nmeaSentence)) {
					break; // overrun
				}
			} else {
				// simply skip
			}
		} else if (nextByte == '$' || nextByte == '!') { // starting delimeters are only '$' or '!'
			startFound = true;
		} else {
			// simply skip
		}
	}

	if (!endFound || sentencePos < 5) {
		return; // NMEA sentence was not found
	}

	// calculate checksum (it is optional, so check only if present)
	if (sentencePos - checksumPos == 3) {
		uint8_t calculatedChecksum = 0;
		for (size_t i = 0; i < checksumPos; ++i) {
			calculatedChecksum ^= nmeaSentence[i];
		}

		const uint8_t originalChecksum = strtoul((const char*)&nmeaSentence[checksumPos + 1], NULL, 16);

		if (originalChecksum != calculatedChecksum) {
			return; // checksum mismatch
		}

		nmeaSentence[checksumPos] = '\0';
		sentencePos -= 3;
	}

	// process NMEA sentence (splitted by ',')
	for (uint16_t i = 0; i < sentencePos; ++i) {
		if (nmeaSentence[i] == ',') {
			nmeaSentence[i] = '\0';
		}
	}
	QueueHandle_t* q = tx_queue();
	if (q) {
		const bool processed = process_nmea_sentence(nmeaSentence, sentencePos, q);
		if (!processed && sentencePos > 5 && debugUnknowSentences) {
			gps_nmea_debug_process_sentence(nmeaSentence, sentencePos, q);
		}
	}
}

// API (for debugging)
bool gps_nmea_debug_process_sentence(const uint8_t* data, const uint16_t dataLength, QueueHandle_t* q)
{
	for (uint16_t i = 0; i < dataLength; ++i) { // un-split sentence fields
		if (data[i] == '\0') {
			((char*)data)[i] = ',';
		}
	}

	nmea_response(GPS_REPLY_HEADER, 0, q);
	nmea_response("_debug_", 0, q);
	nmea_response((const char*)data, dataLength, q);
	nmea_response("\r", 0, q);

	return true;
}

// API
bool gps_nmea_process_sentence(const uint8_t* data, const uint16_t dataLength, QueueHandle_t* q)
{
	// NMEA sentence start with 5-letter message identifier. 
	// The first two letters identify the message source, and the next three letters identify the message format, according to the specific version of the NMEA 0183 protocol.
	if (dataLength < 6) {
		return false;
	}

	if (toupper(data[2]) == 'R' && toupper(data[3]) == 'M' && toupper(data[4]) == 'C') {
		Nmea_RMC msgData;
		if (_process_nmea_RMC(data + 6, dataLength - 6, &msgData)) {
			// serialize
			char buff[DEV_BUFFER_LENGTH];
			snprintf(buff, sizeof(buff), "%sRMC_%c%c_%u_%.8g_%.8g_%.3g\r",
				GPS_REPLY_HEADER,
				data[0], data[1], 
				(unsigned int)msgData.m_utcInMs,
				msgData.m_latitude,  // 8 decimal points are sub-centimeter surveyor level
				msgData.m_longitude, // 8 decimal points are sub-centimeter surveyor level
				msgData.m_speed
			);
			nmea_response(buff, 0, q);
			return true;
		}
	}
	return false;
}

// API
void gps_nmea_send_commands(const char* commands, QueueHandle_t* q)
{
	bool fails = false;

	for (uint16_t i = 0, in = (uint16_t)strlen(commands); i < in; ) {
		uint16_t endPos = i;
		uint16_t asteriskPos = 0;
		for (; endPos < in && commands[endPos] != '\r'; ++endPos) {
			if (commands[endPos] == '*') {
				asteriskPos = endPos;
			}
		}

		if (endPos > i + 1) {
			if (asteriskPos != 0 && endPos - asteriskPos == 3) {
				if (!gps_serial_send_buffer(&commands[i], endPos - i)
					|| !gps_serial_send_buffer("\r\n", 0)
					) {
					fails = true;
					break;
				} else {
					if (q) {
						nmea_response(GPS_REPLY_HEADER, 0, q);
						nmea_response(&commands[i], endPos - i + 1, q); // including separator ('\r')
					}
				}
			}
			else {
				char checksum[3];
				_calc_checksum(&commands[i + 1], endPos - i - 1, checksum);

				if (!gps_serial_send_buffer(&commands[i], endPos - i)
					|| !gps_serial_send_buffer(checksum, 3)
					|| !gps_serial_send_buffer("\r\n", 0)
					) {
					fails = true;
					break;
				} else {
					if (q) {
						nmea_response(GPS_REPLY_HEADER, 0, q);
						nmea_response(&commands[i], endPos - i, q);
						nmea_response(checksum, 3, q);
						nmea_response("\r", 0, q);
					}
				}
			}
			vTaskDelay(pdMS_TO_TICKS(1));
		}

		i = endPos + 1;
	}

	if (fails) {
		nmea_response(GPS_REPLY_HEADER, 0, q);
		nmea_response("_fail_init\r", 0, q);
	}
}
