#include "BluetoothSerial.h"
#include <cmath>
#include <cstring>
#include <cstdlib>

// ==========================================
// ESP32 XBEE NATIVE LED PIN CONFIGURATION
// ==========================================
#define PIN_RED   21  
#define PIN_GREEN 22  
#define PIN_BLUE  23  

// Hardware operates on Common Anode (Active-Low)
#define LED_ON  LOW
#define LED_OFF HIGH

BluetoothSerial SerialBT;

// Structure to pass clean GNSS data from Core 1 to Core 0
struct FilteredData {
    char timeStr[12];
    char dateStr[8];
    double latitude;
    double longitude;
    double height;
    double heading;
    double vel_knots;
    int gpsQuality;
    int numSats;
};

QueueHandle_t gnssQueue = NULL;

// Function Prototypes
void bluetoothTask(void *pvParameters);
void convertNMEACoordinate(double decimalDegree, char* bufferOutput, size_t bufferSize, char* direction, bool isLatitude);
void configureLED();
void updateLED(int quality);
void calculateGPSDate(unsigned long gpsWeek, unsigned long weekSeconds, char* bufferOutput);

void setup() {
    // Primary UART for receiving high-rate data from UM981 (921600 bps)
    Serial.begin(921600, SERIAL_8N1);
    
    // Expanded serial buffer size to prevent hardware overruns at 50Hz/100Hz
    Serial.setRxBufferSize(16384); 
    
    configureLED();

    // Initialize Bluetooth Classic radio
    SerialBT.begin("RaceChrono_UM981");
    SerialBT.setTimeout(0);

    // Queue sized to 64 items to act as a solid buffer for 50 Hz streams
    gnssQueue = xQueueCreate(64, sizeof(FilteredData));
    if (gnssQueue == NULL) while(1);

    // Bluetooth broadcast task pinned to Core 0, keeping Core 1 free for fast UART parsing
    xTaskCreatePinnedToCore(bluetoothTask, "BTTask", 4096, NULL, 3, NULL, 0);
}

void loop() {
    static char lineBuffer[1024];
    static int lineIndex = 0;

    while (Serial.available() > 0) {
        char c = Serial.read();

        // Accumulate characters until newline
        if (c != '\r' && c != '\n' && lineIndex < 1023) {
            lineBuffer[lineIndex++] = c;
        } 
        else if (lineIndex > 0) {
            lineBuffer[lineIndex] = '\0';

            // Validate strict ASCII log preamble emitted by NovAtel/Unicore receivers
            if (lineBuffer[0] == '#' && strncmp(&lineBuffer[1], "INSPVAA", 7) == 0) {
                
                char* ptSemicolon = strchr(lineBuffer, ';');
                if (ptSemicolon != NULL) {
                    *ptSemicolon = '\0';
                    char* messageBody = ptSemicolon + 1;

                    // Tokenize NovAtel Header (Fields before ';')
                    char* headerColumns[15];
                    int headerCount = 0;
                    char* ptr = lineBuffer;
                    headerColumns[headerCount++] = ptr;
                    while (*ptr != '\0' && headerCount < 14) {
                        if (*ptr == ',') {
                            *ptr = '\0';
                            headerColumns[headerCount++] = ptr + 1;
                        }
                        ptr++;
                    }

                    // Tokenize INSPVAA Message Body (Fields after ';')
                    char* bodyColumns[30];
                    int bodyCount = 0;
                    ptr = messageBody;
                    bodyColumns[bodyCount++] = ptr;
                    while (*ptr != '\0' && bodyCount < 28) {
                        if (*ptr == ',' || *ptr == '*') {
                            *ptr = '\0';
                            bodyColumns[bodyCount++] = ptr + 1;
                        }
                        ptr++;
                    }

                    // Structural validation of parsed headers and body tokens
                    if (bodyCount >= 12 && headerCount >= 9) {
                        
                        char* insStatus = bodyColumns[11]; 

                        // Only process valid inertial solutions
                        if (strcmp(insStatus, "INS_SOLUTION_GOOD") == 0 || 
                            strcmp(insStatus, "INS_HIGH_VARIANCE") == 0 ||
                            strcmp(insStatus, "INS_SOLUTION_FREE") == 0) {
                            
                            FilteredData data;
                            std::memset(&data, 0, sizeof(FilteredData));

                            // Extract native decimal coordinates from INSPVAA
                            data.latitude  = strtod(bodyColumns[2], NULL);                        
                            data.longitude = strtod(bodyColumns[3], NULL); 
                            data.height    = strtod(bodyColumns[4], NULL); 
                            
                            double vel_N    = strtod(bodyColumns[5], NULL); 
                            double vel_E    = strtod(bodyColumns[6], NULL); 

                            // ================================================================
                            // TIME PROCESSING: Precise Mathematical Conversion
                            // ================================================================
                            unsigned long gpsWeek = strtoul(headerColumns[5], NULL, 10);
                            double secWeek = strtod(headerColumns[6], NULL);

                            // Correcting GPS time to UTC (Leap Seconds subtraction = 18s)
                            double secWeekUTC = secWeek - 18.0;

                            if (secWeekUTC < 0) {
                                secWeekUTC += 604800.0;
                                if (gpsWeek > 0) gpsWeek--;
                            }

                            unsigned long daySeconds = (unsigned long)secWeekUTC % 86400;

                            unsigned int hourUTC   = daySeconds / 3600;
                            unsigned int minuteUTC = (daySeconds % 3600) / 60;
                            unsigned int secondUTC = daySeconds % 60;

                            // High frequency millisecond rounding mechanism
                            double rMillis = (secWeekUTC - (unsigned long)secWeekUTC) * 1000.0;
                            unsigned long restoMillis = (unsigned long)(rMillis + 0.5);
                            if (restoMillis >= 1000) {
                                restoMillis = 0;
                                secondUTC++;
                                if (secondUTC >= 60) { secondUTC = 0; minuteUTC++; }
                            }

                            snprintf(data.timeStr, sizeof(data.timeStr), "%02u%02u%02u.%03lu", 
                                     hourUTC, minuteUTC, secondUTC, restoMillis);

                            calculateGPSDate(gpsWeek, (unsigned long)secWeekUTC, data.dateStr);

                            // ================================================================
                            // ATTITUDE, SPEED AND SATELLITES CORRELATION
                            // ================================================================
                            data.heading = strtod(bodyColumns[10], NULL);
                            data.numSats = atoi(headerColumns[9]);

                            // Map INS status to NMEA GPS quality indicators expected by apps
                            if (strcmp(insStatus, "INS_SOLUTION_GOOD") == 0) {
                                data.gpsQuality = 4; // RTK Fixed
                            } else {
                                data.gpsQuality = 1; // Autonomous GPS
                            }

                            // Calculate 2D scalar speed via Pythagorean theorem and convert to knots
                            double vel_ms = std::sqrt((vel_E * vel_E) + (vel_N * vel_N));
                            data.vel_knots = vel_ms * 1.94384;

                            // Send frame to queue with a 2-tick timeout to avoid thread starvation
                            xQueueSend(gnssQueue, &data, 2); 
                        }
                    }
                }
            }
            lineIndex = 0;
        }
    }
}

// ====================================================================
// FREERTOS TASK: BLUETOOTH ENCODING & TRANSMISSION (CORE 0)
// ====================================================================
void bluetoothTask(void *pvParameters) {
    FilteredData data;
    
    // Safely padded NMEA sentence string buffers
    char ggaBody[148];
    char rmcBody[148];
    char latNumStr[24], lonNumStr[24];
    char latDir, lonDir; 

    for (;;) {
        // Block until clean data becomes available from Core 1 loop
        if (xQueueReceive(gnssQueue, &data, portMAX_DELAY) == pdTRUE) {
            
            updateLED(data.gpsQuality);

            if (SerialBT.hasClient()) {
                
                // Convert coordinates to High-Precision NMEA DDMM.MMMMMM format
                convertNMEACoordinate(data.latitude, latNumStr, sizeof(latNumStr), &latDir, true);
                convertNMEACoordinate(data.longitude, lonNumStr, sizeof(lonNumStr), &lonDir, false);

                // 1. GENERATE AND TRANSMIT $GNRMC
                char modeIndicator = (data.gpsQuality == 4) ? 'D' : 'A';
                int lenRMC = snprintf(rmcBody, sizeof(rmcBody), "GNRMC,%s,A,%s,%c,%s,%c,%.2f,%.2f,%s,,,%c,A",
                                      data.timeStr, latNumStr, latDir, lonNumStr, lonDir, 
                                      data.vel_knots, data.heading, data.dateStr, modeIndicator);
                
                unsigned char chkRMC = 0;
                for (int i = 0; i < lenRMC; i++) chkRMC ^= rmcBody[i];
                SerialBT.printf("$%s*%02X\r\n", rmcBody, chkRMC);

                // 2. GENERATE AND TRANSMIT $GNGGA
                int lenGGA = snprintf(ggaBody, sizeof(ggaBody), "GNGGA,%s,%s,%c,%s,%c,%d,%d,0.5,%.2f,M,0.0,M,,",
                                      data.timeStr, latNumStr, latDir, lonNumStr, lonDir, 
                                      data.gpsQuality, data.numSats, data.height);
                unsigned char chkGGA = 0;
                for (int i = 0; i < lenGGA; i++) chkGGA ^= ggaBody[i];
                SerialBT.printf("$%s*%02X\r\n", ggaBody, chkGGA);
                // Bluetooth stack native pacing manages transmission.
                // Omitting flush() prevents sequential execution locks.
            }
        }
    }
}

// ====================================================================
// GPS EPOCH TIME TO GREGORIAN CALENDAR CONVERTER ALGORITHM (DDMMAA)
// ====================================================================
void calculateGPSDate(unsigned long gpsWeek, unsigned long weekSeconds, char* bufferOutput) {
    // Pure integer epoch-handling mathematics with all multiplication operators (*) restored
    unsigned long long totalUnixSeconds = ((unsigned long long)gpsWeek * 7ULL * 86400ULL) + weekSeconds - 18ULL;
    unsigned long epochDays = totalUnixSeconds / 86400;
    epochDays += 719468;
    unsigned long era = epochDays / 146097;
    unsigned long doe = epochDays - era * 146097;
    unsigned long yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    unsigned long year = yoe + era * 400;
    unsigned long doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    unsigned long mp = (5 * doy + 2) / 153;
    unsigned long day = doy - (153 * mp + 2) / 5 + 1;
    unsigned long month = mp + (mp < 10 ? 3 : -9);
    
    if (month <= 2) year++;
    int shortYear = year % 100;
    
    snprintf(bufferOutput, 7, "%02lu%02lu%02d", day, month, shortYear);
}

// ==========================================
// RGB LED LOGIC HARDWARE CONTROLLER
// ==========================================
void configureLED() {
    pinMode(PIN_RED, OUTPUT);
    pinMode(PIN_GREEN, OUTPUT);
    pinMode(PIN_BLUE, OUTPUT);
    digitalWrite(PIN_RED, LED_OFF);
    digitalWrite(PIN_GREEN, LED_OFF);
    digitalWrite(PIN_BLUE, LED_ON);
}

void updateLED(int quality) {
    switch (quality) {
        case 4: // Solid GREEN -> RTK Fixed solution acquired
            digitalWrite(PIN_RED, LED_OFF);
            digitalWrite(PIN_GREEN, LED_ON);
            digitalWrite(PIN_BLUE, LED_OFF);
            break;
        case 1: // Solid RED -> Autonomous mode active, lacks RTK correction
            digitalWrite(PIN_RED, LED_ON);
            digitalWrite(PIN_GREEN, LED_OFF);
            digitalWrite(PIN_BLUE, LED_OFF);
            break;
        default: // Solid YELLOW -> Initializing data link/waiting for valid solution
            digitalWrite(PIN_RED, LED_ON);
            digitalWrite(PIN_GREEN, LED_ON);
            digitalWrite(PIN_BLUE, LED_OFF);
            break;
    }
}

// ====================================================================
// GEOGRAPHIC PARSER: DD.DDDDDD -> DDMM.MMMMMM WITH RTK RESOLUTION
// ====================================================================
void convertNMEACoordinate(double decimalDegree, char* bufferOutput, size_t bufferSize, char* direction, bool isLatitude) {
    if (decimalDegree < 0) {
        *direction = isLatitude ? 'S' : 'W';
        decimalDegree = -decimalDegree;
    } else {
        *direction = isLatitude ? 'N' : 'E';
    }
    double integerPart;
    double fractionalPart = std::modf(decimalDegree, &integerPart);
    double minutes = fractionalPart * 60.0;
    
    // Output format updated to %09.6f for sub-millimeter positioning representations
    if (isLatitude) {
        snprintf(bufferOutput, bufferSize, "%02d%09.6f", (int)integerPart, minutes);
    } else {
        snprintf(bufferOutput, bufferSize, "%03d%09.6f", (int)integerPart, minutes);
    }
}