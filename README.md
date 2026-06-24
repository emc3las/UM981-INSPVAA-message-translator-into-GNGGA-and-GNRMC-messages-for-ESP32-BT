# UM981-INSPVAA-message-translator-into-GNGGA-and-GNRMC-messages-for-ESP32-BT
This repository contains an Arduino IDE sketch for the ESP32 that intercepts Unicore Communications #INSPVAA messages via serial, translates them into standard NMEA $GNGGA and $GNRMC strings, and broadcasts them over Classic Bluetooth.

It is specifically tailored for the ESP32 XBee module installed on an Ardusimple simpleRTK3B Fusion board connected to an Android smartphone.

**The Problem**

The UM981 GNSS receiver features an integrated IMU and an Inertial Navigation System (INS). While highly accurate, its native #INSPVAA messages are not recognized by popular GIS software and automotive telemetry apps like RaceChrono. RaceChrono requires standard NMEA data ($GNGGA and $GNRMC) to function properly.

**The Solution**

This firmware acts as a bridge. It listens to the UM981 serial port at 921600 bps, extracts the required PVT (Position, Velocity, Time) data from the raw #INSPVAA packets, constructs valid NMEA messages, and sends them to a Classic Bluetooth master device.

**RTK Correction Setup**

To achieve RTK precision, the UM981 requires incoming RTCM correction data from an NTRIP caster. 

The hardware data flow is configured as follows:

**NTRIP Client:** An Android app (such as GNSS Master or SW Maps) runs in the background on the smartphone to fetch RTCM data from the internet.

**Injected Corrections:** The smartphone feeds these RTCM messages back to the UM981 using a physical USB-to-serial cable.

**Bluetooth Output:** At the same time, RaceChrono receives the translated, INS-enhanced NMEA coordinates wirelessly from the ESP32 via Bluetooth.
