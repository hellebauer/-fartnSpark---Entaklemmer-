# Fart 'N Spark - Entaklemmer ⚡

** PV Eigenverbrauch Optimierung auf die billige Art **



Ohmpilot fuer finanziell Minderbemittelte. Schaltet bis zu 4 Lasten an (4xShelly) wenn genug PV Ueberschuss vorhanden ist. Funktioniert mit Fronius. Mit Arduino IDE auf ESP8266 (z.B. Wemos D1 mini) platform flashen. Ein AP wird unter 192.168.4.1 aufgemacht (password 12345678) . Verbinden, Wifi Kofiguration, Fronius IP, und Shelly IP eintragen. Schaltwerte eintragen. Fertig.


Flash the firmware via Webbrowser (no Arduino IDE needed)

    Download the latest .bin file from the Releases page.
    Connect a Wemos D1 Mini to your computer via USB.
    Open Chrome or Edge and go to https://esp.huhn.me
    Click Connect and select the Wemos COM port from the list.
    Click Add file, set the address to 0x0, and select the .bin file.
    Click Program and wait ~30 seconds until it says Done.
    Unplug and replug the Wemos.

    Firefox does not work — use Chrome or Edge only.


----------------------------------------------
Open-Source-Software ohne jegliche Gewährleistung oder Garantie. Die Nutzung erfolgt auf eigenes Risiko.

Haftungsausschluss

Diese Firmware wird ausschließlich zu Lehr-, Test- und Experimentierzwecken bereitgestellt. Sie dient unter anderem zur Steuerung von netzspannungsführenden Geräten (230 V AC). Der Autor übernimmt keinerlei Haftung für Sachschäden, Personenverletzungen oder Todesfälle, die aus der Verwendung dieser Software entstehen.

Die Installation und Inbetriebnahme dürfen ausschließlich durch eine qualifizierte Elektrofachkraft und unter Einhaltung aller geltenden gesetzlichen Vorschriften sowie örtlichen Bestimmungen erfolgen.

Die Nutzung erfolgt vollständig auf eigene Gefahr und eigenes Risiko.


----------------------------------------------
Open-source, no warranty. Use at your own risk.

Disclaimer

This firmware is provided for educational and experimental purposes only. It involves control of mains-voltage equipment (230 V AC). The author accepts no liability for damage to property, injury, or death resulting from its use. Installation must be carried out by a qualified electrician in accordance with all applicable local regulations. Use entirely at your own risk.
