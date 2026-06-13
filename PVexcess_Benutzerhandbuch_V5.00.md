# PV-Entaklemmer — Benutzerhandbuch V5.00

**Fronius + Shelly · ESP8266 · 4-Kanal-Kaskade**

---

## Inhaltsverzeichnis

1. [Überblick](#überblick)
2. [Erstinbetriebnahme](#erstinbetriebnahme)
3. [Statusanzeige](#statusanzeige)
4. [Konfiguration](#konfiguration)
5. [Kaskaden-Logik](#kaskaden-logik)
6. [Schaltzyklen & Lebensdauer](#schaltzyklen--lebensdauer)
7. [Unterstützte Smartmeter](#unterstützte-smartmeter)
8. [Technische Hinweise](#technische-hinweise)
9. [Haftungsausschluss](#haftungsausschluss)

---

## Überblick

Der PV-Entaklemmer steuert bis zu **4 Shelly-Relais** anhand des aktuellen PV-Überschusses. Er fragt regelmäßig einen Smartmeter (z. B. Fronius) ab und schaltet Verbraucher automatisch ein oder aus — sobald genug Solarstrom ins Netz exportiert wird.

Ziel: **Eigenverbrauch maximieren**, ohne Strom zuzukaufen.

---

## Erstinbetriebnahme

### 1. WLAN einrichten (AP-Modus)

Beim ersten Start (oder wenn keine WLAN-Verbindung besteht) öffnet das Gerät einen eigenen Hotspot:

| Parameter | Wert |
|-----------|------|
| SSID | `EK-XXYYZZ` (letzte 3 Bytes der MAC) |
| Passwort | `12345678` |
| Konfigurations-URL | `http://192.168.4.1` |

Den Hotspot im Konfigurationsformular mit WLAN-SSID, Passwort und Smartmeter-IP befüllen, speichern — das Gerät startet neu und verbindet sich.

> **Hinweis:** Der AP-Modus beendet sich automatisch nach 120 Sekunden, falls keine Einstellungen gespeichert werden.

### 2. Im Heimnetz aufrufen

Nach erfolgreicher WLAN-Verbindung ist das Gerät über seine IP erreichbar (z. B. `http://192.168.22.61`). Die IP steht im Seriellen Monitor und wird im Header der Weboberfläche angezeigt.

---

## Statusanzeige

Der **Status-Tab** zeigt in Echtzeit:

### Übersichtskarten (oben)

| Karte | Bedeutung |
|-------|-----------|
| **Einspeisung** | Aktueller Netzaustausch in W. Grün = Export, Rot = Import |
| **PV Erzeugung** | Aktuelle Solarproduktion (nur bei Fronius verfügbar) |
| **Verbrauch** | Aktueller Hausverbrauch (nur bei Fronius verfügbar, sonst `--`) |
| **Fronius** | HTTP-Status der Smartmeter-Abfrage (OK / Offline / HTTP-Fehler) |

### Shelly-Karten (2×2 Grid)

Jede Karte zeigt:

- **Kreisnummer** — grün = EIN, rot = AUS, orange = gesperrt (Vorgänger AUS)
- **Badge** — `EIN`, `AUS` oder `gesperrt`
- **Live-Leistung** in Watt (wird nur bei eingeschaltetem Shelly abgefragt)
- **Online-Punkt** — grün wenn letzter HTTP-Aufruf erfolgreich
- **Schaltvorgänge** — Anzahl der Halbzyklen geteilt durch 2 (z. B. `886,5`)
- **Lebensdauer-Balken** — grün < 50 %, orange 50–75 %, rot ≥ 75 %
- **Verbleibende Lebensdauer** in Jahren (Hochrechnung auf Basis der bisherigen Rate)
- **Schaltflächen** `EIN` / `AUS` — manuelle Sofortsteuerung

> Die Anzeige aktualisiert sich alle **3 Sekunden** automatisch.

---

## Konfiguration

Der **Konfiguration-Tab** gliedert sich in zwei Bereiche.

### Netzwerk

| Feld | Beschreibung |
|------|-------------|
| WLAN SSID | Name des Heimnetzwerks |
| WLAN Passwort | Leer lassen, um das gespeicherte Passwort beizubehalten |
| Datenquelle | Smartmeter-Typ (siehe [Unterstützte Smartmeter](#unterstützte-smartmeter)) |
| Gerät IP | IP-Adresse des gewählten Smartmeters |
| Peak-Leistung (W) | Nennleistung der PV-Anlage (derzeit informativ) |
| Zykluszeit (ms) | Abfrageintervall des Smartmeters. Standard: `3000` ms |
| Wartezyklen | Pausenzyklen nach jedem Schaltvorgang. `0` = kein Warten |

### Shelly Konfiguration (4 Karten)

Für jeden der 4 Shellys:

| Feld | Beschreibung |
|------|-------------|
| Shelly Generation | `Gen 1` oder `Gen 2 / 3` — pro Shelly individuell einstellbar |
| IP-Adresse | IP des Shelly im Heimnetz |
| Nennleistung (W) | Angeschlossene Geräteleistung (informativ) |
| Einschaltschwelle (W) | Mindest-Überschuss, ab dem dieser Shelly einschaltet |
| Ausschaltschwelle (W) | Überschuss, unter dem dieser Shelly ausschaltet |
| Lebensdauer-Limit | Maximale Schaltzyklen (Vollzyklen). Standard: `100.000` |
| Zähler zurücksetzen | Setzt den Schaltzyklen-Zähler nach Bestätigung auf 0 |

**Speichern / Verwerfen** — Einstellungen werden sofort im EEPROM gespeichert. Kein Neustart erforderlich.

---

## Kaskaden-Logik

Die 4 Shellys arbeiten in einer festen Reihenfolge (Kaskade):

```
Shelly 1 → Shelly 2 → Shelly 3 → Shelly 4
```

**Einschalten (vorwärts):**
- Shelly 1 schaltet ein, sobald Überschuss ≥ Einschaltschwelle SW1
- Shelly 2 schaltet nur ein, wenn Shelly 1 bereits EIN ist
- Shelly 3 nur wenn Shelly 2 EIN, usw.
- Pro Zyklus wird maximal **ein** Shelly eingeschaltet

**Ausschalten (rückwärts):**
- Fällt der Überschuss unter die Ausschaltschwelle, schaltet zuerst **Shelly 4** aus
- Pro Zyklus wird maximal **ein** Shelly ausgeschaltet
- Nach jedem Schaltvorgang wartet das System `Wartezyklen` × `Zykluszeit` ms, bevor erneut entschieden wird

**Gesperrt-Zustand:** Ein Shelly zeigt `gesperrt` (orangefarbener Rand), wenn sein Vorgänger AUS ist. Er wird nicht aktiv ausgeschaltet — er wartet nur, bis er wieder berechtigt ist.

---

## Schaltzyklen & Lebensdauer

Relais haben eine begrenzte mechanische Lebensdauer. Der PV-Entaklemmer zählt jeden Schaltvorgang (EIN oder AUS = 1 Halbzyklus).

- **Anzeige:** Halbzyklen ÷ 2 = Vollzyklen (z. B. `1773` → `886,5`)
- **Speicherung:** Wear-Levelling-Ringpuffer im EEPROM (150 Slots × 4 Byte pro Shelly) — überlebt Neustarts und EEPROM-Alterung
- **Lebensdauer-Hochrechnung:** Rate aus bisherigen Zyklen seit letztem Boot, hochgerechnet auf das gesetzte Limit
- **Zähler zurücksetzen:** Nach einem Relais-Tausch den Zähler in der Konfiguration auf 0 setzen

> Die Lebensdaueranzeige stabilisiert sich erst nach einigen Stunden Betrieb.

---

## Unterstützte Smartmeter

Ab V5.00 können verschiedene Meter als Datenquelle gewählt werden. Alle nutzen die lokale LAN-IP — kein Cloud-Zugriff erforderlich.

| # | Datenquelle | Abfrage-Endpunkt | PV / Verbrauch |
|---|-------------|-----------------|----------------|
| 0 | **Fronius Smartmeter** | `/solar_api/v1/GetPowerFlowRealtimeData.fcgi` | ✅ verfügbar |
| 1 | **Shelly EM / 3EM** | Gen1: `/emeter/0` · Gen2: `/rpc/EM.GetStatus?id=0` | — |
| 2 | **SMA Home Manager** | `/api/v1/measurements/live` | — |
| 3 | **Tasmota (CT-Klemme)** | `/cm?cmnd=Status%2010` | — |
| 4 | **Volkszähler / vzlogger** | `/api/data.json` | — |
| 5 | **Kostal PLENTICORE / PIKO** | `/api/dxs.json?dxsEntries=33556736` | — |

> Bei Quellen ohne PV/Verbrauch-Daten zeigt die Statusseite `-- W` in den entsprechenden Feldern. Die Schaltlogik arbeitet ausschließlich mit dem **Netzaustausch (Einspeisung)** — dieser ist bei allen Quellen verfügbar.

**Vorzeichen-Konvention intern:** Positiv = Export ins Netz = Überschuss vorhanden.

---

## Technische Hinweise

### EEPROM-Kompatibilität

| Version | Änderung | Maßnahme nach Update |
|---------|----------|---------------------|
| V4.06 | Fronius-IP von Adresse 300 → 2779 verschoben | Fronius-IP neu eingeben und speichern |
| V5.00 | Metertyp (Adresse 2879), Shelly-Generation pro Kanal (2880–2883) | Keine — Defaults werden automatisch gesetzt |

### Hostname

Das Gerät meldet sich im Netzwerk als `EK-XXYYZZ` (die letzten 3 Bytes der WLAN-MAC-Adresse in Hex).

### Watchdog

Ein Hardware-Watchdog (10 Sekunden) schützt vor Hängern. Bei mehr als 10 aufeinanderfolgenden fehlgeschlagenen Smartmeter-Abfragen werden alle Shellys als Sicherheitsmaßnahme ausgeschaltet.

### Webschnittstelle — Endpunkte

| Endpunkt | Methode | Beschreibung |
|----------|---------|-------------|
| `/` | GET | Haupt-UI |
| `/getLiveData` | GET | JSON: Echtzeitdaten (alle 3 s vom Browser abgefragt) |
| `/getStoredParameters` | GET | JSON: Konfiguration + Systeminfo |
| `/action_pageRun` | POST | Konfiguration speichern |
| `/shellyForce?sw=N&on=1` | GET | Shelly N manuell ein- (1) oder ausschalten (0) |
| `/resetCycles?sw=N` | GET | Schaltzyklen-Zähler für Shelly N zurücksetzen |

---

---

## Haftungsausschluss

Dieses Projekt ist ein privates Open-Source-Hobbyprojekt und wird **ohne jegliche Gewährleistung** bereitgestellt.

Die Nutzung erfolgt **auf eigene Gefahr**. Insbesondere wird keine Haftung übernommen für:

- Schäden an elektrischen Geräten, Installationen oder der PV-Anlage
- Folgeschäden durch fehlerhafte Schaltlogik, Netzwerkausfälle oder Softwarefehler
- Verstöße gegen lokale Vorschriften, Normen oder Netzbetreiber-Anforderungen
- Datenverlust oder Fehlfunktionen nach Firmware-Updates

**Elektroinstallation:** Die Verdrahtung von Shelly-Relais im 230-V-Bereich darf nur durch eine **Elektrofachkraft** gemäß den geltenden Vorschriften (z. B. VDE, ÖVE, NIV) durchgeführt werden.

**Keine Zertifizierung:** Dieses Gerät ist kein zugelassenes Energiemanagement-System im Sinne von Förderprogrammen oder Netzbetreiber-Anforderungen.

Der Quellcode wird unter der **MIT-Lizenz** veröffentlicht. Die MIT-Lizenz schließt ausdrücklich jede Haftung des Autors aus.

---

*PV-Entaklemmer V5.00 · ESP8266 · MIT License*
