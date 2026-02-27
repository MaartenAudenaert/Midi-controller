# STM32 USB MIDI Device - Opdracht 1

## Overzicht

Dit project configureert de STM32 Nucleo-H533RE microcontroller als een USB MIDI Device. Het board wordt herkend door de computer als een MIDI-instrument en verstuurt automatisch MIDI Note ON en Note OFF messages die zichtbaar zijn in MIDI monitoring software zoals MIDI-view.

## Hardware Configuratie

### USB Voeding Instelling

**BELANGRIJK:** Het Nucleo board moet via USB gevoed worden (niet via ST-Link).

1. **Jumper JP1**: Verplaats deze jumper naar de E5V positie (niet U5V). Dit schakelt de voeding van ST-Link uit.
2. **USB Connector**: Sluit een USB-kabel aan op de **CN13 USB connector** (de USER USB connector, niet de ST-Link USB).

### Pinout
- **PA11**: USB_DM (USB Data Minus)
- **PA12**: USB_DP (USB Data Plus)
- **PA5**: LED (togglet bij elke MIDI message)

## Software Architectuur

### USB Device Middleware Structuur

Het project implementeert een complete USB Device stack:

```
Middlewares/ST/STM32_USB_Device_Library/
├── Core/
│   ├── Inc/
│   │   ├── usbd_core.h        - USB Device core functies
│   │   ├── usbd_def.h         - USB definities en structuren
│   │   ├── usbd_ioreq.h       - IO request functies
│   │   └── usbd_ctlreq.h      - Control request handling
│   └── Src/
│       ├── usbd_core.c
│       ├── usbd_ioreq.c
│       └── usbd_ctlreq.c
└── Class/
    └── MIDI/
        ├── Inc/
        │   └── usbd_midi.h    - USB MIDI Class definitie
        └── Src/
            └── usbd_midi.c    - USB MIDI Class implementatie

Core/
├── Inc/
│   ├── usbd_conf.h            - USB configuratie
│   └── usbd_desc.h            - USB descriptors header
└── Src/
    ├── usbd_conf.c            - Low-Level driver interface (HAL PCD ↔ USB Device)
    └── usbd_desc.c            - USB MIDI descriptors
```

### USB MIDI Descriptors

Het device gebruikt de volgende USB descriptors:

- **Device Descriptor**: Identificeert het apparaat als Audio Device Class
  - Vendor ID: 0x0483 (STMicroelectronics)
  - Product ID: 0x5740
  - Device Class: Audio (0x01)

- **Configuration Descriptor**: Definieert 2 interfaces:
  1. **Audio Control Interface** (Interface 0)
  2. **MIDI Streaming Interface** (Interface 1)
     - Bulk IN endpoint (EP1 IN, 0x81)
     - Bulk OUT endpoint (EP1 OUT, 0x01)

- **MIDI Streaming Descriptors**: 
  - MIDI IN Jack (Embedded + External)
  - MIDI OUT Jack (Embedded + External)

### MIDI Message Format

USB MIDI messages gebruiken een 4-byte formaat:

```
Byte 0: Cable Number (4 bits) + Code Index Number (4 bits)
Byte 1: MIDI Status + Channel
Byte 2: Data byte 1
Byte 3: Data byte 2
```

**Voorbeeld - Note ON (Middle C):**
```c
uint8_t midiData[4] = {
    0x09,  // Cable 0 + Code Index 9 (Note ON)
    0x90,  // Note ON (0x9) + Channel 0
    60,    // Note number (Middle C)
    127    // Velocity
};
```

**Voorbeeld - Note OFF:**
```c
uint8_t midiData[4] = {
    0x08,  // Cable 0 + Code Index 8 (Note OFF)
    0x80,  // Note OFF (0x8) + Channel 0
    60,    // Note number (Middle C)
    0      // Velocity
};
```

## Code Uitleg

### Main Loop (main.c)

De main loop verstuurt elke seconde afwisselend een Note ON en Note OFF message:

```c
while (1)
{
    if (HAL_GetTick() - lastNoteTime > 1000)
    {
        lastNoteTime = HAL_GetTick();
        
        if (noteOn)
        {
            // Note ON: Middle C (60), Velocity 127
            midiData[0] = 0x09;
            midiData[1] = 0x90;
            midiData[2] = 60;
            midiData[3] = 127;
        }
        else
        {
            // Note OFF: Middle C (60), Velocity 0
            midiData[0] = 0x08;
            midiData[1] = 0x80;
            midiData[2] = 60;
            midiData[3] = 0;
        }
        
        USBD_MIDI_SendData(&hUsbDeviceFS, midiData, 4);
        noteOn = !noteOn;
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}
```

### USB MIDI Initialisatie

In `main()` wordt de USB MIDI stack geïnitialiseerd:

```c
/* Initialize USB Device Library */
USBD_Init(&hUsbDeviceFS, &MIDI_Desc, 0);

/* Add MIDI Class */
USBD_RegisterClass(&hUsbDeviceFS, &USBD_MIDI);

/* Start Device */
USBD_Start(&hUsbDeviceFS);
```

## Testen en Verificatie

### Stap 1: Flash de Firmware

1. Open het project in STM32CubeIDE of IAR Embedded Workbench
2. Build het project
3. Flash de firmware via ST-Link

### Stap 2: Schakel naar USB Voeding

1. Disconnect de ST-Link USB-kabel
2. Verplaats JP1 jumper naar E5V
3. Sluit USB-kabel aan op CN13 (USER USB)

### Stap 3: Controleer Device Herkenning

**Windows:**
1. Open Device Manager (Apparaatbeheer)
2. Kijk onder "Audio inputs and outputs" of "Sound, video and game controllers"
3. Je zou "STM32 MIDI Device" moeten zien

**macOS:**
1. Open Audio MIDI Setup
2. Window → Show MIDI Studio
3. "STM32 MIDI Device" zou zichtbaar moeten zijn

**Linux:**
```bash
lsusb | grep "STM"
# Zou moeten tonen: Bus XXX Device XXX: ID 0483:5740 STMicroelectronics

# Controleer MIDI ports:
aconnect -l
```

### Stap 4: Monitor MIDI Messages

**MIDI-view (Windows):**
1. Download en installeer MIDI-view van https://hautetechnique.com/midi/midiview/
2. Start MIDI-view
3. Selecteer "STM32 MIDI Device" als Input
4. Je zou elke seconde afwisselend Note ON (60) en Note OFF (60) moeten zien

**MIDI Monitor (macOS):**
1. Download MIDI Monitor van https://www.snoize.com/MIDIMonitor/
2. Selecteer "STM32 MIDI Device" in Sources
3. Observeer de Note ON/OFF messages

**Linux:**
```bash
# Installeer amidi als nog niet geïnstalleerd:
sudo apt-get install alsa-utils

# Vind MIDI device:
amidi -l

# Monitor MIDI messages:
aseqdump -p "STM32 MIDI"
```

### Verwachte Output

In MIDI monitoring software zou je dit moeten zien (elke seconde):

```
Note ON  - Channel: 0, Note: 60 (C4), Velocity: 127
Note OFF - Channel: 0, Note: 60 (C4), Velocity: 0
Note ON  - Channel: 0, Note: 60 (C4), Velocity: 127
Note OFF - Channel: 0, Note: 60 (C4), Velocity: 0
...
```

De groene LED op het Nucleo board (PA5) zou ook moeten knipperen bij elke message.

## Troubleshooting

### Device wordt niet herkend
- Controleer of JP1 op E5V staat
- Controleer of USB-kabel op CN13 (USER USB) zit, niet op CN1 (ST-Link)
- Probeer een andere USB-kabel (sommige kabels zijn alleen voor opladen)
- Herstart de computer na eerste aansluiting

### Geen MIDI messages zichtbaar
- Controleer of de LED knippert (PA5)
- Herstart de MIDI monitoring software
- Controleer of het juiste MIDI input device is geselecteerd

### Device valt steeds weg
- Gebruik een USB-kabel met goede stroomvoorziening
- Sluit niet aan op een USB hub, gebruik directe PC verbinding
- Controleer of HSI48 clock correct is geconfigureerd (zie main.c)

## MIDI Message Referentie

### Code Index Numbers (CIN)
| CIN | Naam | Beschrijving |
|-----|------|--------------|
| 0x8 | Note OFF | Note uitschakelen |
| 0x9 | Note ON | Note inschakelen |
| 0xA | Poly Aftertouch | Polyphonic key pressure |
| 0xB | Control Change | Controller wijziging |
| 0xC | Program Change | Instrument wijziging |
| 0xD | Channel Aftertouch | Channel pressure |
| 0xE | Pitch Bend | Pitch bend wijziging |

### MIDI Note Numbers
| Note | Number |
|------|--------|
| C4 (Middle C) | 60 |
| C#4 | 61 |
| D4 | 62 |
| ... | ... |

## Volgende Stappen (Opdracht 2)

Na deze opdracht kan je uitbreiden met:
- 4x4 Keypad matrix voor interactieve MIDI input
- Verschillende MIDI notes per toets
- Velocity sensing
- Multiple MIDI channels

## Bronnen

- [USB MIDI Class Specification](https://www.usb.org/sites/default/files/midi10.pdf)
- [MIDI 1.0 Specification](https://www.midi.org/specifications)
- STM32H5 Reference Manual
- STM32 USB Device Library Documentation

## Licentie

Dit project is gemaakt voor educatieve doeleinden als onderdeel van de Entertainment Technology opleiding aan Hogeschool VIVES.
