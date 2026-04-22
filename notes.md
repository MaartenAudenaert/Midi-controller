## SK6812 notes (wat werkt nu)

### 1) Hardware die nu werkt
- Board: `NUCLEO-H533RE`
- Data pin: `PC6`
- Functie van pin: `TIM3_CH1` (alternate function)
- LED type: `SK6812 Mini-E` (addressable RGB)
- Aansluiting:
  - `PC6 -> DIN`
  - `GND -> GND`
  - `5V -> VDD`
  - `100 nF` dicht bij de led tussen `VDD` en `GND`

### 2) Belangrijkste software-fix
- De led werkte eerst niet door foute timing.
- Oplossing in `main.c` bij `MX_TIM3_Init()`:
  - `htim3.Init.Period` is nu gebaseerd op de echte timerklok:
  - `htim3.Init.Period = (HAL_RCC_GetPCLK1Freq() / 800000U) - 1U;`
- Daardoor klopt de SK6812 bitstream (~800 kHz) en reageert de led correct.

### 3) Testmodus die nu actief is
- Er is een tijdelijke testmodus om PC6 zichtbaar te testen:
  - `SK6812_PC6_TEST_ENABLE = 1`
  - `SK6812_PC6_BLINK_MS = 1000U`
- Resultaat: led knippert groen 1x per seconde.
- Dit is puur om de datalijn/timing te verifiëren.

### 4) Waarom D8 niet werkte
- `D8` werd als gewone GPIO getest.
- Een SK6812 werkt niet met simpel HIGH/LOW, maar met een strak protocol.
- Daarom werkt testen op `PC6 + TIM3 + DMA` wel.

### 5) Verder werken naar meerdere leds
- In code:
  - verhoog `SK6812_LED_COUNT` naar het echte aantal leds in de ketting.
  - `SK6812_BUFFER_SIZE` schaalt automatisch mee.
- In hardware:
  - `DOUT` van led 1 naar `DIN` van led 2, enz.
  - gemeenschappelijke `GND` behouden.
  - bij langere strips: extra ontkoppeling en degelijke 5V voeding.
- In software-logica:
  - gebruik `SK6812_SetColor(index, r, g, b)` per led.
  - roep `SK6812_Show()` aan na updates (of laat de bestaande update-flow dit doen).

### 6) Voor terug naar normale app (geen testblink)
- Zet testmodus uit:
  - `SK6812_PC6_TEST_ENABLE = 0`
- Dan draait opnieuw de gewone matrix/MIDI-ledlogica.
