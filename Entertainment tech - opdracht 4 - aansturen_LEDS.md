**Entertainment technology**

**bachelor in de elektronica – ICT - Brugge**

docent: **Van Gaever T.**

academiejaar **2026-2027**

**Opdracht 4: Aansturen SK6812_mini_e**

# Inleiding

**Microcontroller:** Nucleo-H533RE

In deze opdracht breid je de USB MIDI Controller uit met 16 SK6812 Mini-E RGB LEDs. De LEDs worden aangestuurd via een PWM-signaal dat met DMA automatisch wordt gegenereerd, zodat de CPU vrij blijft voor de USB MIDI en matrix scanning functies. De LEDs zijn in een daisy chain verbonden en worden aangestuurd via één enkele datalijn.

Het project bestaat uit drie deelopdrachten die je achtereenvolgens uitvoert.

# Opdracht 1: SK6812 Protocol Studie

**Doel:** Bestudeer het SK6812 protocol en begrijp hoe de data wordt gecodeerd en verstuurd.

**Achtergrond:**

De SK6812 Mini-E is een intelligent aanstuurbare RGB LED met een ingebouwde controller. Het protocol is compatibel met de WS2812B. Elke LED ontvangt 24 bits data in GRB-volgorde (Groen, Rood, Blauw) via een unipolair RZ (Return-to-Zero) protocol op één enkele datalijn.

Zorg dat je het protocol begrijpt en kan uitleggen.

# Opdracht 2: Hardware & CubeMX Configuratie

**Doel:** Configureer TIM3 en GPDMA in CubeMX voor het aansturen van de SK6812 LEDs en sluit de hardware aan.

**Hardware aansluiting:**

- Verbind PB4 (TIM3_CH1) met de DIN van de eerste SK6812 LED
- Verbind DOUT van elke LED met DIN van de volgende LED (daisy chain)
- Voorzie 5V voeding en GND voor alle LEDs
- Plaats een ontkoppelcondensator (100 nF) dicht bij de VDD pin van elke LED
- Optioneel: een pull-up weerstand (1K naar 5V met open-drain GPIO) tussen PB4 en de eerste LED. Als de LED niet werkt met de 3.3V out van de microcontroller (de LEDS werken op 5V). gebruiken we een pull up weerstand naar 5V voor de logic high.

**highCubeMX configuratie TIM3:**

|     |     |     |
| --- | --- | --- |
| **Parameter** | **Waarde** | **Toelichting** |
| Clock Source | Internal Clock | Systeemklok |
| Channel 1 | PWM Generation CH1 | PWM output op PB4 |
| Prescaler (PSC) | 0   | Geen deling |
| Counter Period (ARR) | 312 | kloksnelheid ÷ 800 kHz - 1 |
| PWM Mode | Mode 1 | Uitgang hoog zolang teller < CCR |
| CH Polarity | High | Actief hoog signaal |

**CubeMX configuratie GPDMA1 (voor SK6812):**

|     |     |     |
| --- | --- | --- |
| **Parameter** | **Waarde** | **Toelichting** |
| Channel | GPDMA1 Ch1 | Ch0 is voor ADC |
| Request | TIM3_CH1 | Koppeling timer → DMA |
| Direction | Memory to Peripheral | Buffer → CCR1 register |
| Source Increment | Enable | Door buffer lopen |
| Dest Increment | Disable | Altijd naar zelfde CCR1 |
| Data Width | Half Word (16-bit) | CCR1 is 16-bit register |
| Mode | Normal | Stop na volledige transfer |

# Opdracht 3: LED Driver Software

**Doel:** Implementeer de SK6812 LED driver en koppel deze aan de MIDI controller zodat elke knop op de matrix een visuele LED-feedback geeft.

**Aanpak:**

Je mag gebruik maken van een bestaande library (zoals **Crazy-Geeks/STM32-ARGB-DMA** op GitHub) of de code uit een library als voorbeeld gebruiken om je eigen driver te schrijven. Let op: bestaande libraries ondersteunen de STM32H5 serie (GPDMA) mogelijk niet direct — je zal de DMA-aanroepen eventueel moeten aanpassen.

**Minimale functionaliteit:**

- Een databuffer van het juiste formaat (bereken het totaal aantal waarden inclusief reset)
- Een functie om een individuele LED een kleur te geven: set_led_color(led_index, r, g, b)
- Let op de GRB-volgorde en MSB-first bitvolgorde bij het vullen van de buffer
- Een functie om de buffer via DMA naar de LEDs te versturen
- De Transfer Complete callback afhandelen om de PWM te stoppen na de transfer

**MIDI-LED koppeling:**

Koppel de 16 LEDs aan de 16 knoppen van de matrix. Implementeer het volgende gedrag:

|     |     |     |
| --- | --- | --- |
| **Event** | **LED Actie** | **Kleur** |
| Knop ingedrukt (Note On) | Bijbehorende LED aan | Kies een kleur (bijv. groen) |
| Knop losgelaten (Note Off) | Bijbehorende LED uit | LED uit (R=0, G=0, B=0) |
| Geen activiteit | Standby kleur (optioneel) | Bijv. gedimd blauw |

**Bonus (optioneel):**

- Implementeer verschillende kleuren per rij van de matrix
- Voeg een animatie-effect toe bij het opstarten (bijv. alle LEDs sequentieel laten oplichten)

**Indienen:**

- Demonstratievideo waarin je laat zien dat de LEDs reageren op knopdrukken
- Oscilloscoop-screenshot van het datasignaal op PB4 (gebruik de Siglent SDS1000X-E)

## Tips & Aandachtspunten

- Controleer de DMA initialisatievolgorde in CubeMX: DMA_Init moet vóór TIM_Init staan
- PB4 is na reset geconfigureerd als NJTRST (JTAG). CubeMX stelt dit automatisch om naar TIM3_CH1 (AF2)
- De bufferwaarde 0 (CCR=0) is niet hetzelfde als een 0-bit (CCR=10 bij 32 MHz). CCR=0 houdt de lijn volledig laag en wordt gebruikt voor de reset
- De ADC DMA (Circular mode, GPDMA Ch0) en de LED DMA (Normal mode, GPDMA Ch1) werken volledig onafhankelijk
- Gebruik de oscilloscoop om het signaal te verifiëren. Een 0-bit moet een korte puls tonen (300 ns hoog), een 1-bit een langere puls (600 ns hoog)
- Als de eerste LED niet reageert maar de rest van de keten wel zou moeten werken, controleer dan het spanningsniveau (3,3V vs 5V logica)
- Library referentie: https://github.com/Crazy-Geeks/STM32-ARGB-DMA (je kan gebruik maken van de WS2812 define voor jouw SK6812 Mini-E RGB)