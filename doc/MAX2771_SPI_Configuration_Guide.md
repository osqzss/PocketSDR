# MAX2771 SPI Configuration and GNSS Signal Type Documentation

## Overview

The MAX2771 is a multiband, multi-constellation GNSS RF front-end IC used in PocketSDR devices. This document details the SPI initialization, register configuration, and GNSS signal type settings extracted from the PocketSDR codebase.

## Table of Contents

1. [Hardware Architecture](#hardware-architecture)
2. [SPI Interface Protocol](#spi-interface-protocol)
3. [Register Map](#register-map)
4. [Initialization Sequence](#initialization-sequence)
5. [GNSS Band Configurations](#gnss-band-configurations)
6. [USB Vendor Commands](#usb-vendor-commands)
7. [Configuration Examples](#configuration-examples)

---

## Hardware Architecture

### Device Variants

| Device | Firmware Ver | MAX2771 Channels | USB Controller | IF Data Format |
|--------|-------------|------------------|----------------|----------------|
| FE 2CH | 0x10 | 2 | EZ-USB FX2LP | RAW8 (packed 8-bit) |
| FE 4CH | 0x30 | 4 | EZ-USB FX3 | RAW16 (packed 16-bit) |
| FE 8CH | 0x40 | 8 | EZ-USB FX3 | RAW32 (packed 32-bit) |

### GPIO Pin Assignments (FE 2CH - EZ-USB FX2LP)

```
Pin Assignment (FX2LP Port D):
  PD0 (CSN1)  → MAX2771 CH1 Chip Select (Active Low)
  PD1 (CSN2)  → MAX2771 CH2 Chip Select (Active Low)
  PD2 (SCLK)  → MAX2771 SPI Clock
  PD3 (SDATA) ↔ MAX2771 SPI Data (Bidirectional)
  PD4 (STAT1) ← MAX2771 CH1 PLL Lock Detect
  PD5 (STAT2) ← MAX2771 CH2 PLL Lock Detect
  PD6 (LED1)  → Status LED 1
  PD7 (LED2)  → Status LED 2
```

### GPIO Pin Assignments (FE 4CH - EZ-USB FX3)

```
GPIO Assignments:
  GPIO(36-39) ← MAX2771 CH1-4 Lock Detect (LOCK_A-D)
  GPIO(40-43) → MAX2771 CH1-4 Chip Select (CSN_A-D)
  GPIO(44-47) → RF Switch CH1-4 (RF_SW_A-D)
  GPIO(48-49) → LED1-2
  GPIO(50)    → USB3 Port Select
  GPIO(53)    → SPI Clock (SCLK)
  GPIO(55)    ↔ SPI Data (SDATA) - shared bus
```

### GPIO Pin Assignments (FE 8CH - EZ-USB FX3)

```
GPIO Assignments:
  GPIO(17-24) → MAX2771 CH1-8 Chip Select (CSN_A-H)
  GPIO(25)    → SPI Clock (SCLK)
  GPIO(26)    ↔ SPI Data (SDATA) - shared bus
  GPIO(27-29) → LED1-3
  GPIO(45)    → USB3 Port Select
  GPIO(50-57) ← MAX2771 CH1-8 Lock Detect (LOCK_A-H)
```

---

## SPI Interface Protocol

### SPI Frame Format

The MAX2771 uses a 48-bit SPI frame:

```
┌────────────────────┬──────┬──────────┬────────────────────────────────┐
│   Address (12-bit) │ R/W  │ Reserved │          Data (32-bit)         │
│    [47:36]         │ [35] │ [34:32]  │           [31:0]               │
└────────────────────┴──────┴──────────┴────────────────────────────────┘

Address: 12-bit register address (0x000-0x00A for MAX2771)
R/W:     0 = Write, 1 = Read
Reserved: 3 bits, set to 0
Data:    32-bit register value (MSB first)
```

### SPI Timing

```c
#define SCLK_CYC  5   // FX2LP: 5 SYNCDELAY cycles (~208ns @ 48MHz)
#define SCLK_CYC  10  // FX3: 10 µs BusyWait cycles
```

### SPI Write Operation

```
Source: FE_2CH/FW/v2.1/pocket_fw.c:132-143

static void write_reg(uint8_t cs, uint8_t addr, uint32_t val) {
    // 1. Assert chip select (active low)
    digitalWrite(cs, 0);

    // 2. Write frame header: 12-bit address + mode(0=write) + 3 reserved
    write_head(addr, 0);

    // 3. Write 32-bit data MSB first
    for (int i = 31; i >= 0; i--) {
        write_sdata((uint8_t)(val >> i) & 1);
    }

    // 4. Deassert chip select
    digitalWrite(cs, 1);
}
```

### SPI Read Operation

```
Source: FE_2CH/FW/v2.1/pocket_fw.c:145-159

static uint32_t read_reg(uint8_t cs, uint8_t addr) {
    uint32_t val = 0;

    // 1. Assert chip select (active low)
    digitalWrite(cs, 0);

    // 2. Write frame header: 12-bit address + mode(1=read) + 3 reserved
    write_head(addr, 1);

    // 3. Read 32-bit data MSB first
    for (int i = 31; i >= 0; i--) {
        val <<= 1;
        val |= read_sdata();
    }

    // 4. Deassert chip select
    digitalWrite(cs, 1);
    return val;
}
```

### Frame Header Write Function

```
Source: FE_2CH/FW/v2.1/pocket_fw.c:117-130

static void write_head(uint16_t addr, uint8_t mode) {
    // Write 12-bit address MSB first
    for (int i = 11; i >= 0; i--) {
        write_sdata((uint8_t)(addr >> i) & 1);
    }

    // Write R/W mode bit (0=write, 1=read)
    write_sdata(mode);

    // Write 3 reserved bits (zeros)
    for (int i = 0; i < 3; i++) {
        write_sdata(0);
    }
    spi_delay();
}
```

---

## Register Map

### MAX2771 Register Addresses

| Address | Name | Description |
|---------|------|-------------|
| 0x00 | CONF1 | Configuration 1 (LNA, Mixer, IF Filter) |
| 0x01 | CONF2 | Configuration 2 (AGC, ADC, Data Format) |
| 0x02 | CONF3 | Configuration 3 (PGA, Streaming) |
| 0x03 | PLLCONF | PLL Configuration |
| 0x04 | DIV | PLL Integer Divider |
| 0x05 | FDIV | PLL Fractional Divider |
| 0x06 | Reserved | Test Register (do not write) |
| 0x07 | CLK | Clock Configuration |
| 0x08 | Reserved | Test Register (do not write) |
| 0x09 | Reserved | Test Register (do not write) |
| 0x0A | CLK2 | Clock Configuration 2 |

### Register 0x00 (CONF1) - Bit Fields

```
Source: src/sdr_conf.c:41-52

Bit Position  | Field     | Bits | Description
--------------|-----------|------|------------------------------------------
[31]          | CHIPEN    |  1   | Chip enable (0:disable, 1:enable)
[30]          | IDLE      |  1   | Idle mode (0:operating, 1:idle)
[17]          | MIXPOLE   |  1   | Mixer pole (0:13MHz, 1:36MHz)
[16:15]       | LNAMODE   |  2   | LNA mode (0:high-band, 1:low-band, 2:disable)
[14:13]       | MIXERMODE |  2   | Mixer mode (0:high-band, 1:low-band, 2:disable)
[12:6]        | FCEN      |  7   | IF filter center freq: (128-FCEN)/2*step
[5:3]         | FBW       |  3   | IF filter bandwidth
[2]           | F3OR5     |  1   | Filter order (0:5th, 1:3rd)
[1]           | FCENX     |  1   | Polyphase filter (0:lowpass, 1:bandpass)
[0]           | FGAIN     |  1   | IF filter gain (0:-6dB, 1:normal)
```

### IF Filter Bandwidth (FBW) Settings

| FBW Value | Bandwidth | Frequency Step |
|-----------|-----------|----------------|
| 0 | 2.5 MHz | 0.195 MHz |
| 1 | 8.7 MHz | 0.66 MHz |
| 2 | 4.2 MHz | 0.355 MHz |
| 3 | 23.4 MHz | - |
| 4 | 36.0 MHz | - |
| 7 | 16.4 MHz | - |

### Register 0x01 (CONF2) - Bit Fields

```
Source: src/sdr_conf.c:53-61

Bit Position  | Field           | Bits | Description
--------------|-----------------|------|------------------------------------------
[28]          | ANAIMON         |  1   | Continuous spectrum monitoring
[27]          | IQEN            |  1   | I/Q enable (0:I-only, 1:I/Q)
[26:15]       | GAINREF         | 12   | AGC gain reference (0-4095)
[14:13]       | SPI_SDIO_CONFIG |  2   | SPI SDIO pin config
[12:11]       | AGCMODE         |  2   | AGC mode (0:auto, 2:manual)
[10:9]        | FORMAT          |  2   | Data format (0:unsigned, 1:sign-mag, 2:2's comp)
[8:6]         | BITS            |  3   | ADC bits (0:1-bit, 2:2-bit, 4:3-bit)
[5:4]         | DRVCFG          |  2   | Output driver (0:CMOS, 2:analog)
[1:0]         | DIEID           |  2   | Die ID
```

### Register 0x02 (CONF3) - Bit Fields

```
Source: src/sdr_conf.c:62-74

Bit Position  | Field      | Bits | Description
--------------|------------|------|------------------------------------------
[27:22]       | GAININ     |  6   | PGA gain (0-63 dB steps)
[20]          | HILODEN    |  1   | High load driver enable
[15]          | FHIPEN     |  1   | Highpass coupling enable
[13]          | PGAIEN     |  1   | I-channel PGA enable
[12]          | PGAQEN     |  1   | Q-channel PGA enable
[11]          | STRMEN     |  1   | DSP interface enable
[10]          | STRMSTART  |  1   | Start data streaming
[9]           | STRMSTOP   |  1   | Stop data streaming
[5:4]         | STRMBITS   |  2   | Stream bits (1:I, 3:IQ)
[3]           | STAMPEN    |  1   | Frame number insertion
[2]           | TIMESYNCEN |  1   | Time sync pulse enable
[1]           | DATASYNCEN |  1   | Data sync pulse enable
[0]           | STRMRST    |  1   | Reset counters
```

### Register 0x03 (PLLCONF) - Bit Fields

```
Source: src/sdr_conf.c:75-80

Bit Position  | Field    | Bits | Description
--------------|----------|------|------------------------------------------
[31:29]       | REFDIV   |  3   | Integer clock divider (0:x2, 1:1/4, 2:1/2, 3:x1, 4:x4)
[28]          | LOBAND   |  1   | LO band (0:L1/high, 1:L2/L5/low)
[24]          | REFOUTEN |  1   | Reference clock output enable
[20:19]       | IXTAL    |  2   | XTAL current (1:normal, 3:high)
[9]           | ICP      |  1   | Charge pump current (0:0.5mA, 1:1mA)
[3]           | INT_PLL  |  1   | PLL mode (0:fractional-N, 1:integer-N)
[2]           | PWRSAV   |  1   | PLL power save
```

### Register 0x04 (DIV) - Bit Fields

```
Source: src/sdr_conf.c:81-82

Bit Position  | Field | Bits | Description
--------------|-------|------|------------------------------------------
[27:13]       | NDIV  | 15   | PLL integer division ratio (36-32767)
[12:3]        | RDIV  | 10   | PLL reference division ratio (1-1023)
```

### Register 0x05 (FDIV) - Bit Fields

```
Source: src/sdr_conf.c:83

Bit Position  | Field | Bits | Description
--------------|-------|------|------------------------------------------
[27:8]        | FDIV  | 20   | PLL fractional division ratio (0-1048575)
```

### Register 0x07 (CLK) - Bit Fields

```
Source: src/sdr_conf.c:84-94

Bit Position  | Field       | Bits | Description
--------------|-------------|------|------------------------------------------
[28]          | EXTADCCLK   |  1   | External ADC clock (0:internal, 1:ADC_CLKIN)
[27:16]       | REFCLK_L_CNT| 12   | Clock pre-divider L counter
[15:4]        | REFCLK_M_CNT| 12   | Clock pre-divider M counter
[3]           | FCLKIN      |  1   | ADC clock divider enable
[2]           | ADCCLK      |  1   | Integer clock selection (0:enable, 1:bypass)
[0]           | MODE        |  1   | DSP interface mode
```

### Register 0x0A (CLK2) - Bit Fields

```
Source: src/sdr_conf.c:91-93

Bit Position  | Field         | Bits | Description
--------------|---------------|------|------------------------------------------
[27:16]       | ADCCLK_L_CNT  | 12   | ADC clock divider L counter
[15:4]        | ADCCLK_M_CNT  | 12   | ADC clock divider M counter
[3]           | PREFRACDIV_SEL|  1   | Clock pre-divider enable
[2]           | CLKOUT_SEL    |  1   | CLKOUT selection
```

---

## Initialization Sequence

### Boot Sequence

```
Source: FE_2CH/FW/v2.1/pocket_fw.c:247-276

void setup(void) {
    // 1. Initialize CPU clock (48MHz for FX2LP)
    CPUCS = 0x12;

    // 2. Configure FIFO endpoints
    EP2FIFOCFG = EP4FIFOCFG = EP6FIFOCFG = EP8FIFOCFG = 0x00;

    // 3. Configure SLAVE_FIFO interface
    IFCONFIG = 0x53;  // IFCLK=EXT, OUT_DIS, POL=INV, SLAVE_FIFO
    REVCTL = 0x03;    // SLAVE-FIFO enabled

    // 4. Configure bulk endpoints
    EP2CFG = 0x20;    // EP2: OFF, OUT, BULK
    EP4CFG = 0x20;    // EP4: OFF, OUT, BULK
    EP6CFG = 0xE0;    // EP6: ON, IN, BULK, 512B, 4X buffered
    EP8CFG = 0x60;    // EP8: OFF, IN, BULK

    // 5. Configure AUTOIN packet length
    EP6AUTOINLENH = 0x02;  // 512 bytes
    EP6AUTOINLENL = 0x00;

    // 6. Reset EP6 FIFO
    FIFORESET = 0x86;
    FIFORESET = 0x00;

    // 7. Initialize SPI pins
    digitalWrite(CSN1, 1);  // CSN high (deselected)
    digitalWrite(CSN2, 1);
    digitalWrite(SCLK, 0);  // SCLK idle low

    // 8. Initialize I2C for EEPROM
    EZUSB_InitI2C();

    // 9. Load default MAX2771 settings
    load_default();

    // 10. Load saved settings from EEPROM (if valid)
    load_settings();

    // 11. Start bulk transfer
    delay(255);
    start_bulk();
}
```

### Default Register Values (FE 2CH)

```
Source: FE_2CH/FW/v2.1/pocket_fw.c:71-76

Channel 1 (L1 Band - High-band):
  REG[0x00] = 0xA2241797  // CHIPEN=1, LNAMODE=0, MIXERMODE=0
  REG[0x01] = 0x20550288  // IQEN=0, 2-bit ADC, sign-magnitude
  REG[0x02] = 0x0E9F21DC  // PGA enabled
  REG[0x03] = 0x69888000  // LOBAND=0 (L1), REFOUTEN=1
  REG[0x04] = 0x00082008  // PLL dividers for 1575.42MHz
  REG[0x05] = 0x0647AE70  // Fractional divider
  REG[0x07] = 0x00000000  // Internal ADC clock
  REG[0x0A] = 0x00000004  // Clock config

Channel 2 (L2/L5 Band - Low-band):
  REG[0x00] = 0xA224A019  // CHIPEN=1, LNAMODE=1, MIXERMODE=1
  REG[0x01] = 0x28550288  // IQEN=1, 2-bit ADC
  REG[0x02] = 0x0E9F31DC  // PGA enabled
  REG[0x03] = 0x78888000  // LOBAND=1 (L2/L5), REFOUTEN=1
  REG[0x04] = 0x00062008  // PLL dividers for L2/L5
  REG[0x05] = 0x004CCD70  // Fractional divider
  REG[0x07] = 0x10000000  // External ADC clock
  REG[0x0A] = 0x00000004  // Clock config
```

### Default Register Values (FE 4CH)

```
Source: FE_4CH/FW/v3.0/pocket_fw_v3/pocket_fw_v3.c:127-136

Channel 1: 0xA2240015, 0x28550288, 0x0EAF31D0, 0x698C0008, 0x0CD22C80, ...
Channel 2: 0xA224A011, 0x28550288, 0x0EAF31D0, 0x198C0008, 0x027F6320, ...
Channel 3: 0xA224A03D, 0x28550288, 0x0EAF31D0, 0x198C0008, 0x03D46500, ...
Channel 4: 0xA224A00D, 0x28550288, 0x0EAF31D0, 0x198C0008, 0x00D52100, ...
```

### Load Settings from EEPROM

```
Source: FE_2CH/FW/v2.1/pocket_fw.c:212-228

EEPROM Layout:
  0x3F00: Header (0xABC00CBA = valid settings present)
  0x3F04: Channel 1 registers (11 × 4 bytes = 44 bytes)
  0x3F30: Channel 2 registers (11 × 4 bytes = 44 bytes)

static void load_settings(void) {
    uint32_t reg;
    uint16_t ee_addr = EE_ADDR_S;  // 0x3F04

    // Check for valid header
    read_eeprom(EE_ADDR_H, 4, &reg);  // 0x3F00
    if (reg != HEAD_REG) return;     // 0xABC00CBA

    // Load all channel registers
    for (uint8_t cs = 0; cs < MAX_CH; cs++) {
        for (uint8_t addr = 0; addr < MAX_ADDR; addr++) {
            read_eeprom(ee_addr, 4, &reg);
            write_reg(cs, addr, reg);
            ee_addr += 4;
        }
    }
}
```

---

## GNSS Band Configurations

### GNSS Frequency Bands

| Band | Signal | Center Frequency | Typical Use |
|------|--------|-----------------|-------------|
| L1 | GPS L1 C/A, Galileo E1, BeiDou B1C, QZSS L1 | 1575.42 MHz | Primary navigation |
| L2 | GPS L2C | 1227.60 MHz | Dual-frequency |
| L5 | GPS L5, Galileo E5a, BeiDou B2a, QZSS L5 | 1176.45 MHz | Safety-of-life |
| E5b | Galileo E5b, BeiDou B2b | 1207.14 MHz | Augmentation |
| L6/E6 | QZSS L6, Galileo E6 | 1278.75 MHz | High-accuracy |
| B3 | BeiDou B3I | 1268.52 MHz | Regional service |
| G1 | GLONASS L1 | 1602.00 MHz (center) | FDMA navigation |
| G2 | GLONASS L2 | 1246.00 MHz (center) | FDMA dual-freq |

### LNA/Mixer Band Selection

```
LNAMODE and MIXERMODE settings:
  0 = High-band (L1, G1, E1)     → 1559-1610 MHz
  1 = Low-band (L2, L5, E5, L6)  → 1164-1300 MHz
  2 = Disabled

LOBAND (Register 0x03, bit 28):
  0 = L1 band VCO range
  1 = L2/L5 band VCO range
```

### PLL Frequency Calculation

```
LO Frequency Formula:
  F_LO = F_TCXO / RDIV × (NDIV + FDIV / 2^20)

Where:
  F_TCXO = TCXO frequency (typically 24 MHz)
  RDIV   = Reference divider (1-1023)
  NDIV   = Integer divider (36-32767)
  FDIV   = Fractional divider (0-1048575)

For Integer-N mode (INT_PLL=1): FDIV is not used
For Fractional-N mode (INT_PLL=0): Full fractional synthesis
```

### ADC Sampling Rate Calculation

```
Source: src/sdr_conf.c:287-322

F_ADC = F_TCXO × PreDiv × IntDiv × PostDiv

PreDiv (PREFRACDIV_SEL=1):
  = REFCLK_L_CNT / (4096 - REFCLK_M_CNT + REFCLK_L_CNT)

IntDiv (ADCCLK=0):
  = REFDIV ratio: 0→×2, 1→÷4, 2→÷2, 3→×1, 4→×4

PostDiv (FCLKIN=1):
  = ADCCLK_L_CNT / (4096 - ADCCLK_M_CNT + ADCCLK_L_CNT)
```

---

## USB Vendor Commands

### Command Summary

| Code | Direction | wValue | Bytes | Description |
|------|-----------|--------|-------|-------------|
| 0x40 | IN | - | 6 | Get device status |
| 0x41 | IN | CH<<8 + Addr | 4 | Read MAX2771 register |
| 0x42 | OUT | CH<<8 + Addr | 4 | Write MAX2771 register |
| 0x44 | OUT | - | 0 | Start bulk transfer |
| 0x45 | OUT | - | 0 | Stop bulk transfer |
| 0x46 | OUT | - | 0 | Reset device |
| 0x47 | OUT | - | 0 | Save settings to EEPROM |
| 0x48 | IN | Address | n | Read EEPROM (n ≤ 64) |
| 0x49 | OUT | Address | n | Write EEPROM (n ≤ 64) |

### Device Status Response (0x40)

```
Byte 0: Firmware version (0x10=2CH, 0x30=4CH, 0x40=8CH)
Byte 1: TCXO frequency MSB (kHz)
Byte 2: TCXO frequency LSB (kHz)
Byte 3: Status flags
Byte 4: PLL lock status (channel-dependent)
Byte 5: Bulk transfer status
```

### Register Read/Write (0x41/0x42)

```
wValue format: (CH << 8) | RegisterAddress

Example - Read CH1 Register 0x04:
  wValue = (0 << 8) | 0x04 = 0x0004

Example - Write CH2 Register 0x00:
  wValue = (1 << 8) | 0x00 = 0x0100
  Data: 4 bytes (32-bit register value, big-endian)
```

### Host-Side Register Access

```
Source: src/sdr_conf.c:445-475

// Read register via USB
static uint32_t read_reg(sdr_usb_t *usb, int ch, int addr) {
    uint8_t data[4] = {0};

    sdr_usb_req(usb, 0, SDR_VR_REG_READ,
                (uint16_t)((ch << 8) + addr), data, 4);

    uint32_t val = 0;
    for (int i = 0; i < 4; i++) {
        val = (val << 8) | data[i];
    }
    return val;
}

// Write register via USB
static void write_reg(sdr_usb_t *usb, int ch, int addr, uint32_t val) {
    uint8_t data[4];

    for (int i = 0; i < 4; i++) {
        data[i] = (uint8_t)(val >> (3 - i) * 8);
    }
    sdr_usb_req(usb, 1, SDR_VR_REG_WRITE,
                (uint16_t)((ch << 8) + addr), data, 4);
}
```

---

## Configuration Examples

### Example 1: GPS L1 (1575.42 MHz, 24 MHz ADC, IQ)

```
Configuration: pocket_L1L5_24MHz.conf (Channel 1)

[CH1]
LNAMODE   = 0    # High-band LNA
MIXERMODE = 0    # High-band mixer
FBW       = 2    # 4.2 MHz bandwidth
F3OR5     = 1    # 3rd order filter
FCENX     = 0    # Lowpass mode
FGAIN     = 1    # Normal gain
IQEN      = 1    # I/Q enabled
GAINREF   = 170  # AGC reference
AGCMODE   = 0    # Automatic AGC
GAININ    = 58   # PGA gain
PGAIEN    = 1    # I-channel enabled
PGAQEN    = 1    # Q-channel enabled
LOBAND    = 0    # L1 band VCO
INT_PLL   = 1    # Integer-N mode
NDIV      = 26257  # Integer divider
RDIV      = 400    # Reference divider

PLL Calculation:
  F_LO = 24 MHz / 400 × 26257 = 1575.42 MHz ✓

ADC Clock (master channel):
  F_ADC = 24 MHz × 1 (bypass all dividers) = 24 MHz
```

### Example 2: GPS L5/Galileo E5a (1176.45 MHz, IQ)

```
Configuration: pocket_L1L5_24MHz.conf (Channel 2)

[CH2]
LNAMODE   = 1    # Low-band LNA
MIXERMODE = 1    # Low-band mixer
FBW       = 7    # 16.4 MHz bandwidth
F3OR5     = 1    # 3rd order filter
FCENX     = 0    # Lowpass mode
IQEN      = 1    # I/Q enabled
LOBAND    = 1    # L2/L5 band VCO
INT_PLL   = 1    # Integer-N mode
NDIV      = 7843   # Integer divider
RDIV      = 160    # Reference divider

PLL Calculation:
  F_LO = 24 MHz / 160 × 7843 = 1176.45 MHz ✓

Note: CH2 uses external ADC clock from CH1 (EXTADCCLK=1)
```

### Example 3: GPS L1/L2 with Low IF (8 MHz ADC)

```
Configuration: pocket_L1L2_8MHz.conf

[CH1] - GPS L1 with 2 MHz IF
LNAMODE   = 0      # High-band
MIXERMODE = 0      # High-band
FCEN      = 107    # IF center = (128-107)/2 × 0.195 = 2.0475 MHz
FBW       = 0      # 2.5 MHz bandwidth
F3OR5     = 0      # 5th order filter
FCENX     = 1      # Bandpass mode
IQEN      = 0      # I-only (real sampling)
LOBAND    = 0      # L1 band
INT_PLL   = 0      # Fractional-N mode
NDIV      = 65
RDIV      = 1
FDIV      = 586329

PLL Calculation:
  F_LO = 24 MHz / 1 × (65 + 586329/1048576)
       = 24 × 65.559098 = 1573.42 MHz

IF Frequency:
  F_IF = 1575.42 - 1573.42 = 2.0 MHz ✓

ADC Clock:
  PREFRACDIV_SEL = 1
  REFCLK_L_CNT = 2048
  F_ADC = 24 MHz × (2048 / (4096 - 0 + 2048))
        = 24 × (2048/6144) = 8.0 MHz ✓
```

### Example 4: Multi-constellation (L1/G1/L5/L6, 16 MHz)

```
Configuration: pocket_L1G1L5L6_16MHz.conf

Channel allocations:
  CH1: L1/G1 shared (1568 MHz center, 16.4 MHz BW)
  CH2: GLONASS G1 (1602 MHz, 8.7 MHz BW)
  CH3: L5/E5a (1176.45 MHz, 16.4 MHz BW)
  CH4: L6/E6 (1278.75 MHz, 16.4 MHz BW)

[CH1] - GPS L1 + GLONASS G1
F_LO = 1568.000 MHz (between L1 and G1)
FBW  = 7  # 16.4 MHz to cover both L1 and G1

[CH3] - GPS L5 / Galileo E5a
F_LO = 1176.450 MHz
LNAMODE = 1, LOBAND = 1

[CH4] - QZSS L6 / Galileo E6
F_LO = 1278.750 MHz
NDIV = 1705, RDIV = 32
F_LO = 24 MHz / 32 × 1705 = 1278.75 MHz ✓
```

### Example 5: Wide-band All-signal (48 MHz ADC)

```
Configuration: pocket_ALL_48MHz.conf

[CH1] - Upper L-band: L1/E1/B1C/G1
F_LO = 1588.000 MHz
FBW  = 4  # 36 MHz bandwidth (covers 1559-1610 MHz)
LNAMODE = 0, LOBAND = 0
INT_PLL = 0  # Fractional-N for precise tuning

ADC Clock:
  F_ADC = 24 MHz × 2 (REFDIV=0) = 48 MHz

[CH2] - L2 band
F_LO = 1237.000 MHz
FBW  = 4  # 36 MHz bandwidth

[CH3] - L5/E5 band
F_LO = 1191.000 MHz
FBW  = 4  # 36 MHz bandwidth

[CH4] - L6/B3 band
F_LO = 1273.000 MHz
FBW  = 4  # 36 MHz bandwidth
```

---

## Runtime Configuration API

### Set LNA Gain

```
Source: src/sdr_dev.c:442-465

int sdr_dev_set_gain(sdr_dev_t *dev, int ch, int gain) {
    // gain > 0: Manual gain (1-64 dB)
    // gain = 0: AGC mode

    // Read current registers
    sdr_usb_req(dev->usb, 0, SDR_VR_REG_READ, (ch << 8) + 1, reg1, 4);
    sdr_usb_req(dev->usb, 0, SDR_VR_REG_READ, (ch << 8) + 2, reg2, 4);

    if (gain > 0) {
        reg1[2] = (reg1[2] & ~0x18) + (2 << 3);  // AGCMODE = 2 (manual)
        reg2[0] = (reg2[0] & ~0x0F) + (((gain-1) >> 2) & 0x0F);  // GAININ[5:2]
        reg2[1] = (reg2[1] & ~0xC0) + (((gain-1) << 6) & 0xC0);  // GAININ[1:0]
    } else {
        reg1[2] = (reg1[2] & ~0x18);  // AGCMODE = 0 (auto)
    }

    // Write back
    sdr_usb_req(dev->usb, 1, SDR_VR_REG_WRITE, (ch << 8) + 1, reg1, 4);
    sdr_usb_req(dev->usb, 1, SDR_VR_REG_WRITE, (ch << 8) + 2, reg2, 4);
}
```

### Set IF Filter

```
Source: src/sdr_dev.c:487-517

int sdr_dev_set_filt(sdr_dev_t *dev, int ch, double bw, double freq, int order) {
    // Bandwidth options: 2.5, 8.7, 4.2, 23.4, 36.0, 16.4 MHz
    // freq: Center frequency for bandpass mode (0 = lowpass)
    // order: 0 = 5th order, 1 = 3rd order

    // Find FBW code
    for (FBW = 0; freqs[FBW] >= 0.0; FBW++) {
        if (fabs(freqs[FBW] - bw) < 0.1) break;
    }

    // Calculate FCEN for bandpass mode
    if (FBW < 3 && freq > 0.0) {
        FCENX = 1;  // Bandpass
        FCEN = (uint8_t)(128.0 - freq / fstep[FBW] * 2.0 + 0.5) & 0x7F;
    }

    // Update register 0x00
    reg[2] = (reg[2] & ~0x1F) + (FCEN >> 2);
    reg[3] = (reg[3] & ~0x38) + (FBW << 3);
    reg[3] = (reg[3] & ~0x04) + (F3OR5 << 2);
    reg[3] = (reg[3] & ~0xC0) + (FCEN << 6);
    reg[3] = (reg[3] & ~0x02) + (FCENX << 1);
}
```

---

## Configuration File Format

### Keyword-Value Format

```
# Comment line
[CH1]
FIELD_NAME = value  # Inline comment

Example:
[CH1]
LNAMODE   = 0     # High-band
FCEN      = 107   # IF center frequency
FBW       = 2     # 4.2 MHz bandwidth
NDIV      = 26257 # PLL integer divider
```

### Hexadecimal Format

```
#CH  ADDR       VALUE
  1  0x00  0xA2241C17
  1  0x01  0x20550288
  1  0x02  0x0E9F21DC
  ...
  2  0x00  0xA224A019
  2  0x01  0x28550288
```

### Command Line Usage

```bash
# Read current configuration
./pocket_conf

# Read all register fields
./pocket_conf -a

# Read in hexadecimal format
./pocket_conf -h

# Write configuration
./pocket_conf config_file.conf

# Write and save to EEPROM
./pocket_conf -s config_file.conf

# Specify USB device
./pocket_conf -p 1,2 config_file.conf
```

---

## Appendix: Register Value Decoder

### Decode Register 0x00 (CONF1)

```python
def decode_conf1(reg):
    print(f"CHIPEN    = {(reg >> 31) & 1}")
    print(f"IDLE      = {(reg >> 30) & 1}")
    print(f"MIXPOLE   = {(reg >> 17) & 1}")
    print(f"LNAMODE   = {(reg >> 15) & 3}")
    print(f"MIXERMODE = {(reg >> 13) & 3}")
    print(f"FCEN      = {(reg >> 6) & 0x7F}")
    print(f"FBW       = {(reg >> 3) & 7}")
    print(f"F3OR5     = {(reg >> 2) & 1}")
    print(f"FCENX     = {(reg >> 1) & 1}")
    print(f"FGAIN     = {(reg >> 0) & 1}")

# Example: decode_conf1(0xA2241797)
```

### Calculate LO Frequency

```python
def calc_lo_freq(f_tcxo, rdiv, ndiv, fdiv, int_pll):
    if int_pll:
        return f_tcxo / rdiv * ndiv
    else:
        return f_tcxo / rdiv * (ndiv + fdiv / 2**20)

# Example: GPS L1
f_lo = calc_lo_freq(24.0, 400, 26257, 0, True)
print(f"F_LO = {f_lo} MHz")  # 1575.42 MHz
```

### Calculate ADC Sample Rate

```python
def calc_adc_rate(f_tcxo, prefrac_sel, refclk_l, refclk_m,
                   adcclk, refdiv, fclkin, adcclk_l, adcclk_m):
    ratio = [2.0, 0.25, 0.5, 1.0, 4.0]

    f = f_tcxo
    if prefrac_sel:
        f *= refclk_l / (4096 - refclk_m + refclk_l)
    if not adcclk:
        f *= ratio[refdiv]
    if fclkin:
        f *= adcclk_l / (4096 - adcclk_m + adcclk_l)
    return f

# Example: 24 MHz ADC
f_adc = calc_adc_rate(24.0, 0, 0, 0, 0, 3, 0, 0, 0)
print(f"F_ADC = {f_adc} MHz")  # 24.0 MHz
```

---

## References

1. MAX2771 Datasheet - Maxim Integrated (now Analog Devices)
2. PocketSDR GitHub Repository: https://github.com/tomojitakasu/PocketSDR
3. EZ-USB FX2LP Technical Reference Manual, Cypress/Infineon
4. EZ-USB FX3 Technical Reference Manual, Cypress/Infineon

---

*Document generated from PocketSDR source code analysis*
*Version: Based on PocketSDR v0.14*
