<div align="center">

# ESP32 Scientific Calculator

### A physical scientific calculator built from scratch on NodeMCU-32S with a 9×4 key matrix, LCD1602 display, and a fully custom expression evaluator — no calculator libraries used.

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Platform](https://img.shields.io/badge/platform-ESP32%20%7C%20NodeMCU--32S-red.svg)
![Language](https://img.shields.io/badge/language-C%2FC%2B%2B%20(Arduino)-brightgreen.svg)
![Status](https://img.shields.io/badge/status-Working-success.svg)

</div>

---

## What This Is

A physical scientific calculator running on a NodeMCU-32S (ESP32), driven entirely by custom firmware — no calculator library, no expression parser library, no math shortcut. The expression evaluator implements Dijkstra's shunting-yard algorithm from scratch in C, the trig functions use hand-written Taylor series instead of stdlib, and the matrix keyboard scanner uses active-LOW row driving with external pull-ups on input-only GPIO pins. Built as a learning project to understand embedded systems from first principles: GPIO configuration, I2C peripheral control, debounce logic, and event-driven input handling on bare hardware.

---

## Features

- **Expression mode** — type and evaluate arbitrary math expressions: `sin(30)+2^3*(pi/4)`
- **Degree-based trig** — `sin`, `cos`, `tan`, `asin`, `acos`, `atan`, `csc`, `sec`, `cot` via Taylor series
- **Math functions** — `sqrt`, `log` (base-10), `ln`, `e`, `pi`
- **Equation solvers** — Linear, Quadratic, Cubic (Newton-Raphson + synthetic division), Cramer's Rule 2×2, Cramer's Rule 3×3
- **SHIFT layer** — secondary functions on existing keys (`csc(`, `sec(`, `cot(`, clear, recall answer)
- **Expression scroll** — expressions longer than 16 chars scroll horizontally on the LCD
- **Runtime secret code change** — `SHIFT + .` lets you change the trigger code without reflashing
- **UART debug output** — every keypress logged with timestamp to Serial at 115200 baud

---

## Hardware Required

| Component | Specification | Notes |
|-----------|--------------|-------|
| Microcontroller | NodeMCU-32S (ESP32, 38-pin) | Any ESP32 devboard works with pin remapping |
| Display | LCD1602 with PCF8574 I2C backpack | I2C address `0x27` (A0=A1=A2=HIGH) |
| Tactile switches | 6×6mm through-hole, 36 total | 9 rows × 4 cols |
| Pull-up resistors | 10 kΩ, 4 total | Required on COL pins — GPIO 34/35/36/39 have no internal pull-up |
| Breadboard / PCB | Full-size breadboard or custom PCB | — |
| Power | USB via NodeMCU-32S | LCD VCC must go to 5V rail, not 3.3V |

---

## Pin Connections

### LCD (I2C via PCF8574 backpack)

| LCD Pin | ESP32 Pin | Notes |
|---------|-----------|-------|
| SDA | GPIO 21 | Hardware I2C SDA |
| SCL | GPIO 22 | Hardware I2C SCL |
| VCC | 5V | PCF8574 needs 5V — 3.3V causes dim/blank display |
| GND | GND | — |

### Key Matrix

| Signal | GPIO Pins | Direction | Notes |
|--------|-----------|-----------|-------|
| ROW 0–8 | 32, 33, 25, 26, 27, 14, 12, 13, 2 | Output | Driven LOW one at a time during scan |
| COL 0–3 | 34, 35, 36, 39 | Input | **Must have 10 kΩ pull-up to 3.3V** — no internal pull-up on these pins |

---

## Getting Started

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) 2.x or [PlatformIO](https://platformio.org/)
- ESP32 board package installed (`esp32` by Espressif, v2.x)
- Library: `LiquidCrystal_I2C` by Frank de Brabander (install via Library Manager)

### Clone and Flash

```bash
git clone https://github.com/AnshMi2674/TSC_V1.git
cd TSC_V1
```

Open `ScientificCalculator.ino` in Arduino IDE.

**Board settings:**
```
Board   : NodeMCU-32S  (or ESP32 Dev Module)
Upload Speed : 115200
Port    : your COM/tty port
```

Place `formula_data.h` in the same folder as `ScientificCalculator.ino` before compiling. Click Upload.

### Verify

Open Serial Monitor at **115200 baud**. On power-up you should see:
```
Calculator starting...
```
Every keypress prints its code and timestamp:
```
1042 Key: 26
1093 Key: 32
```

---

## How It Works

### Expression Evaluator — Shunting-Yard Algorithm

The core of the calculator is a two-stack implementation of Dijkstra's shunting-yard algorithm. When the user presses `=` in EXPR mode, `evaluate_expression()` first calls `tokenize()` which walks the raw character buffer and produces a `Token[]` array — each token is a struct with a type tag (`TOKEN_NUMBER`, `TOKEN_OPERATOR`, `TOKEN_IDENTIFIER`, etc.) and a union holding the actual value.

`evaluate()` then processes that token array using three stacks — a value stack (`ValStack`), an operator stack (`OpStack`), and a function-name stack (`FnStack`). Numbers push directly to `ValStack`. Operators check the precedence of whatever is already on `OpStack` and pop-and-apply any higher-or-equal precedence operators before pushing themselves (with the exception of `^` which is right-associative, so it only pops strictly higher). Left parens push as barriers; right parens flush operators down to the matching barrier, then check if a function name is waiting on `FnStack` — if so, it dispatches to `dispatch_function()` with the top of `ValStack` as the argument.

One non-obvious bug fixed during development: Arduino local structs are not zero-initialised. Without `memset(&v, 0, sizeof(v))` before use, `ValStack.data[]` contained garbage that corrupted results when the stack was shallow (e.g. `7-5` would return `1073741824` instead of `2`).

### Keyboard Scanner — Active-LOW Matrix

The 9×4 matrix uses active-LOW row driving: each ROW pin is driven LOW one at a time while the firmware reads all COL pins. A pressed key connects the active ROW to a COL, pulling the COL pin LOW through the switch. COL pins sit HIGH normally via 10 kΩ external resistors to 3.3V. GPIO 34, 35, 36, 39 on the ESP32 are input-only with no internal pull-up hardware — using `INPUT_PULLUP` on these pins in software silently does nothing, which is why external resistors are mandatory.

A 10 µs `delayMicroseconds(10)` settle time is inserted after driving each ROW LOW to let the voltage stabilise before reading — parasitic capacitance on the COL traces can cause false reads without this.

### Trig Without stdlib

`sin_deg()` and `cos_deg()` use Taylor series computed iteratively using the ratio between consecutive terms rather than computing factorials explicitly:

```
sin(x) = x - x³/3! + x⁵/5! - ...
ratio between term i and term i-1: -x² / ((2i)(2i+1))
```

The angle is first normalised to `(-π, π]` by `norm_angle()` to ensure the series converges within 7 iterations to better than 1e-9 accuracy. This avoids including `<math.h>` trig for the forward functions — only the inverse trig (`asin`, `acos`, `atan`) delegates to stdlib since their series are less well-conditioned.

---

## Project Structure

```
ESP32-SciCalc/
├── SciCalculator.ino   # Main firmware — all sections in one file
├── Schematic.png            
└── README.md
```

**Sections inside `ScientificCalculator.ino`:**

| Section | What it contains |
|---------|-----------------|
| Type Definitions | `Token`, `EqSol`, `CubicSol`, `Cr2Sol`, `Cr3Sol` structs |
| 1 — Hardware Config | Pin arrays, LCD object |
| 2 — Key Codes & Layout | `#define KEY_*` constants, `key_layout[9][4]` |
| 3 — Keyboard Scan & Debounce | `scan_keyboard()`, `get_key()` |
| 4 — Tokenizer | `tokenize()` — string → Token array |
| 5 — Trigonometry | Taylor series trig, inverse trig, log, sqrt |
| 6 — Function Dispatch | `dispatch_function()` — name → math function |
| 7 — Shunting-Yard Evaluator | `evaluate()` — Token array → double result |
| 8 — Solvers | Linear, Quadratic, Cubic, Cramer 2×2, Cramer 3×3 |
| 9 — Application State | Global expression buffer, mode, scroll state |
| 10 — LCD Helpers | `display_update()`, `display_line2()` |
| 11 — Expression Buffer | `expr_append()`, `expr_backspace()`, `expr_clear()` |
| 12 — Coefficient Input | `collect_coeff()` — blocking loop for solver input |
| 13 — Change Secret Code | `change_sCode()` — blocking loop for code change |
| 14 — Solver Runners | `run_linear()`, `run_quadratic()`, `run_cubic()`, `run_cramer2()`, `run_cramer3()` |
| 15 — Expression Evaluation | `evaluate_expression()` — top-level eval + secret trigger check |
| 16 — Key Handler | `handle_key()` — full key routing with priority order |
| 17 — Setup & Loop | `setup()`, `loop()` |

---

## Learnings & Challenges

**Uninitialized stack memory** — The most subtle bug. `7 - 5` was returning `1073741824`. Root cause: Arduino local structs contain garbage. `ValStack v; v.top = -1;` leaves `v.data[]` full of random values. On a 2-token expression the stack never goes deep enough to overwrite all slots, so `apply_op()` read garbage. Fix: `memset(&v, 0, sizeof(v))` before use.

**GPIO 34/35/36/39 have no internal pull-up** — `pinMode(pin, INPUT_PULLUP)` silently does nothing on these four pins. The matrix COL lines floated HIGH sometimes and LOW sometimes, producing phantom keypresses. Once external 10 kΩ resistors were added to 3.3V the behaviour became deterministic.

**Wokwi double-press simulation** — The simulator fires two rapid GPIO transitions per mouse click, faster than any debounce timer can catch. The standard debounce pattern (track last key + timestamp) does not help because both transitions arrive within microseconds. A 300ms lockout after any confirmed keypress was the working fix for simulation. On real hardware the 50ms debounce is sufficient.

**Blocking vs event-driven input** — `change_sCode()` was originally written as a single-shot handler (called once per keypress, returned immediately). It could never accumulate multiple characters because SHIFT was consumed on the first call and the function was never called again for subsequent digits. The fix was converting it to a blocking loop like `collect_coeff()` — stay inside until the user presses `=` or `BCK`.

**`char` vs `unsigned char` with `isdigit()`/`isalpha()`** — Passing a plain `char` to `isdigit()` causes undefined behaviour when the value is negative (characters > 127 on some encodings). All tokenizer checks cast explicitly to `(unsigned char)`.

**Safe integer display** — `result == (long long)result` to check for whole numbers caused overflow on large doubles (casting `1e18` to `long long` wraps). Fixed by bounding the check: `result >= -2147483648.0 && result <= 2147483647.0 && result == (long)result`.

---

## Roadmap

- [ ] Rewrite firmware as bare-metal ESP-IDF v5 (no Arduino layer) after completing FreeRTOS study
- [ ] Add PCB design (KiCad) to replace breadboard
- [ ] Add complex number support to the expression evaluator
- [ ] Add matrix operations mode
- [ ] Add unit conversion utility
- [ ] Add statistics mode (mean, variance, standard deviation)
- [ ] Persistent secret code storage in NVS (currently resets on power cycle)

---

## License

MIT — see [LICENSE](LICENSE) for details.

---

<div align="center">

Built by **Ansh Mishra**

[GitHub](https://github.com/AnshMi2674) · [PC Version (original expression engine)](https://github.com/AnshMi2674/PC_SciCalc)

</div>
