/*
 * ============================================================
 *  ESP32 Scientific Calculator
 *  Author  : Ansh Mishra
 *  Hardware: NodeMCU-32S · LCD1602 I2C (0x27) · 9×4 key matrix
 *  License : MIT
 * ============================================================
 *
 *  PIN ASSIGNMENT
 *  ──────────────
 *  LCD  : SDA = GPIO21   SCL = GPIO22   VCC = 5V
 *  ROW  : GPIO 32 33 25 26 27 14 12 13 2   (outputs)
 *  COL  : GPIO 34 35 36 39                  (inputs, needs external 10kΩ pull-up to 3.3V)
 *
 *  KEY MATRIX  (ROW × COL)
 *  ────────────────────────
 *  ROW0 :  .       ←      →      ⌫
 *  ROW1 :  SHIFT   MODE   √      PWR
 *  ROW2 :  sin     cos    tan    ln
 *  ROW3 :  asin    acos   atan   log
 *  ROW4 :  ^       1      2      3
 *  ROW5 :  (       4      5      6
 *  ROW6 :  )       7      8      9
 *  ROW7 :  π       e      0      =
 *  ROW8 :  +       -      ×      ÷
 *
 *  SHIFT COMBINATIONS
 *  ───────────────────
 *  SHIFT + sin     →  csc(
 *  SHIFT + cos     →  sec(
 *  SHIFT + tan     →  cot(
 *  SHIFT + PWR     →  Clear all
 *  SHIFT + =       →  Recall last answer
 * *
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>


/* ════════════════════════════════════════════════════════════
   SHARED TYPE DEFINITIONS
   These structs are used across multiple sections.
   Defined here so the compiler sees them before any function.
   ════════════════════════════════════════════════════════════ */

#define MAX_IDENT_LEN 16   /* max length of a function name e.g. "asin" */

/*
 * Token — one unit produced by the tokenizer.
 * type  : what kind of token (number / operator / paren / identifier / end)
 * value : the actual data — a number, a single operator char, or a name string.
 *         Only one member of the union is valid at a time, chosen by 'type'.
 */
typedef struct {
    int type;
    union {
        double number;              /* used when type == TOKEN_NUMBER     */
        char   oper;                /* used when type == TOKEN_OPERATOR   */
        char   ident[MAX_IDENT_LEN];/* used when type == TOKEN_IDENTIFIER */
    } value;
} Token;

/* Return types for the equation solvers */
typedef struct { int result; double r1, r2; }    EqSol;    /* linear / quadratic */
typedef struct { int num_roots; double roots[3]; } CubicSol; /* cubic              */
typedef struct { int result; double x, y; }       Cr2Sol;   /* Cramer 2×2         */
typedef struct { int ok;     double x, y, z; }    Cr3Sol;   /* Cramer 3×3         */


/* ════════════════════════════════════════════════════════════
   SECTION 1 — HARDWARE CONFIGURATION
   All pin assignments live here. Change wiring? Change here.
   ════════════════════════════════════════════════════════════ */

#define KB_ROWS 9
#define KB_COLS 4

/* ROW pins are driven LOW one at a time during a scan (outputs). */
const int ROW_PINS[KB_ROWS] = { 32, 33, 25, 26, 27, 14, 12, 13, 2 };

/*
 * COL pins are read during a scan (inputs).
 * GPIO 34/35/36/39 are INPUT-ONLY on the ESP32 — they have no internal
 * pull-up hardware. You MUST place 10 kΩ resistors from each COL pin to
 * 3.3 V on the PCB. Without them the pins float and phantom presses occur.
 */
const int COL_PINS[KB_COLS] = { 34, 35, 36, 39 };

#define LCD_ADDR 0x27   /* PCF8574 I2C address — set by A0/A1/A2 = HIGH on the backpack */
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);


/* ════════════════════════════════════════════════════════════
   SECTION 2 — KEY CODES AND MATRIX LAYOUT
   Each physical key maps to a unique integer code.
   key_layout[r][c] gives the code for the key at row r, col c.
   ════════════════════════════════════════════════════════════ */

#define KEY_NONE      0
#define KEY_DECIMAL   1   /* decimal point '.'  */
#define KEY_ARROW_LT  2   /* scroll left / formula scroll */
#define KEY_ARROW_RT  3   /* scroll right / formula scroll */
#define KEY_BCK       4   /* backspace */
#define KEY_SHIFT     5
#define KEY_MODE      6
#define KEY_SQRT      7
#define KEY_PWR       8   /* clear (SHIFT+PWR) or power key */
#define KEY_SIN       9
#define KEY_COS       10
#define KEY_TAN       11
#define KEY_LN        12
#define KEY_ARCSIN    13
#define KEY_ARCCOS    14
#define KEY_ARCTAN    15
#define KEY_LOG       16
#define KEY_POWER     17  /* ^ operator */
#define KEY_N1        18
#define KEY_N2        19
#define KEY_N3        20
#define KEY_LPAREN    21
#define KEY_N4        22
#define KEY_N5        23
#define KEY_N6        24
#define KEY_RPAREN    25
#define KEY_N7        26
#define KEY_N8        27
#define KEY_N9        28
#define KEY_PI        29
#define KEY_E_CONST   30
#define KEY_N0        31
#define KEY_EQUAL     32
#define KEY_ADDN      33  /* + */
#define KEY_SUBN      34  /* - */
#define KEY_MULT      35  /* * */
#define KEY_DIVI      36  /* / */

/* 2-D array: rows = physical rows, cols = physical cols.
   This is the ONLY place wiring meets key names.
   Rewire a button? Change its entry here. */
const int key_layout[KB_ROWS][KB_COLS] = {
    { KEY_DECIMAL, KEY_ARROW_LT, KEY_ARROW_RT, KEY_BCK   },
    { KEY_SHIFT,   KEY_MODE,     KEY_SQRT,     KEY_PWR   },
    { KEY_SIN,     KEY_COS,      KEY_TAN,      KEY_LN    },
    { KEY_ARCSIN,  KEY_ARCCOS,   KEY_ARCTAN,   KEY_LOG   },
    { KEY_POWER,   KEY_N1,       KEY_N2,       KEY_N3    },
    { KEY_LPAREN,  KEY_N4,       KEY_N5,       KEY_N6    },
    { KEY_RPAREN,  KEY_N7,       KEY_N8,       KEY_N9    },
    { KEY_PI,      KEY_E_CONST,  KEY_N0,       KEY_EQUAL },
    { KEY_ADDN,    KEY_SUBN,     KEY_MULT,     KEY_DIVI  },
};


/* ════════════════════════════════════════════════════════════
   SECTION 3 — KEYBOARD SCANNING AND DEBOUNCE
   ════════════════════════════════════════════════════════════ */

#define DEBOUNCE_MS 50   /* ignore re-triggers within 50 ms of a confirmed press */

/* Debounce state — persists between loop() calls */
int  last_key        = KEY_NONE;
long last_press_time = 0;
bool key_handled     = false;

/*
 * scan_keyboard()
 * Drives each ROW LOW one at a time and reads all COL pins.
 * If a COL reads LOW while a ROW is LOW, that key is pressed.
 * Returns the key code, or KEY_NONE if nothing is pressed.
 *
 * Why active-LOW?
 *   COL pins are pulled HIGH by external 10kΩ resistors.
 *   Pressing a key connects COL to the active (LOW) ROW,
 *   pulling the COL LOW — detectable by digitalRead() == LOW.
 */
int scan_keyboard()
{
    for (int r = 0; r < KB_ROWS; r++)
    {
        digitalWrite(ROW_PINS[r], LOW);    /* activate this row   */
        delayMicroseconds(10);             /* let voltage settle (parasitic capacitance) */

        for (int c = 0; c < KB_COLS; c++)
        {
            if (digitalRead(COL_PINS[c]) == LOW)
            {
                digitalWrite(ROW_PINS[r], HIGH);   /* restore row before returning */
                return key_layout[r][c];
            }
        }

        digitalWrite(ROW_PINS[r], HIGH);   /* restore row before scanning next */
    }
    return KEY_NONE;
}

/*
 * get_key()
 * Wraps scan_keyboard() with software debounce.
 * Returns a key code exactly ONCE per physical press.
 *
 * Debounce logic:
 *   - A new key (or the same key after DEBOUNCE_MS) resets the timer
 *     and marks the key as unhandled.
 *   - The first time we see an unhandled key we report it and set
 *     key_handled=true so subsequent scans (while still held) are ignored.
 *   - When no key is pressed, state is reset so the next press is fresh.
 */
int get_key()
{
    int key = scan_keyboard();

    if (key != KEY_NONE)
    {
        long now = millis();

        /* New key pressed, or same key pressed again after debounce window */
        if (key != last_key || (now - last_press_time) > DEBOUNCE_MS)
        {
            last_key         = key;
            last_press_time  = now;
            key_handled      = false;   /* ready to report */
        }

        /* Report the key exactly once per press */
        if (!key_handled)
        {
            key_handled = true;
            return key;
        }
    }
    else
    {
        /* No key held — reset so next press is treated fresh */
        last_key    = KEY_NONE;
        key_handled = false;
    }

    return KEY_NONE;
}


/* ════════════════════════════════════════════════════════════
   SECTION 4 — TOKENIZER
   Converts the raw expression string (e.g. "sin(30)+2")
   into an array of Token structs for the evaluator.
   ════════════════════════════════════════════════════════════ */

#define MAX_TOKENS   64

/* Token type codes */
#define TOKEN_NUMBER     0   /* a numeric literal: 3, 3.14, .5  */
#define TOKEN_OPERATOR   1   /* +  -  *  /  ^                   */
#define TOKEN_LPAREN_T   2   /* (                                */
#define TOKEN_RPAREN_T   3   /* )                                */
#define TOKEN_IDENTIFIER 4   /* function name: sin, cos, sqrt …  */
#define TOKEN_END        5   /* sentinel — marks end of token array */

/*
 * tokenize()
 * Scans 'input' left-to-right and fills 'tokens[]'.
 * Returns number of tokens produced (including TOKEN_END),
 * or -1 on error (unknown character or overflow).
 *
 * Special cases:
 *   "pi"  → TOKEN_NUMBER  (3.14159…)
 *   "e"   → TOKEN_NUMBER  (2.71828…)
 *   other letters → TOKEN_IDENTIFIER (function name passed to evaluator)
 */
int tokenize(const char *input, Token *tokens, int max)
{
    int pos = 0, count = 0;

    while (input[pos])
    {
        /* Skip whitespace */
        if (isspace((unsigned char)input[pos]))
        {
            pos++;
            continue;
        }

        /* Overflow guard */
        if (count >= max - 1)
            return -1;

        /* ── Numeric literal (integer or decimal) ── */
        if (isdigit((unsigned char)input[pos]) || input[pos] == '.')
        {
            char *end;
            tokens[count].type         = TOKEN_NUMBER;
            /* strtod() reads as many digits/dots as it can and sets 'end'
               to the first character it did NOT consume. */
            tokens[count].value.number = strtod(&input[pos], &end);
            pos = end - input;   /* advance past the number */
            count++;
            continue;
        }

        /* ── Alphabetic — function name or constant ── */
        if (isalpha((unsigned char)input[pos]))
        {
            int  len = 0;
            char ident[MAX_IDENT_LEN] = {0};

            /* Read the full word */
            while (isalpha((unsigned char)input[pos]) && len < MAX_IDENT_LEN - 1)
                ident[len++] = input[pos++];
            ident[len] = '\0';

            /* Treat "pi" and "e" as numeric constants */
            if (strcmp(ident, "pi") == 0)
            {
                tokens[count].type         = TOKEN_NUMBER;
                tokens[count].value.number = 3.14159265358979323846;
            }
            else if (strcmp(ident, "e") == 0)
            {
                tokens[count].type         = TOKEN_NUMBER;
                tokens[count].value.number = 2.71828182845904523536;
            }
            else
            {
                /* Everything else is a function name (sin, cos, sqrt …) */
                tokens[count].type = TOKEN_IDENTIFIER;
                strncpy(tokens[count].value.ident, ident, MAX_IDENT_LEN);
            }

            count++;
            continue;
        }

        /* ── Parentheses ── */
        if (input[pos] == '(') { tokens[count++].type = TOKEN_LPAREN_T; pos++; continue; }
        if (input[pos] == ')') { tokens[count++].type = TOKEN_RPAREN_T; pos++; continue; }

        /* ── Arithmetic operators ── */
        if (input[pos] == '+' || input[pos] == '-' ||
            input[pos] == '*' || input[pos] == '/' || input[pos] == '^')
        {
            tokens[count].type       = TOKEN_OPERATOR;
            tokens[count].value.oper = input[pos++];
            count++;
            continue;
        }

        /* Unknown character — fail */
        return -1;
    }

    tokens[count].type = TOKEN_END;
    return count + 1;
}


/* ════════════════════════════════════════════════════════════
   SECTION 5 — TRIGONOMETRY  (degree-based, Taylor series)
   All angles in DEGREES. stdlib trig uses radians, so we
   convert, compute, and convert back where needed.
   ════════════════════════════════════════════════════════════ */

#define MY_PI 3.14159265358979323846

/* Unit conversion helpers */
static double deg2rad(double d) { return d * MY_PI / 180.0; }
static double rad2deg(double r) { return r * 180.0 / MY_PI; }

/*
 * norm_angle()
 * Reduces an angle (in radians) to the range (-π, π].
 * Required before feeding into the Taylor series so the
 * series converges quickly and accurately.
 */
static double norm_angle(double x)
{
    while (x >  MY_PI) x -= 2.0 * MY_PI;
    while (x < -MY_PI) x += 2.0 * MY_PI;
    return x;
}

/*
 * sin_deg() — Taylor series for sin(x)
 *
 *   sin(x) = x  -  x³/3!  +  x⁵/5!  -  x⁷/7!  + …
 *
 * Each iteration multiplies the previous term by  -x²/((2i)(2i+1))
 * which is the ratio between consecutive terms.
 * 7 iterations gives < 1e-9 error for |x| ≤ π.
 */
double sin_deg(double d)
{
    double x    = norm_angle(deg2rad(d));
    double term = x;    /* first term of the series = x */
    double sum  = x;

    for (int i = 1; i <= 7; i++)
    {
        term *= -x * x / ((2*i) * (2*i + 1));   /* ratio to next term */
        sum  += term;
    }
    return sum;
}

/*
 * cos_deg() — Taylor series for cos(x)
 *
 *   cos(x) = 1  -  x²/2!  +  x⁴/4!  -  x⁶/6!  + …
 *
 * Ratio between consecutive terms:  -x²/((2i-1)(2i))
 */
double cos_deg(double d)
{
    double x    = norm_angle(deg2rad(d));
    double term = 1.0;  /* first term = 1 */
    double sum  = 1.0;

    for (int i = 1; i <= 7; i++)
    {
        term *= -x * x / ((2*i - 1) * (2*i));
        sum  += term;
    }
    return sum;
}

/* ── Derived trig functions ── */

/* tan = sin/cos. Guard against cos ≈ 0 (90°, 270° …) */
double tan_deg(double d)
{
    double c = cos_deg(d);
    return (fabs(c) < 1e-9) ? INFINITY : sin_deg(d) / c;
}

/* cot = 1/tan. Guard against tan = 0 or tan = ∞ */
double cot_deg(double d)
{
    double t = tan_deg(d);
    return (isinf(t) || fabs(t) < 1e-9) ? INFINITY : 1.0 / t;
}

/* sec = 1/cos */
double sec_deg(double d)
{
    double c = cos_deg(d);
    return (fabs(c) < 1e-9) ? INFINITY : 1.0 / c;
}

/* csc = 1/sin */
double csc_deg(double d)
{
    double s = sin_deg(d);
    return (fabs(s) < 1e-9) ? INFINITY : 1.0 / s;
}

/* ── Inverse trig — delegates to stdlib, converts output to degrees ── */

/* asin: defined only for x ∈ [-1, 1] */
double asin_deg(double x) { return (x < -1 || x > 1) ? NAN : rad2deg(asin(x)); }

/* acos: defined only for x ∈ [-1, 1] */
double acos_deg(double x) { return (x < -1 || x > 1) ? NAN : rad2deg(acos(x)); }

/* atan: defined for all real x */
double atan_deg(double x) { return rad2deg(atan(x)); }

/* ── Other math functions ── */

/* sqrt: undefined for negative input */
double sqrt_real(double x) { return (x < 0) ? NAN : sqrt(x); }

/* log base-10: undefined for x ≤ 0 */
double log10_fn(double x) { return (x <= 0) ? NAN : log10(x); }

/* natural log: undefined for x ≤ 0 */
double ln_fn(double x) { return (x <= 0) ? NAN : log(x); }


/* ════════════════════════════════════════════════════════════
   SECTION 6 — FUNCTION DISPATCH
   Maps a function name string to the correct math function.
   Called by the evaluator when it sees a TOKEN_IDENTIFIER
   followed by a parenthesised argument.
   Returns 0 on success, -1 if name unknown, -2 if result is
   non-finite (NaN or Infinity).
   ════════════════════════════════════════════════════════════ */

int dispatch_function(const char *name, double arg, double *result)
{
    if      (strcmp(name, "sin")  == 0) *result = sin_deg(arg);
    else if (strcmp(name, "cos")  == 0) *result = cos_deg(arg);
    else if (strcmp(name, "tan")  == 0) *result = tan_deg(arg);
    else if (strcmp(name, "cot")  == 0) *result = cot_deg(arg);
    else if (strcmp(name, "sec")  == 0) *result = sec_deg(arg);
    else if (strcmp(name, "csc")  == 0) *result = csc_deg(arg);
    else if (strcmp(name, "asin") == 0) *result = asin_deg(arg);
    else if (strcmp(name, "acos") == 0) *result = acos_deg(arg);
    else if (strcmp(name, "atan") == 0) *result = atan_deg(arg);
    else if (strcmp(name, "sqrt") == 0) *result = sqrt_real(arg);
    else if (strcmp(name, "log")  == 0) *result = log10_fn(arg);
    else if (strcmp(name, "ln")   == 0) *result = ln_fn(arg);
    else return -1;   /* unknown function */

    if (!isfinite(*result)) return -2;   /* domain error (e.g. sqrt(-1)) */
    return 0;
}


/* ════════════════════════════════════════════════════════════
   SECTION 7 — SHUNTING-YARD EVALUATOR
   Evaluates a token array produced by tokenize().
   Uses Dijkstra's shunting-yard algorithm:
     - Numbers go directly onto the value stack.
     - Operators wait on an operator stack until a lower/equal
       precedence operator arrives, then get applied.
     - Parentheses bracket sub-expressions.
     - Function names are saved and applied when their closing
       parenthesis is seen.
   ════════════════════════════════════════════════════════════ */

#define MAX_STACK 32

/* Three separate stacks used by the evaluator */
typedef struct { double data[MAX_STACK]; int top; } ValStack; /* operand values  */
typedef struct { char   data[MAX_STACK]; int top; } OpStack;  /* operator chars  */
typedef struct { char   data[MAX_STACK][MAX_IDENT_LEN]; int top; } FnStack; /* function names */

/*
 * prec() — operator precedence
 *   +  -  →  1  (lowest)
 *   *  /  →  2
 *   ^     →  3  (highest, right-associative)
 */
static int prec(char op)
{
    if (op == '+' || op == '-') return 1;
    if (op == '*' || op == '/') return 2;
    if (op == '^')              return 3;
    return 0;
}

/*
 * apply_op() — pops two operands, applies the operator, returns result.
 * Division by near-zero returns NaN to signal a math error.
 */
static double apply_op(double a, double b, char op)
{
    switch (op)
    {
        case '+': return a + b;
        case '-': return a - b;
        case '*': return a * b;
        case '/': return (fabs(b) < 1e-12) ? NAN : a / b;
        case '^': return pow(a, b);
    }
    return 0;
}

/*
 * evaluate()
 * Processes token array t[] and writes the numeric result to *res.
 * Returns 0 on success, -1 on error (stack underflow, unknown function).
 *
 * CRITICAL: all three stacks are zero-initialised with memset().
 * Arduino local structs contain garbage — without memset, v.data[]
 * holds random values that would corrupt results on stack underflow.
 */
int evaluate(const Token *t, double *res)
{
    ValStack v; memset(&v, 0, sizeof(v)); v.top = -1;
    OpStack  o; memset(&o, 0, sizeof(o)); o.top = -1;
    FnStack  f; memset(&f, 0, sizeof(f)); f.top = -1;

    for (int i = 0; t[i].type != TOKEN_END; i++)
    {
        /* ── Number: push onto value stack ── */
        if (t[i].type == TOKEN_NUMBER)
        {
            v.data[++v.top] = t[i].value.number;
        }

        /* ── Function name: push onto function stack ── */
        else if (t[i].type == TOKEN_IDENTIFIER)
        {
            strncpy(f.data[++f.top], t[i].value.ident, MAX_IDENT_LEN);
        }

        /* ── Operator: apply higher-precedence operators first ── */
        else if (t[i].type == TOKEN_OPERATOR)
        {
            char cur = t[i].value.oper;

            /*
             * Pop and apply any operator on the op-stack that:
             *   - is not a left-paren, AND
             *   - has HIGHER precedence than cur  (or equal, if cur is left-associative)
             * ^ is right-associative so we only pop strictly-higher precedence.
             */
            while (o.top >= 0 && o.data[o.top] != '(' &&
                   ((cur != '^' && prec(cur) <= prec(o.data[o.top])) ||
                    (cur == '^' && prec(cur) <  prec(o.data[o.top]))))
            {
                double b = v.data[v.top--];
                double a = v.data[v.top--];
                v.data[++v.top] = apply_op(a, b, o.data[o.top--]);
            }
            o.data[++o.top] = cur;
        }

        /* ── Left paren: push as barrier onto op-stack ── */
        else if (t[i].type == TOKEN_LPAREN_T)
        {
            o.data[++o.top] = '(';
        }

        /* ── Right paren: apply until matching left paren ── */
        else if (t[i].type == TOKEN_RPAREN_T)
        {
            /* Flush operators down to the matching '(' */
            while (o.top >= 0 && o.data[o.top] != '(')
            {
                double b = v.data[v.top--];
                double a = v.data[v.top--];
                v.data[++v.top] = apply_op(a, b, o.data[o.top--]);
            }
            if (o.top >= 0) o.top--;   /* discard the '(' */

            /* If a function was waiting, apply it now */
            if (f.top >= 0)
            {
                double arg    = v.data[v.top--];
                double result = 0;
                if (dispatch_function(f.data[f.top--], arg, &result) != 0)
                    return -1;
                v.data[++v.top] = result;
            }
        }
    }

    /* Apply any remaining operators */
    while (o.top >= 0)
    {
        double b = v.data[v.top--];
        double a = v.data[v.top--];
        v.data[++v.top] = apply_op(a, b, o.data[o.top--]);
    }

    if (v.top < 0) return -1;   /* empty expression */
    *res = v.data[v.top];
    return 0;
}


/* ════════════════════════════════════════════════════════════
   SECTION 8 — EQUATION SOLVERS
   ════════════════════════════════════════════════════════════ */

/* Result codes for linear / quadratic solvers */
#define EQ_ONE  1   /* exactly one real root    */
#define EQ_TWO  2   /* two distinct real roots  */
#define EQ_NONE 3   /* no real roots            */
#define EQ_INF  4   /* infinite solutions (0=0) */

/* Result codes for Cramer's rule */
#define CR_UNIQUE 0   /* unique solution         */
#define CR_NONE   1   /* no solution             */
#define CR_INF    2   /* infinite solutions      */

/*
 * sqrt_nr() — Newton-Raphson square root
 * Avoids stdlib sqrt for embedded safety.
 * Iterates:  g_(n+1) = 0.5 * (g_n + x/g_n)
 * Converges quadratically; 20 iterations is overkill but safe.
 */
static double sqrt_nr(double x)
{
    double g = x / 2.0;
    for (int i = 0; i < 20; i++)
        g = 0.5 * (g + x / g);
    return g;
}

/*
 * nr_cubic() — Newton-Raphson root finder for a cubic
 * Solves f(x) = ax³ + bx² + cx + d = 0
 * Iterates:  x_(n+1) = x_n - f(x_n) / f'(x_n)
 *   f'(x) = 3ax² + 2bx + c
 * Stops when step size < 1e-6 or after 100 iterations.
 * x0 = initial guess (0.0 is used as the default).
 */
static double nr_cubic(double a, double b, double c, double d, double x0)
{
    double x = x0;
    for (int i = 0; i < 100; i++)
    {
        double fx  = a*x*x*x + b*x*x + c*x + d;
        double dfx = 3*a*x*x + 2*b*x + c;
        if (fabs(dfx) < 1e-12) break;  /* derivative ≈ 0 — local extremum, stop */
        double xn = x - fx / dfx;
        if (fabs(xn - x) < 1e-6) return xn;   /* converged */
        x = xn;
    }
    return x;
}

/* ── Linear solver: ax + b = 0 ──────────────────────────── */
EqSol solve_linear(double a, double b)
{
    EqSol s;
    if (fabs(a) < 1e-9)
    {
        /* a ≈ 0 → degenerates to 0 = b */
        s.result = (fabs(b) < 1e-9) ? EQ_INF : EQ_NONE;
        return s;
    }
    s.result = EQ_ONE;
    s.r1     = -b / a;   /* x = -b/a */
    s.r2     = 0;
    return s;
}

/*
 * Quadratic solver: ax² + bx + c = 0
 * Uses the discriminant Δ = b² - 4ac:
 *   Δ > 0  →  two distinct real roots
 *   Δ = 0  →  one repeated root
 *   Δ < 0  →  no real roots (complex pair)
 */
EqSol solve_quadratic(double a, double b, double c)
{
    EqSol s;
    if (fabs(a) < 1e-9) return solve_linear(b, c);   /* degenerate to linear */

    double disc = b*b - 4.0*a*c;

    if (disc > 1e-9)
    {
        double sq = sqrt_nr(disc);
        s.result  = EQ_TWO;
        s.r1      = (-b + sq) / (2*a);
        s.r2      = (-b - sq) / (2*a);
    }
    else if (fabs(disc) < 1e-9)
    {
        s.result = EQ_ONE;
        s.r1     = -b / (2*a);
        s.r2     = 0;
    }
    else
    {
        s.result = EQ_NONE;
    }
    return s;
}

/*
 * Cubic solver: ax³ + bx² + cx + d = 0
 *
 * Strategy:
 *   1. Find one real root r1 using Newton-Raphson.
 *   2. Divide out (x - r1) via synthetic division to get a quadratic.
 *      Synthetic division coefficients:
 *        A = a
 *        B = b + A·r1
 *        C = c + B·r1
 *      Remaining factor: Ax² + Bx + C
 *   3. Solve the quadratic for the remaining two roots.
 *
 * Discriminant determines expected number of real roots:
 *   disc = 18abcd - 4b³d + b²c² - 4ac³ - 27a²d²
 *   disc < 0  →  1 real root
 *   disc ≥ 0  →  3 real roots (may include repeated)
 */
CubicSol solve_cubic(double a, double b, double c, double d)
{
    CubicSol sol;
    memset(&sol, 0, sizeof(sol));

    if (fabs(a) < 1e-9)
    {
        /* Not actually cubic — solve as quadratic */
        EqSol q       = solve_quadratic(b, c, d);
        sol.num_roots = (q.result == EQ_TWO) ? 2 : 1;
        sol.roots[0]  = q.r1;
        sol.roots[1]  = q.r2;
        return sol;
    }

    /* Discriminant */
    double disc = 18*a*b*c*d - 4*b*b*b*d + b*b*c*c - 4*a*c*c*c - 27*a*a*d*d;
    sol.num_roots = (disc < -1e-9) ? 1 : 3;

    /* Step 1: one real root via Newton-Raphson */
    sol.roots[0] = nr_cubic(a, b, c, d, 0.0);

    /* Step 2: synthetic division to get quadratic factor */
    double A = a;
    double B = b + A * sol.roots[0];
    double C = c + B * sol.roots[0];

    /* Step 3: solve quadratic for remaining roots */
    EqSol q = solve_quadratic(A, B, C);
    if (sol.num_roots == 3 && q.result == EQ_TWO)
    {
        sol.roots[1] = q.r1;
        sol.roots[2] = q.r2;
    }
    else
    {
        sol.num_roots = 1;   /* numerical edge case */
    }

    return sol;
}

/*
 * Cramer's Rule — 2×2 system:
 *   a1·x + b1·y = c1
 *   a2·x + b2·y = c2
 *
 * Determinants:
 *   D  = |a1 b1|  = a1·b2 - a2·b1
 *        |a2 b2|
 *
 *   Dx = |c1 b1|  = c1·b2 - c2·b1
 *        |c2 b2|
 *
 *   Dy = |a1 c1|  = a1·c2 - a2·c1
 *        |a2 c2|
 *
 *   x = Dx/D,  y = Dy/D  (when D ≠ 0)
 */
Cr2Sol solve_cramer2(double a1, double b1, double c1,
                     double a2, double b2, double c2)
{
    Cr2Sol s;
    double D  = a1*b2 - a2*b1;
    double Dx = c1*b2 - c2*b1;
    double Dy = a1*c2 - a2*c1;

    if (fabs(D) < 1e-9)
    {
        /* D = 0: either no solution or infinitely many */
        s.result = (fabs(Dx) < 1e-9 && fabs(Dy) < 1e-9) ? CR_INF : CR_NONE;
        return s;
    }

    s.result = CR_UNIQUE;
    s.x = Dx / D;
    s.y = Dy / D;
    return s;
}

/*
 * det3() — determinant of a 3×3 matrix using Sarrus' rule:
 *   |a1 b1 c1|
 *   |a2 b2 c2| = a1(b2c3-b3c2) - b1(a2c3-a3c2) + c1(a2b3-a3b2)
 *   |a3 b3 c3|
 */
static double det3(double a1, double b1, double c1,
                   double a2, double b2, double c2,
                   double a3, double b3, double c3)
{
    return a1*(b2*c3 - b3*c2)
         - b1*(a2*c3 - a3*c2)
         + c1*(a2*b3 - a3*b2);
}

/*
 * Cramer's Rule — 3×3 system:
 *   a[i]·x + b[i]·y + c[i]·z = d[i]  for i = 0,1,2
 *
 * x = det(D_x) / det(D)   (replace x-column with d)
 * y = det(D_y) / det(D)   (replace y-column with d)
 * z = det(D_z) / det(D)   (replace z-column with d)
 */
Cr3Sol solve_cramer3(double a[3], double b[3], double c[3], double d[3])
{
    Cr3Sol s;
    double D = det3(a[0],b[0],c[0], a[1],b[1],c[1], a[2],b[2],c[2]);

    if (fabs(D) < 1e-9) { s.ok = 0; return s; }

    s.ok = 1;
    s.x  = det3(d[0],b[0],c[0], d[1],b[1],c[1], d[2],b[2],c[2]) / D;
    s.y  = det3(a[0],d[0],c[0], a[1],d[1],c[1], a[2],d[2],c[2]) / D;
    s.z  = det3(a[0],b[0],d[0], a[1],b[1],d[1], a[2],b[2],d[2]) / D;
    return s;
}


/* ════════════════════════════════════════════════════════════
   SECTION 9 — APPLICATION STATE
   Global variables that persist across loop() iterations.
   ════════════════════════════════════════════════════════════ */

#define EXPR_MAX     63   /* max expression length (62 usable + null terminator) */
#define MODE_EXPR    0
#define MODE_LINEAR  1
#define MODE_QUAD    2
#define MODE_CUBIC   3
#define MODE_CRAMER2 4
#define MODE_CRAMER3 5
#define MODE_COUNT   6

const char *mode_names[] = { "EXPR", "LINEAR", "QUAD", "CUBIC", "CRAM2", "CRAM3" };

char   expr[EXPR_MAX + 1]; /* expression string being typed          */
int    expr_len      = 0;  /* current length of expr[]               */
int    scroll_offset = 0;  /* first visible char index on LCD line 0 */
bool   shift_active  = false;
double last_result   = 0.0;
int    current_mode  = MODE_EXPR;


/* ════════════════════════════════════════════════════════════
   SECTION 10 — LCD HELPERS
   ════════════════════════════════════════════════════════════ */

/*
 * display_line2()
 * Writes msg to LCD row 1 (second line), padded to 16 chars.
 * Padding with spaces erases leftover characters from previous content.
 */
void display_line2(const char *msg)
{
    lcd.setCursor(0, 1);
    char buf[17];
    snprintf(buf, 17, "%-16s", msg);   /* %-16s = left-align, pad with spaces to width 16 */
    lcd.print(buf);
}

/*
 * display_update()
 * Redraws LCD row 0 with the expression, applying scroll_offset
 * so only 16 characters are shown at a time.
 * Optionally redraws row 1 with line2 (pass NULL to skip).
 */
void display_update(const char *line2)
{
    lcd.setCursor(0, 0);
    char buf[17];

    /* Copy 16 chars from expr[] starting at scroll_offset.
       Fill with spaces if expression is shorter. */
    for (int i = 0; i < 16; i++)
    {
        int idx = scroll_offset + i;
        buf[i]  = (idx < expr_len) ? expr[idx] : ' ';
    }
    buf[16] = '\0';
    lcd.print(buf);

    if (line2 != NULL) display_line2(line2);
}


/* ════════════════════════════════════════════════════════════
   SECTION 11 — EXPRESSION BUFFER HELPERS
   ════════════════════════════════════════════════════════════ */

/* Clear the entire expression and reset scroll position */
void expr_clear()
{
    memset(expr, 0, sizeof(expr));
    expr_len      = 0;
    scroll_offset = 0;
}

/*
 * expr_append()
 * Adds one character to the expression.
 * Auto-scrolls right when expression exceeds 16 chars.
 */
void expr_append(char c)
{
    if (expr_len >= EXPR_MAX) return;   /* buffer full — silently ignore */
    expr[expr_len++] = c;
    expr[expr_len]   = '\0';
    if (expr_len > 16) scroll_offset = expr_len - 16;
}

/* Appends a full string (e.g. "sin(") character by character */
void expr_append_str(const char *s)
{
    for (int i = 0; s[i]; i++) expr_append(s[i]);
}

/*
 * expr_backspace()
 * Removes the last character. Adjusts scroll so the display
 * doesn't show blank space on the left.
 */
void expr_backspace()
{
    if (expr_len == 0) return;
    expr[--expr_len] = '\0';
    if (scroll_offset > 0 && scroll_offset >= expr_len)
        scroll_offset = (expr_len > 16) ? expr_len - 16 : 0;
}



/* ════════════════════════════════════════════════════════════
   SECTION 13 — COEFFICIENT INPUT  (blocking loop)
   Used by all solver modes to collect one numeric coefficient
   at a time before solving.
   Stays inside the loop until the user presses =.
   ════════════════════════════════════════════════════════════ */

double collect_coeff(const char *prompt)
{
    expr_clear();
    display_update(prompt);

    while (true)
    {
        int key = get_key();
        if (key == KEY_NONE) { delay(1); continue; }

        if (key == KEY_EQUAL)
        {
            expr[expr_len] = '\0';
            return atof(expr);   /* convert string to double and return */
        }
        else if (key == KEY_BCK)     expr_backspace();
        else if (key == KEY_N0)      expr_append('0');
        else if (key == KEY_N1)      expr_append('1');
        else if (key == KEY_N2)      expr_append('2');
        else if (key == KEY_N3)      expr_append('3');
        else if (key == KEY_N4)      expr_append('4');
        else if (key == KEY_N5)      expr_append('5');
        else if (key == KEY_N6)      expr_append('6');
        else if (key == KEY_N7)      expr_append('7');
        else if (key == KEY_N8)      expr_append('8');
        else if (key == KEY_N9)      expr_append('9');
        else if (key == KEY_SUBN)    expr_append('-');   /* negative coefficients */
        else if (key == KEY_DECIMAL) expr_append('.');   /* decimal point         */

        display_update(prompt);
        delay(1);
    }
}



/* ════════════════════════════════════════════════════════════
   SECTION 15 — SOLVER RUNNERS
   Each function collects coefficients then calls the solver
   and displays the result.
   ════════════════════════════════════════════════════════════ */

void run_linear()
{
    double a = collect_coeff("a (ax+b=0):");
    double b = collect_coeff("b:");

    EqSol s = solve_linear(a, b);
    char  buf[17];

    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("LINEAR SOLVER");

    if      (s.result == EQ_ONE)  snprintf(buf, 17, "x=%.5g", s.r1);
    else if (s.result == EQ_NONE) snprintf(buf, 17, "No solution");
    else                          snprintf(buf, 17, "Infinite sol.");

    display_line2(buf);
    delay(2000);
}

void run_quadratic()
{
    double a = collect_coeff("a(ax2+bx+c):");
    double b = collect_coeff("b:");
    double c = collect_coeff("c:");

    EqSol s = solve_quadratic(a, b, c);
    char  buf[17];

    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("QUADRATIC");

    if      (s.result == EQ_TWO)  snprintf(buf, 17, "%.4g , %.4g", s.r1, s.r2);
    else if (s.result == EQ_ONE)  snprintf(buf, 17, "x=%.5g", s.r1);
    else if (s.result == EQ_NONE) snprintf(buf, 17, "No real roots");
    else                          snprintf(buf, 17, "Infinite sol.");

    display_line2(buf);
    delay(2000);
}

void run_cubic()
{
    double a = collect_coeff("a(ax3+bx2+cx):");
    double b = collect_coeff("b:");
    double c = collect_coeff("c:");
    double d = collect_coeff("d:");

    CubicSol s = solve_cubic(a, b, c, d);
    char     buf[17];

    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("CUBIC");

    snprintf(buf, 17, "r1=%.5g", s.roots[0]);
    display_line2(buf);
    delay(3000);

    if (s.num_roots == 3)
    {
        snprintf(buf, 17, "%.4g , %.4g", s.roots[1], s.roots[2]);
        display_line2(buf);
        delay(3000);
    }
}

void run_cramer2()
{
    double a1 = collect_coeff("a1(a1x+b1y=c1):");
    double b1 = collect_coeff("b1:");
    double c1 = collect_coeff("c1:");
    double a2 = collect_coeff("a2:");
    double b2 = collect_coeff("b2:");
    double c2 = collect_coeff("c2:");

    Cr2Sol s = solve_cramer2(a1, b1, c1, a2, b2, c2);
    char   buf[17];

    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("CRAMER 2x2");

    if      (s.result == CR_UNIQUE) snprintf(buf, 17, "x=%.4g y=%.4g", s.x, s.y);
    else if (s.result == CR_NONE)   snprintf(buf, 17, "No solution");
    else                            snprintf(buf, 17, "Infinite sol.");

    display_line2(buf);
    delay(3000);
}

void run_cramer3()
{
    double a[3], b[3], c[3], d[3];
    char   prompt[17];

    for (int i = 0; i < 3; i++)
    {
        snprintf(prompt, 17, "a%d:", i+1); a[i] = collect_coeff(prompt);
        snprintf(prompt, 17, "b%d:", i+1); b[i] = collect_coeff(prompt);
        snprintf(prompt, 17, "c%d:", i+1); c[i] = collect_coeff(prompt);
        snprintf(prompt, 17, "d%d:", i+1); d[i] = collect_coeff(prompt);
    }

    Cr3Sol s = solve_cramer3(a, b, c, d);
    char   buf[17];

    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("CRAMER 3x3");

    if (s.ok) snprintf(buf, 17, "x=%.3g y=%.3g", s.x, s.y);
    else      snprintf(buf, 17, "No unique sol.");

    display_line2(buf);
    delay(2000);

    if (s.ok)
    {
        snprintf(buf, 17, "z=%.5g", s.z);
        display_line2(buf);
        delay(2000);
    }
}


/* ════════════════════════════════════════════════════════════
   SECTION 16 — EXPRESSION EVALUATION
   ════════════════════════════════════════════════════════════ */

void evaluate_expression()
{
    if (expr_len == 0) { display_line2("Empty!"); return; }
    expr[expr_len] = '\0';

    Token tokens[MAX_TOKENS];
    if (tokenize(expr, tokens, MAX_TOKENS) < 0)
    {
        display_line2("Syntax error");
        return;
    }

    double result;
    if (evaluate(tokens, &result) != 0 || !isfinite(result))
    {
        display_line2("Math error");
        return;
    }

    last_result = result;

    char buf[17];
    /*
     * Display as integer if the result has no fractional part
     * AND fits in a 32-bit signed integer range.
     * This prevents overflow when casting large doubles to long.
     */
    if (result >= -2147483648.0 && result <= 2147483647.0 && result == (long)result)
        snprintf(buf, 17, "=%ld", (long)result);
    else
        snprintf(buf, 17, "=%.6g", result);

    display_line2(buf);
}


/* ════════════════════════════════════════════════════════════
   SECTION 17 — KEY HANDLER
   Central routing of all keypresses.
   Priority order:
     1. SHIFT key toggle
     2. MODE key
     3. SHIFT layer combinations
     4. EXPR mode keys
     5. Solver mode trigger (= key in non-EXPR modes)
   ════════════════════════════════════════════════════════════ */

void handle_key(int key)
{
    /* ── 1. SHIFT toggles the shift layer ── */
    if (key == KEY_SHIFT)
    {
        shift_active = !shift_active;
        display_line2(shift_active ? "SHIFT ON" : mode_names[current_mode]);
        return;
    }

    /* ── 2. MODE cycles calculator modes ── */
    if (key == KEY_MODE)
    {
        current_mode = (current_mode + 1) % MODE_COUNT;
        expr_clear();
        lcd.clear();
        display_update(mode_names[current_mode]);
        display_line2(mode_names[current_mode]);
        shift_active = false;
        return;
    }

    /* ── 3. SHIFT layer ── */
    if (shift_active)
    {
        shift_active = false;   /* consume SHIFT — next key is normal */

        switch (key)
        {
            case KEY_EQUAL:
            {
                /* Recall last answer */
                char buf[17];
                snprintf(buf, 17, "Ans=%.6g", last_result);
                display_line2(buf);
                return;
            }
            case KEY_PWR:
                /* Clear all */
                expr_clear();
                lcd.clear();
                display_update(mode_names[current_mode]);
                return;

            case KEY_SIN: expr_append_str("csc("); break;
            case KEY_COS: expr_append_str("sec("); break;
            case KEY_TAN: expr_append_str("cot("); break;

            default: break;
        }

        display_update(mode_names[current_mode]);
        return;
    }

    /* ── 4. EXPR mode ── */
    if (current_mode == MODE_EXPR)
    {
        switch (key)
        {
            case KEY_N0: expr_append('0'); break;
            case KEY_N1: expr_append('1'); break;
            case KEY_N2: expr_append('2'); break;
            case KEY_N3: expr_append('3'); break;
            case KEY_N4: expr_append('4'); break;
            case KEY_N5: expr_append('5'); break;
            case KEY_N6: expr_append('6'); break;
            case KEY_N7: expr_append('7'); break;
            case KEY_N8: expr_append('8'); break;
            case KEY_N9: expr_append('9'); break;

            case KEY_ADDN:    expr_append('+');         break;
            case KEY_SUBN:    expr_append('-');         break;
            case KEY_MULT:    expr_append('*');         break;
            case KEY_DIVI:    expr_append('/');         break;
            case KEY_POWER:   expr_append('^');         break;
            case KEY_LPAREN:  expr_append('(');         break;
            case KEY_RPAREN:  expr_append(')');         break;
            case KEY_DECIMAL: expr_append('.');         break;

            case KEY_PI:      expr_append_str("pi");    break;
            case KEY_E_CONST: expr_append('e');         break;

            case KEY_SIN:     expr_append_str("sin(");  break;
            case KEY_COS:     expr_append_str("cos(");  break;
            case KEY_TAN:     expr_append_str("tan(");  break;
            case KEY_LN:      expr_append_str("ln(");   break;
            case KEY_LOG:     expr_append_str("log(");  break;
            case KEY_ARCSIN:  expr_append_str("asin("); break;
            case KEY_ARCCOS:  expr_append_str("acos("); break;
            case KEY_ARCTAN:  expr_append_str("atan("); break;
            case KEY_SQRT:    expr_append_str("sqrt("); break;

            case KEY_BCK:      expr_backspace(); break;
            case KEY_ARROW_LT: if (scroll_offset > 0)          scroll_offset--; break;
            case KEY_ARROW_RT: if (scroll_offset < expr_len-1) scroll_offset++; break;

            case KEY_EQUAL: evaluate_expression(); return;

            default: break;
        }

        display_update(mode_names[current_mode]);
        return;
    }

    /* ── 5. Solver modes — = triggers coefficient collection ── */
    if (key == KEY_EQUAL)
    {
        switch (current_mode)
        {
            case MODE_LINEAR:  run_linear();    break;
            case MODE_QUAD:    run_quadratic(); break;
            case MODE_CUBIC:   run_cubic();     break;
            case MODE_CRAMER2: run_cramer2();   break;
            case MODE_CRAMER3: run_cramer3();   break;
            default: break;
        }

        expr_clear();
        display_update(mode_names[current_mode]);
        display_line2(mode_names[current_mode]);
    }
}


/* ════════════════════════════════════════════════════════════
   SECTION 18 — SETUP AND MAIN LOOP
   ════════════════════════════════════════════════════════════ */

void setup()
{
    Serial.begin(115200);
    Serial.println("Calculator starting...");

    /* Configure ROW pins as outputs, initially HIGH (inactive) */
    for (int r = 0; r < KB_ROWS; r++)
    {
        pinMode(ROW_PINS[r], OUTPUT);
        digitalWrite(ROW_PINS[r], HIGH);
    }

    /* Configure COL pins as inputs with internal pull-up where supported.
       Note: GPIO 34/35/36/39 ignore INPUT_PULLUP — use external resistors. */
    for (int c = 0; c < KB_COLS; c++)
        pinMode(COL_PINS[c], INPUT_PULLUP);

    /* Initialise LCD over I2C */
    Wire.begin(21, 22);   /* SDA=21, SCL=22 */
    lcd.init();
    lcd.backlight();
    lcd.clear();

    /* Splash screen */
    lcd.setCursor(0, 0); lcd.print("Burdened With");
    lcd.setCursor(0, 1); lcd.print("Glorious Purpose");
    delay(1500);
    lcd.clear();

    /* Show default mode */
    display_update(mode_names[current_mode]);
    display_line2(mode_names[current_mode]);
}

void loop()
{
    int key = get_key();

    if (key != KEY_NONE)
    {
        /* Debug: print timestamp and key code to serial monitor */
        Serial.print(millis());
        Serial.print(" Key: ");
        Serial.println(key);

        handle_key(key);
    }

    delay(1);   /* 1 ms yield — keeps timing predictable */
}
