#include <keyboard.h>
#include <csmwrap.h>
#include <stddef.h>

#define KEYBOARD_QUEUE_SIZE 64

/* UEFI Simple Text Input scan-code values used for non-printable keys. */
#define CSMWRAP_EFI_SCAN_UP     0x0001
#define CSMWRAP_EFI_SCAN_DOWN   0x0002
#define CSMWRAP_EFI_SCAN_RIGHT  0x0003
#define CSMWRAP_EFI_SCAN_LEFT   0x0004

/* PS/2 Set 1 make codes consumed by a legacy INT 9/i8042 emulation path. */
#define SET1_EXTENDED_PREFIX 0xe0
#define SET1_BREAK_BIT       0x80
#define SET1_LEFT_CTRL       0x1d
#define SET1_LEFT_SHIFT      0x2a
#define SET1_LEFT_ALT        0x38
#define SET1_ENTER           0x1c
#define SET1_SPACE           0x39
#define SET1_ARROW_UP        0x48
#define SET1_ARROW_LEFT      0x4b
#define SET1_ARROW_RIGHT     0x4d
#define SET1_ARROW_DOWN      0x50

/*
 * Minimal Set 1 make-code table for letters.  EFI reports text input as a
 * Unicode character, so upper and lower case both map to the same physical key
 * make code.  Uppercase text from Simple Text Input is represented as a
 * left-shift key tap around the letter tap because that protocol does not
 * report modifier press/release state separately.
 */
static const uint8_t letter_set1_make_codes[26] = {
    0x1e, /* a */
    0x30, /* b */
    0x2e, /* c */
    0x20, /* d */
    0x12, /* e */
    0x21, /* f */
    0x22, /* g */
    0x23, /* h */
    0x17, /* i */
    0x24, /* j */
    0x25, /* k */
    0x26, /* l */
    0x32, /* m */
    0x31, /* n */
    0x18, /* o */
    0x19, /* p */
    0x10, /* q */
    0x13, /* r */
    0x1f, /* s */
    0x14, /* t */
    0x16, /* u */
    0x2f, /* v */
    0x11, /* w */
    0x2d, /* x */
    0x15, /* y */
    0x2c, /* z */
};

/*
 * Single-producer/single-consumer ring buffer of raw scancode bytes.
 * One slot is deliberately kept empty so head == tail means "empty" and the
 * next head equaling tail means "full".  Overflow is non-blocking: complete
 * key sequences are dropped when there is insufficient room, which prevents
 * malformed partial E0-prefixed or make/break pairs from reaching INT 9.
 */
static uint8_t keyboard_queue[KEYBOARD_QUEUE_SIZE];
static volatile unsigned int keyboard_queue_head;
static volatile unsigned int keyboard_queue_tail;

static bool modifier_shift_down;
static bool modifier_ctrl_down;
static bool modifier_alt_down;
static bool scancode_down[128];

static void compiler_barrier(void)
{
    __asm__ __volatile__("" ::: "memory");
}

static unsigned int queue_next(unsigned int index)
{
    index++;
    if (index == KEYBOARD_QUEUE_SIZE)
        index = 0;
    return index;
}

static unsigned int queue_free_slots(void)
{
    unsigned int head = keyboard_queue_head;
    unsigned int tail = keyboard_queue_tail;

    if (head >= tail)
        return (KEYBOARD_QUEUE_SIZE - 1) - (head - tail);

    return (tail - head) - 1;
}

bool QueuePush(uint8_t scancode)
{
    unsigned int head = keyboard_queue_head;
    unsigned int next = queue_next(head);

    if (next == keyboard_queue_tail)
        return false;

    keyboard_queue[head] = scancode;
    compiler_barrier();
    keyboard_queue_head = next;
    return true;
}

bool QueuePop(uint8_t *scancode)
{
    unsigned int tail;

    if (scancode == NULL)
        return false;

    tail = keyboard_queue_tail;
    if (keyboard_queue_head == tail)
        return false;

    compiler_barrier();
    *scancode = keyboard_queue[tail];
    keyboard_queue_tail = queue_next(tail);
    return true;
}

bool GetNextScancode(uint8_t *out)
{
    return QueuePop(out);
}

void KeyboardQueueReset(void)
{
    keyboard_queue_head = 0;
    keyboard_queue_tail = 0;

    for (unsigned int i = 0; i < sizeof(scancode_down); i++)
        scancode_down[i] = false;

    modifier_shift_down = false;
    modifier_ctrl_down = false;
    modifier_alt_down = false;
}

static unsigned int scancode_sequence_len(const struct keyboard_scancode_sequence *seq)
{
    if (seq == NULL)
        return 0;

    return seq->len;
}

static bool sequence_push_make(const struct keyboard_scancode_sequence *seq)
{
    if (seq == NULL || seq->len == 0)
        return false;

    if (queue_free_slots() < scancode_sequence_len(seq))
        return false;

    for (uint8_t i = 0; i < seq->len; i++)
        QueuePush(seq->bytes[i]);

    return true;
}

static bool sequence_push_break(const struct keyboard_scancode_sequence *seq)
{
    if (seq == NULL || seq->len == 0)
        return false;

    if (queue_free_slots() < scancode_sequence_len(seq))
        return false;

    if (seq->len == 1) {
        QueuePush(seq->bytes[0] | SET1_BREAK_BIT);
        return true;
    }

    if (seq->len == 2 && seq->bytes[0] == SET1_EXTENDED_PREFIX) {
        QueuePush(SET1_EXTENDED_PREFIX);
        QueuePush(seq->bytes[1] | SET1_BREAK_BIT);
        return true;
    }

    return false;
}

static bool sequence_push_event(const struct keyboard_scancode_sequence *seq,
                                bool pressed)
{
    if (pressed)
        return sequence_push_make(seq);

    return sequence_push_break(seq);
}

static bool sequence_push_tap(const struct keyboard_scancode_sequence *seq)
{
    unsigned int needed;

    if (seq == NULL || seq->len == 0)
        return false;

    needed = scancode_sequence_len(seq) * 2;
    if (queue_free_slots() < needed)
        return false;

    sequence_push_make(seq);
    sequence_push_break(seq);
    return true;
}

static bool set_single(uint8_t scancode, struct keyboard_scancode_sequence *seq)
{
    seq->bytes[0] = scancode;
    seq->len = 1;
    return true;
}

static bool set_extended(uint8_t scancode, struct keyboard_scancode_sequence *seq)
{
    seq->bytes[0] = SET1_EXTENDED_PREFIX;
    seq->bytes[1] = scancode;
    seq->len = 2;
    return true;
}

static bool set_modifier(bool *state, uint8_t scancode, bool down)
{
    struct keyboard_scancode_sequence seq;

    if (*state == down)
        return true;

    if (queue_free_slots() < 1)
        return false;

    set_single(scancode, &seq);
    if (!sequence_push_event(&seq, down))
        return false;

    *state = down;
    scancode_down[scancode] = down;
    return true;
}

bool KeyboardSetModifierState(bool shift, bool ctrl, bool alt)
{
    unsigned int needed = 0;

    if (modifier_shift_down != shift)
        needed++;
    if (modifier_ctrl_down != ctrl)
        needed++;
    if (modifier_alt_down != alt)
        needed++;

    if (queue_free_slots() < needed)
        return false;

    if (!set_modifier(&modifier_ctrl_down, SET1_LEFT_CTRL, ctrl))
        return false;
    if (!set_modifier(&modifier_alt_down, SET1_LEFT_ALT, alt))
        return false;
    if (!set_modifier(&modifier_shift_down, SET1_LEFT_SHIFT, shift))
        return false;

    return true;
}

bool KeyboardTranslateEfiKeyToSet1(const EFI_INPUT_KEY *key,
                                   struct keyboard_scancode_sequence *seq)
{
    if (key == NULL || seq == NULL)
        return false;

    seq->len = 0;

    if (key->UnicodeChar >= L'a' && key->UnicodeChar <= L'z')
        return set_single(letter_set1_make_codes[key->UnicodeChar - L'a'], seq);

    if (key->UnicodeChar >= L'A' && key->UnicodeChar <= L'Z')
        return set_single(letter_set1_make_codes[key->UnicodeChar - L'A'], seq);

    switch (key->UnicodeChar) {
    case L' ':
        return set_single(SET1_SPACE, seq);
    case L'\r':
        return set_single(SET1_ENTER, seq);
    default:
        break;
    }

    /* Arrow keys are non-printable EFI scan codes and use Set 1 E0 prefixes. */
    switch (key->ScanCode) {
    case CSMWRAP_EFI_SCAN_UP:
        return set_extended(SET1_ARROW_UP, seq);
    case CSMWRAP_EFI_SCAN_DOWN:
        return set_extended(SET1_ARROW_DOWN, seq);
    case CSMWRAP_EFI_SCAN_RIGHT:
        return set_extended(SET1_ARROW_RIGHT, seq);
    case CSMWRAP_EFI_SCAN_LEFT:
        return set_extended(SET1_ARROW_LEFT, seq);
    default:
        return false;
    }
}

bool KeyboardProcessEfiKeyEvent(const EFI_INPUT_KEY *key, bool pressed)
{
    struct keyboard_scancode_sequence seq;
    uint8_t make_code;

    if (!KeyboardTranslateEfiKeyToSet1(key, &seq))
        return false;

    make_code = seq.bytes[seq.len - 1];
    if (make_code >= sizeof(scancode_down))
        return false;

    if (scancode_down[make_code] == pressed)
        return true;

    if (!sequence_push_event(&seq, pressed))
        return false;

    scancode_down[make_code] = pressed;
    return true;
}

static bool keyboard_queue_simple_text_key(const EFI_INPUT_KEY *key)
{
    struct keyboard_scancode_sequence seq;
    bool uppercase;

    if (!KeyboardTranslateEfiKeyToSet1(key, &seq))
        return false;

    uppercase = key->UnicodeChar >= L'A' && key->UnicodeChar <= L'Z';
    if (!uppercase || modifier_shift_down)
        return sequence_push_tap(&seq);

    if (queue_free_slots() < 4)
        return false;

    QueuePush(SET1_LEFT_SHIFT);
    sequence_push_tap(&seq);
    QueuePush(SET1_LEFT_SHIFT | SET1_BREAK_BIT);
    return true;
}

EFI_STATUS UefiKeyboardPoll(EFI_SIMPLE_TEXT_INPUT_PROTOCOL *con_in)
{
    EFI_STATUS status;
    EFI_INPUT_KEY key;

    if (con_in == NULL)
        return EFI_INVALID_PARAMETER;

    for (;;) {
        status = con_in->ReadKeyStroke(con_in, &key);
        if (status == EFI_NOT_READY)
            return EFI_SUCCESS;
        if (EFI_ERROR(status))
            return status;

        keyboard_queue_simple_text_key(&key);
    }
}

EFI_STATUS UefiKeyboardPollSystemTable(void)
{
    if (gST == NULL)
        return EFI_NOT_READY;

    return UefiKeyboardPoll(gST->ConIn);
}
