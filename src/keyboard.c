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
#define SET1_ENTER           0x1c
#define SET1_SPACE           0x39
#define SET1_ARROW_UP        0x48
#define SET1_ARROW_LEFT      0x4b
#define SET1_ARROW_RIGHT     0x4d
#define SET1_ARROW_DOWN      0x50

/*
 * Minimal Set 1 make-code table for letters.  EFI reports text input as a
 * Unicode character, so upper and lower case both map to the same physical key
 * make code here.  Modifier state (Shift/Caps Lock) is intentionally outside
 * this small bridge because EFI_SIMPLE_TEXT_INPUT_PROTOCOL only gives us the
 * resulting character and basic firmware scan code.
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
 * next head equaling tail means "full".  Overflow is non-blocking: new bytes
 * are dropped and QueuePush() reports false, which keeps firmware polling
 * simple and avoids dynamic allocation.
 */
static uint8_t keyboard_queue[KEYBOARD_QUEUE_SIZE];
static unsigned int keyboard_queue_head;
static unsigned int keyboard_queue_tail;

static unsigned int queue_next(unsigned int index)
{
    index++;
    if (index == KEYBOARD_QUEUE_SIZE)
        index = 0;
    return index;
}

static unsigned int queue_free_slots(void)
{
    if (keyboard_queue_head >= keyboard_queue_tail)
        return (KEYBOARD_QUEUE_SIZE - 1) -
               (keyboard_queue_head - keyboard_queue_tail);

    return (keyboard_queue_tail - keyboard_queue_head) - 1;
}

bool QueuePush(uint8_t scancode)
{
    unsigned int next = queue_next(keyboard_queue_head);

    if (next == keyboard_queue_tail)
        return false;

    keyboard_queue[keyboard_queue_head] = scancode;
    keyboard_queue_head = next;
    return true;
}

bool QueuePop(uint8_t *scancode)
{
    if (scancode == NULL)
        return false;

    if (keyboard_queue_head == keyboard_queue_tail)
        return false;

    *scancode = keyboard_queue[keyboard_queue_tail];
    keyboard_queue_tail = queue_next(keyboard_queue_tail);
    return true;
}

void KeyboardQueueReset(void)
{
    keyboard_queue_head = 0;
    keyboard_queue_tail = 0;
}

static bool sequence_push(const struct keyboard_scancode_sequence *seq)
{
    if (seq == NULL || seq->len == 0)
        return false;

    /* Keep multi-byte extended key sequences contiguous, or drop them whole. */
    if (queue_free_slots() < seq->len)
        return false;

    for (uint8_t i = 0; i < seq->len; i++)
        QueuePush(seq->bytes[i]);

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

bool KeyboardTranslateEfiKeyToSet1(const EFI_INPUT_KEY *key,
                                   struct keyboard_scancode_sequence *seq)
{
    if (key == NULL || seq == NULL)
        return false;

    seq->len = 0;

    if (key->UnicodeChar >= L'a' && key->UnicodeChar <= L'z') {
        return set_single(letter_set1_make_codes[key->UnicodeChar - L'a'], seq);
    }

    if (key->UnicodeChar >= L'A' && key->UnicodeChar <= L'Z') {
        return set_single(letter_set1_make_codes[key->UnicodeChar - L'A'], seq);
    }

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

EFI_STATUS UefiKeyboardPoll(EFI_SIMPLE_TEXT_INPUT_PROTOCOL *con_in)
{
    EFI_STATUS status;
    EFI_INPUT_KEY key;
    struct keyboard_scancode_sequence seq;

    if (con_in == NULL)
        return EFI_INVALID_PARAMETER;

    for (;;) {
        status = con_in->ReadKeyStroke(con_in, &key);
        if (status == EFI_NOT_READY)
            return EFI_SUCCESS;
        if (EFI_ERROR(status))
            return status;

        if (KeyboardTranslateEfiKeyToSet1(&key, &seq))
            sequence_push(&seq);
    }
}

EFI_STATUS UefiKeyboardPollSystemTable(void)
{
    if (gST == NULL)
        return EFI_NOT_READY;

    return UefiKeyboardPoll(gST->ConIn);
}
