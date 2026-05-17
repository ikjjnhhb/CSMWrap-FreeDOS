#ifndef CSMWRAP_KEYBOARD_H
#define CSMWRAP_KEYBOARD_H

#include <stdbool.h>
#include <stdint.h>
#include <efi.h>

/*
 * Fixed-size byte queue exposed to the legacy keyboard path.  The BIOS INT 9
 * / i8042 emulation layer should pop bytes from here and present them exactly
 * as if they had arrived from a PS/2 controller data port.
 */
bool QueuePush(uint8_t scancode);
bool QueuePop(uint8_t *scancode);
void KeyboardQueueReset(void);

struct keyboard_scancode_sequence {
    uint8_t bytes[2];
    uint8_t len;
};

/* Translation layer: EFI key event -> PS/2 Set 1 make-code byte sequence. */
bool KeyboardTranslateEfiKeyToSet1(const EFI_INPUT_KEY *key,
                                   struct keyboard_scancode_sequence *seq);

/* UEFI input layer: drain all pending firmware key events into the queue. */
EFI_STATUS UefiKeyboardPoll(EFI_SIMPLE_TEXT_INPUT_PROTOCOL *con_in);
EFI_STATUS UefiKeyboardPollSystemTable(void);

#endif
