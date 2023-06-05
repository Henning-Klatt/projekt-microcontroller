#include <Arduino.h>

#include "main.h"
#include "pitches.h"

int melody[] = {

    // Nokia Ringtone
    // Score available at https://musescore.com/user/29944637/scores/5266155

    NOTE_E5,
    8,
    NOTE_D5,
    8,
    NOTE_FS4,
    4,
    NOTE_GS4,
    4,
    NOTE_CS5,
    8,
    NOTE_B4,
    8,
    NOTE_D4,
    4,
    NOTE_E4,
    4,
    NOTE_B4,
    8,
    NOTE_A4,
    8,
    NOTE_CS4,
    4,
    NOTE_E4,
    4,
    NOTE_A4,
    2,
};

int notes = sizeof(melody) / sizeof(melody[0]) / 2;
int wholenote = (60000 * 4) / 180;

int divider = 0, noteDuration = 0;

void playNokia()
{
    for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2)
    {

        // calculates the duration of each note
        divider = melody[thisNote + 1];
        if (divider > 0)
        {
            // regular note, just proceed
            noteDuration = (wholenote) / divider;
        }
        else if (divider < 0)
        {
            // dotted notes are represented with negative durations!!
            noteDuration = (wholenote) / abs(divider);
            noteDuration *= 1.5; // increases the duration in half for dotted notes
        }

        // we only play the note for 90% of the duration, leaving 10% as a pause
        tone(PUMP, melody[thisNote] * 6, noteDuration * 0.9);

        // Wait for the specief duration before playing the next note.
        delay(noteDuration);

        // stop the waveform generation before the next note.
        noTone(PUMP);
    }
}