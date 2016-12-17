void play(){
 //play the sound from the first not to the last
 int size = sizeof(melody) / sizeof(int);
  
 for (int thisNote = 0; thisNote < size; thisNote++) {

       tone(13, melody[thisNote],8);
       // to distinguish the notes, set a minimum time between them.
       // the note's duration + 30% seems to work well:
       int pauseBetweenNotes = 8 * 1.30;
       delay(pauseBetweenNotes);}
}

