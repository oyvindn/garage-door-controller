#ifndef Debug_h
#define Debug_h

// #define DEBUG   //If you comment this line, the DPRINT & DPRINTLN lines are defined as blank.
#ifdef DEBUG
  #define DPRINT(...)    Serial.print(__VA_ARGS__)     // DPRINT is a macro, debug print
  #define DPRINTLN(...)  Serial.println(__VA_ARGS__)   // DPRINTLN is a macro, debug print with new line
#else
  #define DPRINT(...)     // Defines a blank line when debug is undefined
  #define DPRINTLN(...)   // Defines a blank line when debug is undefined
#endif

#endif