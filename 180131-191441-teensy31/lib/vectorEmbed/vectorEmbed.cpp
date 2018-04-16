#include <vectorEmbed.h>
#include <Arduino.h>

void std::__throw_bad_alloc(){
    Serial.println("Unable to allocate memory");
}

void std::__throw_length_error( char const*e ){
    Serial.print("Length Error :");
    Serial.println(e);
}