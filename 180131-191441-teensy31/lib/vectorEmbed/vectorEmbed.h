#ifndef _VECTOR_
#define _VECTOR_

#include <vector>



namespace std {
  void __throw_bad_alloc();
  
    //Serial.println("Unable to allocate memory");
  

  void __throw_length_error( char const*e );
  
    //Serial.print("Length Error :");
    //Serial.println(e);
  
}

#endif