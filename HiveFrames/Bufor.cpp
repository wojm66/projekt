#include <Arduino.h>
#include "RemoteNode.h"
#include "rbuffor.h"
#include "Bufor.h"




int Bufor::Size(){
    return _buforRotacyjny.size();
}

const RemoteNode& Bufor::Last(){
    return _buforRotacyjny.ostatni();
}

void Bufor::Add(RemoteNode r){
    _buforRotacyjny.dodaj(r);
}

void Bufor::Reset(){
    _buforRotacyjny.reset();
}
const RemoteNode Bufor::Next(){
   return _buforRotacyjny.nastepny();
}