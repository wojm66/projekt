#include <Arduino.h>
#include "RemoteNode.h"
#include "Bufor.h"
#include "rbuffor.h"


rbufor<RemoteNode,40> _buforRotacyjny;

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
const RemoteNode& Bufor::Next(){
    _buforRotacyjny.nastepny();
}