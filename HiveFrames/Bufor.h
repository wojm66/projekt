#pragma once


class Bufor{
    public:
    rbufor<RemoteNode,40> _buforRotacyjny;

    int Size();
    const RemoteNode& Last();
    void Add(RemoteNode);
    void Reset();
    const RemoteNode& Next();

};