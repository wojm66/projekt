#pragma once


class Bufor{
    public:

    int Size();
    const RemoteNode& Last();
    void Add(RemoteNode);
    void Reset();
    const RemoteNode& Next();

};