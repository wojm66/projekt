template<typename T,int N>
class rbufor
{
public:
  rbufor():
    start(0),
    stop(0),
    iterator(-1)
    { };

  const T& ostatni() const {
    int idx = (stop - 1 + N) % N;
    return tablica[idx];
  }

  int size() const {
    if (stop >= start)
        return stop - start;
    else
        return N - (start - stop);
  }

  const T nastepny()
  {
    if(iterator>=0)
      {
  int index=iterator;
  ++iterator%=N;
  if(iterator==stop)
    {
      iterator=-1;
    }
  return tablica[index];
      }
    return T();
  };

  void dodaj(T rekord)
  {
    tablica[stop]=rekord;
    (++stop)%=N;
    if(stop==start)
      {
  (++start)%=N;
      }
  };

  void reset()
  {
    iterator=start;
  };

private:
  T tablica[N];
  int start;
  int stop;
  int iterator;
};
