template<typename T,int N, T DUMMY>
class rbufor
{
public:
  rbufor(): dummy(DUMMY),
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

  const T& nastepny()
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
    return dummy;
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
  const T dummy;
  int start;
  int stop;
  int iterator;
};
