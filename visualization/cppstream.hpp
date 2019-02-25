#ifndef CPP_STREAM_H
#define CPP_STREAM_H

#include <iostream>

class mystream : public std::streambuf
{
  protected:
    virtual std::streamsize xsputn(const char *s, std::streamsize n)
    {
        mexPrintf("%.*s", n, s);
        return n;
    }
    virtual int overflow(int c = EOF)
    {
        if (c != EOF)
        {
            mexPrintf("%.1s", &c);
        }
        return 1;
    }
};
class scoped_redirect_cout
{
  public:
    scoped_redirect_cout()
    {
        old_buf = std::cout.rdbuf();
        std::cout.rdbuf(&mout);
    }
    ~scoped_redirect_cout() { std::cout.rdbuf(old_buf); }

  private:
    mystream mout;
    std::streambuf *old_buf;
};

#endif