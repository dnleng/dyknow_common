/**
 * Alternative to Boost::shared_ptr
 * Source: http://stackoverflow.com/a/7793459 (with minor modifications)
 *
 * Copyright (c) 2016 Emilio Garavaglia, Daniel de Leng
 */

#ifndef INCLUDE_MY_SHARED_PTR_HPP_
#define INCLUDE_MY_SHARED_PTR_HPP_

namespace dyknow {

template<class T>
class my_shared_ptr
{
    template<class U>
    friend class my_shared_ptr;
public:
    my_shared_ptr() :p(), c() {}
    explicit my_shared_ptr(T* s) :p(s), c(new unsigned(1)) {}

    my_shared_ptr(const my_shared_ptr& s) :p(s.p), c(s.c) { if(c) ++*c; }

    my_shared_ptr& operator=(const my_shared_ptr& s)
    { if(this!=&s) { clear(); p=s.p; c=s.c; if(c) ++*c; } return *this; }

    template<class U>
    my_shared_ptr(const my_shared_ptr<U>& s) :p(s.p), c(s.c) { if(c) ++*c; }

    ~my_shared_ptr() { clear(); }

    void clear()
    {
        if(c)
        {
            if(*c==1) delete p;
            if(!--*c) delete c;
        }
        c=0; p=0;
    }

    T* get() const { return (c)? p: 0; }
    T* operator->() const { return get(); }
    T& operator*() const { return *get(); }

private:
    T* p;
    unsigned* c;
};

} //namespace

#endif /* INCLUDE_MY_SHARED_PTR_HPP_ */
