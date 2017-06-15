/*
 * list.h
 *
 *  Created on: 21 нояб. 2016 г.
 *      Author: wera
 */

#ifndef SOURCE_ENGIE_MEMORY_RPLIST_H_
#define SOURCE_ENGIE_MEMORY_RPLIST_H_

namespace real_physics
{


template<class T>
class  rpListElement
{
  public:

    rpListElement()
    : m_next(0),
      m_prev(0)
    {
    }

    rpListElement(rpListElement *next, rpListElement *prev)
    : m_next(next),
      m_prev(prev)
    {
    }


    rpListElement *getNext() const { return m_next; }
    rpListElement *getPrev() const { return m_prev; }


    bool isHead() const { return m_prev == 0; }
    bool isTail() const { return m_next == 0; }


    void insertBefore(rpListElement *link)
    {
        m_next         = link;
        m_prev         = link->m_prev;
        m_next->m_prev = this;
        m_prev->m_next = this;
    }


    void insertAfter(rpListElement *link)
    {
        m_next         = link->m_next;
        m_prev         = link;
        m_next->m_prev = this;
        m_prev->m_next = this;
    }


    void remove()
    {
        m_next->m_prev = m_prev;
        m_prev->m_next = m_next;
    }



    ///Get real value
    T *getPointer() const;

    void setPointer(T *pointer);

private:

    rpListElement  *m_next;
    rpListElement  *m_prev;

    ///pointer value
    T *m_pointer;
};


///Get real value
template< class T >
T *rpListElement<T>::getPointer() const
{
   return m_pointer;
}

template< class T >
void rpListElement<T>::setPointer(T *pointer)
{
   m_pointer = pointer;
}


template<class T>
class rpList
{
  public:

    rpList()
    : m_head(&m_tail, 0),
	  m_tail(0, &m_head)
    {
    }

    rpListElement<T> *getHead() const { return m_head.getNext(); }
    rpListElement<T> *getTail() const { return m_tail.getPrev(); }

    void addHead(rpListElement<T> *link) { link->insertAfter(&m_head); }
    void addTail(rpListElement<T> *link) { link->insertBefore(&m_tail); }

 private:

    rpListElement<T> m_head;
    rpListElement<T> m_tail;
};




} /* namespace real_physics */

#endif /* SOURCE_ENGIE_MEMORY_RPLIST_H_ */
