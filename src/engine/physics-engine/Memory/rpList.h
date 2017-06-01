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


class rpGEN_Link
{
  public:

    rpGEN_Link() : m_next(0), m_prev(0)
    {
    }



    rpGEN_Link(rpGEN_Link *next, rpGEN_Link *prev)
    : m_next(next), m_prev(prev)
    {
    }


    rpGEN_Link *getNext() const { return m_next; }
    rpGEN_Link *getPrev() const { return m_prev; }


    bool isHead() const { return m_prev == 0; }
    bool isTail() const { return m_next == 0; }


    void insertBefore(rpGEN_Link *link)
    {
        m_next         = link;
        m_prev         = link->m_prev;
        m_next->m_prev = this;
        m_prev->m_next = this;
    }


    void insertAfter(rpGEN_Link *link)
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

 private:

    rpGEN_Link  *m_next;
    rpGEN_Link  *m_prev;
};


class rpGEN_List
{
  public:

    rpGEN_List()
    : m_head(&m_tail, 0),
	  m_tail(0, &m_head)
    {
    }

    rpGEN_Link *getHead() const { return m_head.getNext(); }
    rpGEN_Link *getTail() const { return m_tail.getPrev(); }

    void addHead(rpGEN_Link *link) { link->insertAfter(&m_head); }
    void addTail(rpGEN_Link *link) { link->insertBefore(&m_tail); }

 private:

    rpGEN_Link m_head;
    rpGEN_Link m_tail;
};


} /* namespace real_physics */

#endif /* SOURCE_ENGIE_MEMORY_RPLIST_H_ */
