/*
 * BlockAllocator.h
 *
 *  Created on: 25 июн. 2017 г.
 *      Author: werqa
 */

#ifndef BLOCKALLOCATOR_H_
#define BLOCKALLOCATOR_H_

#include <stddef.h>
#include <stdlib.h>
#include <new>
#include <vector>


namespace real_physics
{

// todo: implement nothrow_t overloads, according to borisko' comment
// http://habrahabr.ru/post/148657/#comment_5020297


inline size_t align(size_t x, size_t a) { return ((x-1) | (a-1)) + 1; }
//#define align(x, a) ((((x)-1) | ((a)-1)) + 1)

template<size_t PageSize = 65536>
class PagePool
{
public:
        void* GetPage()
        {
                void* page = malloc(PageSize);
                pages.push_back(page);
                return page;
        }

        ~PagePool()
        {
        	for (auto i = pages.begin(); i != pages.end(); ++i)
        	{
        		free(*i);
        	}
        }
private:
        std::vector<void*> pages;
};


template<class T, size_t PageSize = 65536, size_t Alignment = 8 /* sizeof(void*) */>
class BlockPool : PagePool<PageSize>
{
public:
        BlockPool() : head(NULL)
        {
                BlockSize = align(sizeof(T), Alignment);
                count = PageSize / BlockSize;
        }

        void* AllocBlock()
        {
                // todo: lock(this)
                if (!head) FormatNewPage();
                void* tmp = head;
                head = *(void**)head;
                return tmp;
        }

        void FreeBlock(void* tmp)
        {
                // todo: lock(this)
                *(void**)tmp = head;
                head = tmp;
        }
private:
        void* head;
        size_t BlockSize;
        size_t count;

        void FormatNewPage()
        {

                void* tmp = this->GetPage();
                head = tmp;
                for(size_t i = 0; i < count-1; i++) {
                        void* next = (char*)tmp + BlockSize;
                        *(void**)tmp = next;
                        tmp = next;
                }
                *(void**)tmp = NULL;
        }
};


template<class T>
class BlockAlloc
{
public:
	static void* operator new(size_t s)
	{
//        if (s != sizeof(T))
//        {
//            return ::operator new(s);
//        }
		return pool.AllocBlock();
	}
	static void operator delete(void* m, size_t s)
	{
//        if (s != sizeof(T))
//        {
//            ::operator delete(m);
//        }
//        else
        if (m != NULL)
		{
			pool.FreeBlock(m);
		}
	}


	// Avoid hiding placement new that's needed by the stl containers...
	static void* operator new(size_t, void* m)
	{
		return m;
	}
	// ...and the warning about missing placement delete...
	static void operator delete(void*, void*)
	{
	}

private:
	static BlockPool<T> pool;
};


template<class T> BlockPool<T> BlockAlloc<T>::pool;


//***************************************************************************//

template<size_t PageSize = 65536, size_t Alignment = 8 /* sizeof(void*) */>
class PointerBumpAllocator
{
public:
        PointerBumpAllocator()
        : free(0) { }

        void* AllocBlock(size_t block)
        {
                // todo: lock(this)
                block = align(block, Alignment);
                if (block > free) {
                        free = align(block, PageSize);
                        head = GetPage(free);
                }
                void* tmp = head;
                head = (char*)head + block;
                free -= block;
                return tmp;
        }

        ~PointerBumpAllocator()
        {
        	for (auto i = pages.begin(); i != pages.end(); ++i)
        	{
        		free(*i);
        	}
        	pages.clear();
        }


private:
        void* GetPage(size_t size)
        {
                void* page = malloc(size);
                pages.push_back(page);
                return page;
        }

        std::vector<void*> pages;
        void* head;
        size_t free;
};
typedef PointerBumpAllocator<> DefaultAllocator;


template<class T, class A = DefaultAllocator>
struct ChildObject
{
        static void* operator new(size_t s, A& allocator)
        {
                return allocator.AllocBlock(s);
        }

        static void* operator new(size_t s, A* allocator)
        {
                return allocator->AllocBlock(s);
        }

        static void operator delete(void*, size_t) { } // *1
        static void operator delete(void*, A*) { }
        static void operator delete(void*, A&) { }
private:
        static void* operator new(size_t s);
};

}


#endif /* BLOCKALLOCATOR_H_ */
