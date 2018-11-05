#include "board.h"
#include "mm.h"

#define MM_MODULE_STATIC            1 
#define MM_MODULE_DYNAMIC_ADDR      2 
#define MM_MODULE_DYNAMIC_SIZE      3 

#define USE_MM  MM_MODULE_DYNAMIC_ADDR

#define BYTE_ALIGNMENT_MASK         (MM_BYTE_ALIGNMENT-1)


typedef struct BLOCK_LINK
{
	struct BLOCK_LINK *next;	
	size_t size;				
} BlockLink_t;

#define BLOCK_STRUCT_SIZE	   (sizeof(BlockLink_t) + ((size_t) (MM_BYTE_ALIGNMENT-1))) & ~((size_t)BYTE_ALIGNMENT_MASK);

static __align(4) uint8_t heap[MM_HEAP_SIZE];
static size_t mm_used = 0;

static BlockLink_t start, *end = NULL;


#if USE_MM == MM_MODULE_STATIC

void* mm_malloc(uint8_t s)
{
    void* m = NULL;

    if(s == 0) return m;

    if(s & BYTE_ALIGNMENT_MASK ) {
        s += ( MM_BYTE_ALIGNMENT - ( s & BYTE_ALIGNMENT_MASK ) );
    }

    if(((mm_used+s) < MM_HEAP_SIZE)) {
        m = heap + mm_used;
        mm_used += s;
    }    

    return m;
}

void mm_free(void* m)
{

}

#elif USE_MM == MM_MODULE_DYNAMIC_ADDR
void mm_init(void)
{
    size_t addr;
    BlockLink_t *first;

    start.next = (void*)heap;
	start.size = (size_t)0;

	addr = ((size_t)heap) + MM_HEAP_SIZE;
	addr -= BLOCK_STRUCT_SIZE;
	addr &= ~((size_t)BYTE_ALIGNMENT_MASK);
	end = (void*) addr;
	end->size = 0;
	end->next = NULL;    

    first = (void*)heap;
	first->size = addr - (size_t)first;
	first->next = end;    
}

void* mm_malloc(uint8_t s)
{
    BlockLink_t *curr, *prev, *new;
    void *ret = NULL;

    if(s == 0) return m;

    s += BLOCK_STRUCT_SIZE;
    if((s & BYTE_ALIGNMENT_MASK ) != 0x00 ) {
        s += (MM_BYTE_ALIGNMENT - (s & BYTE_ALIGNMENT_MASK));
    }

    
}

void mm_free(void* m)
{

}
#elif MM_USE MM_MODULE_DYNAMIC_SIZE
#else
    #error "must select one mm module"
#endif
