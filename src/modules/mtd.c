#include "board.h"
#include <string.h>

#include "mtd.h"
#ifdef F3_EVO
#include "spi_flash.h"
#endif
#include "fifo.h"


#define BUF_SIZE 4096

enum mtd_status
{
	MTD_ERASE,
	MTD_PROGRAM,
	MTD_PROGRAM_CONTINUE,
	MTD_IDLE,
};

static struct fifo_s write_fifo;
static uint8_t buf[BUF_SIZE];
#ifdef F3_EVO
static uint8_t page_buf[M25P16_PAGESIZE];
#else
static uint8_t page_buf[100];
#endif

static uint32_t write_addr;
static uint32_t read_addr;

static enum mtd_status status;

static bool has_erase;
static bool full;


void mtd_init()
{
	write_addr = 0;
	read_addr = 0;
	status = MTD_IDLE;
	has_erase = false;
	full = false;
	fifo_create(&write_fifo, buf, BUF_SIZE);
}

void mtd_test()
{
#ifdef F3_EVO	
	if(spi_flash_eraseSector(0))
//	while(1)
	{
		for(uint16_t i=0; i<spi_flash_getGeometry()->pageSize; i++)
		{
			page_buf[i] = i;
		}

		if(spi_flash_pageProgram(0, page_buf, spi_flash_getGeometry()->pageSize))
		{
			memset(page_buf, 0, spi_flash_getGeometry()->pageSize);

			uint16_t read_len = spi_flash_readBytes(0, page_buf, spi_flash_getGeometry()->pageSize);

			while(!spi_flash_eraseSector(read_addr));
 
            memset(page_buf, 0, spi_flash_getGeometry()->pageSize);
            spi_flash_readBytes(0, page_buf, spi_flash_getGeometry()->pageSize);

		}
	}
#endif	
}

void mtd_write(uint8_t* data, uint16_t len)
{
	if(mtd_get_space() > len)
	{
		for(uint16_t i=0; i<len; i++)
		{
			fifo_write(&write_fifo, data[i]);
		}
		full = false;
	}
	else
	{
		full = true;
	}
}

uint16_t mtd_read(uint32_t offset, uint8_t* data, uint16_t len)
{
#ifdef F3_EVO	
	uint16_t read_len = spi_flash_readBytes(offset, data, len);

	read_addr += read_len;

	if(read_addr == write_addr) read_addr = 0;

	return read_len;
#else 
	return 0;	
#endif	
}


void mtd_sync()
{
#ifdef F3_EVO	
	if(!has_erase && write_addr % spi_flash_getGeometry()->sectorSize ==0)
	{
		status = MTD_ERASE;
	}
	else if(fifo_get_count(&write_fifo) > spi_flash_getGeometry()->pageSize)
	{
		if(status != MTD_PROGRAM_CONTINUE)
		{
			status = MTD_PROGRAM;
		}
	}


	if(status == MTD_ERASE)
	{
		if(spi_flash_eraseSector(write_addr))
		{
			has_erase = true;
            status = MTD_IDLE;
//			memset(_page_buf, 0, spi_flash_getGeometry()->pageSize);
//
//			uint16_t read_len = spi_flash_readBytes(0, _page_buf, spi_flash_getGeometry()->pageSize);

		}
	}
	else if(status == MTD_PROGRAM)
	{
		for(uint16_t i=0; i<spi_flash_getGeometry()->pageSize; i++)
		{
			 fifo_read(&write_fifo, &page_buf[i]);
		}

		if(spi_flash_pageProgram(write_addr, page_buf, spi_flash_getGeometry()->pageSize))
		{
			write_addr += spi_flash_getGeometry()->pageSize;
			has_erase = false;
            status = MTD_IDLE;
            
//			memset(_page_buf, 0x5C, spi_flash_getGeometry()->pageSize);

//			uint16_t read_len = spi_flash_readBytes(0, _page_buf, spi_flash_getGeometry()->pageSize);
            

		}
		else
		{
			status = MTD_PROGRAM_CONTINUE;
		}
	}
	else if(status == MTD_PROGRAM_CONTINUE)
	{
		if(spi_flash_pageProgram(write_addr, page_buf, spi_flash_getGeometry()->pageSize))
		{
			write_addr += spi_flash_getGeometry()->pageSize;
			has_erase = false;
            status = MTD_IDLE;
		}
	}
#endif	
}

uint32_t mtd_get_space()
{
#ifdef F3_EVO	
	return spi_flash_getGeometry()->totalSize - write_addr;
#else 
	return 0;
#endif		
}

bool mtd_is_full()
{
	return full;
}

uint32_t mtd_get_store()
{
	return write_addr;
}

